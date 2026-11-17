"""Ground truth for GRACE-3D 2.0.0 (docs/v2_spec.md §1): the v0.4 terrain, radio and contention, a multizone ToF
sensor, and the slot loop. Hosts v2 agents and the v0.4 baselines (none, flood, grace, cairn) on the same
terrain, links and sensor for a given seed, so comparisons are paired."""
import math
from dataclasses import dataclass, field, replace
from functools import lru_cache

import numpy as np

from swarm_dil.nav3 import agent3
from swarm_dil.nav3.world3 import Links3, Mission3, make_world, sense
from swarm_dil.radio import contention

from . import agent as v2
from .geom import STEPS, cells_of, h3
from .ops import BUDGET


@dataclass(frozen=True)
class Mission(Mission3):
    protocol: str = "v2"
    K: int = 12                    # v2 map memory, tiles
    zones: int = 8                 # each ToF array is zones x zones
    fov_deg: float = 60.0
    p_esc: float = 0.05
    mu: float = 4.0
    rho: float = 0.25
    slice: int = 40
    arena: int = 2048
    horizon: int = 2


@dataclass
class Outcome:
    path: np.ndarray               # (team,) voxels travelled, plus the true distance left if not arrived
    arrived_at: np.ndarray
    bytes_sent: np.ndarray
    d_star: np.ndarray
    committed: np.ndarray
    commit_ratio: list             # U / V*(A) at every commit (T2.8: <= 1 + theta)
    cycles: np.ndarray             # cycles of every active agent-slot (v2 only)
    ingest_cycles: np.ndarray      # cycles of step 1 per agent-slot (v2 only)
    memory_peak: int               # persistent bytes (v2) or map bytes (v0.4), max over agents and time
    plans: np.ndarray
    escapes: np.ndarray
    fit: list = field(default_factory=list)    # per arrived v2 agent: did its walk fit the 12-waypoint record
    sent: dict = field(default_factory=dict)

    @property
    def regret(self):
        return float(np.mean(self.path / self.d_star - 1.0))

    @property
    def arrival_rate(self):
        return float(np.isfinite(self.arrived_at).mean())


@lru_cache(maxsize=8)
def zone_rays(r, zones, fov_deg):
    """Per heading and phase: integer offsets (rays, steps, 3) of two zones x zones arrays, one along the
    heading, one along one of the four perpendicular directions (phase = slot mod 4). One ray per zone,
    through its centre; the world keeps the first hit."""
    steps = np.arange(0.5, r + 1e-9, 0.25)
    tans = np.tan(np.radians((np.arange(zones) + 0.5) / zones * fov_deg - fov_deg / 2))
    out = []
    for d in STEPS:
        axis = np.array(d, float) / np.linalg.norm(d)
        e1 = np.cross(axis, (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0))
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(axis, e1)

        def array(a, b, c):
            dirs = a[None, None] + tans[:, None, None] * b[None, None] + tans[None, :, None] * c[None, None]
            dirs = (dirs / np.linalg.norm(dirs, axis=2, keepdims=True)).reshape(-1, 3)
            return np.rint(dirs[:, None, :] * steps[None, :, None]).astype(int)

        front = array(axis, e1, e2)
        out.append(tuple(np.concatenate([front, array(side, axis, other)])
                         for side, other in ((e1, e2), (-e1, e2), (e2, e1), (-e2, e1))))
    return tuple(out)


def run(m=Mission(), seed=0, check=False, trace=None):
    """One mission. check=True asserts every bound a v2 agent holds against the ground truth. A list passed as trace
    receives one record per slot (positions, plans, radio deliveries, map items each agent sent, arrivals, and a v2
    agent's tiles), for animations."""
    rng = np.random.default_rng(seed)
    blocked, start, goal, true = make_world(m, rng)
    nx, ny, nz = m.shape
    N = m.team
    ix = lambda c: (c[0] * ny + c[1]) * nz + c[2]  # noqa: E731
    bands = tuple((0, 0) if i >= N - m.ground else (0, nz - 1) for i in range(N))
    is_v2 = m.protocol.startswith("v2")
    if is_v2:
        agents = [v2.Agent(i, v2.Spec(shape=m.shape, start=start, goal=goal, team=N, launch_at=i * m.launch_every,
                                      band=bands[i], protocol=m.protocol, K=m.K, budget=m.budget, price=m.price,
                                      theta=m.theta, r=m.sensor_range, p_esc=m.p_esc, mu=m.mu, rho=m.rho, slice=m.slice,
                                      arena=m.arena, horizon=m.horizon, seed=seed)) for i in range(N)]
    else:
        agents = [agent3.Agent(i, agent3.Spec(shape=m.shape, start=start, goal=goal, team=N,
                                              launch_at=i * m.launch_every, bands=bands, protocol=m.protocol,
                                              sensor_range=m.sensor_range, memory_bytes=m.memory_bytes,
                                              budget=m.budget, price=m.price, theta=m.theta, delta=m.delta,
                                              burst_lambda=m.burst_lambda), rng) for i in range(N)]
    pos = [start] * N
    rays = zone_rays(m.sensor_range, m.zones, m.fov_deg)
    links = Links3(m, blocked, N, [b == (0, 0) for b in bands], rng)
    arrived_at = np.full(N, np.inf)
    d_star = np.array([true[b][ix(start)] for b in bands])
    was_committed = [False] * N
    commit_ratio, cycles, ingest, mem_peak = [], [], [], 0
    map_sent = [0] * N
    for t in range(m.t_max):
        for i, a in enumerate(agents):
            hits = sense(blocked, pos[i], rays[a.heading][t % 4])
            a.observe(hits)
        packets = [a.transmit() for a in agents]
        tx = np.array([p is not None for p in packets])
        edges = []
        if tx.any():
            p = links.step(np.array(pos, float))
            airtime = np.array([len(q) * 8 / m.radio.rate_bps if q else 0.0 for q in packets])
            got = rng.random((N, N)) < p * contention(p, airtime)[:, None]
            for j in range(N):
                for i in np.flatnonzero(tx & got[j]):
                    if i != j:
                        agents[j].receive(packets[i])
                        edges.append((int(i), j))
        for i, a in enumerate(agents):
            active = not a.arrived
            d = a.decide()
            ok = False
            if d is not None:
                c = tuple(pos[i][k] + STEPS[d][k] for k in range(3))
                ok = (all(0 <= c[k] < m.shape[k] for k in range(3)) and not blocked[c]
                      and bands[i][0] <= c[2] <= bands[i][1])
                if ok:
                    pos[i] = c
            if not is_v2:
                a.heading = d if d is not None else a.heading
            a.moved(d, ok)
            if a.committed and not was_committed[i]:
                commit_ratio.append(float((a.U_ver if is_v2 else a.plan_cost) / max(true[bands[i]][ix(start if is_v2 else a.pos)], 1e-9)))
            was_committed[i] = a.committed
            if a.arrived and not np.isfinite(arrived_at[i]):
                arrived_at[i] = t + 1
            if is_v2:
                if active:
                    cycles.append(a.ops.cycles())
                    ingest.append(a.ingest_cycles)
                if check:
                    v = true[bands[i]]
                    assert a.pos == pos[i], "dead reckoning drifted"
                    assert a.L <= v[ix(pos[i])] + 1e-6, f"agent {i}: unsound lower bound at slot {t}"
                    assert a.L_team <= v[ix(start)] + 1e-6, f"agent {i}: unsound team bound at slot {t}"
                    assert a.U_ver is None or a.U_ver >= v[ix(start)] - 1e-6, f"agent {i}: unsound route cost"
                    assert a.memory_used <= 256 and len(a.plan) <= 8 and len(a.tiles) <= a.s.K
                    for key, mask in a.tiles.items():
                        assert all(blocked[c] for c in cells_of(key, mask)), f"agent {i} believes a false obstacle"
        mem_peak = max(mem_peak, max(a.memory_used for a in agents))
        if trace is not None:
            now = [a.sent.get("tile", 0) + a.sent.get("box", 0) for a in agents]
            trace.append(dict(pos=[list(map(int, q)) for q in pos], edges=edges,
                              map_tx=[x - y for x, y in zip(now, map_sent)],
                              plan=[[list(map(int, a.pos))] + [list(map(int, w)) for w in a.plan] for a in agents],
                              tiles=[dict(a.tiles) for a in agents] if is_v2 else None,
                              arrived=[int(a.arrived) for a in agents], bytes=int(sum(a.bytes_sent for a in agents))))
            map_sent = now
        if np.isfinite(arrived_at).all():
            break
    left = np.array([0.0 if a.arrived else true[bands[i]][ix(pos[i])] for i, a in enumerate(agents)])
    sent = {k: sum(a.sent.get(k, 0) for a in agents) for k in agents[0].sent}
    return Outcome(np.array([a.path_len for a in agents]) + left, arrived_at,
                   np.array([a.bytes_sent for a in agents]), d_star, np.array([a.committed for a in agents]),
                   commit_ratio, np.array(cycles), np.array(ingest), mem_peak,
                   np.array([getattr(a, "n_plans", getattr(a, "n_fields", 0)) for a in agents]),
                   np.array([getattr(a, "escapes", 0) for a in agents]),
                   [a.walk_ok for a in agents if is_v2 and a.arrived], sent)
