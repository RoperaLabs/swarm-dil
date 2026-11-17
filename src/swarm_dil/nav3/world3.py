"""Ground truth for the 3-D A -> B study: terrain, cone sensing, motion and radio.

The world owns everything agents must not see. Each slot it hands every agent only the blocked
voxels inside its cone (relative offsets), whether its last move happened, and the raw bytes
other agents broadcast that survived the channel.
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from ..radio import Radio, contention, path_loss_db
from .agent3 import DIRS, Agent, Spec, distance_field

CLEAR = 3  # voxels kept free around A and B


@dataclass(frozen=True)
class Mission3:
    shape: tuple = (40, 40, 10)    # voxels
    cell_m: float = 5.0            # metres per voxel
    team: int = 10
    ground: int = 0                # how many are ground vehicles (the last ids); the rest fly
    launch_every: int = 10
    t_max: int = 500
    sensor_range: float = 5.0      # voxels
    half_angle_deg: float = 60.0
    towers: int = 6                # boxes from the ground up, some to the ceiling
    walls: int = 2                 # long thin walls with one window (and a door if ground > 0)
    cups: int = 3                  # roofed pockets that open towards A: 3-D dead ends
    slabs: int = 3                 # floating slabs: fly under or over
    mounds: int = 4                # ellipsoid hills
    radio: Radio = Radio(tx_dbm=-16.0, rate_bps=9_600.0)
    voxel_db: float = 10.0         # loss per blocked voxel crossed by the line of sight
    block_cap_db: float = 40.0
    ground_db: float = 6.0         # extra loss per end of a link that sits on the ground
    interferers: tuple = ()        # (x, y, z, dBm)
    protocol: str = "cairn"
    memory_bytes: int | None = None
    budget: int = 64
    price: float = 0.5
    theta: float = 0.1
    delta: float = 1.0
    burst_lambda: float = 0.9


@dataclass
class Outcome3:
    path: np.ndarray        # (team,) voxels travelled (+ true distance left if not arrived)
    arrived_at: np.ndarray  # (team,) slot of arrival, inf if never
    bytes_sent: np.ndarray
    d_star: np.ndarray      # (team,) true shortest A -> B distance for each agent's class
    committed: np.ndarray   # (team,) the agent ended on a certified route
    commit_ratio: list      # U / V* at every commit (Thm. 2 says <= 1 + theta)
    fields: np.ndarray      # (team,) shortest-path searches run on board
    memory_peak: int        # bytes of map memory, max over agents and time
    sent: dict              # packet items by kind, team total
    bracket: list           # (slot, best lower bound on V*(A), best upper bound) over the team
    replay: dict | None

    @property
    def regret(self):
        """Mean extra distance over the true shortest path, as a fraction of it."""
        return float(np.mean(self.path / self.d_star - 1.0))

    @property
    def arrival_rate(self):
        return float(np.isfinite(self.arrived_at).mean())


def make_world(m, rng):
    """Random 3-D terrain; retried until B is reachable from A for every mobility class."""
    nx, ny, nz = m.shape
    start, goal = (2, 2, 0), (nx - 3, ny - 3, 0)
    bands = [(0, nz - 1)] + ([(0, 0)] if m.ground else [])
    X, Y, Z = np.indices(m.shape)
    for _ in range(300):
        b = np.zeros(m.shape, bool)
        for _ in range(m.towers):
            w, d = rng.integers(3, 9, 2)
            x, y = rng.integers(0, nx - w), rng.integers(0, ny - d)
            b[x:x + w, y:y + d, :nz if rng.random() < 0.5 else rng.integers(4, nz - 1)] = True
        for _ in range(m.mounds):
            a, c = rng.uniform(3, 7, 2)
            hgt = rng.uniform(3, nz - 2)
            cx, cy = rng.uniform(4, nx - 4), rng.uniform(4, ny - 4)
            b |= ((X - cx) / a) ** 2 + ((Y - cy) / c) ** 2 + (Z / hgt) ** 2 <= 1.0
        for _ in range(m.slabs):
            w, d = rng.integers(6, 15, 2)
            x, y, z = rng.integers(0, nx - w), rng.integers(0, ny - d), rng.integers(3, nz - 2)
            b[x:x + w, y:y + d, z:z + rng.integers(1, 3)] = True
        for _ in range(m.cups):
            L, W, H = rng.integers(6, 11), rng.integers(6, 11), rng.integers(4, nz)
            x, y = rng.integers(4, nx - L - 5), rng.integers(4, ny - W - 5)
            b[x + L, y:y + W + 1, :H + 1] = True          # far wall in x
            b[x:x + L + 1, y + W, :H + 1] = True          # far wall in y
            b[x:x + L + 1, y:y + W + 1, H] = True         # roof
        for _ in range(m.walls):
            length, thick = rng.integers(16, 31), rng.integers(1, 3)
            p, a0 = rng.integers(8, nx - 9), rng.integers(0, nx - length)
            top = nz if rng.random() < 0.6 else rng.integers(6, nz)
            wall = np.ones((length, thick, top), bool)
            g, zw = rng.integers(0, length - 3), rng.integers(0, max(top - 3, 1))
            wall[g:g + rng.integers(2, 4), :, zw:zw + 3] = False       # the window
            if m.ground:
                g = rng.integers(0, length - 2)
                wall[g:g + 2, :, :2] = False                          # a door at ground level
            if rng.random() < 0.5:
                b[a0:a0 + length, p:p + thick, :top] |= wall
            else:
                b[p:p + thick, a0:a0 + length, :top] |= wall.transpose(1, 0, 2)
        for c in (start, goal):
            b[max(c[0] - CLEAR, 0):c[0] + CLEAR + 1, max(c[1] - CLEAR, 0):c[1] + CLEAR + 1, :] = False
        fields = {}
        for zlo, zhi in bands:
            off = np.ones(m.shape, bool)
            off[:, :, zlo:zhi + 1] = False
            fields[(zlo, zhi)] = distance_field(m.shape, (b | off).ravel(), (goal[0] * ny + goal[1]) * nz + goal[2])
        if all(np.isfinite(f[(start[0] * ny + start[1]) * nz + start[2]]) for f in fields.values()):
            return b, start, goal, fields
    raise RuntimeError("could not generate a connected terrain")


@lru_cache(maxsize=4)
def cone_rays(r, half_angle_deg):
    """Per heading: integer voxel offsets, shape (rays, steps, 3), along rays filling the cone."""
    steps = np.arange(0.5, r + 1e-9, 0.25)
    out = []
    for d in DIRS:
        axis = np.array(d, float) / np.linalg.norm(d)
        e1 = np.cross(axis, (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0))
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(axis, e1)
        dirs = [axis]
        for polar in np.radians(np.arange(10.0, half_angle_deg + 1e-9, 10.0)):
            for az in np.linspace(0, 2 * math.pi, max(6, round(36 * math.sin(polar))), endpoint=False):
                dirs.append(math.cos(polar) * axis + math.sin(polar) * (math.cos(az) * e1 + math.sin(az) * e2))
        out.append(np.rint(np.array(dirs)[:, None, :] * steps[None, :, None]).astype(int))
    return tuple(out)


def sense(blocked, pos, rays):
    """Blocked voxels visible in the cone (relative offsets): each ray stops at its first hit."""
    c = np.asarray(pos) + rays
    inb = ((c >= 0) & (c < blocked.shape)).all(2)
    hit = np.zeros(inb.shape, bool)
    hit[inb] = blocked[c[inb][:, 0], c[inb][:, 1], c[inb][:, 2]]
    stop = hit | ~inb
    first = stop.argmax(1)
    rows = np.flatnonzero(stop.any(1))
    rows = rows[hit[rows, first[rows]]]
    return np.unique(rays[rows, first[rows]], axis=0).tolist()


class Links3:
    """Per-slot delivery probabilities in 3-D: path loss, solid matter on the line of sight,
    AR(1) shadowing per link (bursty fades), interferers and near-ground loss."""

    def __init__(self, m, blocked, n, on_ground, rng):
        self.m, self.b, self.rng, self.n = m, blocked, rng, n
        self.gain = -m.ground_db * np.asarray(on_ground, float)
        self.shadow = self._noise()

    def _noise(self):
        s = np.triu(self.rng.normal(0, self.m.radio.shadow_sigma_db, (self.n, self.n)), 1)
        return s + s.T

    def _solid(self, a, b):
        """Voxels of solid matter crossed by each segment a -> b (voxel coordinates)."""
        t = np.linspace(0.0, 1.0, 48)[:, None]
        pts = np.rint(a[..., None, :] + (b - a)[..., None, :] * t).astype(int)
        pts = np.clip(pts, 0, np.array(self.b.shape) - 1)
        frac = self.b[pts[..., 0], pts[..., 1], pts[..., 2]].mean(-1)
        return frac * np.linalg.norm(b - a, axis=-1)

    def step(self, pos):
        """pos: (n, 3) voxel coordinates. Returns p[i, j] = P(packet from j reaches i)."""
        m, r = self.m, self.m.radio
        self.shadow = r.shadow_corr * self.shadow + math.sqrt(1 - r.shadow_corr ** 2) * self._noise()
        a, b = np.broadcast_arrays(pos[:, None, :], pos[None, :, :])
        d = np.linalg.norm(a - b, axis=2) * m.cell_m
        loss = np.minimum(m.block_cap_db, m.voxel_db * self._solid(a, b))
        rx = r.tx_dbm + self.gain[:, None] + self.gain[None] - path_loss_db(r, d) - loss + self.shadow
        floor_mw = np.full(self.n, 10 ** (r.noise_dbm / 10))
        for x, y, z, dbm in m.interferers:
            src = np.broadcast_to(np.array([x, y, z], float), pos.shape)
            di = np.linalg.norm(pos - src, axis=1) * m.cell_m
            li = np.minimum(m.block_cap_db, m.voxel_db * self._solid(pos, src))
            floor_mw += 10 ** ((dbm + self.gain - path_loss_db(r, di) - li) / 10)
        sinr = rx - 10 * np.log10(floor_mw)[:, None]
        p = 1 / (1 + np.exp(-(sinr - r.sinr50_db) / r.sinr_width_db))
        np.fill_diagonal(p, 0.0)
        return p


def run(m=Mission3(), seed=0, record=False, check=False):
    """One mission. check=True asserts every bound an agent holds against the ground truth."""
    rng = np.random.default_rng(seed)
    blocked, start, goal, true = make_world(m, rng)
    nx, ny, nz = m.shape
    N = m.team
    ix = lambda c: (c[0] * ny + c[1]) * nz + c[2]
    bands = tuple((0, 0) if i >= N - m.ground else (0, nz - 1) for i in range(N))
    agents = [Agent(i, Spec(shape=m.shape, start=start, goal=goal, team=N, launch_at=i * m.launch_every,
                            bands=bands, protocol="none" if m.protocol.startswith("oracle") else m.protocol,
                            sensor_range=m.sensor_range,
                            memory_bytes=m.memory_bytes, budget=m.budget, price=m.price, theta=m.theta,
                            delta=m.delta, burst_lambda=m.burst_lambda), rng) for i in range(N)]
    pos = [start] * N
    heading = [a.heading for a in agents]
    rays = cone_rays(m.sensor_range, m.half_angle_deg)
    links = Links3(m, blocked, N, [b == (0, 0) for b in bands], rng)
    arrived_at = np.full(N, np.inf)
    d_star = np.array([true[b][ix(start)] for b in bands])
    was_committed = [False] * N
    commit_ratio, bracket, mem_peak = [], [], 0
    replay = {"shape": list(m.shape), "blocked": np.argwhere(blocked).tolist(), "start": start, "goal": goal,
              "ground": m.ground, "steps": []} if record else None

    for t in range(m.t_max):
        for i, a in enumerate(agents):
            a.observe(sense(blocked, pos[i], rays[heading[i]]))
        if m.protocol.startswith("oracle"):    # references: obstacles shared for free
            own = [dict(a.tiles) for a in agents]
            if m.protocol == "oracle":         # ... with everybody, instantly
                hears = np.ones((N, N), bool)
            else:                              # "oracle-links": over the real radio links, unlimited bytes
                hears = rng.random((N, N)) < links.step(np.array(pos, float))
            for j, a in enumerate(agents):
                for i in np.flatnonzero(hears[j]):
                    if i != j:
                        for key, mask in own[i].items():
                            a._merge(key, mask)
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
                        edges.append((int(i), int(j)))
        for i, a in enumerate(agents):
            d = a.decide()
            ok = False
            if d is not None:
                c = tuple(pos[i][k] + DIRS[d][k] for k in range(3))
                ok = all(0 <= c[k] < m.shape[k] for k in range(3)) and not blocked[c] and bands[i][0] <= c[2] <= bands[i][1]
                heading[i] = d
                if ok:
                    pos[i] = c
            if a.committed and not was_committed[i]:
                commit_ratio.append(float(a.plan_cost / max(true[bands[i]][ix(a.pos)], 1e-9)))
            was_committed[i] = a.committed
            a.moved(d, ok)
            if a.arrived and not np.isfinite(arrived_at[i]):
                arrived_at[i] = t + 1
            if check:
                v = true[bands[i]]
                assert a.pos == pos[i], "dead reckoning drifted"
                assert a.lb <= v[ix(pos[i])] + 1e-6, f"agent {i} holds an unsound lower bound at slot {t}"
                assert a.team_v <= v[ix(start)] + 1e-6, f"agent {i} holds an unsound route value at slot {t}"
                assert a.ver_u is None or a.ver_u >= v[ix(start)] - 1e-6, f"agent {i} holds an unsound upper bound"
                assert not (a.blocked & ~blocked.ravel()).any(), f"agent {i} believes in a false obstacle"
        mem_peak = max(mem_peak, max(a.memory_used for a in agents))
        fly = [a for i, a in enumerate(agents) if bands[i][1] > 0] or agents
        bracket.append((t, max(a.team_v for a in fly), min((a.ver_u for a in fly if a.ver_u is not None), default=math.inf)))
        if record:
            replay["steps"].append({"pos": [list(q) for q in pos], "edges": edges,
                                    "state": [2 if a.committed else 1 if a.lost else 0 for a in agents]})
        if np.isfinite(arrived_at).all():
            break

    left = np.array([0.0 if a.arrived else true[bands[i]][ix(pos[i])] for i, a in enumerate(agents)])
    sent = {k: sum(a.sent[k] for a in agents) for k in agents[0].sent}
    return Outcome3(np.array([a.path_len for a in agents]) + left, arrived_at,
                    np.array([a.bytes_sent for a in agents]), d_star, np.array([a.committed for a in agents]),
                    commit_ratio, np.array([a.n_fields for a in agents]), mem_peak, sent, bracket, replay)
