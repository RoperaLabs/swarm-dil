"""Ground truth for the A -> B navigation study: terrain, vision cones, motion and radio.

The world owns everything agents must not see. Each slot it hands every agent only its own
cone readings (relative offsets), whether its last move happened, and the raw bytes other
agents broadcast that survived the radio channel (`swarm_dil.radio`).
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from ..radio import Links, Radio, contention
from .agent import DIRS, Agent, Spec, distance_field

START_CLEAR = 3  # cells kept free around A and B


@dataclass(frozen=True)
class Mission:
    n: int = 48                    # grid side, cells
    cell_m: float = 5.0            # metres per cell
    team: int = 8
    launch_every: int = 12         # slots between consecutive launches from A
    t_max: int = 600
    sensor_range: float = 5.0      # cells
    half_angle_deg: float = 60.0   # vision cone half-angle
    n_blocks: int = 10             # random rectangular obstacles
    n_traps: int = 4               # L-shaped pockets that open towards A (dead ends)
    radio: Radio = Radio(tx_dbm=-16.0, rate_bps=9_600.0)
    interferers: tuple = ()        # (x_cell, y_cell, dBm)
    protocol: str = "grace"
    memory_tiles: int | None = None
    budget: int = 64
    price: float = 0.5
    burst_lambda: float = 0.9


@dataclass
class Outcome:
    path: np.ndarray        # (team,) cells flown (+ true distance left if not arrived)
    arrived_at: np.ndarray  # (team,) slot of arrival, inf if never
    bytes_sent: np.ndarray  # (team,)
    d_star: float           # true shortest A -> B distance, cells
    replay: dict | None

    @property
    def regret(self):
        """Mean extra distance over the true shortest path, as a fraction of it."""
        return float(np.mean(self.path / self.d_star - 1.0))

    @property
    def arrival_rate(self):
        return float(np.isfinite(self.arrived_at).mean())


def make_terrain(n, rng, n_blocks, n_traps):
    """Random blocks plus L-shaped pockets facing A; retried until B is reachable from A."""
    start, goal = (2, 2), (n - 3, n - 3)
    for _ in range(200):
        rects = []
        for _ in range(n_blocks):
            w, h = rng.integers(2, 9, 2)
            x, y = rng.integers(0, n - w), rng.integers(0, n - h)
            rects.append((x, y, x + w, y + h))
        for _ in range(n_traps):
            L, k = rng.integers(7, 15), rng.integers(1, 3)
            x, y = rng.integers(4, n - L - 4), rng.integers(4, n - L - 4)
            rects.append((x, y + L, x + L + k, y + L + k))   # roof
            rects.append((x + L, y, x + L + k, y + L))       # east wall
        rects = [r for r in rects if not any(r[0] - START_CLEAR <= p[0] < r[2] + START_CLEAR and
                                             r[1] - START_CLEAR <= p[1] < r[3] + START_CLEAR for p in (start, goal))]
        blocked = np.zeros((n, n), bool)
        for x0, y0, x1, y1 in rects:
            blocked[x0:x1, y0:y1] = True
        field = distance_field(n, blocked.ravel(), goal)
        if np.isfinite(field[start[0] * n + start[1]]):
            return blocked, rects, start, goal, field
    raise RuntimeError("could not generate a connected terrain")


@lru_cache(maxsize=4)
def cone_rays(r, half_angle_deg):
    """Per heading, a list of rays; each ray is the ordered cells (dx, dy) it crosses."""
    out = []
    for hx, hy in DIRS:
        base = math.atan2(hy, hx)
        rays = []
        for a in np.arange(-half_angle_deg, half_angle_deg + 1e-9, 4.0):
            ang = base + math.radians(a)
            cells = []
            for d in np.arange(0.5, r + 1e-9, 0.25):
                c = (round(d * math.cos(ang)), round(d * math.sin(ang)))
                if c != (0, 0) and (not cells or cells[-1] != c):
                    cells.append(c)
            rays.append(tuple(cells))
        out.append(tuple(rays))
    return tuple(out)


def sense(blocked, pos, heading, rays):
    """Cells visible inside the cone, with line of sight: a ray stops at the first blocked cell."""
    n = blocked.shape[0]
    seen = {}
    for ray in rays[heading]:
        for dx, dy in ray:
            x, y = pos[0] + dx, pos[1] + dy
            if not (0 <= x < n and 0 <= y < n):
                break
            hit = bool(blocked[x, y])
            seen[(dx, dy)] = hit
            if hit:
                break
    return [(dx, dy, hit) for (dx, dy), hit in seen.items()]


def run(m=Mission(), seed=0, record=False):
    rng = np.random.default_rng(seed)
    blocked, rects, start, goal, true_field = make_terrain(m.n, rng, m.n_blocks, m.n_traps)
    n, N = m.n, m.team
    agents = [Agent(i, Spec(n=n, start=start, goal=goal, team=N, launch_at=i * m.launch_every,
                            protocol=m.protocol, sensor_range=m.sensor_range,
                            memory_tiles=m.memory_tiles, budget=m.budget, price=m.price,
                            burst_lambda=m.burst_lambda), rng) for i in range(N)]
    pos = [start] * N           # truth
    heading = [0] * N
    rays = cone_rays(m.sensor_range, m.half_angle_deg)
    c = m.cell_m
    links = Links(m.radio, np.zeros(N), [(x0 * c, y0 * c, x1 * c, y1 * c) for x0, y0, x1, y1 in rects],
                  [((x + 0.5) * c, (y + 0.5) * c, dbm) for x, y, dbm in m.interferers], rng)
    arrived_at = np.full(N, np.inf)
    replay = {"n": n, "blocked": np.argwhere(blocked).tolist(), "start": start, "goal": goal,
              "steps": []} if record else None

    for t in range(m.t_max):
        for i, a in enumerate(agents):
            a.observe(sense(blocked, pos[i], heading[i], rays))
        packets = [a.transmit() for a in agents]
        tx = np.array([p is not None for p in packets])
        edges = []
        if tx.any():
            xy = (np.array(pos, float) + 0.5) * c
            p = links.step(xy, np.ones(N, bool))
            airtime = np.array([len(p_) * 8 / m.radio.rate_bps if p_ else 0.0 for p_ in packets])
            q = p * contention(p, airtime)[:, None]
            got = rng.random((N, N)) < q
            for j in range(N):
                for i in np.flatnonzero(tx & got[j]):
                    if i != j:
                        agents[j].receive(packets[i])
                        edges.append((int(i), int(j)))
        for i, a in enumerate(agents):
            d = a.decide()
            ok = False
            if d is not None:
                dx, dy = DIRS[d]
                x, y = pos[i][0] + dx, pos[i][1] + dy
                ok = (0 <= x < n and 0 <= y < n and not blocked[x, y]
                      and not (dx and dy and (blocked[x, pos[i][1]] or blocked[pos[i][0], y])))
                heading[i] = d
                if ok:
                    pos[i] = (x, y)
            a.moved(d, ok)
            if a.arrived and not np.isfinite(arrived_at[i]):
                arrived_at[i] = t + 1
        if record:
            replay["steps"].append({"pos": [list(p_) for p_ in pos], "heading": list(heading),
                                    "edges": edges, "known0": sorted(map(list, agents[0].tiles))})
        if np.isfinite(arrived_at).all():
            break

    left = np.array([0.0 if a.arrived else true_field[pos[i][0] * n + pos[i][1]] for i, a in enumerate(agents)])
    return Outcome(np.array([a.path_len for a in agents]) + left, arrived_at,
                   np.array([a.bytes_sent for a in agents]), float(true_field[start[0] * n + start[1]]), replay)
