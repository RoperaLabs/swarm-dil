"""Area search by a UAV swarm over degraded radio links, under three coordination strategies.

centralised:   UAVs report to a ground station (C2), which uploads each one a route of
               `mission_cells` cells; a UAV flies its route even out of link and, once done
               without new orders for `stale_after` s, returns towards C2 (lost-link failsafe).
               UAVs do not talk to each other.
decentralised: UAVs gossip searched cells, located targets and goal claims over a mesh and
               resolve clashing claims locally (the closest UAV keeps the cell). The ground
               station only listens.
partition:     UAVs do not coordinate; each sweeps a strip fixed before launch, then searches
               whatever it has not seen itself. They only report located targets to C2.

Every message has a size and goes through the radio model in `radio.py`: range, obstacles,
interference, correlated fading and channel contention. A UAV transmits at most `msg_bytes`
per second and remembers at most `memory_cells` searched cells. A target counts as rescued
once C2 knows about it, not when a UAV sees it.
"""
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .radio import Links, Radio, contention

STRATEGIES = ("centralised", "decentralised", "partition")
NO_GOAL = -1
HEADER_BYTES = 32   # sender id, time, position, goal, sequence number
EVENT_BYTES = 8     # one located target
CELL_BYTES = 2      # one searched-cell index (uint16)


@dataclass(frozen=True)
class Scenario:
    n_agents: int = 20
    area: float = 1000.0              # m, side of the square search area
    cell: float = 25.0                # m, grid cell = sensor footprint
    speed: float = 10.0               # m/s
    n_targets: int = 10
    base_xy: tuple = (50.0, 50.0)     # ground station and launch point, at the area's edge
    obstacles: tuple = ()             # (x0, y0, x1, y1): no-fly, not searchable, blocks radio
    interferers: tuple = ()           # (x, y, dBm) emitters raising the noise floor
    radio: Radio = Radio()
    msg_bytes: int = 256              # max bytes a UAV transmits per second
    memory_cells: int | None = None   # max searched cells a UAV remembers; None = whole map
    attrition: float = 0.0            # fraction of UAVs that fail during the mission
    fail_window: float = 300.0        # s, failures happen uniformly in [0, fail_window]
    stale_after: int = 30             # s, reports and claims older than this are ignored
    mission_cells: int = 20           # cells per route C2 uploads to a UAV (centralised)
    t_max: int = 1200                 # s, mission time limit; one step = 1 s


# Reference terrain: a ridge, a cliff band, two clusters of buildings; C2 post at the SW corner.
REFERENCE_OBSTACLES = ((300, 150, 340, 650), (550, 820, 620, 1000), (600, 480, 760, 540), (760, 150, 830, 260))
REFERENCE_INTERFERER_XY = (650.0, 700.0)


def reference(interferer_dbm=None, **kw):
    """Reference scenario used by the experiments, optionally with one field interferer."""
    intf = () if interferer_dbm is None else ((*REFERENCE_INTERFERER_XY, float(interferer_dbm)),)
    return Scenario(obstacles=REFERENCE_OBSTACLES, interferers=intf, **kw)


@dataclass
class Result:
    coverage: np.ndarray    # (t_max,) fraction of free cells searched after each second
    found: np.ndarray       # (n_targets,) first time a UAV saw each target, inf if never
    reported: np.ndarray    # (n_targets,) first time C2 knew about each target, inf if never
    covered_at: np.ndarray  # (cells,) second each cell was first searched, inf if never
    trace: np.ndarray       # (t_max, n_agents, 2) positions in m
    alive: np.ndarray       # (t_max, n_agents) bool
    targets_xy: np.ndarray  # (n_targets, 2) m
    free: np.ndarray        # (cells,) bool, False inside obstacles
    bytes_per_uav: float    # mean bytes transmitted per live UAV per second
    peak_memory: int        # most searched cells any UAV held at once

    @property
    def time_all_found(self):
        return float(self.found.max())

    @property
    def time_all_reported(self):
        return float(self.reported.max())


@lru_cache(maxsize=4)
def grid(n, cell, obstacles):
    """Free cells, 8-neighbour table and all-pairs travel distance (m) around obstacles."""
    ix, iy = np.indices((n, n)).reshape(2, -1)
    cx, cy = (ix + 0.5) * cell, (iy + 0.5) * cell
    free = np.ones(n * n, bool)
    for x0, y0, x1, y1 in obstacles:
        free &= ~((cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1))
    F = free.reshape(n, n)

    moves = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy]
    nbr = np.full((n * n, 8), NO_GOAL)
    step = np.full((n * n, 8), np.inf)
    for m, (dx, dy) in enumerate(moves):
        tx, ty = ix + dx, iy + dy
        txc, tyc = np.clip(tx, 0, n - 1), np.clip(ty, 0, n - 1)
        ok = (tx >= 0) & (tx < n) & (ty >= 0) & (ty < n) & free & F[txc, tyc]
        if dx and dy:  # no cutting corners past an obstacle
            ok &= F[txc, iy] & F[ix, tyc]
        nbr[ok, m] = (tx * n + ty)[ok]
        step[ok, m] = np.hypot(dx, dy) * cell

    # Bellman-Ford style relaxation on the image grid, all sources at once
    D = np.full((n * n, n, n), np.inf, np.float32)
    D.reshape(n * n, -1)[np.arange(n * n), np.arange(n * n)] = 0
    shifts = []
    for m, (dx, dy) in enumerate(moves):
        dst = (slice(max(dx, 0), n + min(dx, 0)), slice(max(dy, 0), n + min(dy, 0)))
        src = (slice(max(-dx, 0), n - max(dx, 0)), slice(max(-dy, 0), n - max(dy, 0)))
        cost = step[:, m].reshape(n, n)[src].astype(np.float32)  # cost of leaving src that way
        shifts.append((dst, src, cost))
    while True:
        before = D.copy()
        for dst, src, cost in shifts:
            np.minimum(D[:, dst[0], dst[1]], D[:, src[0], src[1]] + cost, out=D[:, dst[0], dst[1]])
        if np.array_equal(before, D):
            break
    return free, nbr, step, D.reshape(n * n, n * n)


def run(strategy, sc=Scenario(), seed=0):
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown strategy {strategy!r}; choose from {STRATEGIES}")
    n = round(sc.area / sc.cell)
    if not np.isclose(n * sc.cell, sc.area):
        raise ValueError("area must be a whole number of cells")
    N, G, rng = sc.n_agents, n * n, np.random.default_rng(seed)
    free, nbr, step, D = grid(n, sc.cell, tuple(map(tuple, sc.obstacles)))
    ix, iy = np.indices((n, n)).reshape(2, -1)
    centres = (np.stack([ix, iy], 1) + 0.5) * sc.cell
    sector = ix * N // n  # strip index per cell, for the partition strategy

    def cell_of(p):
        c = np.clip((p // sc.cell).astype(int), 0, n - 1)
        return c[..., 0] * n + c[..., 1]

    def nearest(c, mask):
        idx = np.flatnonzero(mask)
        if idx.size == 0:
            return NO_GOAL
        d = D[c, idx]
        return idx[np.argmin(d)] if np.isfinite(d).any() else NO_GOAL

    base = np.asarray(sc.base_xy, float)
    if not free[cell_of(base)]:
        raise ValueError("ground station is inside an obstacle")
    reachable_cells = free & np.isfinite(D[cell_of(base)])
    targets = rng.choice(np.flatnonzero(reachable_cells), sc.n_targets, replace=False)
    pos = np.tile(centres[cell_of(base)], (N, 1))
    wp = cell_of(pos)  # next cell centre each UAV is flying to
    fail_at = np.full(N, np.inf)
    n_fail = round(sc.attrition * N)
    fail_at[rng.choice(N, n_fail, replace=False)] = rng.uniform(0, sc.fail_window, n_fail)

    known = np.zeros((N, G), bool)           # searched cells each UAV remembers
    learned = np.full((N, G), -np.inf)       # when it learned each one
    known_found = np.zeros((N, sc.n_targets), bool)
    covered_at = np.full(G, np.inf)
    found = np.full(sc.n_targets, np.inf)
    reported = np.full(sc.n_targets, np.inf)
    goal = np.full(N, NO_GOAL)
    trace = np.empty((sc.t_max, N, 2))
    alive_log = np.empty((sc.t_max, N), bool)
    links = Links(sc.radio, np.r_[np.zeros(N), sc.radio.gs_gain_db], sc.obstacles, sc.interferers, rng)
    BS = N  # node index of the ground station
    sent_bytes, uav_seconds, peak = 0.0, 0, 0

    c2_known = np.zeros(G, bool)
    c2_pos = pos.copy()
    c2_heard = np.zeros(N)
    mission = [np.empty(0, int) for _ in range(N)]      # route C2 has planned for each UAV
    uav_mission = [np.empty(0, int) for _ in range(N)]  # route each UAV last received
    last_order = np.zeros(N)
    claim = np.full((N, N), NO_GOAL)  # claim[i, j]: goal of j as last heard by i
    claim_pos = np.zeros((N, N, 2))
    heard = np.full((N, N), -np.inf)

    def learn(rows, cells, t):
        sub = np.ix_(rows, cells)
        learned[sub] = np.where(known[sub], learned[sub], t)
        known[sub] = True

    def payload(i):
        """Searched cells i can fit in its message: mostly the newest, some older at random."""
        room = (sc.msg_bytes - HEADER_BYTES - EVENT_BYTES * known_found[i].sum()) // CELL_BYTES
        idx = np.flatnonzero(known[i])
        if room <= 0:
            return idx[:0]
        if idx.size <= room:
            return idx
        n_new = room - room // 4
        order = np.argpartition(-learned[i, idx], n_new)
        return np.r_[idx[order[:n_new]], rng.choice(idx[order[n_new:]], room - n_new, replace=False)]

    t_end = sc.t_max
    for t in range(sc.t_max):
        alive = fail_at > t
        live = np.flatnonzero(alive)
        trace[t], alive_log[t] = pos, alive

        # sense
        cells = cell_of(pos[live])
        new = ~known[live, cells]
        learned[live[new], cells[new]] = t
        known[live, cells] = True
        covered_at[cells] = np.minimum(covered_at[cells], t)
        seen = cells[:, None] == targets[None]
        known_found[live] |= seen
        found[seen.any(0)] = np.minimum(found[seen.any(0)], t)

        # communicate
        nodes = np.vstack([pos, base])
        active = np.r_[alive, True]
        p = links.step(nodes, active)
        tx = np.zeros(N + 1, bool)
        size = np.zeros(N + 1)
        msgs = {}
        for i in live:
            msgs[i] = payload(i) if strategy != "partition" else np.empty(0, int)
            size[i] = HEADER_BYTES + EVENT_BYTES * known_found[i].sum() + CELL_BYTES * msgs[i].size
            tx[i] = True
        if strategy == "centralised":
            tx[BS] = True
            size[BS] = HEADER_BYTES + sum(4 + CELL_BYTES * m.size for m in mission)
        airtime = size * 8 / sc.radio.rate_bps
        q = p * contention(p, airtime)[:, None] * tx[None]
        got = rng.random(q.shape) < q  # got[i, j]: i received j's message
        sent_bytes += size[:N].sum()
        uav_seconds += live.size

        if strategy == "decentralised":
            mesh = got[:N, :N]
            for j in live:
                rows = np.flatnonzero(mesh[:, j])
                if rows.size:
                    learn(rows, msgs[j], t)
                    known_found[rows] |= known_found[j]
            rows, cols = np.nonzero(mesh)
            claim[rows, cols] = goal[cols]
            claim_pos[rows, cols] = pos[cols]
            heard[rows, cols] = t
        to_c2 = np.flatnonzero(got[BS, :N])
        if to_c2.size:
            new = known_found[to_c2].any(0) & np.isinf(reported)
            reported[new] = t
            if strategy == "centralised":
                for j in to_c2:
                    c2_known[msgs[j]] = True
                c2_pos[to_c2], c2_heard[to_c2] = pos[to_c2], t

        if sc.memory_cells is not None:
            for i in live:
                excess = known[i].sum() - sc.memory_cells
                if excess > 0:
                    oldest = np.argpartition(np.where(known[i], learned[i], np.inf), excess)[:excess]
                    known[i, oldest] = False
                    learned[i, oldest] = -np.inf
        peak = max(peak, int(known.sum(1).max()))

        if np.isfinite(reported).all():
            t_end = t + 1
            break

        # plan
        if strategy == "centralised":
            fresh = t - c2_heard <= sc.stale_after
            todo = ~c2_known & free
            for j in rng.permutation(N):
                if fresh[j] and (~c2_known[mission[j]]).sum() < sc.mission_cells // 2:
                    taken = np.concatenate([mission[k] for k in np.flatnonzero(fresh) if k != j] + [[]]).astype(int)
                    mask = todo.copy()
                    mask[taken] = False
                    route, here = [], cell_of(c2_pos[j])
                    for _ in range(sc.mission_cells):  # greedy nearest-neighbour route
                        c = nearest(here, mask)
                        if c == NO_GOAL:
                            break
                        route.append(c)
                        mask[c], here = False, c
                    mission[j] = np.array(route, int)
            for i in np.flatnonzero(got[:N, BS] & alive):
                uav_mission[i], last_order[i] = mission[i].copy(), t
            for i in live:
                left = uav_mission[i][~known[i, uav_mission[i]]]
                if left.size:
                    goal[i] = left[0]
                elif t - last_order[i] > sc.stale_after:
                    goal[i] = cell_of(base)  # lost-link failsafe (as in PX4/ArduPilot): return to C2
        else:
            for i in live:
                g, here = goal[i], cell_of(pos[i])
                fresh = (t - heard[i] <= sc.stale_after) & (claim[i] != NO_GOAL)
                if strategy == "decentralised" and g != NO_GOAL:
                    rivals = np.flatnonzero(fresh & (claim[i] == g))
                    if rivals.size:
                        dj = D[cell_of(claim_pos[i, rivals]), g]
                        di = D[here, g]
                        if np.any((dj < di) | ((dj == di) & (rivals < i))):
                            g = NO_GOAL
                if g == NO_GOAL or known[i, g]:
                    todo = ~known[i] & free
                    if strategy == "decentralised":
                        mask = todo.copy()
                        mask[claim[i, fresh]] = False
                    else:
                        mask = todo & (sector == i)
                    g = nearest(here, mask if mask.any() else todo)
                goal[i] = g

        # move along the grid towards the goal
        for i in live:
            if goal[i] == NO_GOAL:
                continue
            budget = sc.speed
            while budget > 0:
                to_wp = centres[wp[i]] - pos[i]
                dist = np.hypot(*to_wp)
                if dist > 1e-9:
                    hop = min(budget, dist)
                    pos[i] += to_wp * hop / dist
                    budget -= hop
                elif wp[i] == goal[i]:
                    break
                else:
                    k = np.argmin(step[wp[i]] + np.where(nbr[wp[i]] >= 0, D[np.maximum(nbr[wp[i]], 0), goal[i]], np.inf))
                    if nbr[wp[i], k] == NO_GOAL:
                        break
                    wp[i] = nbr[wp[i], k]

    trace[t_end:], alive_log[t_end:] = pos, fail_at[None] > np.arange(t_end, sc.t_max)[:, None]
    coverage = (covered_at[free][None] <= np.arange(sc.t_max)[:, None]).mean(1)
    return Result(coverage, found, reported, covered_at, trace, alive_log, centres[targets], free,
                  sent_bytes / max(uav_seconds, 1), peak)
