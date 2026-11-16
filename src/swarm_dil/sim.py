"""Area search by a UAV swarm over DIL links, under three coordination strategies.

centralised:   agents report to a C2 node, which assigns each one the next cell.
               The C2 link has unlimited range; it only suffers message loss.
decentralised: agents share coverage maps and goal claims over a range-limited
               mesh and resolve clashing claims locally (closest agent keeps it).
partition:     no communication; each agent sweeps a strip fixed before launch,
               then searches whatever it has not seen itself.
"""
from dataclasses import dataclass

import numpy as np

from .link import Channel

STRATEGIES = ("centralised", "decentralised", "partition")
NO_GOAL = -1


@dataclass(frozen=True)
class Scenario:
    n_agents: int = 20
    area: float = 1000.0        # m, side of the square search area
    cell: float = 25.0          # m, grid cell = sensor footprint
    speed: float = 10.0         # m/s
    n_targets: int = 10
    comm_range: float = 300.0   # m, agent-to-agent mesh range
    loss: float = 0.0           # long-run fraction of messages dropped on every link
    burst: float | None = None  # mean loss-burst length in s; None = independent losses
    attrition: float = 0.0      # fraction of agents that fail during the mission
    fail_window: float = 300.0  # s, failures happen uniformly in [0, fail_window]
    stale_after: int = 30       # s, reports and claims older than this are ignored
    t_max: int = 1200           # s, mission time limit; one step = 1 s


@dataclass
class Result:
    coverage: np.ndarray    # (t_max,) fraction of cells searched after each second
    found: np.ndarray       # (n_targets,) detection time in s, inf if never found
    covered_at: np.ndarray  # (cells,) second each cell was first searched, inf if never
    trace: np.ndarray       # (t_max, n_agents, 2) positions in m
    alive: np.ndarray       # (t_max, n_agents) bool
    targets_xy: np.ndarray  # (n_targets, 2) m

    @property
    def time_all_found(self):
        return float(self.found.max())


def run(strategy, sc=Scenario(), seed=0):
    if strategy not in STRATEGIES:
        raise ValueError(f"unknown strategy {strategy!r}; choose from {STRATEGIES}")
    n = round(sc.area / sc.cell)
    if not np.isclose(n * sc.cell, sc.area):
        raise ValueError("area must be a whole number of cells")
    N, G, rng = sc.n_agents, n * n, np.random.default_rng(seed)

    ix, iy = np.indices((n, n)).reshape(2, -1)
    centres = (np.stack([ix, iy], 1) + 0.5) * sc.cell
    sector = ix * N // n  # strip index per cell, for the partition strategy

    def cell_of(p):
        c = np.clip((p // sc.cell).astype(int), 0, n - 1)
        return c[:, 0] * n + c[:, 1]

    def nearest(p, mask):
        idx = np.flatnonzero(mask)
        if idx.size == 0:
            return NO_GOAL
        return idx[np.argmin(((centres[idx] - p) ** 2).sum(1))]

    targets = rng.choice(G, sc.n_targets, replace=False)
    pos = sc.area / 2 + rng.uniform(-20, 20, (N, 2))  # launch point at the centre
    fail_at = np.full(N, np.inf)
    n_fail = round(sc.attrition * N)
    fail_at[rng.choice(N, n_fail, replace=False)] = rng.uniform(0, sc.fail_window, n_fail)

    own = np.zeros((N, G), bool)        # cells each agent knows are searched
    covered_at = np.full(G, np.inf)
    goal = np.full(N, NO_GOAL)
    trace = np.empty((sc.t_max, N, 2))
    alive_log = np.empty((sc.t_max, N), bool)

    if strategy == "centralised":
        up = Channel(N, sc.loss, sc.burst, rng)
        down = Channel(N, sc.loss, sc.burst, rng)
        c2_map = np.zeros(G, bool)
        c2_pos = pos.copy()
        c2_heard = np.zeros(N)
        assign = np.full(N, NO_GOAL)
    elif strategy == "decentralised":
        mesh = Channel((N, N), sc.loss, sc.burst, rng)
        claim = np.full((N, N), NO_GOAL)  # claim[i, j]: goal of j as last heard by i
        claim_pos = np.zeros((N, N, 2))
        heard = np.full((N, N), -np.inf)

    t_end = sc.t_max
    for t in range(sc.t_max):
        alive = fail_at > t
        trace[t], alive_log[t] = pos, alive

        # sense
        live = np.flatnonzero(alive)
        cells = cell_of(pos[live])
        own[live, cells] = True
        covered_at[cells] = np.minimum(covered_at[cells], t)
        if np.isfinite(covered_at).all():
            t_end = t + 1
            break

        # communicate, then plan
        if strategy == "centralised":
            ok = up.step() & alive
            c2_map |= own[ok].any(0)
            c2_pos[ok], c2_heard[ok] = pos[ok], t
            reachable = t - c2_heard <= sc.stale_after
            for j in rng.permutation(N):
                if reachable[j] and (assign[j] == NO_GOAL or c2_map[assign[j]]):
                    free = ~c2_map
                    taken = assign[reachable & (assign != NO_GOAL)]
                    mask = free.copy()
                    mask[taken] = False
                    assign[j] = nearest(c2_pos[j], mask if mask.any() else free)
            ok = down.step() & alive
            goal[ok] = assign[ok]  # without a fresh order an agent holds its last goal
        else:
            if strategy == "decentralised":
                d = np.linalg.norm(pos[:, None] - pos[None], axis=2)
                link = mesh.step() & (d <= sc.comm_range) & alive[:, None] & alive[None]
                np.fill_diagonal(link, False)
                merged = link.astype(np.float32) @ own.astype(np.float32)
                own |= merged > 0
                rows, cols = np.nonzero(link)  # i hears j
                claim[rows, cols] = goal[cols]
                claim_pos[rows, cols] = pos[cols]
                heard[rows, cols] = t
            for i in live:
                g = goal[i]
                if strategy == "decentralised":
                    fresh = (t - heard[i] <= sc.stale_after) & (claim[i] != NO_GOAL)
                    if g != NO_GOAL:
                        rivals = np.flatnonzero(fresh & (claim[i] == g))
                        if rivals.size:
                            dj = np.linalg.norm(claim_pos[i, rivals] - centres[g], axis=1)
                            di = np.linalg.norm(pos[i] - centres[g])
                            if np.any((dj < di) | ((dj == di) & (rivals < i))):
                                g = NO_GOAL
                if g == NO_GOAL or own[i, g]:
                    free = ~own[i]
                    if strategy == "decentralised":
                        mask = free.copy()
                        mask[claim[i, fresh]] = False
                    else:
                        mask = free & (sector == i)
                    g = nearest(pos[i], mask if mask.any() else free)
                goal[i] = g

        # move
        m = alive & (goal != NO_GOAL)
        delta = centres[goal[m]] - pos[m]
        dist = np.linalg.norm(delta, axis=1, keepdims=True)
        pos[m] += delta * np.minimum(1.0, sc.speed / np.maximum(dist, 1e-9))

    trace[t_end:], alive_log[t_end:] = pos, fail_at[None] > np.arange(t_end, sc.t_max)[:, None]
    coverage = (covered_at[None] <= np.arange(sc.t_max)[:, None]).mean(1)
    return Result(coverage, covered_at[targets], covered_at, trace, alive_log, centres[targets])
