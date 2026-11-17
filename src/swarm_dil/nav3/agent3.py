"""On-board software of one agent in a 3-D voxel world (v0.4).

An agent only ever receives three things: the blocked voxels its sensor cone hits (relative
offsets), whether its last move happened, and raw bytes from teammates. Its position, map,
plan, bounds on the cost-to-go and beliefs about teammates all live here. This module never
imports the world.

Protocols (all agents of a mission run the same one):
    none    no communication
    flood   newest-first round robin of 4x4x4 obstacle tiles
    grace   the v0.3 protocol ported to 3-D (goal-relevant tiles, loss model)
    cairn   this work: route claims with certified bounds, plus priced obstacle cuts
            options, appended with '-': noclaims, nocuts, noboxes, noverify, noreply
"""
import math
from collections import deque
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from . import wire3 as wire
from .theory3 import SQRT2, SQRT3, h3

DIRS = tuple((a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0))
DIR_INDEX = {d: k for k, d in enumerate(DIRS)}
_D = np.array(DIRS)
_W = np.sqrt((_D ** 2).sum(1))
TS = 4          # tile side, voxels (Cor. 2)
NEVER = -10**9


@dataclass(frozen=True)
class Spec:
    """Everything an agent is told before launch: hardware limits and mission order."""
    shape: tuple                  # (nx, ny, nz) voxels
    start: tuple
    goal: tuple
    team: int
    launch_at: int
    bands: tuple                  # per teammate id: (z_low, z_high) it can move in (its mobility class)
    protocol: str = "cairn"
    sensor_range: float = 5.0
    memory_bytes: int | None = None   # map memory (10 B per tile); None = unbounded
    budget: int = 64              # max bytes per packet, one packet per slot at most
    price: float = 0.5            # cells of travel a byte of obstacle data must save to be sent
    theta: float = 0.1            # commit to a verified route when U <= (1 + theta) L
    delta: float = 1.0            # cells by which a bound must improve to be worth a claim
    backtrack: float = 0.25       # accept a claim that needs backtracking if cost <= (1 + this) L
    burst_lambda: float = 0.9     # design constant: per-slot correlation of link state
    stale: int = 20
    heartbeat: int = 5
    refresh: int = 3              # slots a cost-to-go field may be reused when advising others
    announce: int = 10            # re-announce my plan at least this often


# ---------------------------------------------------------------------------- lattice geometry
@lru_cache(maxsize=4)
def lattice(shape):
    """CSR arrays of the 26-connected lattice: indptr, indices, weights, row of every edge."""
    nx, ny, nz = shape
    n = nx * ny * nz
    x, y, z = np.indices(shape).reshape(3, -1)
    src, dst, w = [], [], []
    for (dx, dy, dz), c in zip(DIRS, _W):
        tx, ty, tz = x + dx, y + dy, z + dz
        ok = (tx >= 0) & (tx < nx) & (ty >= 0) & (ty < ny) & (tz >= 0) & (tz < nz)
        src.append(np.flatnonzero(ok))
        dst.append(((tx * ny + ty) * nz + tz)[ok])
        w.append(np.full(int(ok.sum()), c))
    g = csr_matrix((np.concatenate(w), (np.concatenate(src), np.concatenate(dst))), shape=(n, n))
    g.sort_indices()
    return g.indptr, g.indices, g.data, np.repeat(np.arange(n), np.diff(g.indptr))


def distance_field(shape, blocked, goal):
    """Cost-to-go to voxel index `goal` when a move is possible iff its target is not in
    `blocked` (unknown voxels count as free: optimistic). Flat array, inf where unreachable.
    Dijkstra runs from the goal on the reversed graph: blocked voxels have no outgoing edge."""
    indptr, indices, base, rows = lattice(tuple(shape))
    data = np.where(blocked[rows], np.inf, base)
    n = blocked.size
    return dijkstra(csr_matrix((data, indices, indptr), shape=(n, n)), directed=True, indices=goal)


def segment(p, q):
    """Canonical lattice geodesic p -> q (p excluded): move along every axis that still has
    displacement left, so 3-D diagonals come first, then 2-D diagonals, then straight steps.
    Any contiguous piece of a canonical segment is itself canonical."""
    d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]]
    c, out = list(p), []
    while d[0] or d[1] or d[2]:
        for a in range(3):
            if d[a] > 0:
                c[a] += 1
                d[a] -= 1
            elif d[a] < 0:
                c[a] -= 1
                d[a] += 1
        out.append(tuple(c))
    return out


def expand(wps):
    """Cells of the path through the waypoints (first waypoint excluded)."""
    out = []
    for a, b in zip(wps, wps[1:]):
        out += segment(a, b)
    return out


def path_cost(cells):
    return sum(h3(a, b) for a, b in zip(cells, cells[1:]))


def simplify(cells, block, shape, exact=False):
    """Waypoints whose canonical segments give a path of the same cost as `cells` that avoids
    the flat mask `block` (exact=True: the very same cells, used for walked trails)."""
    ny, nz = shape[1], shape[2]
    cum = [0.0]
    for a, b in zip(cells, cells[1:]):
        cum.append(cum[-1] + h3(a, b))
    wps, i, n = [cells[0]], 0, len(cells) - 1
    while i < n:
        j = i + 1
        for k in range(i + 2, n + 1):
            if abs(h3(cells[i], cells[k]) - (cum[k] - cum[i])) > 1e-6:
                break
            seg = segment(cells[i], cells[k])
            if exact:
                if seg != cells[i + 1:k + 1]:
                    break
            elif any(block[(x * ny + y) * nz + z] for x, y, z in seg):
                break
            j = k
        wps.append(cells[j])
        i = j
    return wps


def hit_any(agent, idx):
    return bool(agent.blocked[idx].any())


class Agent:
    def __init__(self, ident, spec, rng):
        base, *opts = spec.protocol.split("-")
        if base not in ("none", "flood", "grace", "cairn"):
            raise ValueError(f"unknown protocol {spec.protocol!r}")
        self.base, self.off = base, {o[2:] for o in opts if o.startswith("no")}
        self.opts = {o for o in opts if not o.startswith("no")}
        self.id, self.s, self.rng = ident, spec, rng
        self.shape = tuple(spec.shape)
        nx, ny, nz = self.shape
        self.N = nx * ny * nz
        zlo, zhi = spec.bands[ident]
        wall = np.ones(self.shape, bool)
        wall[:, :, zlo:zhi + 1] = False
        self.wall = wall.ravel()               # voxels outside my mobility band
        self.blocked = np.zeros(self.N, bool)  # my map knowledge, decoded
        self.start, self.goal = tuple(spec.start), tuple(spec.goal)
        self.t = 0
        self.pos = self.start                  # dead reckoning from start + own successful moves
        self.heading = DIR_INDEX[(1, 1, 0)]
        self.arrived = self.pos == self.goal
        self.path_len = 0.0
        self.seq = 0
        self.last_tx = NEVER
        self.bytes_sent = 0
        self.n_fields = 0                      # shortest-path searches run on board
        self.sent = dict(advice=0, team=0, verified=0, relay=0, plan=0, tile=0, box=0)
        gx, gy, gz = self.goal
        ix, iy, iz = np.indices(self.shape).reshape(3, -1)
        self.cx, self.cy, self.cz = ix, iy, iz
        self.h_goal = self._h(ix - gx, iy - gy, iz - gz)
        self.h0 = h3(self.start, self.goal)
        # map knowledge
        self.tiles, self.learned = {}, {}      # key -> 64-bit mask, key -> slot of last change
        self.u, self.last_sent = {}, {}        # cut bookkeeping
        self.queue = deque()                   # flood round robin
        self.urgent = deque()                  # tiles that refute a plan somebody believes in
        self.sensed = np.zeros(self.N, bool)   # blocked voxels I saw with my own sensor
        self.pool = np.zeros(self.N, bool)     # the certificate: obstacles that have refuted a route
        self.pool_keys = set()
        self.field, self.field_any, self.field_t = None, None, NEVER
        # plan and bounds
        self.plan, self.plan_ix, self.plan_cost, self.plan_exact = [], np.zeros(0, int), 0.0, False
        self.lb = self.h0                      # sound lower bound on my true cost-to-go
        self.trail, self.trail_len, self.trail_ix = [self.pos], [0.0], {self.pos: 0}
        self.team_v, self.team_wps = self.h0, None   # best route claim anchored at the start
        self.ver_u, self.ver_wps, self.ver_cells, self.ver_togo = None, None, [], {}
        self.committed = False
        self.changed = False                   # my plan changed since I last announced it
        self.announced = NEVER                 # slot of my last plan announcement
        self.plan_rem = {}                     # cell on my plan -> cost left from there
        self.vb = {}                           # cell ahead -> sound lower bound to apply on arrival
        self.replies = deque(maxlen=4)         # (teammate, cell) where my trail refutes its plan
        # neighbour table
        T = spec.team
        self.nb_heard = np.full(T, NEVER)
        self.nb_first = np.full(T, NEVER)
        self.nb_pos = [self.start] * T
        self.nb_seq = [None] * T
        self.nb_rx, self.nb_expected = np.zeros(T), np.zeros(T)
        self.nb_lb = np.full(T, self.h0)
        self.nb_lost = np.zeros(T, bool)
        self.nb_va = np.full(T, self.h0)
        self.nb_ua = np.full(T, np.inf)
        self.told = {k: np.full(T, NEVER) for k in ("advice", "team", "verified")}

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def _h(dx, dy, dz):
        a = np.sort(np.abs(np.stack([dx, dy, dz])), axis=0)
        return a[2] + (SQRT2 - 1.0) * a[1] + (SQRT3 - SQRT2) * a[0]

    def _ix(self, c):
        return (c[0] * self.shape[1] + c[1]) * self.shape[2] + c[2]

    def _inside(self, c):
        return all(0 <= c[a] < self.shape[a] for a in range(3))

    @property
    def lost(self):
        """My plan is cheaper than a bound I hold, so it is certainly blocked somewhere."""
        return not self.arrived and (not self.plan or self.plan_cost < self.lb - self.s.delta)

    @property
    def memory_used(self):
        return wire.TILE * len(self.tiles)

    # ------------------------------------------------------------------ map knowledge
    @staticmethod
    def _key_bit(x, y, z):
        return ((x // TS) * 64 + y // TS) * 16 + z // TS, ((x % TS) * TS + y % TS) * TS + z % TS

    @staticmethod
    def _cells_of(key, mask):
        tz, ty, tx = key % 16, key // 16 % 64, key // 1024
        return [(tx * TS + b // 16, ty * TS + b // 4 % 4, tz * TS + b % 4) for b in range(64) if mask >> b & 1]

    def _add_blocked(self, c):
        if not self._inside(c):
            return False
        key, bit = self._key_bit(*c)
        return self._merge(key, 1 << bit)

    def _merge(self, key, mask, sender=None):
        old = self.tiles.get(key, 0)
        new = old | mask
        if new != old:
            self.tiles[key] = new
            self.learned[key] = self.t
            for c in self._cells_of(key, new & ~old):
                self.blocked[self._ix(c)] = True
                if key in self.pool_keys:
                    self.pool[self._ix(c)] = True
            self.field = None
            self.last_sent.pop(key, None)
            if self.base == "grace":
                self.u[key] = np.ones(self.s.team)
                self.u[key][self.id] = 0.0
            else:
                self.u[key] = 1.0
            if key not in self.queue:
                self.queue.appendleft(key)
        if sender is not None and self.base == "grace" and mask | new == mask:
            self.u[key][sender] = 0.0
        return new != old

    def _cell_table(self):
        """Blocked voxels I know, grouped by tile: (keys, flat cell index, start of each group)."""
        cells = np.flatnonzero(self.blocked)
        tid = ((self.cx[cells] // TS) * 64 + self.cy[cells] // TS) * 16 + self.cz[cells] // TS
        order = np.argsort(tid, kind="stable")
        cells, tid = cells[order], tid[order]
        starts = np.flatnonzero(np.r_[True, tid[1:] != tid[:-1]])
        return tid[starts], cells, starts

    def _margins(self, cells, starts, at, cost_to_go):
        """Per tile: (spheroid margin, reach). margin = min over its voxels of h(at, y) + h(y, B)
        minus the believed cost-to-go; positive means outside the informed set (Prop. 7)."""
        h_at = self._h(self.cx[cells] - at[0], self.cy[cells] - at[1], self.cz[cells] - at[2])
        return (np.minimum.reduceat(h_at + self.h_goal[cells], starts) - cost_to_go,
                np.minimum.reduceat(h_at, starts))

    def _evict(self):
        cap = self.s.memory_bytes
        if cap is None or self.memory_used <= cap:
            return
        n_drop = len(self.tiles) - cap // wire.TILE
        if self.base == "flood":
            victims = sorted(self.tiles, key=self.learned.get)[:n_drop]
        else:
            keys, cells, starts = self._cell_table()
            ref = self.plan_cost if self.plan else self.lb
            margin, _ = self._margins(cells, starts, self.pos, ref)
            victims = keys[np.argsort(-margin, kind="stable")[:n_drop]].tolist()
        for key in victims:
            for c in self._cells_of(key, self.tiles[key]):
                self.blocked[self._ix(c)] = False
                self.pool[self._ix(c)] = False
            self.pool_keys.discard(key)
            for d in (self.tiles, self.learned, self.u, self.last_sent):
                d.pop(key, None)
            if key in self.queue:
                self.queue.remove(key)
        self.field = None

    # ------------------------------------------------------------------ planning
    def _field(self):
        if self.field is None:
            self.field = self.field_any = distance_field(self.shape, self.blocked | self.wall, self._ix(self.goal))
            self.field_t = self.t
            self.n_fields += 1
            if self.base == "cairn" and "claims" not in self.off:
                v = self.field[self._ix(self.start)]
                if np.isfinite(v) and v > self.team_v + self.s.delta:   # my hindsight route from the start
                    path = self._descent(self.field, self.start)
                    if path is not None:
                        self.team_v = v
                        self.team_wps = simplify([self.start] + path, self.blocked | self.wall, self.shape)
            if self.base == "cairn" and "cert" not in self.off:
                self._certify()
        return self.field

    def _descent(self, f, start, block=None):
        """Cells of a shortest path start -> goal down the field f (start excluded), or None."""
        ny, nz = self.shape[1], self.shape[2]
        block = self.blocked | self.wall if block is None else block
        i = self._ix(start)
        if not np.isfinite(f[i]):
            return None
        c, out = np.array(start), []
        for _ in range(4 * sum(self.shape)):
            if tuple(c) == self.goal:
                return out
            n = c + _D
            inb = ((n >= 0) & (n < self.shape)).all(1)
            j = np.where(inb, (n[:, 0] * ny + n[:, 1]) * nz + n[:, 2], 0)
            cost = np.where(inb & ~block[j] & (f[j] < f[i] - 1e-9), _W + f[j], np.inf)
            k = int(np.argmin(cost))
            if not np.isfinite(cost[k]):
                return None
            c, i = n[k], int(j[k])
            out.append((int(c[0]), int(c[1]), int(c[2])))
        return None

    def _set_plan(self, cells, exact):
        self.plan = list(cells)
        self.plan_ix = np.array([self._ix(c) for c in cells], int)
        self.plan_cost = path_cost([self.pos] + self.plan)
        left, prev, self.plan_rem = self.plan_cost, self.pos, {}
        for c in self.plan:
            left -= h3(prev, c)
            self.plan_rem[c] = max(left, 0.0)
            prev = c
        self.plan_exact = exact
        self.changed = True

    def _replan(self):
        """Shortest path under my own knowledge. By construction its cost is a lower bound."""
        f = self._field()
        self.committed = False
        path = self._descent(f, self.pos)
        if path is None:
            self._set_plan([], False)
            return
        self.lb = max(self.lb, f[self._ix(self.pos)])
        self._set_plan(expand(simplify([self.pos] + path, self.blocked | self.wall, self.shape)), True)

    def _check_plan(self):
        if self.arrived:
            return
        hit = self.blocked[self.plan_ix] if self.plan else None
        if hit is None or hit.any():
            if hit is not None and self.base == "cairn":
                c = int(self.plan_ix[int(np.argmax(hit))])
                if self.sensed[c]:      # an obstacle I discovered on a route we believed in
                    self._report(self._key_bit(int(self.cx[c]), int(self.cy[c]), int(self.cz[c]))[0])
            self._replan()
            if self.base == "cairn" and self.pos == self.start and self.team_wps:
                self._consider(self.start, expand(self.team_wps), self.team_v, True, True)
        self._commit_check()

    def _bound_on_track(self, c):
        """Sound lower bound on the cost-to-go from a cell of my own walked trail (Lemma 3) or
        of my plan when its cost is a proved bound; None elsewhere."""
        k = self.trail_ix.get(c)
        if k is not None and k < len(self.trail) - 1:
            return self.lb - (self.trail_len[-1] - self.trail_len[k])
        if self.plan_exact and c in self.plan_rem:
            return self.plan_rem[c]
        return None

    def _route_on_track(self, c):
        k = self.trail_ix.get(c)
        if k is not None and k < len(self.trail) - 1:
            return self.trail[k:] + self.plan
        return [c] + self.plan[self.plan.index(c) + 1:]

    def _promote(self, key):
        """Make what I know of a tile part of the certificate."""
        if key in self.tiles:
            self.pool_keys.add(key)
            for c in self._cells_of(key, self.tiles[key]):
                self.pool[self._ix(c)] = True

    def _report(self, key):
        self._promote(key)
        if key not in self.urgent:
            self.urgent.append(key)

    def _certify(self, rounds=12):
        """Keep the certificate able to prove my bound from the start (Def. 2): while the best
        route through the pool alone is cheaper than what my full knowledge proves, it must
        cross an obstacle I know; that obstacle joins the pool. Lazy cutting planes."""
        a = self._ix(self.start)
        target = self.field[a]
        if not np.isfinite(target):
            return
        for _ in range(rounds):
            block = self.pool | self.wall
            fp = distance_field(self.shape, block, self._ix(self.goal))
            self.n_fields += 1
            if fp[a] >= target - self.s.delta:
                return
            path = self._descent(fp, self.start, block)
            hit = next((c for c in path or [] if self.blocked[self._ix(c)]), None)
            if hit is None:
                return
            self._promote(self._key_bit(*hit)[0])

    def _commit_check(self):
        """Thm. 2: with a verified route of length U from here and a lower bound L, following
        the verified route costs at most (1 + theta) times the true optimum if U <= (1+theta) L."""
        if self.committed or self.ver_u is None or "verify" in self.off or self.arrived:
            return
        k = self.ver_togo.get(self.pos)
        if k is not None and k[1] <= (1.0 + self.s.theta) * self.lb + 1e-9:
            self._set_plan(self.ver_cells[k[0] + 1:], False)
            self.committed = True

    def _set_verified(self, wps, u):
        self.ver_u, self.ver_wps = u, wps
        self.ver_cells = [wps[0]] + expand(wps)
        togo, self.ver_togo = u, {}
        for k, c in enumerate(self.ver_cells):
            if k:
                togo -= h3(self.ver_cells[k - 1], c)
            self.ver_togo[c] = (k, max(togo, 0.0))

    # ------------------------------------------------------------------ sensing and motion
    def observe(self, hits):
        """hits: (dx, dy, dz) of every blocked voxel my cone sees. Free voxels carry no decision
        value under optimistic planning (v0.3, Lemma 1), so the sensor driver drops them."""
        new = False
        for dx, dy, dz in hits:
            c = (self.pos[0] + dx, self.pos[1] + dy, self.pos[2] + dz)
            if self._inside(c):
                self.sensed[self._ix(c)] = True
                new |= self._add_blocked(c)
        if new:
            self._evict()

    def decide(self):
        """Direction index to move this slot, or None to hold."""
        if self.arrived or self.t < self.s.launch_at:
            return None
        self._check_plan()
        if not self.plan:
            return None
        nxt = self.plan[0]
        self.heading = DIR_INDEX[(nxt[0] - self.pos[0], nxt[1] - self.pos[1], nxt[2] - self.pos[2])]
        return self.heading

    def moved(self, direction, ok):
        """Proprioception: the world tells me whether my commanded step happened."""
        if direction is not None:
            d = DIRS[direction]
            nxt = (self.pos[0] + d[0], self.pos[1] + d[1], self.pos[2] + d[2])
            if ok:
                step = float(_W[direction])
                self.pos = nxt
                self.path_len += step
                self.lb = max(self.lb - step, h3(nxt, self.goal), self.vb.pop(nxt, 0.0))   # Lemma 3: bounds travel with me
                self.plan_rem.pop(self.plan.pop(0), None)
                self.plan_ix = self.plan_ix[1:]
                self.plan_cost = max(self.plan_cost - step, 0.0)
                k = self.trail_ix.get(nxt)
                if k is not None:                                       # closed a loop: erase it
                    for c in self.trail[k + 1:]:
                        del self.trail_ix[c]
                    del self.trail[k + 1:], self.trail_len[k + 1:]
                else:
                    self.trail_ix[nxt] = len(self.trail)
                    self.trail.append(nxt)
                    self.trail_len.append(self.trail_len[-1] + step)
                self.arrived = nxt == self.goal
                if self.arrived and self.base == "cairn" and "verify" not in self.off:
                    u = self.trail_len[-1]                              # a walked path: an upper bound
                    if self.ver_u is None or u < self.ver_u - 1e-9:
                        self._set_verified(simplify(self.trail, None, self.shape, exact=True), u)
            elif self._inside(nxt):                                     # bumped into it
                self.sensed[self._ix(nxt)] = True
                self._add_blocked(nxt)
                self._evict()
        self.t += 1

    # ------------------------------------------------------------------ receiving
    def receive(self, packet):
        sender, seq, pos, digest, tiles, boxes, claims = wire.decode(packet)
        if sender == self.id:
            return
        if self.nb_heard[sender] < self.t - self.s.stale:
            self.nb_first[sender] = self.t
        self.nb_heard[sender] = self.t
        self.nb_pos[sender] = pos
        last = self.nb_seq[sender]
        self.nb_expected[sender] += 1 if last is None else (seq - last) % 256
        self.nb_rx[sender] += 1
        self.nb_seq[sender] = seq
        if digest:
            flags, lbx, vax, uax = digest
            self.nb_lost[sender] = bool(flags & wire.LOST)
            self.nb_lb[sender] = h3(pos, self.goal) + lbx / 2.0
            self.nb_va[sender] = self.h0 + vax / 2.0
            self.nb_ua[sender] = np.inf if uax == 255 else self.h0 + uax / 2.0
        for key, mask in tiles:
            self._merge(key, mask, sender)
            if self.base == "cairn":
                self._promote(key)
        for lo, ext in boxes:
            masks = {}
            for x in range(lo[0], lo[0] + ext[0]):
                for y in range(lo[1], lo[1] + ext[1]):
                    for z in range(lo[2], lo[2] + ext[2]):
                        key, bit = self._key_bit(x, y, z)
                        masks[key] = masks.get(key, 0) | 1 << bit
            for key, mask in masks.items():
                self._merge(key, mask, sender)
                if self.base == "cairn":
                    self._promote(key)
        if claims and self.plan and self.blocked[self.plan_ix].any():
            self._check_plan()          # weigh claims against a plan that still stands
        for kind, _target, value, wps in claims:
            self._on_claim(kind, value / 8.0, wps, sender)
        self._evict()

    def _on_claim(self, kind, value, wps, sender):
        if len(wps) < 2 or not all(self._inside(w) for w in wps):
            return
        cells = expand(wps)
        idx = np.array([self._ix(c) for c in cells], int)
        mine, theirs = self.s.bands[self.id], self.s.bands[sender]
        walkable = not (self.wall[idx].any() or self.wall[self._ix(wps[0])])
        if kind == wire.VERIFIED:
            # somebody walked it: a true upper bound for every class that can move there
            if walkable and wps[0] == self.start and cells[-1] == self.goal and (self.ver_u is None or value < self.ver_u - 1e-9):
                self._set_verified(wps, value)
            return
        # a lower bound proved in a larger free space also holds in mine (Prop. 3)
        lower_ok = theirs[0] <= mine[0] and theirs[1] >= mine[1]
        if kind == wire.TEAM and lower_ok and wps[0] == self.start and value > self.team_v + 1e-9:
            self.team_v, self.team_wps = value, wps
        if kind in (wire.PLAN, wire.PLAN_X) and lower_ok and theirs == mine and not hit_any(self, idx):
            arc, prev = 0.0, wps[0]
            for c in [wps[0]] + cells:
                arc += h3(prev, c)
                prev = c
                w = self._bound_on_track(c)
                if w is not None and w > value - arc + self.s.delta:
                    self.replies.append((sender, c))
                    break
        hit = self.blocked[idx]
        if hit.any():
            # my knowledge refutes that route: the obstacle on it is worth telling at once
            self._report(self._key_bit(*cells[int(np.argmax(hit))])[0])
        # a plain PLAN is only a route somebody follows: its cost is not a bound
        self._consider(wps[0], cells, value, walkable and not hit.any(), lower_ok and kind != wire.PLAN,
                       kind in (wire.ADVICE, wire.TEAM, wire.PLAN_X))

    def _consider(self, anchor, cells, value, walk, lower_ok, exact=True):
        """Use a claim (anchor, path, certified value): raise my bound, maybe adopt the path."""
        full = bool(cells) and cells[-1] == self.goal
        k_pos = cells.index(self.pos) if self.pos in cells else None
        k_tr = self.trail_ix.get(anchor)
        if lower_ok:
            if self.pos == anchor:
                self.lb = max(self.lb, value)
            elif k_pos is not None and exact and full:
                self.lb = max(self.lb, value - path_cost([anchor] + cells[:k_pos + 1]))
            elif k_tr is not None:      # the anchor is on my own trail: Lemma 3 with a walked path
                self.lb = max(self.lb, value - (self.trail_len[-1] - self.trail_len[k_tr]))
        ahead = self.plan_rem.get(anchor)
        if lower_ok and ahead is not None and anchor != self.pos and anchor in self.plan:
            self.vb[anchor] = max(self.vb.get(anchor, 0.0), value)
            if len(self.vb) > 16:
                self.vb.pop(next(iter(self.vb)))
            if walk and not self.committed and not self.arrived and value > ahead + self.s.delta:
                k = self.plan.index(anchor)
                tail = cells if full else cells + (self._descent(self._field(), cells[-1]) or [])
                if tail and tail[-1] == self.goal:
                    self._set_plan(self.plan[:k + 1] + tail, False)
            return
        if self.committed or self.arrived or not walk:
            return
        back = False
        if self.pos == anchor:
            cand = cells
        elif k_pos is not None:
            cand = cells[k_pos + 1:]
        else:
            cand, best = None, np.inf
            block = self.blocked | self.wall
            for k in range(min(8, len(cells))):          # splice in over a short free segment
                seg = segment(self.pos, cells[k])
                c = h3(self.pos, cells[k]) + path_cost(cells[k:])
                if c < best and not any(block[self._ix(q)] for q in seg):
                    cand, best = seg + cells[k + 1:], c
            if k_tr is not None and self.trail_len[-1] - self.trail_len[k_tr] + path_cost([anchor] + cells) < best:
                cand = self.trail[k_tr:-1][::-1] + cells
            if cand is None:
                return
            back = True
        if not full:                    # truncated claim: continue with my own knowledge
            tail = self._descent(self._field(), cand[-1] if cand else self.pos)
            if tail is None:
                return
            cand = cand + tail
        cost = path_cost([self.pos] + cand)
        if cost < self.lb - self.s.delta:                       # the candidate is refuted by value
            return
        if back and cost > (1.0 + self.s.backtrack) * self.lb:
            return
        if self.lost or cost < self.plan_cost - 1e-6:           # cheapest route nobody has refuted
            self._set_plan(cand, exact and full and not back and lower_ok)

    # ------------------------------------------------------------------ transmitting
    def transmit(self):
        """Bytes to broadcast this slot, or None."""
        if self.base == "none":
            return None
        s, now = self.s, self.t
        cairn = self.base == "cairn"
        fresh = [j for j in range(s.team) if j != self.id and now - self.nb_heard[j] <= s.stale]
        room = s.budget - wire.header_len(cairn)
        tiles, boxes, claims = [], [], []
        if cairn:
            if now >= s.launch_at and not self.arrived:
                self._check_plan()
            room = self._urgent(room, tiles, boxes)
            if fresh:
                room = self._claims(fresh, room, claims)
                room = self._announce(room, claims)
                if "cuts" not in self.off:
                    self._cuts(fresh, room, tiles, boxes)
        elif self.base == "flood":
            for _ in range(min(room // wire.TILE, len(self.queue), 15)):
                key = self.queue.popleft()
                tiles.append((key, self.tiles[key]))
                self.queue.append(key)
        elif fresh:
            self._cuts(fresh, room, tiles, boxes)
        beacon = cairn and self.lost and now >= s.launch_at and now - self.last_tx >= 2
        if not (tiles or boxes or claims or beacon) and now - self.last_tx < s.heartbeat:
            return None
        self.seq += 1
        self.last_tx = now
        digest = None
        if cairn:
            flags = (wire.LOST if self.lost else 0) | (wire.ARRIVED if self.arrived else 0) | (wire.COMMITTED if self.committed else 0)
            lbx = min(254, int(math.floor(2.0 * (self.lb - h3(self.pos, self.goal)) + 1e-9)))   # floor: stays a lower bound
            vax = min(254, int(math.floor(2.0 * (self.team_v - self.h0) + 1e-9)))
            uax = 255 if self.ver_u is None else int(math.ceil(2.0 * (self.ver_u - self.h0) - 1e-9))
            digest = (flags, max(lbx, 0), max(vax, 0), uax if uax < 255 else 255)
        packet = wire.encode(self.id, self.seq, self.pos, digest, tiles, boxes, claims)
        self.bytes_sent += len(packet)
        self.sent["tile"] += len(tiles)
        self.sent["box"] += len(boxes)
        return packet

    def _urgent(self, room, tiles, boxes):
        """Refutations: obstacles that block a plan somebody announced or that I was following."""
        while self.urgent and room >= wire.TILE and len(tiles) < 15 and len(boxes) < 15:
            key = self.urgent.popleft()
            if key not in self.tiles:
                continue
            box = None if "boxes" in self.off else self._grow_box(key)
            if box is not None:
                boxes.append(box[:2])
                room -= wire.BOX
            else:
                tiles.append((key, self.tiles[key]))
                room -= wire.TILE
            for k in (box[2] if box else [key]):
                self.last_sent[k] = self.t
                self.u[k] = 0.0
        return room

    def _announce(self, room, out):
        """Announce my plan so that anyone whose knowledge blocks it can refute it (Thm. 1)."""
        s = self.s
        if "announce" in self.off or self.arrived or self.t < s.launch_at or not self.plan or self.committed:
            return room
        if not (self.changed or self.t - self.announced >= s.announce):
            return room
        fit = (room - 4) // 3
        if fit < 2:
            return room
        wps = simplify([self.pos] + self.plan, self.blocked | self.wall, self.shape)[:fit]
        exact = self.plan_exact and wps[-1] == self.goal
        out.append((wire.PLAN_X if exact else wire.PLAN, self.id, int(min(math.floor(self.plan_cost * 8 + 1e-9), 65535)), wps))
        self.announced, self.changed = self.t, False
        self.sent["plan"] += 1
        return room - wire.claim_len(len(wps))

    def _may_tell(self, kind, j):
        """Reply-on-hear (Prop. 9): a claim goes to j only in answer to a header from j heard
        after my previous copy. That header is both the acknowledgement (its digest shows the
        claim is still needed) and the evidence that the link is up right now."""
        if "reply" in self.off:
            return self.t - self.told[kind][j] >= 3
        return self.nb_heard[j] > self.told[kind][j]

    def _claims(self, fresh, room, out):
        s, block = self.s, self.blocked | self.wall

        def push(kind, name, j, value, wps):
            nonlocal room
            fit = (room - 4) // 3
            if fit < 2:
                return
            value = math.ceil(value * 8 - 1e-9) if kind == wire.VERIFIED else math.floor(value * 8 + 1e-9)
            if len(wps) > fit:
                if kind in (wire.VERIFIED, wire.RELAY):
                    return                      # these are useless when cut short
                wps = wps[:fit]
            out.append((kind, j, int(min(value, 65535)), wps))
            room -= wire.claim_len(len(wps))
            self.told[name][j] = self.t
            self.sent[name if kind != wire.RELAY else "relay"] += 1

        if self.ver_u is not None and "verify" not in self.off:
            need = [j for j in fresh if self.nb_ua[j] > self.ver_u + 0.5 and self._may_tell("verified", j)]
            if need:
                push(wire.VERIFIED, "verified", need[0], self.ver_u, self.ver_wps)
                for j in need:
                    self.told["verified"][j] = self.t
        if "claims" in self.off:
            return room
        mine = s.bands[self.id]
        f = None
        for j in fresh:
            if s.bands[j] != mine or not self._may_tell("advice", j):
                continue
            xj = self.nb_pos[j]
            if xj == self.goal or not self._inside(xj):
                continue
            if f is None:
                f = self.field_any if self.field is None and self.t - self.field_t <= s.refresh else self._field()
            v = f[self._ix(xj)]
            if np.isfinite(v) and (v > self.nb_lb[j] + s.delta or (self.nb_lost[j] and v >= self.nb_lb[j] - s.delta)):
                path = self._descent(f, xj) or self._descent(self._field(), xj)
                if path:
                    push(wire.ADVICE, "advice", j, v, simplify([xj] + path, block, self.shape))
                    continue
            # relay what I was told: sound where my own walked trail links us (Lemma 3)
            k = self.trail_ix.get(xj)
            if k is not None and self.plan and k < len(self.trail) - 1:
                v = self.lb - (self.trail_len[-1] - self.trail_len[k])
                if v > self.nb_lb[j] + s.delta:
                    push(wire.RELAY, "advice", j, v, simplify(self.trail[k:] + self.plan, block, self.shape))
        while self.replies:
            j, c = self.replies.popleft()
            w = self._bound_on_track(c)
            if w is not None and self.plan and not self.arrived:
                push(wire.RELAY, "advice", j, w, simplify(self._route_on_track(c), block, self.shape))
        if self.team_wps:
            need = [j for j in fresh if self.team_v > self.nb_va[j] + s.delta and self._may_tell("team", j)]
            if need:
                push(wire.TEAM, "team", need[0], self.team_v, self.team_wps)
                for j in need:
                    self.told["team"][j] = self.t
        return room

    def _loss(self, fresh):
        """Probability that a copy sent now to each fresh neighbour is lost (v0.3, Prop. 4)."""
        pi_b = 1.0 - (self.nb_rx[fresh] + 1) / (self.nb_expected[fresh] + 2)
        return pi_b * (1.0 - self.s.burst_lambda ** np.maximum(self.t - self.nb_heard[fresh], 1))

    def _cuts(self, fresh, room, tiles, boxes):
        """Obstacle data worth its bytes: tiles valued by the informed-set prior (Prop. 8)."""
        s, now = self.s, self.t
        if not self.tiles or room < wire.BOX:
            return
        fresh = np.array(fresh)
        keys, cells, starts = self._cell_table()
        ends = np.r_[starts[1:], len(cells)]
        grace = self.base == "grace"
        mu = max(1.0, self.path_len + (self.plan_cost if self.plan else self.lb) - self.h0)
        V = np.zeros((len(keys), len(fresh)))
        shared = f0 = None
        if grace:
            U = np.array([self.u[k][fresh] for k in keys.tolist()])
            shared = np.zeros(self.N, bool)
            for a in np.flatnonzero(U.max(1) <= 0.5):
                shared[cells[starts[a]:ends[a]]] = True
            f0 = distance_field(self.shape, shared | self.wall, self._ix(self.goal))
            self.n_fields += 1
            cost = np.array([f0[self._ix(self.nb_pos[j])] for j in fresh])
        else:
            u = np.array([self.u[k] for k in keys.tolist()])
            sent = np.array([self.last_sent.get(k, NEVER) for k in keys.tolist()])
            U = np.where(self.nb_first[fresh][None] > sent[:, None], 1.0, u[:, None])   # new contacts lack it
            cost = self.nb_lb[fresh]
        for col, j in enumerate(fresh):
            if not np.isfinite(cost[col]) or self.nb_pos[j] == self.goal:
                continue
            margin, reach = self._margins(cells, starts, self.nb_pos[j], cost[col])
            V[:, col] = np.exp(-np.maximum(margin, 0.0) / mu) * 2.0 * np.maximum(0.0, reach - s.sensor_range)
        if grace:
            self._closure(keys, cells, starts, ends, fresh, U, V, shared, f0)
        loss = self._loss(fresh)
        gain = (V * U * (1.0 - loss)).sum(1)
        if self.base == "cairn" and "allcuts" not in self.opts:
            gain = gain * np.array([k in self.pool_keys for k in keys.tolist()])
        done = set(k for k, _ in tiles)
        for a in np.argsort(-gain):
            key = int(keys[a])
            if gain[a] < s.price * wire.TILE or room < wire.BOX or len(tiles) >= 15:
                break
            if key in done:
                continue
            box = None if grace or "boxes" in self.off or len(boxes) >= 15 else self._grow_box(key)
            covered = [key]
            if box is not None:
                lo, ext, covered = box
                boxes.append((lo, ext))
                room -= wire.BOX
            elif room >= wire.TILE:
                tiles.append((key, self.tiles[key]))
                room -= wire.TILE
            else:
                continue
            for k in covered:
                done.add(k)
                self.last_sent[k] = now
                if grace:
                    self.u[k][fresh] = self.u[k][fresh] * loss
                else:
                    self.u[k] = self.u[k] * float(loss.mean())

    def _grow_box(self, key):
        """Largest axis-aligned box of known-blocked voxels around a voxel of the tile, if it
        says the same as two or more tiles (a 'deep cut': a flat wall collapses to 6 bytes)."""
        b3 = self.blocked.reshape(self.shape)
        seed = self._cells_of(key, self.tiles[key])[0]
        lo, hi = list(seed), [c + 1 for c in seed]
        grew = True
        while grew:
            grew = False
            for a in range(3):
                sl = [slice(lo[i], hi[i]) for i in range(3)]
                if hi[a] < self.shape[a]:
                    sl[a] = slice(hi[a], hi[a] + 1)
                    if b3[tuple(sl)].all():
                        hi[a] += 1
                        grew = True
                sl = [slice(lo[i], hi[i]) for i in range(3)]
                if lo[a] > 0:
                    sl[a] = slice(lo[a] - 1, lo[a])
                    if b3[tuple(sl)].all():
                        lo[a] -= 1
                        grew = True
        covered = []
        for tx in range(lo[0] // TS, (hi[0] - 1) // TS + 1):
            for ty in range(lo[1] // TS, (hi[1] - 1) // TS + 1):
                for tz in range(lo[2] // TS, (hi[2] - 1) // TS + 1):
                    k = (tx * 64 + ty) * 16 + tz
                    if k in self.tiles and all(lo[i] <= c[i] < hi[i] for c in self._cells_of(k, self.tiles[k]) for i in range(3)):
                        covered.append(k)
        if key not in covered or len(covered) < 2 or max(hi[i] - lo[i] for i in range(3)) > 255:
            return None
        return tuple(lo), tuple(hi[i] - lo[i] for i in range(3)), covered

    def _closure(self, keys, cells, starts, ends, fresh, U, V, shared, f0, rounds=2):
        """v0.3 plan-equivalence closure: tiles the teammate's predicted plan runs into."""
        index = {int(k): a for a, k in enumerate(keys)}
        mine = self.blocked
        for col, j in enumerate(fresh):
            if self.nb_pos[j] == self.goal:
                continue
            known, f = shared, f0
            for _ in range(rounds):
                self.blocked = known                    # walk as the teammate would, on what we share
                path = self._descent(f, self.nb_pos[j])
                self.blocked = mine
                if not path:
                    break
                hits, arc, prev = {}, 0.0, self.nb_pos[j]
                for c in path:
                    arc += h3(prev, c)
                    prev = c
                    if mine[self._ix(c)]:
                        a = index.get(self._key_bit(*c)[0])
                        if a is not None and a not in hits and U[a, col] > 0:
                            hits[a] = arc
                if not hits:
                    break
                known = known.copy()
                for a, arc in hits.items():
                    V[a, col] = max(V[a, col], 2.0 * max(0.0, arc - self.s.sensor_range))
                    known[cells[starts[a]:ends[a]]] = True
                f = distance_field(self.shape, known | self.wall, self._ix(self.goal))
                self.n_fields += 1
