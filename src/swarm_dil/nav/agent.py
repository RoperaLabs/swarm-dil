"""On-board software of one agent.

An agent only ever receives three things: its own sensor readings (cells at relative offsets
inside its vision cone, blocked or free), whether its last move succeeded, and raw bytes from
teammates. Its position, map, teammates' positions and knowledge, and link quality are all
estimated here. This module never imports the world.

Protocols (all agents of a mission run the same one):
    none         no communication
    flood_raw    newest-first round robin of blocked cells (2 B each)
    flood_tile   newest-first round robin of 4x4 tiles (4 B each)
    grace        Goal-Relevant Adaptive Compressed Exchange (this work)
    grace_norel  ablation: every unknown tile is worth 1 (no goal relevance)
    grace_noloss ablation: assumes every copy is delivered (no loss model)
    grace_fifo   ablation: forgets oldest tiles instead of least relevant
"""
from collections import deque
from dataclasses import dataclass
from functools import lru_cache

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

from . import theory, wire

DIRS = ((1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))
PROTOCOLS = ("none", "flood_raw", "flood_tile", "grace", "grace_norel", "grace_noloss", "grace_fifo")
TS = 4  # tile side, cells (Cor. 3: optimal for what a range sensor sees)


@dataclass(frozen=True)
class Spec:
    """Everything an agent is told before launch: hardware limits and mission order."""
    n: int                        # grid side, cells
    start: tuple
    goal: tuple
    team: int                     # team size (ids 0..team-1)
    launch_at: int                # slot at which this agent may start moving
    protocol: str = "grace"
    sensor_range: float = 5.0     # cells
    memory_tiles: int | None = None  # map memory, in 4 B tiles; None = unbounded
    budget: int = 64              # max bytes per message (one message per slot at most)
    price: float = 0.5            # GRACE: cells of travel a byte must save to be sent
    burst_lambda: float = 0.9     # design constant: per-slot correlation of link state
    stale: int = 20               # slots after which a silent neighbour is ignored
    heartbeat: int = 5            # send at least a header every this many slots


@lru_cache(maxsize=4)
def _edges(n):
    """All 8-neighbour edges of an n x n grid with the cells a diagonal must not cut."""
    ix, iy = np.indices((n, n)).reshape(2, -1)
    u, v, w, c1, c2 = [], [], [], [], []
    for dx, dy in DIRS:
        tx, ty = ix + dx, iy + dy
        ok = (tx >= 0) & (tx < n) & (ty >= 0) & (ty < n)
        a, b = (ix * n + iy)[ok], (tx * n + ty)[ok]
        u.append(a)
        v.append(b)
        w.append(np.full(a.size, theory.SQRT2 if dx and dy else 1.0))
        c1.append((tx * n + iy)[ok])   # orthogonal cells a diagonal step passes between
        c2.append((ix * n + ty)[ok])
    return tuple(np.concatenate(x) for x in (u, v, w, c1, c2))


def distance_field(n, blocked, goal):
    """Optimistic cost-to-go: shortest 8-connected path length to `goal` avoiding `blocked`
    (unknown cells count as free). Returns an (n*n,) array, inf where unreachable."""
    u, v, w, c1, c2 = _edges(n)
    ok = ~(blocked[u] | blocked[v] | blocked[c1] | blocked[c2])
    graph = csr_matrix((w[ok], (u[ok], v[ok])), shape=(n * n, n * n))
    return dijkstra(graph, directed=False, indices=goal[0] * n + goal[1])


class Agent:
    def __init__(self, ident, spec, rng):
        if spec.protocol not in PROTOCOLS:
            raise ValueError(f"unknown protocol {spec.protocol!r}")
        self.id, self.s, self.rng = ident, spec, rng
        n = spec.n
        self.t = 0
        self.pos = tuple(spec.start)       # dead reckoning from start + own successful moves
        self.heading = 0
        self.arrived = self.pos == tuple(spec.goal)
        self.path_len = 0.0
        self.seq = 0
        self.last_tx = -10**9
        self.bytes_sent = 0
        self.tiles = {}                    # (tx, ty) -> 16-bit blocked mask
        self.learned = {}                  # (tx, ty) -> slot learned / last changed
        self.blocked = np.zeros(n * n, bool)
        self.field = None
        gx, gy = spec.goal
        ix, iy = np.indices((n, n)).reshape(2, -1)
        self.h_goal = np.maximum(abs(ix - gx), abs(iy - gy)) + (theory.SQRT2 - 1) * np.minimum(abs(ix - gx), abs(iy - gy))
        # neighbour table: last heard slot, estimated cell, seq bookkeeping
        self.nb_heard = np.full(spec.team, -10**9)
        self.nb_pos = [tuple(spec.start)] * spec.team
        self.nb_seq = [None] * spec.team
        self.nb_rx = np.zeros(spec.team)
        self.nb_expected = np.zeros(spec.team)
        # u[key][j]: probability teammate j does not know my current version of tile key
        self.u = {}
        self.last_sent = {}
        self.queue = deque()               # round robin for the flood baselines

    # ------------------------------------------------------------------ knowledge
    def _cells_of(self, key, mask):
        tx, ty = key
        return [(tx * TS + b // TS, ty * TS + b % TS) for b in range(TS * TS) if mask >> b & 1]

    def _add_blocked(self, x, y, now):
        key, bit = (x // TS, y // TS), (x % TS) * TS + (y % TS)
        return self._merge(key, 1 << bit, now)

    def _merge(self, key, mask, now, sender=None):
        old = self.tiles.get(key, 0)
        new = old | mask
        changed = new != old
        if changed:
            self.tiles[key] = new
            self.learned[key] = now
            for x, y in self._cells_of(key, new & ~old):
                self.blocked[x * self.s.n + y] = True
            self.field = None
            self.u[key] = np.ones(self.s.team)
            self.u[key][self.id] = 0.0
            if key not in self.queue:
                self.queue.appendleft(key)
        if sender is not None and mask | new == mask:  # sender's version covers mine
            self.u[key][sender] = 0.0
        return changed

    def _evict(self):
        cap = self.s.memory_tiles
        while cap is not None and len(self.tiles) > cap:
            keys = list(self.tiles)
            if self.s.protocol == "grace_fifo" or not self.s.protocol.startswith("grace"):
                victim = min(keys, key=self.learned.get)
            else:
                victim = keys[int(np.argmax([self._margin(k) for k in keys]))]
            for x, y in self._cells_of(victim, self.tiles[victim]):
                self.blocked[x * self.s.n + y] = False
            for d in (self.tiles, self.learned, self.u, self.last_sent):
                d.pop(victim, None)
            if victim in self.queue:
                self.queue.remove(victim)
            self.field = None

    def _margin(self, key):
        """Ellipse margin (Lemma 3): min over the tile's cells of h(x, y) + h(y, B), minus my
        believed cost-to-go. Positive margin: the tile cannot touch my current plan."""
        d_here = self._field()[self.pos[0] * self.s.n + self.pos[1]]
        cells = self._cells_of(key, self.tiles[key])
        best = min(theory.octile(x - self.pos[0], y - self.pos[1]) + self.h_goal[x * self.s.n + y] for x, y in cells)
        return best - d_here if np.isfinite(d_here) else -self.learned[key]

    def _field(self):
        if self.field is None:
            self.field = distance_field(self.s.n, self.blocked, self.s.goal)
        return self.field

    # ------------------------------------------------------------------ sensing and motion
    def observe(self, readings):
        """readings: iterable of (dx, dy, blocked) relative to my cell. Free cells carry no
        decision value under optimistic planning (Lemma 1), so only blocked ones are kept."""
        for dx, dy, hit in readings:
            x, y = self.pos[0] + dx, self.pos[1] + dy
            if hit and 0 <= x < self.s.n and 0 <= y < self.s.n:
                self._add_blocked(x, y, self.t)
        self._evict()

    def decide(self):
        """Direction index to move this slot, or None to hold."""
        if self.arrived or self.t < self.s.launch_at:
            return None
        n, (x, y) = self.s.n, self.pos
        f = self._field()
        best, choice = np.inf, None
        for k, (dx, dy) in enumerate(DIRS):
            tx, ty = x + dx, y + dy
            if not (0 <= tx < n and 0 <= ty < n) or self.blocked[tx * n + ty]:
                continue
            if dx and dy and (self.blocked[tx * n + y] or self.blocked[x * n + ty]):
                continue
            cost = (theory.SQRT2 if dx and dy else 1.0) + f[tx * n + ty]
            if cost < best:
                best, choice = cost, k
        if choice is not None:
            self.heading = choice
        return choice

    def moved(self, direction, ok):
        """Proprioception: the world tells me whether my commanded step happened."""
        if ok and direction is not None:
            dx, dy = DIRS[direction]
            self.pos = (self.pos[0] + dx, self.pos[1] + dy)
            self.path_len += theory.SQRT2 if dx and dy else 1.0
            self.arrived = self.pos == tuple(self.s.goal)
        self.t += 1

    # ------------------------------------------------------------------ communication
    def receive(self, packet):
        raw = self.s.protocol == "flood_raw"
        sender, seq, pos, items = wire.decode(packet, raw=raw)
        if sender == self.id:
            return
        self.nb_heard[sender] = self.t
        self.nb_pos[sender] = pos
        last = self.nb_seq[sender]
        self.nb_expected[sender] += 1 if last is None else (seq - last) % 256
        self.nb_rx[sender] += 1
        self.nb_seq[sender] = seq
        for item in items:
            if raw:
                self._add_blocked(*item, self.t)
            else:
                self._merge(item[0], item[1], self.t, sender=sender)
        self._evict()

    def transmit(self):
        """Bytes to broadcast this slot, or None."""
        p = self.s.protocol
        if p == "none":
            return None
        room = self.s.budget - wire.HEADER
        tiles, cells = [], []
        if p == "flood_tile":
            for _ in range(min(room // wire.TILE, len(self.queue))):
                key = self.queue.popleft()
                tiles.append((key, self.tiles[key]))
                self.queue.append(key)
        elif p == "flood_raw":
            while self.queue and len(cells) < room // wire.CELL:
                key = self.queue.popleft()
                cells += self._cells_of(key, self.tiles[key])
                self.queue.append(key)
            cells = cells[: room // wire.CELL]
        else:
            tiles = self._grace_select(room // wire.TILE)
        if not tiles and not cells and self.t - self.last_tx < self.s.heartbeat:
            return None
        self.seq += 1
        self.last_tx = self.t
        packet = wire.encode(self.id, self.seq, self.pos, tiles, cells)
        self.bytes_sent += len(packet)
        return packet

    def _grace_select(self, slots):
        """Pick the tiles with the largest expected team value this slot (Sec. 6)."""
        s, n, now = self.s, self.s.n, self.t
        fresh = [j for j in range(s.team) if j != self.id and now - self.nb_heard[j] <= s.stale]
        if not fresh or slots <= 0 or not self.tiles:
            return []
        keys = list(self.tiles)
        U = np.array([self.u[k][fresh] for k in keys])                 # (tiles, fresh)
        # loss of a copy sent now, given the receiver does not have it yet (Prop. 4)
        q_hat = (self.nb_rx[fresh] + 1) / (self.nb_expected[fresh] + 2)
        pi_b = 1.0 - q_hat
        lam = s.burst_lambda
        since_heard = now - self.nb_heard[fresh]
        beta = np.empty_like(U)
        for a, k in enumerate(keys):
            t1 = self.last_sent.get(k, -10**9)
            after_copy = t1 > self.nb_heard[fresh]
            beta[a] = np.where(after_copy, pi_b + (1 - pi_b) * lam ** (now - t1),
                               pi_b * (1 - lam ** since_heard))
        if s.protocol == "grace_noloss":
            beta[:] = 0.0
        if s.protocol == "grace_norel":
            V = (U > 0).astype(float)
        else:
            V = self._relevance(keys, fresh, U)
        gain = (V * U * (1 - beta)).sum(1)
        order = np.argsort(-gain)[:slots]
        chosen = [a for a in order if gain[a] >= s.price * wire.TILE]
        for a in chosen:
            k = keys[a]
            self.last_sent[k] = now
            self.u[k][fresh] = U[a] * beta[a]
        return [(keys[a], self.tiles[keys[a]]) for a in chosen]

    def _crossings(self, f, start, index):
        """Walk the optimistic path from `start` down field f; return {tile: arc length of the
        first cell of that tile I know is blocked}."""
        n, goal = self.s.n, tuple(self.s.goal)
        (x, y), arc, hits = start, 0.0, {}
        if not np.isfinite(f[x * n + y]):
            return hits
        for _ in range(4 * n):
            if self.blocked[x * n + y]:
                k = (x // TS, y // TS)
                if k in index and k not in hits:
                    hits[k] = arc
            if (x, y) == goal:
                break
            nxt, best = None, np.inf
            for dx, dy in DIRS:  # steepest descent along the optimistic cost-to-go
                tx, ty = x + dx, y + dy
                if 0 <= tx < n and 0 <= ty < n and f[tx * n + ty] < f[x * n + y]:
                    c = (theory.SQRT2 if dx and dy else 1.0) + f[tx * n + ty]
                    if c < best:
                        best, nxt = c, (tx, ty, c - f[tx * n + ty])
            if nxt is None:
                break
            x, y, step = nxt
            arc += step
        return hits

    def _relevance(self, keys, fresh, U, rounds=4):
        """V[a, j]: value of tile a for teammate j (Sec. 4).

        Round 0 predicts j's plan on the map I believe we all share (common knowledge) and
        values every tile of mine it crosses at 2 (s - r)^+ (Prop. 1). Those tiles are then
        added to j's predicted map and the plan recomputed, so tiles that bound j's *next*
        plan are valued too: the plan-equivalence closure (Def. 3), truncated at `rounds`.
        """
        n, s = self.s.n, self.s
        index = {k: a for a, k in enumerate(keys)}

        def cells(ks):
            return [x * n + y for k in ks for x, y in self._cells_of(k, self.tiles[k])]

        V = np.zeros((len(keys), len(fresh)))
        shared = [k for a, k in enumerate(keys) if U[a].max() <= 0.5]
        base = np.zeros(n * n, bool)
        base[cells(shared)] = True
        f0 = distance_field(n, base, s.goal)

        # Ellipse prior (Prop. 3): a tile with margin m w.r.t. j can only enter a future plan
        # of j if j's believed cost rises by >= m; with that rise ~ Exp(mu), P <= exp(-m / mu).
        # mu is my own observed cost inflation so far (self-calibrating, >= 1 cell).
        own = self._field()[self.pos[0] * n + self.pos[1]]
        sx, sy = s.start
        mu = max(1.0, self.path_len + own - theory.octile(s.goal[0] - sx, s.goal[1] - sy)) if np.isfinite(own) else 1.0
        tile_of, xs, ys = [], [], []
        for a, k in enumerate(keys):
            for x, y in self._cells_of(k, self.tiles[k]):
                tile_of.append(a)
                xs.append(x)
                ys.append(y)
        tile_of, xs, ys = np.array(tile_of), np.array(xs), np.array(ys)
        starts = np.flatnonzero(np.r_[True, tile_of[1:] != tile_of[:-1]])
        h_goal = self.h_goal[xs * n + ys]
        for col, j in enumerate(fresh):
            jx, jy = self.nb_pos[j]
            d_j = f0[jx * n + jy]
            if not np.isfinite(d_j):
                continue
            dx, dy = np.abs(xs - jx), np.abs(ys - jy)
            h_j = np.maximum(dx, dy) + (theory.SQRT2 - 1) * np.minimum(dx, dy)
            margin = np.minimum.reduceat(h_j + h_goal, starts) - d_j
            reach = np.minimum.reduceat(h_j, starts)
            V[tile_of[starts], col] = (np.exp(-np.maximum(margin, 0.0) / mu)
                                       * 2.0 * np.maximum(0.0, reach - s.sensor_range))
        for col, j in enumerate(fresh):
            if tuple(self.nb_pos[j]) == tuple(s.goal):
                continue
            known_j, f = base, f0
            for _ in range(rounds):
                hits = {k: arc for k, arc in self._crossings(f, self.nb_pos[j], index).items()
                        if U[index[k], col] > 0}
                if not hits:
                    break
                for k, arc in hits.items():
                    a = index[k]
                    V[a, col] = max(V[a, col], theory.voi_bound(arc, s.sensor_range))
                known_j = known_j.copy()
                known_j[cells(hits)] = True
                f = distance_field(n, known_j, s.goal)
        return V
