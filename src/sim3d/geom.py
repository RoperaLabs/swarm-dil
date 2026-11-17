"""Lattice geometry of GRACE-3D 2.0.0 (docs/v2_spec.md §1-2): step cost h3, the canonical segment, the exact
segment test against stored tiles, the box-visibility planner (BVP) and anytime A* in a bounded arena.
Pure functions of the agent's own data; no world access. `ops` (an Ops counter or None) receives the
primitive operations for the cycle model."""
import heapq
from math import inf, sqrt

W2, W3 = sqrt(2) - 1, sqrt(3) - sqrt(2)
STEPS = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1) if dx or dy or dz]
TS = 4                                         # tile side, voxels


def h3(d):
    """Exact 26-lattice distance of displacement d on the empty lattice."""
    a, b, c = sorted((abs(d[0]), abs(d[1]), abs(d[2])), reverse=True)
    return a + W2 * b + W3 * c


def sub(p, q):
    return p[0] - q[0], p[1] - q[1], p[2] - q[2]


def segment(p, q):
    """Canonical segment sigma(p, q), p excluded (spec §2): nested Bresenham, the minor axis moves only
    when the middle one does, so the cost is exactly h3(q - p) and every cell stays within l-inf
    distance 1 of the straight segment (T2.5a)."""
    d = sub(q, p)
    hi, mid, lo = sorted(range(3), key=lambda i: -abs(d[i]))
    n_hi, n_mid, n_lo = abs(d[hi]), abs(d[mid]), abs(d[lo])
    sign = [1 if v >= 0 else -1 for v in d]
    out = []
    for k in range(1, n_hi + 1):
        c = [0, 0, 0]
        c[hi] = k
        c[mid] = (2 * k * n_mid + n_hi) // (2 * n_hi)
        c[lo] = (2 * c[mid] * n_lo + n_mid) // (2 * n_mid) if n_mid else 0
        out.append((p[0] + sign[0] * c[0], p[1] + sign[1] * c[1], p[2] + sign[2] * c[2]))
    return out


# ------------------------------------------------------------------------------- tiles

def key_bit(c):
    """Tile key and mask bit of voxel c (bit 16 dx + 4 dy + dz, spec §4)."""
    return (c[0] // TS, c[1] // TS, c[2] // TS), 1 << ((c[0] % TS) * 16 + (c[1] % TS) * 4 + c[2] % TS)


def cells_of(key, mask):
    out = []
    while mask:
        low = mask & -mask
        b = low.bit_length() - 1
        out.append((key[0] * TS + b // 16, key[1] * TS + b // 4 % 4, key[2] * TS + b % 4))
        mask ^= low
    return out


def box_of(key, mask):
    """Occupied bounding box (lo, hi), inclusive voxel coordinates, of a non-empty tile."""
    cs = cells_of(key, mask)
    return tuple(min(c[a] for c in cs) for a in range(3)), tuple(max(c[a] for c in cs) for a in range(3))


def blocked_in(tiles, c, ops=None):
    if ops:
        ops.add("hash")
        ops.add("bit")
    key, bit = key_bit(c)
    return bool(tiles.get(key, 0) & bit)


def _slab_hit(p, q, lo, hi):
    """Does the closed segment p-q meet the closed box [lo, hi] (floats; boxes passed already dilated)?"""
    t0, t1 = 0.0, 1.0
    for a in range(3):
        d = q[a] - p[a]
        if d == 0:
            if not lo[a] <= p[a] <= hi[a]:
                return False
            continue
        u, v = (lo[a] - p[a]) / d, (hi[a] - p[a]) / d
        t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
        if t0 > t1:
            return False
    return True


def segment_block(p, q, tiles, boxes, ops=None):
    """Exact test (spec step 4, T2.5): the key of a stored tile holding a cell of sigma(p, q), or None if
    the segment is believed free. A tile whose box, dilated by one voxel (plus a float margin, the safe
    side), misses the straight segment holds no cell of sigma (T2.5a(d)); otherwise the cells of sigma
    inside the box are checked against the mask."""
    cells = None
    e = 1.0 + 1e-6
    for key, (lo, hi) in boxes.items():
        if ops:
            ops.add("slab")
        if not _slab_hit(p, q, (lo[0] - e, lo[1] - e, lo[2] - e), (hi[0] + e, hi[1] + e, hi[2] + e)):
            continue
        if cells is None:
            cells = segment(p, q)
            if ops:
                ops.add("dda", len(cells))
        mask = tiles[key]
        for c in cells:
            if lo[0] <= c[0] <= hi[0] and lo[1] <= c[1] <= hi[1] and lo[2] <= c[2] <= hi[2]:
                if ops:
                    ops.add("bit")
                if mask & key_bit(c)[1]:
                    return key
    return None


def bvp(start, goal, tiles, boxes, inside, ops=None):
    """The box-visibility planner run to completion: the node path or None."""
    steps = bvp_steps(start, goal, tiles, boxes, inside, ops)
    while True:
        try:
            next(steps)
        except StopIteration as done:
            return done.value


def bvp_steps(start, goal, tiles, boxes, inside, ops=None):
    """Box-visibility planner (spec step 4): lazy weighted A* (an edge is tested exactly only when the
    entry that uses it leaves the heap) with heuristic h3 on the complete graph whose nodes are start, goal
    and the free lattice points diagonally adjacent to the corners of every stored tile's occupied box.
    A generator that yields after every heap pop, so the caller can spread it over slots; it returns the
    node path or None. Any returned path is a walk of G(K) (T2.5)."""
    nodes = [start, goal]
    seen = {start, goal}
    for lo, hi in boxes.values():
        for x in (lo[0] - 1, hi[0] + 1):
            for y in (lo[1] - 1, hi[1] + 1):
                for z in (lo[2] - 1, hi[2] + 1):
                    c = (x, y, z)
                    if c not in seen and inside(c) and not blocked_in(tiles, c, ops):
                        seen.add(c)
                        nodes.append(c)
    n = len(nodes)
    hgoal = [h3(sub(goal, v)) for v in nodes]
    if ops:
        ops.add("h3", n)
    parent, closed = [-1] * n, [False] * n
    heap = [(hgoal[0], 0.0, 0, -1)]            # (f, g, node, via): the edge via -> node is not yet tested
    while heap:
        yield
        _, gv, v, u = heapq.heappop(heap)
        if ops:
            ops.add("heap")
        if closed[v]:
            continue
        if u >= 0 and segment_block(nodes[u], nodes[v], tiles, boxes, ops) is not None:
            continue
        closed[v], parent[v] = True, u
        if v == 1:
            path = [1]
            while path[-1] != 0:
                path.append(parent[path[-1]])
            return [nodes[i] for i in reversed(path)]
        for w in range(n):
            if closed[w]:
                continue
            gw = gv + h3(sub(nodes[w], nodes[v]))
            if ops:
                ops.add("relax")
                ops.add("h3")
            heapq.heappush(heap, (gw + hgoal[w], gw, w, v))     # every entry: a cheaper one may fail its test
            if ops:
                ops.add("heap")
    return None


# ------------------------------------------------------------------------------- anytime A*

def astar(free, start, goal, budget=64, arena=None, ops=None, parent=None, info=None):
    """Anytime A* with the consistent heuristic h3 (T2.6). Yields, every `budget` expansions, the
    open-list minimum f, a sound lower bound on the cost-to-go from `start`; the last value yielded is
    the exact cost (inf if `goal` is unreachable). With `arena`, at most that many nodes are stored: on
    overflow the run yields the f of the node being expanded (read before the overflowing expansion,
    hence still sound) and stops. `parent`, if a dict, receives the search tree for path recovery; `info`,
    if a dict, gets info["end"] in {"goal", "overflow", "unreachable"} when the run stops.
    `free(cell)` is the agent's belief, bounds included."""
    end = info if info is not None else {}
    g = {start: 0.0}
    heap = [(h3(sub(goal, start)), start)]
    closed = set()
    n = 0
    while heap:
        f, c = heap[0]
        if c in closed:
            heapq.heappop(heap)
            continue
        if c == goal:
            end["end"] = "goal"
            yield g[c]
            return
        if n and n % budget == 0:
            yield f
        heapq.heappop(heap)
        if ops:
            ops.add("heap")
        closed.add(c)
        n += 1
        for s in STEPS:
            y = (c[0] + s[0], c[1] + s[1], c[2] + s[2])
            if ops:
                ops.add("relax")
            if y in closed or not free(y):
                continue
            gy = g[c] + h3(s)
            if gy < g.get(y, inf):
                if arena is not None and y not in g and len(g) >= arena:
                    end["end"] = "overflow"
                    yield f
                    return
                g[y] = gy
                if parent is not None:
                    parent[y] = c
                heapq.heappush(heap, (gy + h3(sub(goal, y)), y))
                if ops:
                    ops.add("heap")
                    ops.add("h3")
    end["end"] = "unreachable"
    yield inf
