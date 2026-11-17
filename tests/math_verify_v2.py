"""GRACE-3D 2.0.0, gate F2B layer 2: symbolic (SymPy), SMT (Z3) and exhaustive / Monte Carlo checks of the
statements in docs/v2_spec.md §6 against the code in src/sim3d.

    python tests/math_verify_v2.py     # JSON summary on stdout, exit code 1 on any failure

Also collected by pytest (see pyproject.toml).
"""
import heapq
import json
import random
import sys
from fractions import Fraction
from itertools import permutations, product
from math import inf
from pathlib import Path

import numpy as np
import sympy as sp
import z3

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sim3d import geom, wire  # noqa: E402
from swarm_dil.nav3.agent3 import distance_field  # noqa: E402  independent reference (v0.4 Dijkstra)

REPORT = {}


def note(key, value):
    REPORT[key] = value


def z3_valid(claim):
    s = z3.Solver()
    s.add(z3.Not(claim))
    return s.check() == z3.unsat


# ---------------------------------------------------------------- wire format (spec §4)

FIELDS = {  # word: [(name, width)] from the most significant bit, as in the spec table
    "H0": [("sender", 5), ("flags", 3)],
    "H2H3": [("x", 5), ("y", 5), ("z", 5), ("digest", 1)],
    "H4": [("tiles", 4), ("claims", 4)],
    "claim": [("target", 5), ("kind", 3), ("n", 8), ("value", 16)],
    "tilekey": [("tx", 5), ("ty", 5), ("tz", 5), ("reserved", 1)],
}


def test_wire_layout_z3():
    """Every word: fields tile it exactly, and shift-or packing / shift-mask unpacking are inverse
    for all values (bit-vector proof, not sampling)."""
    for word, fields in FIELDS.items():
        width = sum(w for _, w in fields)
        assert width % 8 == 0, word
        vs = [z3.BitVec(f"{word}_{n}", w) for n, w in fields]
        packed, shift, shifts = z3.BitVecVal(0, width), width, []
        for v, (_, w) in zip(vs, fields):
            shift -= w
            shifts.append(shift)
            packed = packed | z3.ZeroExt(width - w, v) << shift
        back = [z3.Extract(w - 1, 0, z3.LShR(packed, s)) for s, (_, w) in zip(shifts, fields)]
        assert z3_valid(z3.And([b == v for b, v in zip(back, vs)])), word
    note("wire_layout_z3", "proved for " + ", ".join(FIELDS))


def bits(value, width):
    return format(value, f"0{width}b")


def reference_encode(p):
    """Independent bit-string encoder written from the spec table, not from wire.py."""
    s = bits(p.sender, 5) + bits(p.flags, 3) + bits(p.seq % 256, 8)
    s += "".join(bits(c % 32, 5) for c in p.pose) + bits(p.digest is not None, 1)
    s += bits(len(p.tiles), 4) + bits(len(p.claims), 4)
    if p.digest is not None:
        s += bits(p.digest, 32)
    for c in p.claims:
        s += bits(c.target, 5) + bits(c.kind, 3) + bits(len(c.waypoints), 8) + bits(c.value, 16)
        s += "".join(bits(v, 8) for w in c.waypoints for v in w)
    for (tx, ty, tz), mask in p.tiles:
        s += bits(tx, 5) + bits(ty, 5) + bits(tz, 5) + "0" + bits(mask, 64)
    return int(s, 2).to_bytes(len(s) // 8, "big")


def random_packet(rng):
    claims = [wire.Claim(rng.randrange(32), rng.randint(wire.ADVICE, wire.PLAN_X), rng.randrange(65536),
                         tuple(tuple(rng.randrange(256) for _ in range(3)) for _ in range(rng.randint(0, 3))))
              for _ in range(rng.randint(0, 15))]
    tiles = [((rng.randrange(32), rng.randrange(32), rng.randrange(32)), rng.getrandbits(64))
             for _ in range(rng.randint(0, 15))]
    return wire.Packet(rng.randrange(32), rng.randrange(1000), tuple(rng.randint(-300, 300) for _ in range(3)),
                       rng.randrange(8), rng.getrandbits(32) if rng.random() < 0.5 else None, claims, tiles)


def test_wire_bit_exact_and_roundtrip():
    rng = random.Random(0)
    for _ in range(3000):
        p = random_packet(rng)
        b = wire.encode(p)
        assert b == reference_encode(p)
        assert len(b) == wire.size(p)
        q = wire.decode(b)
        assert q.pose == tuple(c % 32 for c in p.pose)
        assert (q.sender, q.seq, q.flags, q.digest, q.claims, q.tiles) == (p.sender, p.seq % 256, p.flags, p.digest,
                                                                          p.claims, p.tiles)
        for cut in range(len(b)):           # every truncation is rejected
            try:
                wire.decode(b[:cut])
            except ValueError:
                continue
            raise AssertionError("truncated packet accepted")
    note("wire_bit_exact", "3000 random packets equal the spec-table encoder; all truncations rejected")


def test_zero_global_coordinate_bytes():
    """Shifting the pose by any multiple of 32 voxels leaves the bytes unchanged: the header carries no
    coordinate beyond its 5-bit wrap (target 4)."""
    rng = random.Random(1)
    for _ in range(500):
        p = random_packet(rng)
        k = [32 * rng.randint(-5, 5) for _ in range(3)]
        p2 = wire.Packet(p.sender, p.seq, tuple(a + b for a, b in zip(p.pose, k)), p.flags, p.digest, p.claims, p.tiles)
        assert wire.encode(p) == wire.encode(p2)
    note("global_coordinate_bytes", 0)


# ---------------------------------------------------------------- T2.4 wrapped pose

def test_unwrap():
    for e in range(-50, 50):
        for p in range(e - 40, e + 40):
            assert (wire.unwrap(p % 32, e) == p) == (-16 <= p - e < 16)
    p, e = z3.Ints("p e")
    un = e + (p % 32 - e + 16) % 32 - 16
    assert z3_valid((un == p) == z3.And(-16 <= p - e, p - e < 16))
    note("T2.4_unwrap", "exhaustive |p-e|<40 and Z3 proof over all integers")


# ---------------------------------------------------------------- h3 (spec §1)

def test_h3_is_lattice_distance():
    """Dijkstra on the empty 26-lattice (independent of the formula) equals h3 for |d|_inf <= 4."""
    r = 6
    dist = {(0, 0, 0): 0.0}
    heap = [(0.0, (0, 0, 0))]
    while heap:
        d, c = heapq.heappop(heap)
        if d > dist[c]:
            continue
        for s in geom.STEPS:
            y = (c[0] + s[0], c[1] + s[1], c[2] + s[2])
            if max(map(abs, y)) <= r and d + geom.h3(s) < dist.get(y, inf) - 1e-12:
                dist[y] = d + geom.h3(s)
                heapq.heappush(heap, (dist[y], y))
    worst = max(abs(dist[d] - geom.h3(d)) for d in product(range(-4, 5), repeat=3))
    assert worst < 1e-9
    note("h3_vs_dijkstra_max_err", worst)


def test_h3_weights_and_rearrangement():
    s2, s3 = sp.sqrt(2), sp.sqrt(3)
    assert (1 - (s2 - 1)).is_positive and ((s2 - 1) - (s3 - s2)).is_positive and (s3 - s2).is_positive
    nest = s3 - 2 * s2 + 1                     # used in the proof of T2.5a
    assert nest.is_negative
    a1, a2, a3, x, y, z = z3.Reals("a1 a2 a3 x y z")
    hyp = z3.And(a1 >= a2, a2 >= a3, a3 >= 0, x >= y, y >= z, z >= 0)
    sorted_form = a1 * x + a2 * y + a3 * z
    for perm in permutations((x, y, z)):
        assert z3_valid(z3.Implies(hyp, a1 * perm[0] + a2 * perm[1] + a3 * perm[2] <= sorted_form))
    rng = random.Random(2)
    for _ in range(20000):
        a, b = [tuple(rng.randint(-20, 20) for _ in range(3)) for _ in range(2)]
        assert geom.h3((a[0] + b[0], a[1] + b[1], a[2] + b[2])) <= geom.h3(a) + geom.h3(b) + 1e-12
    note("h3_rearrangement", "Z3 proof: sorted closed form = max over the 6 weight assignments (Lean hmax)")
    note("nesting_coefficient", float(nest))


# ---------------------------------------------------------------- T2.5a canonical segment

def test_segment_exhaustive():
    worst = Fraction(0)
    for d in product(range(-10, 11), repeat=3):
        cells = geom.segment((0, 0, 0), d)
        prev, cost = (0, 0, 0), 0.0
        for c in cells:
            step = geom.sub(c, prev)
            assert step in geom.STEPS
            cost += geom.h3(step)
            prev = c
        assert prev == d
        assert abs(cost - geom.h3(d)) < 1e-9, d
        n = max(map(abs, d))
        for k, c in enumerate(cells, 1):        # deviation at the line point matching the major axis
            dev = max(abs(c[a] - Fraction(k * d[a], n)) for a in range(3))
            worst = max(worst, dev)
    assert worst <= 1
    note("T2.5a_segment", f"all 9261 displacements |d|<=10: geodesic, max l-inf deviation {float(worst):.3f}")


def seg_hits_box(p, q, lo, hi):
    """Exact slab test: does the closed straight segment p-q meet the closed box [lo, hi]?"""
    t0, t1 = Fraction(0), Fraction(1)
    for a in range(3):
        d = q[a] - p[a]
        if d == 0:
            if not lo[a] <= p[a] <= hi[a]:
                return False
            continue
        u, v = Fraction(lo[a] - p[a], d), Fraction(hi[a] - p[a], d)
        t0, t1 = max(t0, min(u, v)), min(t1, max(u, v))
    return t0 <= t1


def test_segment_box_corollary():
    rng = random.Random(3)
    misses = 0
    for _ in range(20000):
        p, q = [tuple(rng.randint(0, 15) for _ in range(3)) for _ in range(2)]
        lo = tuple(rng.randint(0, 13) for _ in range(3))
        hi = tuple(a + rng.randint(0, 3) for a in lo)
        if seg_hits_box(p, q, tuple(a - 1 for a in lo), tuple(a + 1 for a in hi)):
            continue
        misses += 1
        assert not any(all(lo[a] <= c[a] <= hi[a] for a in range(3)) for c in geom.segment(p, q))
    note("T2.5a_box_corollary", f"{misses} segments clear of the dilated box: no cell inside")


# ---------------------------------------------------------------- §3 and T2.1 coding

def test_surface_code():
    s = sp.symbols("s", positive=True)
    g = (2 + s**3 / 8) / s**2
    star = sp.solve(sp.diff(g, s), s)
    assert len(star) == 1 and sp.simplify(star[0] ** 3 - 32) == 0     # s* = 32^(1/3), b_id = 16 bits
    assert g.subs(s, 4) == sp.Rational(5, 8) and g.subs(s, 3) == sp.Rational(43, 72)
    w, h = sp.symbols("w h", positive=True)
    plane = sp.limit(sp.limit(10 * (w + 3) * (h + 3) * 4 / 64 / (w * h), w, sp.oo), h, sp.oo)
    assert plane == sp.Rational(5, 8)
    rng = random.Random(4)                                     # tiles touched by a box, exact (v1 Lemma 4 in 3-D)
    for _ in range(30):
        dims = [rng.randint(1, 9) for _ in range(3)]
        total = Fraction(0)
        for off in product(range(4), repeat=3):
            n = 1
            for a in range(3):
                n *= (off[a] + dims[a] - 1) // 4 + 1
            total += n
        assert total / 64 == np.prod([Fraction(d + 3, 4) for d in dims])
    for a in range(1, 11):                                     # height fields: 2^a distinct voxel sets of size a
        sets = {frozenset((i, 0, z[i]) for i in range(a)) for z in product((0, 1), repeat=a)}
        assert len(sets) == 2 ** a and all(len(x) == a for x in sets)
    worst = 0.0                                                # m x m height fields, heights {0,1}, every offset
    for m in range(4, 13):
        for off in product(range(4), repeat=3):
            for _ in range(3):
                z = [[rng.randint(0, 1) for _ in range(m)] for _ in range(m)]
                keys = {((off[0] + i) // 4, (off[1] + j) // 4, (off[2] + z[i][j]) // 4) for i in range(m) for j in range(m)}
                b = 80 * len(keys) / m ** 2
                assert b <= 10 * (1 + 6 / m) ** 2
                if m % 4 == 0 and off[:2] == (0, 0) and off[2] <= 2:
                    assert b == 5
                worst = max(worst, b / (10 * (1 + 6 / m) ** 2))
    note("surface_code", {"s_star": float(star[0]), "B_per_voxel_s4": 0.625, "B_per_voxel_s3": float(43 / 72),
                          "bits_per_voxel_aligned": 5, "worst_vs_bound_10(1+6/m)^2": round(worst, 3),
                          "lower_bound_bits_per_voxel": 1})


# ---------------------------------------------------------------- T2.6, T2.7 on random worlds

def random_world(rng, shape=(8, 8, 4), density=0.3):
    while True:
        blocked = np.array([rng.random() < density for _ in range(np.prod(shape))])
        a, b = (0, 0, 0), (shape[0] - 1, shape[1] - 1, shape[2] - 1)
        ia, ib = 0, blocked.size - 1
        blocked[ia] = blocked[ib] = False
        to_b = distance_field(shape, blocked, ib)
        if np.isfinite(to_b[ia]) or rng.random() < 0.1:
            return blocked, a, b, to_b, distance_field(shape, blocked, ia)


def cells(shape):
    return list(product(range(shape[0]), range(shape[1]), range(shape[2])))


def test_informed_and_anytime():
    rng = random.Random(5)
    shape = (8, 8, 4)
    cs = cells(shape)
    checked = unreachable = 0
    for w in range(150):
        blocked, a, b, to_b, from_a = random_world(rng, shape)
        if w % 10 == 0:                                         # seal B in: the unreachable case
            for s in geom.STEPS:
                y = (b[0] + s[0], b[1] + s[1], b[2] + s[2])
                if all(0 <= y[k] < shape[k] for k in range(3)):
                    blocked[(y[0] * shape[1] + y[1]) * shape[2] + y[2]] = True
            to_b = distance_field(shape, blocked, blocked.size - 1)
        dstar = to_b[0]
        if np.isfinite(dstar):
            for slack in (1.0, 1.1, 1.5):
                U = dstar * slack
                for i, y in enumerate(cs):                      # T2.7
                    if not blocked[i] and from_a[i] + to_b[i] <= U + 1e-9:
                        assert geom.h3(geom.sub(y, a)) + geom.h3(geom.sub(b, y)) <= U + 1e-9
                        checked += 1
        else:
            unreachable += 1

        def free(c):
            return all(0 <= c[k] < shape[k] for k in range(3)) and not blocked[(c[0] * shape[1] + c[1]) * shape[2] + c[2]]
        for budget in (1, 3, 7):                                # T2.6
            seq = list(geom.astar(free, a, b, budget))
            assert all(x <= dstar + 1e-9 for x in seq)
            assert all(x <= y + 1e-9 for x, y in zip(seq, seq[1:]))
            assert seq[-1] == dstar or abs(seq[-1] - dstar) < 1e-9
    note("T2.7_informed_cells_checked", checked)
    note("T2.6_anytime", f"150 worlds ({unreachable} unreachable), budgets 1/3/7: bounds sound, monotone, exact at end")


# ---------------------------------------------------------------- T2.9 liveness with bounded memory

def bloom_bits(c):
    h = hash(c)
    return 1 << (h & 31) | 1 << (h >> 5 & 31)


def bounded_run(blocked, shape, a, b, K, escape, rng, t_max=20000, mu=8.0, p_esc=0.5):
    """Toy agent of spec §2 with one-voxel tiles: sense the 26 neighbours, keep at most K blocked voxels
    (evict the largest ellipse margin), step down the optimistic field (so the plan cost Phi is the believed
    cost-to-go). Escape rule of step 6: the slot did not lower Phi by >= 1 and no acquisition was fresh.
    Returns (arrived, steps, cycled)."""
    ix = lambda c: (c[0] * shape[1] + c[1]) * shape[2] + c[2]  # noqa: E731
    inside = lambda c: all(0 <= c[k] < shape[k] for k in range(3))  # noqa: E731
    mem, bloom, pos, seen, escaping, phi = [], 0, a, set(), False, None   # phi: Phi at the previous decision
    for t in range(t_max):
        if pos == b:
            return True, t, False
        fresh = False
        for s in geom.STEPS:
            y = (pos[0] + s[0], pos[1] + s[1], pos[2] + s[2])
            if inside(y) and blocked[ix(y)] and y not in mem:
                fresh |= (bloom & bloom_bits(y)) != bloom_bits(y)
                mem.append(y)
        belief = np.zeros(blocked.size, bool)
        for y in mem:
            belief[ix(y)] = True
        field = distance_field(shape, belief, ix(b))
        while len(mem) > K:
            D = field[ix(pos)]
            y = max(mem, key=lambda c: (geom.h3(geom.sub(c, pos)) + geom.h3(geom.sub(b, c)) - D, c))
            mem.remove(y)
            bloom |= bloom_bits(y)
            belief[ix(y)] = False
            field = distance_field(shape, belief, ix(b))
        D = field[ix(pos)]
        stalled = phi is not None and D > phi - 1 + 1e-9
        if escape and not escaping and stalled and not fresh and rng.random() < p_esc:
            escaping = True
        if escaping:
            s = rng.choice(geom.STEPS)
            y = (pos[0] + s[0], pos[1] + s[1], pos[2] + s[2])
            escaping = rng.random() < 1 - 1 / mu
            phi = None                                  # no plan is followed during an escape
        else:
            if not escape:
                state = (pos, frozenset(mem))
                if state in seen:
                    return False, t, True
                seen.add(state)
            y = min((c for c in ((pos[0] + s[0], pos[1] + s[1], pos[2] + s[2]) for s in geom.STEPS)
                     if inside(c) and not belief[ix(c)]),
                    key=lambda c: (geom.h3(geom.sub(c, pos)) + field[ix(c)], c))
            phi = D
        if inside(y) and not blocked[ix(y)]:
            pos = y
    return False, t_max, False


def test_liveness():
    rng = random.Random(6)
    shape = (10, 10, 3)
    worlds = []
    while len(worlds) < 40:
        blocked = np.array([rng.random() < 0.35 for _ in range(np.prod(shape))])
        blocked[0] = blocked[-1] = False
        if np.isfinite(distance_field(shape, blocked, blocked.size - 1)[0]):
            worlds.append(blocked)
    a, b = (0, 0, 0), (shape[0] - 1, shape[1] - 1, shape[2] - 1)
    out = {}
    for K in (2, 4, 8):
        cycled = arrived = 0
        steps = []
        for w in worlds:
            ok, _, cyc = bounded_run(w, shape, a, b, K, False, rng)
            cycled += cyc
            ok2, t, _ = bounded_run(w, shape, a, b, K, True, random.Random(len(steps)))
            arrived += ok2
            steps.append(t)
        out[K] = {"deterministic_cycles": cycled, "arrived_with_escape": arrived, "worlds": len(worlds),
                  "steps_p50": float(np.percentile(steps, 50)), "steps_max": max(steps)}
        assert arrived == len(worlds), out
    assert out[2]["deterministic_cycles"] > 0      # the escape is needed, not decorative
    note("T2.9_liveness", out)


# ---------------------------------------------------------------- F4.5 candidates and audit

def test_ge_closed_form():
    """F45.4: P(B after tau | G) = pi_B (1 - lambda^tau): symbolic recurrence and Monte Carlo."""
    p, q, t = sp.symbols("p q tau", positive=True)
    lam = 1 - p - q
    f = lambda n: p / (p + q) * (1 - lam ** n)  # noqa: E731
    assert sp.simplify(f(0)) == 0
    assert sp.simplify(f(t + 1) - (p + lam * f(t))) == 0
    rng = np.random.default_rng(7)
    worst = 0.0
    for pp, qq in ((0.05, 0.15), (0.2, 0.3), (0.1, 0.02)):
        bad = np.zeros(200_000, bool)                      # every chain starts good (the receiver was heard)
        for tau in range(1, 13):
            u = rng.random(bad.size)
            bad = np.where(bad, u >= qq, u < pp)
            exact = pp / (pp + qq) * (1 - (1 - pp - qq) ** tau)
            sd = np.sqrt(exact * (1 - exact) / bad.size)
            worst = max(worst, abs(bad.mean() - exact) / max(sd, 1e-12))
    assert worst < 4.5
    note("F45.4_ge_closed_form", f"SymPy recurrence proof; Monte Carlo worst deviation {worst:.2f} sd")


def test_belief_update_identity():
    """F45.5: the value-weighted scalar update keeps the expected value still to deliver."""
    u = sp.symbols("u", positive=True)
    V, B = sp.symbols("V0:3", positive=True), sp.symbols("b0:3", positive=True)
    bbar = sum(v * b for v, b in zip(V, B)) / sum(V)
    assert sp.simplify(sum(v * u * b for v, b in zip(V, B)) - sum(V) * u * bbar) == 0
    note("F45.5_belief_update", "SymPy identity, 3 receivers")


def test_futility_invariant():
    """F45.1-F45.2 on random worlds: with maps that only grow, the carried bound never exceeds d_K(p)
    (no futility); with forgetting, every futile plan (cost < L) enters a truly blocked voxel."""
    rng = random.Random(8)
    shape = (10, 10, 3)
    ix = lambda c: (c[0] * shape[1] + c[1]) * shape[2] + c[2]  # noqa: E731
    inside = lambda c: all(0 <= c[k] < shape[k] for k in range(3))  # noqa: E731
    futile = checked = 0
    for w in range(150):                                   # a wall with one hole forces a detour; plus 10 % noise
        blocked = np.array([rng.random() < 0.1 for _ in range(int(np.prod(shape)))])
        wx, gy, gz = rng.randint(3, 6), rng.choice((0, 1, 8, 9)), rng.randrange(3)
        for y in range(shape[1]):
            for z in range(shape[2]):
                blocked[ix((wx, y, z))] = (y, z) != (gy, gz)
        a, b = (0, rng.randrange(10), 0), (9, rng.randrange(10), 0)
        blocked[ix(a)] = blocked[ix(b)] = False
        to_b = distance_field(shape, blocked, ix(b))
        if not np.isfinite(to_b[ix(a)]):
            continue
        walk = [a]                                         # the agent walks a true shortest path
        while walk[-1] != b:
            c = walk[-1]
            walk.append(min((y for y in ((c[0] + s[0], c[1] + s[1], c[2] + s[2]) for s in geom.STEPS)
                             if inside(y) and not blocked[ix(y)]),
                            key=lambda y: (geom.h3(geom.sub(y, c)) + to_b[ix(y)], y)))
        forget = w % 2 == 1
        K = np.zeros(blocked.size, bool)
        memory = []                                        # learning order, for FIFO forgetting
        L = 0.0
        for i, p in enumerate(walk[:-1]):
            for d in product(range(-2, 3), repeat=3):      # learn what a short-range sensor sees (true obstacles)
                y = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
                if inside(y) and blocked[ix(y)] and not K[ix(y)]:
                    K[ix(y)] = True
                    memory.append(ix(y))
            while forget and len(memory) > 12:              # bounded memory: forget the oldest
                K[memory.pop(0)] = False
            field = distance_field(shape, K, ix(b))
            if rng.random() < 0.5:
                L = max(L, field[ix(p)])                   # a completed search
            if not forget:
                assert L <= field[ix(p)] + 1e-9            # F45.2
                checked += 1
            elif L > field[ix(p)] + 1e-9:                  # futile: the believed optimal plan must cross O
                futile += 1
                c, crossed = p, False
                while c != b:
                    c = min((y for y in ((c[0] + s[0], c[1] + s[1], c[2] + s[2]) for s in geom.STEPS)
                             if inside(y) and not K[ix(y)]),
                            key=lambda y: (geom.h3(geom.sub(y, c)) + field[ix(y)], y))
                    crossed |= bool(blocked[ix(c)])
                assert crossed                             # F45.1
            L -= geom.h3(geom.sub(walk[i + 1], p))         # transport along the free walk
    assert futile > 0 and checked > 0
    note("F45.1-2_futility", f"{checked} invariant checks without forgetting, {futile} futile plans all crossing O")


def test_bloom_saturation():
    """Audit: the 32-bit, 2-bit eviction Bloom filter against (1 - (1 - 1/32)^(2n))^2, over the default
    world's tile keys."""
    from sim3d.agent import _bloom_bits
    keys = [(x, y, z) for x in range(10) for y in range(10) for z in range(3)]
    rng = random.Random(9)
    rows = {}
    for n in (5, 10, 20, 40, 80):
        fp = []
        for _ in range(200):
            ins = rng.sample(keys, n)
            bloom = 0
            for k in ins:
                bloom |= _bloom_bits(k)
            rest = [k for k in keys if k not in set(ins)]
            fp.append(np.mean([bloom & _bloom_bits(k) == _bloom_bits(k) for k in rest]))
        rows[n] = (round(float(np.mean(fp)), 3), round((1 - (1 - 1 / 32) ** (2 * n)) ** 2, 3))
        assert abs(rows[n][0] - rows[n][1]) < 0.12
    note("audit_bloom_fp(measured, analytic)", rows)


# ---------------------------------------------------------------- T2.11 memory

def test_memory_layout():
    layout = {"tiles": 12 * 10, "plan": 8 * 2, "bounds": 4 * 2, "pose_seq_flags_rng_step": 2 + 1 + 1 + 1 + 2 + 1,
              "bloom": 4, "neighbours": 9 * (2 + 1 + 1), "walk_record": 12 * 2, "send_age": 12}
    total = sum(layout.values())
    assert total == 228 <= 256                  # 255 until F4.5 removed the unread link counters (9 x 3 B)
    note("T2.11_persistent_bytes", total)


if __name__ == "__main__":
    failed = []
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
            except AssertionError as e:
                failed.append(f"{name}: {e}"[:300])
    REPORT["failed"] = failed
    print(json.dumps(REPORT, indent=1, default=str))
    sys.exit(1 if failed else 0)
