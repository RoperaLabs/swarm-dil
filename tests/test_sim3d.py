"""GRACE-3D 2.0.0 simulator and on-board agent (src/sim3d): isolation, planner soundness, bounded search and a
checked mission."""
import ast
import random
import sys
from math import inf
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sim3d import geom  # noqa: E402
from sim3d.world import Mission, run  # noqa: E402

SRC = Path(__file__).resolve().parents[1] / "src" / "sim3d"


def test_agent_is_isolated():
    """Def. 1: the agent module imports only its own wire, geometry and cycle model, and the stdlib."""
    tree = ast.parse((SRC / "agent.py").read_text(encoding="utf-8"))
    mods = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            mods.add("." * node.level + (node.module or ""))
    assert mods <= {"math", "random", "dataclasses", ".", ".geom", ".ops"}, mods


def random_tiles(rng, shape, n):
    tiles = {}
    for _ in range(n):
        c = tuple(rng.randrange(s) for s in shape)
        key, bit = geom.key_bit(c)
        tiles[key] = tiles.get(key, 0) | bit
    return tiles, {k: geom.box_of(k, m) for k, m in tiles.items()}


def test_segment_block_is_exact():
    rng = random.Random(0)
    for _ in range(300):
        tiles, boxes = random_tiles(rng, (16, 16, 8), 60)
        p, q = [tuple(rng.randrange(s) for s in (16, 16, 8)) for _ in range(2)]
        brute = any(geom.blocked_in(tiles, c) for c in geom.segment(p, q))
        assert (geom.segment_block(p, q, tiles, boxes) is not None) == brute


def test_bvp_returns_walks_of_the_believed_graph():
    """T2.5(a): every returned plan avoids the stored obstacles, stays inside, and costs at least d_K."""
    rng = random.Random(1)
    shape = (16, 16, 6)

    def inside(c):
        return all(0 <= c[k] < shape[k] for k in range(3))
    found = 0
    for _ in range(60):
        tiles, boxes = random_tiles(rng, shape, 120)
        a, b = (0, 0, 0), (15, 15, 5)
        if geom.blocked_in(tiles, a) or geom.blocked_in(tiles, b):
            continue
        path = geom.bvp(a, b, tiles, boxes, inside)
        if path is None:
            continue
        found += 1
        assert path[0] == a and path[-1] == b
        cost = 0.0
        for u, v in zip(path, path[1:]):
            cells = geom.segment(u, v)
            assert all(inside(c) and not geom.blocked_in(tiles, c) for c in cells)
            cost += geom.h3(geom.sub(v, u))
        exact = list(geom.astar(lambda c: inside(c) and not geom.blocked_in(tiles, c), a, b, 10**6))[-1]
        assert cost >= exact - 1e-9
    assert found > 30


def test_astar_arena_bound_is_sound():
    """T2.6 with overflow: the value kept on overflow never exceeds the exact cost."""
    rng = random.Random(2)
    shape = (20, 20, 6)

    def inside(c):
        return all(0 <= c[k] < shape[k] for k in range(3))
    for _ in range(30):
        tiles, _ = random_tiles(rng, shape, 300)
        free = lambda c: inside(c) and not geom.blocked_in(tiles, c)  # noqa: E731
        a, b = (0, 0, 0), (19, 19, 5)
        exact = list(geom.astar(free, a, b, 10**6))[-1]
        for arena in (20, 100, 400):
            vals = list(geom.astar(free, a, b, 16, arena))
            assert all(v <= exact + 1e-9 for v in vals) and (vals[-1] == exact or vals[-1] < inf)


def test_checked_mission():
    """A whole mission with every bound, the memory budget and every belief checked against the truth."""
    o = run(Mission(t_max=500), seed=1, check=True)
    assert o.memory_peak <= 256
    assert o.arrival_rate == 1.0
