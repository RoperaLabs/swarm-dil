"""v0.3 navigation study: wire format, agent encapsulation and every closed-form result."""
import ast
import itertools
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from swarm_dil.nav import theory, wire
from swarm_dil.nav.agent import DIRS, TS, Agent, Spec, distance_field
from swarm_dil.nav.world import Mission, make_terrain, run

SMALL = Mission(n=32, team=4, launch_every=8, n_blocks=6, n_traps=2, t_max=400)


def descend(f, n, start, goal):
    x, y = start
    path, arc = [(x, y, 0.0)], 0.0
    while (x, y) != goal:
        best, nxt = np.inf, None
        for dx, dy in DIRS:
            tx, ty = x + dx, y + dy
            if 0 <= tx < n and 0 <= ty < n and f[tx * n + ty] < f[x * n + y]:
                c = (theory.SQRT2 if dx and dy else 1.0) + f[tx * n + ty]
                if c < best:
                    best, nxt = c, (tx, ty)
        arc += best - f[nxt[0] * n + nxt[1]]
        x, y = nxt
        path.append((x, y, arc))
    return path


def test_wire_round_trip():
    tiles = [((1, 2), 0xBEEF), ((11, 0), 1)]
    pkt = wire.encode(3, 300, (7, 9), tiles=tiles)
    assert len(pkt) == wire.HEADER + 2 * wire.TILE
    assert wire.decode(pkt) == (3, 300 % 256, (7, 9), tiles)
    assert wire.decode(wire.encode(0, 1, (0, 0), cells=[(4, 5)]), raw=True)[3] == [(4, 5)]


def test_agent_code_cannot_reach_the_world():
    src = Path(__file__).parents[1] / "src" / "swarm_dil" / "nav" / "agent.py"
    imported = {a.name for node in ast.walk(ast.parse(src.read_text())) if isinstance(node, ast.ImportFrom)
                for a in node.names} | {node.module or "" for node in ast.walk(ast.parse(src.read_text()))
                                       if isinstance(node, ast.ImportFrom)}
    assert not imported & {"world", "radio", "Links", "Mission", "sim"}


def test_gilbert_elliott_all_lost_matches_monte_carlo():
    p, q, gaps = 0.05, 0.25, (1, 3, 6)
    pi_b, lam = p / (p + q), 1 - p - q
    rng = np.random.default_rng(0)
    T = 400_000
    bad = np.empty(T, bool)
    bad[0] = rng.random() < pi_b
    u = rng.random(T)
    for t in range(1, T):
        bad[t] = u[t] >= q if bad[t - 1] else u[t] < p
    offs = np.cumsum((0,) + gaps)
    idx = np.arange(T - offs[-1])
    mc = np.all([bad[idx + o] for o in offs], axis=0).mean()
    assert mc == pytest.approx(theory.p_all_lost(gaps, pi_b, lam), abs=0.004)
    good = ~bad[idx]
    assert bad[idx + 4][good].mean() == pytest.approx(theory.p_bad_given_good(4, pi_b, lam), abs=0.005)


def test_k_star_matches_brute_force():
    for v, c, pi_b, rho in itertools.product((5.0, 50.0, 500.0), (1.0, 4.0), (0.2, 0.6), (None, 0.8)):
        r = pi_b if rho is None else rho
        worth = [v * (1 - pi_b)] + [v * pi_b * r ** (m - 2) * (1 - r) for m in range(2, 200)]
        assert theory.k_star(v, c, pi_b, rho) == sum(w >= c for w in worth)


def test_expected_tiles_is_exact_and_s4_is_optimal_for_curves():
    for w, h, s in itertools.product((1, 3, 7), (1, 5), (2, 4)):
        brute = np.mean([np.ceil((ox + w) / s) * np.ceil((oy + h) / s) for ox in range(s) for oy in range(s)])
        assert theory.expected_tiles(w, h, s) == pytest.approx(brute)
    assert theory.best_tile_side_for_curves(2) == pytest.approx(TS)


@pytest.mark.parametrize("seed", range(5))
def test_ellipse_lemma_positive_margin_never_on_plan(seed):
    rng = np.random.default_rng(seed)
    n = 32
    blocked, _, start, goal, _ = make_terrain(n, rng, 8, 2)
    known = blocked.ravel() & (rng.random(n * n) < 0.5)
    f = distance_field(n, known, goal)
    on_plan = {(x, y) for x, y, _ in descend(f, n, start, goal)}
    d0 = f[start[0] * n + start[1]]
    for x, y in zip(*np.nonzero(blocked)):
        margin = theory.octile(x - start[0], y - start[1]) + theory.octile(goal[0] - x, goal[1] - y) - d0
        if margin > 1e-9:
            assert (x, y) not in on_plan


@pytest.mark.parametrize("seed", range(5))
def test_voi_bound_holds(seed):
    """Ignorance cost W = l + d'(q) - d'(x) of an obstacle met after arc l is in [0, 2 l]."""
    rng = np.random.default_rng(seed)
    n = 32
    blocked, _, start, goal, _ = make_terrain(n, rng, 8, 2)
    f = distance_field(n, np.zeros(n * n, bool), goal)
    path = descend(f, n, start, goal)
    hits = [(x, y, arc) for x, y, arc in path if blocked[x, y]]
    if not hits:
        pytest.skip("straight line is clear")
    x, y, _ = hits[0]
    tile = np.zeros((n, n), bool)
    tile[x // TS * TS:(x // TS + 1) * TS, y // TS * TS:(y // TS + 1) * TS] = True
    f2 = distance_field(n, (blocked & tile).ravel(), goal)
    for qx, qy, ell in path:
        if (blocked & tile)[qx, qy]:
            break
        W = ell + f2[qx * n + qy] - f2[start[0] * n + start[1]]
        assert -1e-9 <= W <= 2 * ell + 1e-9


def test_memory_bound_is_respected():
    a = Agent(0, Spec(n=32, start=(2, 2), goal=(29, 29), team=2, launch_at=0, memory_tiles=3), np.random.default_rng(0))
    for k in range(6):
        a.observe([(4 * k, 1, True), (4 * k, 2, True)])
        assert len(a.tiles) <= 3


def test_everyone_arrives_without_communication():
    for seed in range(4):
        assert run(replace(SMALL, protocol="none"), seed).arrival_rate == 1.0


def test_grace_beats_silence_with_few_bytes():
    regret = {p: [] for p in ("none", "grace")}
    sent = []
    for seed in range(6):
        for p in regret:
            o = run(replace(SMALL, protocol=p, price=0.2), seed)
            regret[p].append(o.regret)
            if p == "grace":
                sent.append(o.bytes_sent.sum())
    assert np.mean(regret["grace"]) < np.mean(regret["none"])
    assert np.mean(sent) < 10_000
