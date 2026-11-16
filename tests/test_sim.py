import xml.etree.ElementTree as ET

import numpy as np
import pytest

from swarm_dil import STRATEGIES, Links, Radio, Scenario, run
from swarm_dil.cot import ORIGIN, UAV, event
from swarm_dil.radio import blocked_count, contention
from swarm_dil.sim import grid

WALL = ((300.0, 0.0, 350.0, 700.0),)


def delivery(pos, **kw):
    radio = Radio(shadow_sigma_db=0.0, **{k: v for k, v in kw.items() if k in Radio.__dataclass_fields__})
    links = Links(radio, np.zeros(len(pos)), kw.get("obstacles", ()), kw.get("interferers", ()),
                  np.random.default_rng(0))
    return links.step(np.asarray(pos, float), np.ones(len(pos), bool))[1, 0]


def test_delivery_falls_with_distance_and_default_range_is_about_550_m():
    p = [delivery([[0, 0], [d, 0]]) for d in (100, 400, 550, 700, 1000)]
    assert all(a > b for a, b in zip(p, p[1:]))
    assert p[2] == pytest.approx(0.5, abs=0.1)


def test_obstacle_and_interferer_degrade_the_link():
    clear = delivery([[0, 0], [300, 0]])
    assert delivery([[0, 0], [300, 0]], obstacles=((140, -10, 160, 10),)) < clear - 0.5
    assert delivery([[0, 0], [300, 0]], interferers=((300, 20, 0.0),)) < clear - 0.5


def test_blocked_count_segments():
    rects = np.array([[1, -1, 2, 1.0]])
    a = np.array([[0, 0], [0, 5.0]])
    b = np.array([[3, 0], [3, 5.0]])
    np.testing.assert_array_equal(blocked_count(a, b, rects), [1, 0])


def test_contention_only_bites_when_the_channel_saturates():
    p = np.ones((3, 3)) - np.eye(3)
    assert contention(p, np.full(3, 0.1)).min() == 1.0
    assert contention(p, np.full(3, 0.5)).max() < 1.0


def test_geodesic_distance_goes_around_a_wall():
    free, _, _, D = grid(40, 25.0, WALL)
    a, b = 10 * 40 + 5, 20 * 40 + 5  # either side of the wall, 250 m apart
    assert D[a, b] > 1000 and not free[13 * 40 + 5]


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_clear_field_finds_and_reports_everything_reproducibly(strategy):
    a, b = run(strategy, Scenario(), seed=3), run(strategy, Scenario(), seed=3)
    assert a.time_all_reported < 900
    assert np.all(a.reported >= a.found)
    np.testing.assert_array_equal(a.reported, b.reported)


def test_uavs_never_enter_obstacles():
    r = run("decentralised", Scenario(obstacles=WALL, t_max=400), seed=0)
    x, y = r.trace[..., 0], r.trace[..., 1]
    assert not np.any((x > 300) & (x < 350) & (y < 700))


def test_map_sharing_speeds_up_decentralised_search():
    shared = run("decentralised", Scenario(), seed=1)
    deaf = run("decentralised", Scenario(radio=Radio(tx_dbm=-100.0, gs_gain_db=200.0)), seed=1)
    assert shared.coverage[250] > deaf.coverage[250] + 0.1


def test_memory_limit_is_enforced_and_costs_time():
    small = run("decentralised", Scenario(memory_cells=100, t_max=600), seed=1)
    assert small.peak_memory <= 100
    assert small.coverage[300] < run("decentralised", Scenario(), seed=1).coverage[300]


def test_c2_never_learns_anything_when_its_link_is_dead():
    r = run("partition", Scenario(radio=Radio(gs_gain_db=-100.0), t_max=400), seed=0)
    assert np.isfinite(r.found).any() and np.isinf(r.reported).all()


def test_cot_event_is_valid_and_placed_at_origin():
    ev = ET.fromstring(event("uav-01", UAV, 0.0, 0.0, "UAV-01"))
    assert ev.get("type") == UAV and ev.get("uid") == "uav-01"
    pt = ev.find("point")
    assert float(pt.get("lat")) == pytest.approx(ORIGIN[0])
    assert float(pt.get("lon")) == pytest.approx(ORIGIN[1])
