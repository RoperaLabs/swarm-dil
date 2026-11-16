import xml.etree.ElementTree as ET

import numpy as np
import pytest

from swarm_dil import STRATEGIES, Channel, Scenario, run
from swarm_dil.cot import ORIGIN, UAV, event


def drops(ch, n):
    return np.array([~ch.step() for _ in range(n)])


def test_iid_channel_hits_target_loss():
    lost = drops(Channel(200, 0.3, None, np.random.default_rng(0)), 500)
    assert abs(lost.mean() - 0.3) < 0.01


def test_bursty_channel_hits_target_loss_and_burst_length():
    lost = drops(Channel(200, 0.4, 10, np.random.default_rng(0)), 2000)
    assert abs(lost.mean() - 0.4) < 0.02
    x = lost.T.astype(int)
    starts = (np.diff(x, axis=1) == 1).sum()  # good -> bad transitions
    assert abs(x.sum() / starts - 10) < 1.0


def test_unreachable_loss_rejected():
    with pytest.raises(ValueError):
        Channel(5, 0.9, 2, np.random.default_rng(0))


@pytest.mark.parametrize("strategy", STRATEGIES)
def test_perfect_links_find_everything_reproducibly(strategy):
    a, b = run(strategy, Scenario(), seed=3), run(strategy, Scenario(), seed=3)
    assert a.time_all_found < 400  # lower bound is 1600 cells * 2.5 s / 20 agents = 200 s
    np.testing.assert_array_equal(a.found, b.found)


def test_map_sharing_speeds_up_decentralised_search():
    shared = run("decentralised", Scenario(), seed=1)
    deaf = run("decentralised", Scenario(comm_range=0.0), seed=1)
    assert shared.coverage[150] > deaf.coverage[150] + 0.1


def test_centralised_stalls_when_c2_link_is_cut():
    good = run("centralised", Scenario(), seed=1)
    cut = run("centralised", Scenario(loss=0.95), seed=1)
    assert cut.coverage[300] < good.coverage[300] - 0.5


def test_partition_leaves_dead_strips_unsearched_for_a_while():
    sc = Scenario(attrition=0.5, fail_window=1.0, t_max=200)
    assert run("partition", sc, seed=0).coverage[-1] < run("decentralised", sc, seed=0).coverage[-1]


def test_cot_event_is_valid_and_placed_at_origin():
    ev = ET.fromstring(event("uav-01", UAV, 0.0, 0.0, "UAV-01"))
    assert ev.get("type") == UAV and ev.get("uid") == "uav-01"
    pt = ev.find("point")
    assert float(pt.get("lat")) == pytest.approx(ORIGIN[0])
    assert float(pt.get("lon")) == pytest.approx(ORIGIN[1])
