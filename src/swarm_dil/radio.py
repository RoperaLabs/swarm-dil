"""Physical-layer link model: path loss, correlated shadowing, line-of-sight blockage,
field interference and channel contention.

Delivery probability of a message from node j to node i:
    rx  = tx + G_i + G_j - PL(d) - obstacle_db * (#obstacles crossed) + shadowing
    PER = logistic(SINR) with SINR = rx - (noise + interference at i)
    p   = (1 - PER) * contention(i)
Shadowing follows an AR(1) process per link, so outages last several seconds instead of
being independent coin flips. Air-to-air links at low altitude are close to free space;
the default exponent 2.7 adds ground reflection and vegetation.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Radio:
    tx_dbm: float = 10.0           # 10 mW, lightweight mesh radio
    path_loss_exp: float = 2.7
    pl_1m_db: float = 40.0         # free-space loss at 1 m, 2.4 GHz
    noise_dbm: float = -110.0      # thermal noise + receiver noise figure in the channel
    shadow_sigma_db: float = 4.0
    shadow_corr: float = 0.9       # per-second correlation of shadowing (mean outage ~ seconds)
    sinr50_db: float = 6.0         # SINR at which half the packets are lost
    sinr_width_db: float = 1.5     # steepness of the packet-error curve
    obstacle_db: float = 20.0      # extra loss per obstacle crossing the line of sight
    gs_gain_db: float = 12.0       # ground-station antenna gain over a UAV antenna
    rate_bps: float = 250_000.0    # shared channel data rate


def path_loss_db(radio, d):
    return radio.pl_1m_db + 10 * radio.path_loss_exp * np.log10(np.maximum(d, 1.0))


def blocked_count(a, b, rects):
    """Number of axis-aligned rectangles (x0, y0, x1, y1) crossed by each segment a -> b."""
    if len(rects) == 0:
        return np.zeros(np.broadcast_shapes(a.shape, b.shape)[:-1])
    a = a[..., None, :]
    d = b[..., None, :] - a
    lo, hi = rects[:, :2], rects[:, 2:]
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - a) / d, (hi - a) / d
    inside = (a >= lo) & (a <= hi)
    tmin = np.where(d == 0, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
    tmax = np.where(d == 0, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
    enter, leave = tmin.max(-1), tmax.min(-1)
    return ((enter <= leave) & (leave >= 0) & (enter <= 1)).sum(-1)


class Links:
    """Per-second delivery probabilities between all nodes (UAVs and ground station)."""

    def __init__(self, radio, gain_db, obstacles, interferers, rng):
        self.r, self.rng = radio, rng
        self.gain = np.asarray(gain_db, float)
        self.rects = np.asarray(obstacles, float).reshape(-1, 4)
        self.intf = np.asarray(interferers, float).reshape(-1, 3)
        self.shadow = self._symmetric_noise()

    def _symmetric_noise(self):
        s = np.triu(self.rng.normal(0, self.r.shadow_sigma_db, (self.gain.size,) * 2), 1)
        return s + s.T

    def step(self, pos, active):
        """pos: (nodes, 2) m; active: (nodes,) bool. Returns (nodes, nodes) p[i, j] = P(j -> i)."""
        r = self.r
        rho = r.shadow_corr
        self.shadow = rho * self.shadow + np.sqrt(1 - rho**2) * self._symmetric_noise()
        d = np.linalg.norm(pos[:, None] - pos[None], axis=2)
        rx = (r.tx_dbm + self.gain[:, None] + self.gain[None] - path_loss_db(r, d)
              - r.obstacle_db * blocked_count(pos[:, None], pos[None], self.rects) + self.shadow)
        floor_mw = np.full(len(pos), 10 ** (r.noise_dbm / 10))
        if len(self.intf):
            src = self.intf[:, :2]
            di = np.linalg.norm(pos[:, None] - src[None], axis=2)
            i_dbm = (self.intf[None, :, 2] + self.gain[:, None] - path_loss_db(r, di)
                     - r.obstacle_db * blocked_count(pos[:, None], src[None], self.rects))
            floor_mw += (10 ** (i_dbm / 10)).sum(1)
        sinr = rx - 10 * np.log10(floor_mw)[:, None]
        p = 1 / (1 + np.exp(-(sinr - r.sinr50_db) / r.sinr_width_db))
        np.fill_diagonal(p, 0.0)
        p[~active] = 0.0
        p[:, ~active] = 0.0
        return p


def contention(p, airtime, audible=0.05):
    """Fraction of airtime a receiver gets: 1 while the channel it hears is not saturated.

    load_i = own airtime (half duplex) + airtime of every sender it can hear.
    """
    load = airtime + ((p > audible) * airtime[None]).sum(1)
    return np.minimum(1.0, 1.0 / np.maximum(load, 1e-12))
