"""Closed-form results behind the GRACE protocol (derivations in docs/theory/theory.pdf).

Shared by the on-board agent code, the tests and the figures, so the simulation runs
exactly the mathematics the document states.
"""
import math

SQRT2 = math.sqrt(2.0)


def octile(dx, dy):
    """Shortest 8-connected distance on an empty grid (orthogonal 1, diagonal sqrt 2)."""
    dx, dy = abs(dx), abs(dy)
    return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)


# --- Gilbert-Elliott link: state G delivers, state B loses ------------------------------
# p = P(G -> B), q = P(B -> G) per slot; pi_B = p / (p + q); lam = 1 - p - q.

def p_bad_given_bad(tau, pi_b, lam):
    """P(B at t + tau | B at t) = pi_B + pi_G lam^tau  (Prop. 4)."""
    return pi_b + (1.0 - pi_b) * lam ** tau


def p_bad_given_good(tau, pi_b, lam):
    """P(B at t + tau | G at t) = pi_B (1 - lam^tau)  (Prop. 4)."""
    return pi_b * (1.0 - lam ** tau)


def p_all_lost(gaps, pi_b, lam):
    """P(k copies all lost) for copies separated by `gaps` slots (Prop. 5)."""
    out = pi_b
    for g in gaps:
        out *= p_bad_given_bad(g, pi_b, lam)
    return out


def spacing_for(eps, pi_b, lam):
    """Smallest gap tau with P(B|B, tau) <= (1 + eps) pi_B: copies become almost independent."""
    if lam <= 0.0 or pi_b >= 1.0:
        return 1
    return max(1, math.ceil(math.log(eps * pi_b / (1.0 - pi_b)) / math.log(lam)))


def k_star(value, cost, pi_b, rho=None):
    """Myopically optimal number of copies of one item (Prop. 6).

    The first copy is worth value (1 - pi_B); copy m >= 2 is worth value pi_B rho^(m-2)
    (1 - rho), with rho = P(lost | previous copy lost) (rho = pi_B for independent losses).
    Copies are sent while their worth is at least `cost`.
    """
    rho = pi_b if rho is None else rho
    if value * (1.0 - pi_b) < cost:
        return 0
    if pi_b <= 0.0 or rho <= 0.0:
        return 1
    extra = math.log(cost / (value * pi_b * (1.0 - rho))) / math.log(rho)
    return 1 + max(0, math.floor(extra) + 1)


# --- Value of information for a shared goal ---------------------------------------------

def voi_bound(s, r):
    """Upper bound on the detour an obstacle report saves (Prop. 1): 2 (s - r)^+, where s is
    the path distance to the obstacle and r the receiver's sensor range."""
    return 2.0 * max(0.0, s - r)


# --- Geometry of the code: tiles ----------------------------------------------------------

def expected_tiles(w, h, s):
    """Expected number of s x s tiles touched by a w x h block of cells at a uniform random
    offset (exact for integer w, h; Lemma 4)."""
    return ((w + s - 1) / s) * ((h + s - 1) / s)


def tile_bytes(w, h, s, id_bytes=2):
    """Expected bytes to send a w x h blocked block as tiles (id + s^2-bit mask)."""
    return expected_tiles(w, h, s) * (id_bytes + s * s / 8.0)


def best_tile_side_for_curves(id_bytes=2):
    """Tile side minimising bytes per cell for 1-cell-thick sets (what a range sensor sees of
    an obstacle): minimise (id + s^2/8) / s  ->  s* = sqrt(8 id) (Cor. 3)."""
    return math.sqrt(8.0 * id_bytes)
