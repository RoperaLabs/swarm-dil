"""Closed forms behind CAIRN (v0.4). Each function is checked numerically in tests/test_nav3.py.

Numbering follows the technical report (docs/theory/cairn.pdf).
"""
import math

SQRT2, SQRT3 = math.sqrt(2.0), math.sqrt(3.0)


def h3(a, b):
    """26-connected lattice distance between voxels a and b (the 3-D octile metric)."""
    lo, mid, hi = sorted((abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])))
    return hi + (SQRT2 - 1.0) * mid + (SQRT3 - SQRT2) * lo


def lattice_stretch():
    """Worst-case ratio between h3 and the Euclidean distance (Lemma 1): attained at (1, a, b)
    with a = sqrt(2) - 1, b = sqrt(3) - sqrt(2); about 1.128."""
    a, b = SQRT2 - 1.0, SQRT3 - SQRT2
    return math.sqrt(1.0 + a * a + b * b)


def spheroid_volume(u, h):
    """Volume of the informed set {y : |x - y| + |y - B| <= u} with |x - B| = h (Prop. 7):
    a prolate spheroid with semi-axes u/2 and sqrt(u^2 - h^2)/2."""
    return 0.0 if u <= h else math.pi / 6.0 * u * (u * u - h * h)


def spheroid_area_bound(u, h):
    """Upper bound on the area of a plane section of the informed set (largest: through both
    foci): pi/4 * u * sqrt(u^2 - h^2). Bounds the wall area a relevant cut can have."""
    return 0.0 if u <= h else math.pi / 4.0 * u * math.sqrt(u * u - h * h)


def tile_bits_per_cell(s, dim, set_dim, id_bits):
    """Bits per occupied voxel when a set of dimension `set_dim` in a `dim`-dimensional grid is
    coded as tiles of side s (identifier + s^dim occupancy mask), Lemma 5."""
    return (id_bits + s ** dim) / s ** set_dim


def best_tile_side(dim, set_dim, id_bits):
    """Real-valued minimiser of tile_bits_per_cell (Cor. 2): s* = (d b / (D - d))^(1/D)."""
    return (set_dim * id_bits / (dim - set_dim)) ** (1.0 / dim)


def certificate_bits_lower(area, layers):
    """Bits any sound voxel-level certificate needs on the layered-wall family (Thm. 3):
    one blocked layer out of `layers` per wall column inside the informed set."""
    return area * math.log2(layers)


def claim_bytes(waypoints):
    """Wire size of a route claim with that many waypoints."""
    return 4 + 3 * waypoints


def loss_after_hearing(tau, pi_b, lam):
    """P(a copy sent tau slots after hearing the peer is lost) on a Gilbert-Elliott link with
    reciprocity (Prop. 9): pi_B (1 - lam^tau), increasing in tau."""
    return pi_b * (1.0 - lam ** tau)


def expected_copies_reply(pi_b, lam):
    """Expected copies until delivery when each copy is sent right after hearing the peer."""
    return 1.0 / (1.0 - loss_after_hearing(1, pi_b, lam))


def certified_regret(theta):
    """Largest relative regret of an agent that commits to a verified route when
    U <= (1 + theta) L (Thm. 2)."""
    return theta
