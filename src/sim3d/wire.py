"""GRACE-3D 2.0.0 wire format, bit-exact (docs/v2_spec.md §4). Big-endian. No global coordinates:
the header pose is wrapped modulo 32 per axis; tile keys and waypoints are in the swarm frame."""
import struct
from dataclasses import dataclass, field

LOST, ARRIVED, COMMITTED = 4, 2, 1
ADVICE, TEAM, VERIFIED, RELAY, PLAN, PLAN_X = range(1, 7)


@dataclass
class Claim:
    target: int            # 0..31
    kind: int              # ADVICE..PLAN_X
    value: int             # 1/8 voxel, 0..65535
    waypoints: tuple = ()  # up to 3 (x, y, z), each 0..255


@dataclass
class Packet:
    sender: int            # 0..31
    seq: int
    pose: tuple            # (x, y, z) in the swarm frame; sent modulo 32
    flags: int = 0
    digest: int | None = None
    claims: list = field(default_factory=list)
    tiles: list = field(default_factory=list)    # ((tx, ty, tz) each 0..31, 64-bit mask)


def size(p):
    return 5 + 4 * (p.digest is not None) + sum(4 + 3 * len(c.waypoints) for c in p.claims) + 10 * len(p.tiles)


def encode(p):
    if not (0 <= p.sender < 32 and 0 <= p.flags < 8 and len(p.claims) < 16 and len(p.tiles) < 16):
        raise ValueError("header field out of range")
    x, y, z = (c % 32 for c in p.pose)
    out = bytearray(struct.pack(">BBHB", p.sender << 3 | p.flags, p.seq % 256,
                                x << 11 | y << 6 | z << 1 | (p.digest is not None), len(p.tiles) << 4 | len(p.claims)))
    if p.digest is not None:
        out += struct.pack(">I", p.digest)
    for c in p.claims:
        if not (0 <= c.target < 32 and ADVICE <= c.kind <= PLAN_X and 0 <= c.value < 65536 and len(c.waypoints) <= 3):
            raise ValueError("claim field out of range")
        out += struct.pack(">I", c.target << 27 | c.kind << 24 | len(c.waypoints) << 16 | c.value)
        for w in c.waypoints:
            out += bytes(w)
    for (tx, ty, tz), mask in p.tiles:
        if not (0 <= tx < 32 and 0 <= ty < 32 and 0 <= tz < 32 and 0 <= mask < 1 << 64):
            raise ValueError("tile field out of range")
        out += struct.pack(">HQ", tx << 11 | ty << 6 | tz << 1, mask)
    return bytes(out)


def decode(b):
    """Inverse of encode; the pose comes back wrapped (0..31 per axis), see unwrap. Raises
    ValueError on malformed input (bytes from the radio are untrusted)."""
    try:
        h0, seq, pose, counts = struct.unpack_from(">BBHB", b, 0)
        k = 5
        digest = None
        if pose & 1:
            digest, = struct.unpack_from(">I", b, k)
            k += 4
        claims = []
        for _ in range(counts & 15):
            word, = struct.unpack_from(">I", b, k)
            n, kind = word >> 16 & 255, word >> 24 & 7
            if n > 3 or not ADVICE <= kind <= PLAN_X:
                raise ValueError("bad claim")
            k += 4
            claims.append(Claim(word >> 27, kind, word & 65535, tuple(tuple(b[k + 3 * i:k + 3 * i + 3]) for i in range(n))))
            k += 3 * n
            if k > len(b):
                raise ValueError("truncated")
        tiles = []
        for _ in range(counts >> 4):
            key, mask = struct.unpack_from(">HQ", b, k)
            k += 10
            tiles.append(((key >> 11, key >> 6 & 31, key >> 1 & 31), mask))
    except struct.error as e:
        raise ValueError("truncated") from e
    if k != len(b):
        raise ValueError("trailing bytes")
    return Packet(h0 >> 3, seq, (pose >> 11, pose >> 6 & 31, pose >> 1 & 31), h0 & 7, digest, claims, tiles)


def unwrap(w, e):
    """Coordinate sent modulo 32, decoded against the estimate e: exact iff -16 <= p - e < 16 (T2.4)."""
    return e + (w - e + 16) % 32 - 16
