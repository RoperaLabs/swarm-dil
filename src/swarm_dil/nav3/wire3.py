"""Wire format of v0.4 packets.

header   sender|0x80 if digest (1), seq (1), x y z (3), [flags lbx vax uax (4)], counts (1)
tile     id (2) + 64-bit mask of a 4x4x4 tile (8)                          = 10 B
box      low corner (3) + extents (3): every voxel inside is blocked       =  6 B
claim    kind<<5|target (1), waypoints n (1), value in 1/8 cell (2), n x (x y z)
"""
import struct

TILE, BOX = 10, 6
ADVICE, TEAM, VERIFIED, RELAY, PLAN, PLAN_X = 1, 2, 3, 4, 5, 6
LOST, ARRIVED, COMMITTED = 1, 2, 4


def header_len(digest):
    return 10 if digest else 6


def claim_len(n_wp):
    return 4 + 3 * n_wp


def encode(sender, seq, pos, digest=None, tiles=(), boxes=(), claims=()):
    out = bytearray(struct.pack("BBBBB", sender | (0x80 if digest else 0), seq & 255, *pos))
    if digest:
        out += struct.pack("BBBB", *digest)
    out.append(len(tiles) << 4 | len(boxes))
    for key, mask in tiles:
        out += struct.pack(">HQ", key, mask)
    for lo, ext in boxes:
        out += struct.pack("BBBBBB", *lo, *ext)
    for kind, target, value, wps in claims:
        out += struct.pack(">BBH", kind << 5 | target, len(wps), value)
        for w in wps:
            out += bytes(w)
    return bytes(out)


def decode(packet):
    """-> sender, seq, pos, digest or None, tiles, boxes, claims."""
    b0, seq, x, y, z = struct.unpack_from("BBBBB", packet)
    at, digest = 5, None
    if b0 & 0x80:
        digest = struct.unpack_from("BBBB", packet, at)
        at += 4
    counts = packet[at]
    at += 1
    tiles, boxes, claims = [], [], []
    for _ in range(counts >> 4):
        tiles.append(struct.unpack_from(">HQ", packet, at))
        at += TILE
    for _ in range(counts & 15):
        v = struct.unpack_from("BBBBBB", packet, at)
        boxes.append((v[:3], v[3:]))
        at += BOX
    while at < len(packet):
        kt, n, value = struct.unpack_from(">BBH", packet, at)
        at += 4
        wps = [tuple(packet[at + 3 * i: at + 3 * i + 3]) for i in range(n)]
        at += 3 * n
        claims.append((kt >> 5, kt & 31, value, wps))
    return b0 & 0x7F, seq, (x, y, z), digest, tiles, boxes, claims
