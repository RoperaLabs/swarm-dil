"""Byte layout of GRACE messages. Every field is an unsigned byte; grids are at most 255 cells.

    header  [sender][seq][x][y]                    4 B
    tile    [tx][ty][mask_hi][mask_lo]             4 B  (4 x 4 cells, bit = blocked)
    cell    [x][y]                                 2 B  (raw baseline only)
"""
import struct

HEADER = 4
TILE = 4
CELL = 2
_H = struct.Struct("BBBB")
_T = struct.Struct(">BBH")
_C = struct.Struct("BB")


def encode(sender, seq, pos, tiles=(), cells=()):
    out = bytearray(_H.pack(sender, seq % 256, *pos))
    for (tx, ty), mask in tiles:
        out += _T.pack(tx, ty, mask)
    for x, y in cells:
        out += _C.pack(x, y)
    return bytes(out)


def decode(packet, raw=False):
    """Returns (sender, seq, (x, y), items); items are ((tx, ty), mask) or (x, y) if raw."""
    sender, seq, x, y = _H.unpack_from(packet, 0)
    body = packet[HEADER:]
    if raw:
        items = [_C.unpack_from(body, k) for k in range(0, len(body), CELL)]
    else:
        items = [((tx, ty), m) for tx, ty, m in (_T.unpack_from(body, k) for k in range(0, len(body), TILE))]
    return sender, seq, (x, y), items
