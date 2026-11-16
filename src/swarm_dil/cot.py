"""Cursor on Target (CoT) events, so any TAK client (ATAK, WinTAK, iTAK) can show a run."""
import math
import xml.etree.ElementTree as ET
from datetime import datetime, timedelta, timezone

ORIGIN = (46.50, 7.50)    # lat, lon of the search area's south-west corner (synthetic)
UAV = "a-f-A-M-F-Q"       # friendly / air / military / fixed wing / UAV
LOCATED = "a-n-G"         # neutral / ground: a located person in the SAR scenario
TAK_MULTICAST = ("239.2.3.1", 6969)  # default TAK situational-awareness multicast group

_M_PER_DEG = 111_320.0


def to_latlon(x, y, origin=ORIGIN):
    """Local metres east/north of the origin to WGS84 (equirectangular, fine at km scale)."""
    lat0, lon0 = origin
    return lat0 + y / _M_PER_DEG, lon0 + x / (_M_PER_DEG * math.cos(math.radians(lat0)))


def event(uid, cot_type, x, y, callsign, now=None, stale_s=30, origin=ORIGIN):
    """One CoT 2.0 event as UTF-8 XML bytes."""
    now = now or datetime.now(timezone.utc)
    def iso(d):
        return d.astimezone(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

    t, stale = iso(now), iso(now + timedelta(seconds=stale_s))
    lat, lon = to_latlon(x, y, origin)
    ev = ET.Element("event", version="2.0", uid=uid, type=cot_type, how="m-g",
                    time=t, start=t, stale=stale)
    ET.SubElement(ev, "point", lat=f"{lat:.7f}", lon=f"{lon:.7f}", hae="0", ce="10", le="10")
    ET.SubElement(ET.SubElement(ev, "detail"), "contact", callsign=callsign)
    return ET.tostring(ev, encoding="utf-8")
