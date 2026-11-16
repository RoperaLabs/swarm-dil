"""Replay a run as Cursor on Target over UDP so a TAK client on the LAN shows it live.

    python experiments/stream_cot.py --strategy decentralised --speedup 10
    python experiments/stream_cot.py --host 127.0.0.1 --port 4242   # unicast to one client

Defaults to the TAK situational-awareness multicast group, which WinTAK and ATAK listen on.
"""
import argparse
import socket
import time

import numpy as np

from swarm_dil import STRATEGIES, Scenario, run
from swarm_dil.cot import LOCATED, TAK_MULTICAST, UAV, event

ap = argparse.ArgumentParser()
ap.add_argument("--strategy", choices=STRATEGIES, default="decentralised")
ap.add_argument("--loss", type=float, default=0.5)
ap.add_argument("--burst", type=float, default=10)
ap.add_argument("--attrition", type=float, default=0.3)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--speedup", type=float, default=10.0)
ap.add_argument("--host", default=TAK_MULTICAST[0])
ap.add_argument("--port", type=int, default=TAK_MULTICAST[1])
a = ap.parse_args()

sc = Scenario(loss=a.loss, burst=a.burst, attrition=a.attrition)
r = run(a.strategy, sc, a.seed)
end = int(min(r.time_all_found, sc.t_max - 1)) + 1
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)  # stay on the local network

print(f"{a.strategy}: streaming {end} s of mission at {a.speedup}x to {a.host}:{a.port}")
for t in range(end):
    for i in np.flatnonzero(r.alive[t]):
        x, y = r.trace[t, i]
        sock.sendto(event(f"swarm-dil-uav-{i:02d}", UAV, x, y, f"UAV-{i:02d}"), (a.host, a.port))
    for k in np.flatnonzero(r.found <= t):
        x, y = r.targets_xy[k]
        sock.sendto(event(f"swarm-dil-target-{k:02d}", LOCATED, x, y, f"FOUND-{k:02d}", stale_s=600),
                    (a.host, a.port))
    time.sleep(1.0 / a.speedup)
sock.close()
