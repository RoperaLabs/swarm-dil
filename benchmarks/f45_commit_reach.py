"""Prop. F45.6 (c), measured: in the missions whose first arrival leaves time for a commit (T1 <= 88), does any packet
from an arrived agent, or any VERIFIED chunk, reach an agent still waiting at A? The agent is observed from outside
(its receive method is wrapped here, not changed).

    python benchmarks/f45_commit_reach.py        # writes benchmarks/v2_f45_commit.json
"""
import json
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from sim3d import agent, wire  # noqa: E402
from sim3d.world import Mission, run  # noqa: E402

SEEDS = (5, 9, 108, 111, 113)            # T1 <= 88 in benchmarks/v2_f45_runs.csv, for F4 and the adopted agent
SEEN = []
_receive = agent.Agent.receive


def _watch(self, data):
    try:
        p = wire.decode(data)
    except ValueError:
        p = None
    if p is not None and self.t < self.s.launch_at and p.flags & wire.ARRIVED:
        SEEN.append(sum(c.kind == wire.VERIFIED for c in p.claims))
    _receive(self, data)


agent.Agent.receive = _watch


def one(job):
    protocol, seed = job
    SEEN.clear()
    o = run(replace(Mission(), protocol=protocol), seed)
    return dict(protocol=protocol, seed=seed, first_arrival=float(o.arrived_at.min()), commits=int(o.committed.sum()),
                verified_sent=o.sent.get("verified", 0), packets_from_arrived_at_A=len(SEEN),
                verified_chunks_at_A=sum(SEEN))


if __name__ == "__main__":
    with Pool(5) as pool:
        rows = pool.map(one, [(p, s) for p in ("v2-nofutile", "v2") for s in SEEDS])
    (ROOT / "benchmarks" / "v2_f45_commit.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print(json.dumps([[r["protocol"], r["seed"], r["packets_from_arrived_at_A"], r["commits"]] for r in rows]))
