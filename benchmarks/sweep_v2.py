"""Paired sweeps for the two main planes of RL-TR-2026-04. Every point runs the same 20 terrains, links and sensors.

  front   (kB on the radio per mission, regret): the price κ of GRACE-3D 2.0.0, GRACE ported to 3-D and CAIRN v0.4, the
          packet budget of flooding; silence is one point.
  memory  (persistent bytes, arrival rate within 500 slots): the tiles K of GRACE-3D 2.0.0, whose whole state is
          11 K + 96 B; the map bytes of the v0.4 agents (only their map is bounded).

    python benchmarks/sweep_v2.py      # resumable; rows in benchmarks/v2_sweep_runs.csv, summary in v2_sweep.json
"""
import csv
import json
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from sim3d.world import Mission, run  # noqa: E402

OUT = ROOT / "benchmarks"
SEEDS = list(range(20))
WORKERS = 4                                  # one process left for experiments/v2_media.py
NAMES = {"v2": "GRACE-3D 2.0.0", "grace": "GRACE ported to 3-D", "cairn": "CAIRN v0.4", "flood": "Flooding",
         "none": "Silence"}
BASE = {"v2": dict(price=0.5, K=12), "grace": dict(price=2), "cairn": dict(price=2), "flood": dict(budget=36),
        "none": {}}                          # the bench defaults, so shared points run once


def cfg(protocol, **kw):
    over = dict(protocol=protocol, **BASE[protocol])
    if protocol != "v2":
        over["memory_bytes"] = 120
    over.update(kw)
    return over


POINTS = ([("front", "v2", "price", k, cfg("v2", price=k)) for k in (0.25, 0.5, 1, 2, 4)]
          + [("front", p, "price", k, cfg(p, price=k)) for p in ("grace", "cairn") for k in (0.5, 2, 10)]
          + [("front", "flood", "budget", b, cfg("flood", budget=b)) for b in (16, 36, 66)]
          + [("front", "none", "-", 0, cfg("none"))]
          + [("memory", "v2", "K", k, cfg("v2", K=k)) for k in (3, 6, 9, 12, 14)]
          + [("memory", p, "map_bytes", m, cfg(p, memory_bytes=m)) for p in ("none", "grace", "cairn", "flood")
             for m in (60, 120, 240)])
KEY = lambda over: json.dumps(over, sort_keys=True)  # noqa: E731
FIELDS = ["key", "seed", "regret", "arrival", "bytes", "memory_peak"]


def one(job):
    key, seed = job
    o = run(replace(Mission(), **json.loads(key)), seed)
    return dict(key=key, seed=seed, regret=o.regret, arrival=o.arrival_rate, bytes=int(o.bytes_sent.sum()),
                memory_peak=o.memory_peak)


def sweep():
    path = OUT / "v2_sweep_runs.csv"
    done = set()
    if path.exists():
        with path.open(newline="") as f:
            done = {(r["key"], int(r["seed"])) for r in csv.DictReader(f)}
    keys = list(dict.fromkeys(KEY(p[4]) for p in POINTS))
    jobs = [(k, s) for k in keys for s in SEEDS if (k, s) not in done]
    jobs.sort(key=lambda j: '"flood"' not in j[0] and '"none"' not in j[0])    # 500-slot missions first
    print(f"{len(jobs)} missions to run", file=sys.stderr, flush=True)
    if jobs:
        with path.open("a", newline="") as f, Pool(WORKERS) as pool:
            w = csv.DictWriter(f, FIELDS)
            if not done:
                w.writeheader()
            for n, row in enumerate(pool.imap_unordered(one, jobs), 1):
                w.writerow(row)
                f.flush()
                if n % 40 == 0:
                    print(f"  {n}/{len(jobs)}", file=sys.stderr, flush=True)


def ms(x):
    x = np.asarray(x, float)
    return [round(float(x.mean()), 4), round(float(x.std(ddof=1) / np.sqrt(len(x))), 4)]


def summarise():
    with (OUT / "v2_sweep_runs.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))
    by = {}
    for r in rows:
        by.setdefault(r["key"], {})[int(r["seed"])] = r
    out = []
    for study, proto, param, value, over in POINTS:
        rs = [by[KEY(over)][s] for s in SEEDS]
        col = lambda k: [float(r[k]) for r in rs]  # noqa: E731
        out.append(dict(study=study, protocol=NAMES[proto], param=param, value=value,
                        kB=ms(np.array(col("bytes")) / 1000), regret=ms(col("regret")), arrival=ms(col("arrival")),
                        memory_bytes=int(max(col("memory_peak"))) if proto == "v2" else over["memory_bytes"]))
    (OUT / "v2_sweep.json").write_text(json.dumps(dict(seeds=SEEDS, points=out), indent=1), encoding="utf-8")
    print(json.dumps([[p["study"], p["protocol"], p["value"], p["kB"][0], p["regret"][0], p["arrival"][0]]
                      for p in out]))


if __name__ == "__main__":
    sweep()
    summarise()
