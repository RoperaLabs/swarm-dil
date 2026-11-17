"""Gate F3/F4 benchmark of GRACE-3D 2.0.0 (docs/v2_spec.md §7), paired over seeds: every configuration runs on the
same terrains.

    python benchmarks/eval_v2.py              # 20 seeds, resumable; prints a short JSON summary
    python benchmarks/eval_v2.py --seeds=5    # quicker

Per-mission rows go to benchmarks/v2_runs.csv; aggregates and constraint status to benchmarks/v2_results.json.
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
from sim3d.ops import BUDGET  # noqa: E402
from sim3d.world import Mission, run  # noqa: E402

OUT = ROOT / "benchmarks"
WORKERS = 5                     # 5 of 8 hardware threads: with the laptop's own load, under the 85 % ceiling
ETA_2D = (0.013, 0.026)         # v1 information efficiency, regret removed per kB (docs/v1_baseline.md)
CONFIGS = {                     # label: Mission overrides. v0.4 baselines get the same 120 B of map memory.
    "none@120": dict(protocol="none", memory_bytes=120),
    "flood@120": dict(protocol="flood", budget=36, memory_bytes=120),
    "grace@120": dict(protocol="grace", price=2, memory_bytes=120),
    "cairn@120": dict(protocol="cairn", price=2, memory_bytes=120),
    "none@unbounded": dict(protocol="none"),
    "v2-silent": dict(protocol="v2-silent"),
    "v2": dict(protocol="v2"),
    "v2-noescape": dict(protocol="v2-noescape"),
    "v2-fifo": dict(protocol="v2-fifo"),
    "v2-bvp": dict(protocol="v2-bvp"),
    "v2-nofutile": dict(protocol="v2-nofutile"),          # the F4 agent (no futility trigger, F4.5)
    "v2@K6": dict(protocol="v2", K=6),
    "v2@T20000": dict(protocol="v2", t_max=20000),        # liveness: does every agent arrive eventually?
    "v2@K6,T5000": dict(protocol="v2", K=6, t_max=5000),   # stress test below the obstacle-certificate size
}
FIELDS = ["label", "seed", "regret", "arrival", "bytes", "memory_peak", "cycles_p99", "cycles_max", "slots_over",
          "slots", "ingest_p99", "escapes", "plans", "commits", "fit", "arrived", "tiles", "voxels"]


def one(job):
    label, seed = job
    o = run(replace(Mission(), **CONFIGS[label]), seed)
    c = o.cycles if len(o.cycles) else np.zeros(1)
    return dict(label=label, seed=seed, regret=o.regret, arrival=o.arrival_rate, bytes=int(o.bytes_sent.sum()),
                memory_peak=o.memory_peak, cycles_p99=float(np.percentile(c, 99)), cycles_max=float(c.max()),
                slots_over=int((o.cycles > BUDGET).sum()), slots=len(o.cycles),
                ingest_p99=float(np.percentile(o.ingest_cycles, 99)) if len(o.ingest_cycles) else 0.0,
                escapes=int(o.escapes.sum()), plans=float(o.plans.mean()), commits=int(o.committed.sum()),
                fit=sum(o.fit), arrived=len(o.fit), tiles=o.sent.get("tile", 0), voxels=o.sent.get("voxels", 0))


def sweep(seeds):
    path = OUT / "v2_runs.csv"
    done = set()
    if path.exists():
        with path.open() as f:
            done = {(r["label"], int(r["seed"])) for r in csv.DictReader(f)}
    jobs = [(label, s) for label in CONFIGS for s in seeds if (label, s) not in done]
    jobs.sort(key=lambda j: -CONFIGS[j[0]].get("t_max", 500))     # long missions first: less idle tail
    print(f"{len(jobs)} missions to run", file=sys.stderr, flush=True)
    if not jobs:
        return
    with path.open("a", newline="") as f, Pool(WORKERS) as pool:
        w = csv.DictWriter(f, FIELDS)
        if not done:
            w.writeheader()
        for k, row in enumerate(pool.imap_unordered(one, jobs), 1):
            w.writerow(row)
            f.flush()
            if k % 20 == 0:
                print(f"  {k}/{len(jobs)}", file=sys.stderr, flush=True)


def stats(x):
    x = np.asarray(x, float)
    return dict(mean=round(float(x.mean()), 3), se=round(float(x.std(ddof=1) / np.sqrt(len(x))), 3) if len(x) > 1 else 0.0,
                p95=round(float(np.percentile(x, 95)), 3), p99=round(float(np.percentile(x, 99)), 3))


def summarise(seeds):
    with (OUT / "v2_runs.csv").open() as f:
        rows = [r for r in csv.DictReader(f) if int(r["seed"]) in seeds]
    by = {}
    for r in rows:
        by.setdefault(r["label"], {})[int(r["seed"])] = r
    common = sorted(set.intersection(*(set(v) for v in by.values())))
    col = lambda label, k: np.array([float(by[label][s][k]) for s in common])  # noqa: E731
    table = {}
    for label in CONFIGS:
        if label in by:
            table[label] = dict(regret=stats(col(label, "regret")), arrival=round(float(col(label, "arrival").mean()), 3),
                                kB=round(float(col(label, "bytes").mean() / 1000), 1),
                                memory_peak=int(col(label, "memory_peak").max()))
    v2 = lambda k: col("v2", k)  # noqa: E731
    paired = {b: stats(v2("regret") - col(b, "regret")) for b in ("v2-silent", "cairn@120", "grace@120", "flood@120",
                                                                  "none@120", "v2-noescape", "v2-fifo", "v2-bvp",
                                                                  "v2-nofutile")}
    eta = (col("v2-silent", "regret").mean() - v2("regret").mean()) / max(v2("bytes").mean() / 1000, 1e-9)
    b_per_voxel = 10 * v2("tiles").sum() / max(v2("voxels").sum(), 1)
    fit = v2("fit").sum() / max(v2("arrived").sum(), 1)
    constraints = {
        "C1_bytes_per_voxel": dict(value=round(float(b_per_voxel), 3), target="<= 0.7", ok=bool(b_per_voxel <= 0.7)),
        "C1_eta_ratio": dict(value=round(float(eta / np.mean(ETA_2D)), 3), target=">= 0.5 (no loss against 2-D)",
                             ok=bool(eta / np.mean(ETA_2D) >= 0.5)),
        "C2_ingest_cycles_p99": dict(value=float(np.percentile(v2("ingest_p99"), 99)), target="< 20000",
                                     ok=bool(np.percentile(v2("ingest_p99"), 99) < 20000)),
        "C3_cycles_max": dict(value=float(v2("cycles_max").max()), target=f"< {BUDGET}",
                              ok=bool(v2("cycles_max").max() < BUDGET),
                              share_of_slots_over=round(float(v2("slots_over").sum() / v2("slots").sum()), 4)),
        "C3_memory": dict(value=int(v2("memory_peak").max()), target="<= 256", ok=bool(v2("memory_peak").max() <= 256)),
        "C3_liveness_K12": dict(value=round(float(col("v2@T20000", "arrival").mean()), 3),
                                target="1.0 within 20000 slots", ok=bool(col("v2@T20000", "arrival").min() == 1)),
        "C3_stress_K6_5000": dict(value=round(float(col("v2@K6,T5000", "arrival").mean()), 3), target="report", ok=None),
        "C3_arrival_500_K12": dict(value=round(float(v2("arrival").mean()), 3), target="report", ok=None),
        "C3_arrival_500_K6": dict(value=round(float(col("v2@K6", "arrival").mean()), 3), target="report", ok=None),
        "C3_verified_route_fit": dict(value=round(float(fit), 3), target="report", ok=None),
        "C4_global_coordinate_bytes": dict(value=0, target="0", ok=True, evidence="tests/math_verify_v2.py"),
    }
    res = dict(seeds=common, configs=table, v2_minus_other_regret=paired, eta_3d=round(float(eta), 4),
               constraints=constraints)
    (OUT / "v2_results.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    short = dict(n=len(common), regret={k: v["regret"]["mean"] for k, v in table.items()},
                 arrival={k: v["arrival"] for k, v in table.items()}, kB={k: v["kB"] for k, v in table.items()},
                 constraints={k: [v["value"], v["ok"]] for k, v in constraints.items()})
    print(json.dumps(short))


if __name__ == "__main__":
    n = int(next((a.split("=")[1] for a in sys.argv if a.startswith("--seeds=")), 20))
    seeds = list(range(n))
    sweep(seeds)
    summarise(seeds)
