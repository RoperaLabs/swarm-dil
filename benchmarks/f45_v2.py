"""Gate F4.5 for GRACE-3D 2.0.0: one bounded post-empirical refinement pass (docs/v2_proofs.md F4.5).

Run 1: each candidate against the F4 agent on the 20 main seeds (0-19) and 20 unseen seeds (100-119), paired. A
candidate passes if it improves at least one Pareto metric (regret, bytes, p99 slot cycles) significantly (paired
t-test, p < 0.05/3 Bonferroni over the three candidates, same sign on both halves), degrades none and keeps every
constraint. Run 2: the composite of the passing candidates, under the rule fixed in F4.5.3 before it ran.

Decision: futility adopted (the default, protocol "v2"; "v2-nofutile" is the F4 agent); the loss model rejected and
its code removed, so its rows (None below) come from the evaluation run and are not re-run.

    python benchmarks/f45_v2.py          # resumable; rows in benchmarks/v2_f45_runs.csv, verdict in v2_f45.json
"""
import csv
import json
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from scipy import stats

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from sim3d.ops import BUDGET  # noqa: E402
from sim3d.world import Mission, run  # noqa: E402

OUT = ROOT / "benchmarks"
WORKERS = 5
MAIN, VALIDATION = list(range(20)), list(range(100, 120))
CONFIGS = {"F4": dict(protocol="v2-nofutile"), "F4+loss": None, "F4+futile": dict(protocol="v2"),
           "F4+futile(p_esc=0.01)": dict(protocol="v2", p_esc=0.01), "F4+loss+futile": None}
REF, COMPOSITE, COMPONENTS = "F4", "F4+loss+futile", ("F4+loss", "F4+futile")   # run 2: rule in F4.5.3
ALPHA = 0.05 / 3                 # Bonferroni over the three candidates
FIELDS = ["label", "seed", "regret", "arrival", "bytes", "cycles_p99", "cycles_max", "memory_peak", "escapes",
          "first_arrival", "slots", "commits", "fit", "arrived"]     # slots: agent-slots before arrival
METRICS = {"regret": "lower", "bytes": "lower", "cycles_p99": "lower"}     # the Pareto metrics compared


def one(job):
    label, seed = job
    o = run(replace(Mission(), **CONFIGS[label]), seed)
    return dict(label=label, seed=seed, regret=o.regret, arrival=o.arrival_rate, bytes=int(o.bytes_sent.sum()),
                cycles_p99=float(np.percentile(o.cycles, 99)), cycles_max=float(o.cycles.max()),
                memory_peak=o.memory_peak, escapes=int(o.escapes.sum()), first_arrival=float(o.arrived_at.min()),
                slots=len(o.cycles), commits=int(o.committed.sum()), fit=sum(o.fit), arrived=len(o.fit))


def sweep():
    path = OUT / "v2_f45_runs.csv"
    done = set()
    if path.exists():
        with path.open() as f:
            done = {(r["label"], int(r["seed"])) for r in csv.DictReader(f)}
    jobs = [(label, s) for label, cfg in CONFIGS.items() if cfg for s in MAIN + VALIDATION if (label, s) not in done]
    print(f"{len(jobs)} missions to run", file=sys.stderr, flush=True)
    if jobs:
        with path.open("a", newline="") as f, Pool(WORKERS) as pool:
            w = csv.DictWriter(f, FIELDS)
            if not done:
                w.writeheader()
            for row in pool.imap_unordered(one, jobs):
                w.writerow(row)
                f.flush()


def verdict():
    with (OUT / "v2_f45_runs.csv").open() as f:
        rows = list(csv.DictReader(f))
    by = {(r["label"], int(r["seed"])): r for r in rows}

    def col(label, key, seeds):
        return np.array([float(by[(label, s)][key]) for s in seeds])

    def paired(cand, ref, m):
        d_all = col(cand, m, MAIN + VALIDATION) - col(ref, m, MAIN + VALIDATION)
        halves = [float((col(cand, m, s) - col(ref, m, s)).mean()) for s in (MAIN, VALIDATION)]
        p = float(stats.ttest_rel(col(cand, m, MAIN + VALIDATION), col(ref, m, MAIN + VALIDATION)).pvalue) \
            if np.any(d_all) else 1.0
        return dict(diff=round(float(d_all.mean()), 4), se=round(float(d_all.std(ddof=1) / np.sqrt(len(d_all))), 4),
                    p=round(p, 4), main=round(halves[0], 4), validation=round(halves[1], 4))

    def compare(cand, ref):
        res = {m: paired(cand, ref, m) for m in METRICS}
        improved = [m for m, r in res.items() if r["p"] < ALPHA and r["diff"] < 0 and r["main"] < 0 and r["validation"] < 0]
        degraded = [m for m, r in res.items() if r["p"] < ALPHA and r["diff"] > 0]
        return res, improved, degraded
    out = {}
    for cand in CONFIGS:
        if cand == REF:
            continue
        res, improved, degraded = compare(cand, REF)
        arrival = col(cand, "arrival", MAIN + VALIDATION).mean() - col(REF, "arrival", MAIN + VALIDATION).mean()
        constraints_ok = bool(col(cand, "cycles_max", MAIN + VALIDATION).max() < BUDGET
                              and col(cand, "memory_peak", MAIN + VALIDATION).max() <= 256)
        out[cand] = dict(metrics=res, arrival_diff=round(float(arrival), 4), constraints_ok=constraints_ok,
                         improved=improved, degraded=degraded,
                         adopt=bool(improved and not degraded and constraints_ok and arrival >= -0.01))
    out["futile_variants"] = {m: paired("F4+futile(p_esc=0.01)", "F4+futile", m) for m in [*METRICS, "arrival", "escapes"]}
    worse_than = {c: compare(COMPOSITE, c)[2] for c in COMPONENTS}       # (ii): metrics where C is worse than c alone
    out[COMPOSITE]["worse_than_component"] = worse_than
    better_alone = [c for c in COMPONENTS if worse_than[c]]               # adding the other component hurt c
    if out[COMPOSITE]["adopt"] and not better_alone:
        out["decision"] = COMPOSITE
    elif len(better_alone) == 1:
        out["decision"] = better_alone[0]
    else:
        out["decision"] = min(COMPONENTS, key=lambda c: min(out[c]["metrics"][m]["p"] for m in out[c]["improved"]))
    (OUT / "v2_f45.json").write_text(json.dumps(out, indent=1), encoding="utf-8")
    print(json.dumps({k: dict(adopt=v["adopt"], improved=v["improved"], degraded=v["degraded"],
                              regret=v["metrics"]["regret"], bytes=v["metrics"]["bytes"])
                      for k, v in out.items() if k in CONFIGS} | {"decision": out["decision"]}))


if __name__ == "__main__":
    sweep()
    verdict()
