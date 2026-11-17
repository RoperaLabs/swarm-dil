"""v0.4 study: CAIRN against flooding, 3-D GRACE and two free-bandwidth references.

    python experiments/v4_study.py                 # run every study (resumable), then plot
    python experiments/v4_study.py pareto memory   # only these studies
    python experiments/v4_study.py --plot          # figures and tables from stored results

Each row of results/v4/<study>.csv is one seeded mission, so everything is reproducible.
"""
import csv
import sys
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from swarm_dil.nav3 import Mission3, run  # noqa: E402
from swarm_dil.radio import Radio  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "results" / "v4"
SEEDS = range(20)
WORKERS = 6
ACOUSTIC = dict(radio=Radio(tx_dbm=-16.0, rate_bps=400.0), budget=32)   # 50 B/s shared channel
FIELDS = ["study", "label", "x", "seed", "regret", "arrival", "bytes", "fields", "committed", "worst_commit",
          "memory_peak", "lower", "upper", "d_star", "advice", "team", "verified", "relay", "plan", "tile", "box"]


def studies():
    """label, x (the swept value), Mission3 overrides."""
    base = Mission3()
    s = {}
    s["pareto"] = (
        [("none", 0, dict(protocol="none")), ("oracle", 0, dict(protocol="oracle")),
         ("oracle-links", 0, dict(protocol="oracle-links"))]
        + [("flood", b, dict(protocol="flood", budget=b)) for b in (16, 26, 36, 46, 66)]
        + [("grace", k, dict(protocol="grace", price=k)) for k in (0.5, 2, 5, 10)]
        + [("cairn", k, dict(protocol="cairn", price=k)) for k in (0.5, 2, 5, 10, 20)]
        + [("cairn-nocuts", 0, dict(protocol="cairn-nocuts"))])
    s["ablation"] = [(p, 0, dict(protocol=p, price=2)) for p in
                     ("cairn", "cairn-noclaims", "cairn-nocert", "cairn-allcuts", "cairn-noboxes", "cairn-noverify",
                      "cairn-noreply", "cairn-noannounce")]
    s["theta"] = [("cairn", th, dict(protocol="cairn", price=2, theta=th)) for th in (0.0, 0.05, 0.1, 0.2, 0.4)]
    s["memory"] = [(p, mem, dict(protocol=p, price=k, memory_bytes=mem))
                   for p, k in (("flood", 0.5), ("grace", 2), ("cairn", 2))
                   for mem in (100, 200, 400, 800, None)]
    s["interference"] = [(p, dbm, dict(protocol=p, price=k, interferers=() if dbm is None else
                                       ((14, 26, 3, dbm), (26, 14, 3, dbm))))
                         for p, k in (("flood", 0.5), ("grace", 2), ("cairn", 2), ("none", 0))
                         for dbm in (None, -30, -20, -10)]
    s["acoustic"] = [(p, 0, dict(protocol=p, price=k, **ACOUSTIC))
                     for p, k in (("none", 0), ("flood", 0.5), ("grace", 2), ("cairn", 2), ("cairn-nocuts", 0))]
    s["mixed"] = [(p, 0, dict(protocol=p, price=k, team=12, ground=4))
                  for p, k in (("none", 0), ("flood", 0.5), ("cairn", 2), ("oracle-links", 0))]
    return base, s


def one(job):
    study, label, x, seed, mission = job
    o = run(mission, seed)
    lower, upper = o.bracket[-1][1:]
    return dict(study=study, label=label, x=x, seed=seed, regret=o.regret, arrival=o.arrival_rate,
                bytes=int(o.bytes_sent.sum()), fields=float(o.fields.mean()), committed=int(o.committed.sum()),
                worst_commit=max(o.commit_ratio, default=0.0), memory_peak=o.memory_peak, lower=lower, upper=upper,
                d_star=float(o.d_star[0]), **o.sent)


def sweep(names, seeds=SEEDS):
    base, all_studies = studies()
    OUT.mkdir(parents=True, exist_ok=True)
    for name in names:
        path = OUT / f"{name}.csv"
        done = set()
        if path.exists():
            with path.open() as f:
                done = {(r["label"], r["x"], r["seed"]) for r in csv.DictReader(f)}
        jobs = [(name, label, x, seed, replace(base, **kw)) for label, x, kw in all_studies[name] for seed in seeds
                if (label, str(x), str(seed)) not in done]
        print(f"{name}: {len(jobs)} missions to run", flush=True)
        if not jobs:
            continue
        with path.open("a", newline="") as f, Pool(WORKERS) as pool:
            w = csv.DictWriter(f, FIELDS)
            if not done:
                w.writeheader()
            for k, row in enumerate(pool.imap_unordered(one, jobs), 1):
                w.writerow(row)
                f.flush()
                if k % 20 == 0:
                    print(f"  {k}/{len(jobs)}", flush=True)


def load(name):
    with (OUT / f"{name}.csv").open() as f:
        rows = list(csv.DictReader(f))
    groups = {}
    for r in rows:
        groups.setdefault((r["label"], r["x"]), []).append(r)
    return groups


def summary(name, cols=("regret", "bytes", "arrival", "fields", "committed", "worst_commit", "memory_peak")):
    """Mean and standard error per (label, x)."""
    out = []
    for (label, x), rows in load(name).items():
        vals = {c: np.array([float(r[c]) for r in rows]) for c in cols}
        out.append(dict(label=label, x=x, n=len(rows), **{c: vals[c].mean() for c in cols},
                        regret_se=vals["regret"].std(ddof=1) / np.sqrt(len(rows)) if len(rows) > 1 else 0.0))
    return out


def table(names):
    lines = []
    for name in names:
        if not (OUT / f"{name}.csv").exists():
            continue
        lines += [f"## {name}", "", "| protocol | x | n | regret | s.e. | kB | arrival | searches/agent | committed | worst U/V* | peak map B |",
                  "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in summary(name):
            lines.append(f"| {r['label']} | {r['x']} | {r['n']} | {r['regret']:.3f} | {r['regret_se']:.3f} | {r['bytes'] / 1000:.1f} | "
                         f"{r['arrival']:.2f} | {r['fields']:.0f} | {r['committed']:.1f} | {r['worst_commit']:.3f} | {r['memory_peak']:.0f} |")
        lines.append("")
    (OUT / "summary.md").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    names = args or list(studies()[1])
    if "--plot" not in sys.argv:
        seeds = range(int(next((a.split("=")[1] for a in sys.argv if a.startswith("--seeds=")), len(SEEDS))))
        sweep(names, seeds)
    table(names)
