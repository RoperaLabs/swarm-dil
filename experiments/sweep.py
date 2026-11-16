"""Sweep message loss x burstiness x attrition for every strategy; write CSV, figure and table.

    python experiments/sweep.py            # full sweep (~5 min on 6 cores), then plot
    python experiments/sweep.py --plot     # re-plot from results/sweep.csv
"""
import argparse
import csv
import itertools
from multiprocessing import Pool
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from swarm_dil import STRATEGIES, Scenario, run

OUT = Path(__file__).resolve().parent.parent / "results"
LOSSES = [round(x, 1) for x in np.arange(0, 1.0, 0.1)]
BURSTS = [None, 10]
ATTRITION = [0.0, 0.3]
SEEDS = range(30)
COLOURS = {"centralised": "#c0392b", "decentralised": "#2471a3", "partition": "#7f8c8d"}


def one(args):
    strategy, loss, burst, attrition, seed = args
    sc = Scenario(loss=loss, burst=burst, attrition=attrition)
    r = run(strategy, sc, seed)
    return dict(strategy=strategy, loss=loss, burst=burst or 0, attrition=attrition, seed=seed,
                time_all_found=min(r.time_all_found, sc.t_max),
                success=int(np.isfinite(r.time_all_found)),
                coverage_300s=round(float(r.coverage[299]), 4))


def sweep():
    jobs = list(itertools.product(STRATEGIES, LOSSES, BURSTS, ATTRITION, SEEDS))
    with Pool(6) as pool:  # 6 of 8 threads: stays inside the 85 % CPU budget
        rows = pool.map(one, jobs, chunksize=8)
    OUT.mkdir(exist_ok=True)
    with open(OUT / "sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0])
        w.writeheader()
        w.writerows(rows)


def load():
    with open(OUT / "sweep.csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("loss", "burst", "attrition", "time_all_found", "coverage_300s"):
            r[k] = float(r[k])
        r["success"] = int(r["success"])
    return rows


def select(rows, **kw):
    return [r for r in rows if all(r[k] == v for k, v in kw.items())]


def plot(rows):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for ax, (burst, title) in zip(axes, [(0.0, "independent losses"), (10.0, "bursty losses (mean outage 10 s)")]):
        for s in STRATEGIES:
            for attrition, ls in [(0.0, "-"), (0.3, "--")]:
                med, q1, q3 = [], [], []
                for loss in LOSSES:
                    t = [r["time_all_found"] for r in select(rows, strategy=s, loss=loss, burst=burst, attrition=attrition)]
                    med.append(np.median(t))
                    q1.append(np.percentile(t, 25))
                    q3.append(np.percentile(t, 75))
                label = f"{s}{' + 30 % attrition' if attrition else ''}"
                ax.plot(LOSSES, med, ls, color=COLOURS[s], label=label, lw=2)
                if attrition == 0.0:
                    ax.fill_between(LOSSES, q1, q3, color=COLOURS[s], alpha=0.15)
        ax.set_title(title)
        ax.set_xlabel("message loss rate (every link)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("time to find all 10 targets (s, median)\ncapped at 1200 s mission limit")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("20-UAV area search, 1 km², 30 seeds per point", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / "fig_time_vs_loss.png", dpi=150)


def table(rows):
    lines = ["| strategy | loss | burst | attrition | median time to find all (s) | success within 1200 s |",
             "|---|---|---|---|---|---|"]
    for s, loss, burst, att in itertools.product(STRATEGIES, [0.0, 0.5, 0.9], [0.0, 10.0], ATTRITION):
        sel = select(rows, strategy=s, loss=loss, burst=burst, attrition=att)
        med = np.median([r["time_all_found"] for r in sel])
        ok = np.mean([r["success"] for r in sel])
        lines.append(f"| {s} | {loss:.1f} | {'10 s' if burst else 'iid'} | {att:.0%} | {med:.0f} | {ok:.0%} |")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true", help="skip the sweep, re-plot existing CSV")
    if not ap.parse_args().plot:
        sweep()
    rows = load()
    plot(rows)
    table(rows)
