"""v0.2 sweeps on the reference terrain: field interference, channel data rate and UAV memory.

    python experiments/sweep.py            # ~1,100 missions (~6 min on 6 cores), then plot
    python experiments/sweep.py --plot     # re-plot from results/sweep.csv
"""
import argparse
import csv
import itertools
from multiprocessing import Pool
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from swarm_dil import STRATEGIES, Radio, run
from swarm_dil.sim import reference

OUT = Path(__file__).resolve().parent.parent / "results"
SEEDS = range(20)
COLOURS = {"centralised": "#c0392b", "decentralised": "#2471a3", "partition": "#7f8c8d"}

FREE_CELLS = 1509  # free cells on the reference terrain (asserted in one())

INTERFERENCE_DBM = [None, -10, 0, 10, 20]
RATES_BPS = [9_600, 19_200, 64_000, 250_000]
MEMORY_FRACTION = [0.05, 0.1, 0.25, 0.5, 1.0]


def scenario(experiment, value, attrition):
    if experiment == "interference":
        return reference(value, attrition=attrition)
    if experiment == "rate":
        return reference(radio=Radio(rate_bps=float(value)))
    return reference(memory_cells=None if value == 1.0 else round(value * FREE_CELLS))


def jobs():
    for s, v, a, seed in itertools.product(STRATEGIES, INTERFERENCE_DBM, [0.0, 0.3], SEEDS):
        yield "interference", s, v, a, seed
    for s, v, seed in itertools.product(STRATEGIES, RATES_BPS, SEEDS):
        yield "rate", s, v, 0.0, seed
    for s, v, seed in itertools.product(STRATEGIES, MEMORY_FRACTION, SEEDS):
        yield "memory", s, v, 0.0, seed


def one(job):
    experiment, strategy, value, attrition, seed = job
    sc = scenario(experiment, value, attrition)
    r = run(strategy, sc, seed)
    assert r.free.sum() == FREE_CELLS
    return dict(experiment=experiment, strategy=strategy, value="none" if value is None else value,
                attrition=attrition, seed=seed,
                time_reported=min(r.time_all_reported, sc.t_max),
                time_found=min(r.time_all_found, sc.t_max),
                success=int(np.isfinite(r.time_all_reported)),
                bytes_per_uav=round(r.bytes_per_uav, 1), peak_memory=r.peak_memory)


def sweep():
    with Pool(6) as pool:  # 6 of 8 threads: inside the 85 % CPU budget
        rows = pool.map(one, list(jobs()), chunksize=4)
    OUT.mkdir(exist_ok=True)
    with open(OUT / "sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0])
        w.writeheader()
        w.writerows(rows)


def load():
    with open(OUT / "sweep.csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("attrition", "time_reported", "time_found", "bytes_per_uav"):
            r[k] = float(r[k])
        r["success"] = int(r["success"])
    return rows


def stat(rows, key="time_reported", fn=np.median, **kw):
    sel = [r[key] for r in rows if all(str(r[k]) == str(v) for k, v in kw.items())]
    return fn(sel)


def plot(rows):
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), sharey=True)
    xs = ["none" if v is None else str(v) for v in INTERFERENCE_DBM]
    for s in STRATEGIES:
        for a, ls in [(0.0, "-"), (0.3, "--")]:
            y = [stat(rows, experiment="interference", strategy=s, value=x, attrition=a) for x in xs]
            axes[0].plot(range(len(xs)), y, ls, color=COLOURS[s], lw=2,
                         label=f"{s}{' + 30 % UAVs lost' if a else ''}")
        y = [stat(rows, experiment="rate", strategy=s, value=v) for v in RATES_BPS]
        axes[1].plot(RATES_BPS, y, "-o", color=COLOURS[s], lw=2, label=s)
        y = [stat(rows, experiment="memory", strategy=s, value=v) for v in MEMORY_FRACTION]
        axes[2].plot([100 * v for v in MEMORY_FRACTION], y, "-o", color=COLOURS[s], lw=2, label=s)
    axes[0].set_xticks(range(len(xs)), ["none"] + [f"{v} dBm" for v in INTERFERENCE_DBM[1:]])
    axes[0].set_xlabel("field interferer power (one emitter mid-area)")
    axes[0].set_title("interference")
    axes[1].set_xscale("log")
    axes[1].set_xticks(RATES_BPS, ["9.6k", "19.2k", "64k", "250k"])
    axes[1].set_xlabel("shared channel data rate (bit/s)")
    axes[1].set_title("bandwidth")
    axes[2].set_xlabel("UAV map memory (% of the search area)")
    axes[2].set_title("memory")
    axes[0].set_ylabel("time until C2 knows all 10 targets\n(s, median, capped at 1200)")
    axes[0].legend(fontsize=7.5)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle("20 UAVs, 1 km² with obstacles, radio model with range, fading and contention; "
                 "20 seeds per point", fontsize=11)
    fig.tight_layout()
    fig.savefig(OUT / "fig_v02.png", dpi=150)


def table(rows):
    lines = ["| experiment | value | strategy | attrition | median time C2 knows all (s) | success | bytes/UAV/s |",
             "|---|---|---|---|---|---|---|"]
    keys = sorted({(r["experiment"], r["value"], r["strategy"], r["attrition"]) for r in rows})
    for e, v, s, a in keys:
        kw = dict(experiment=e, value=v, strategy=s, attrition=a)
        lines.append(f"| {e} | {v} | {s} | {a:.0%} | {stat(rows, **kw):.0f} | "
                     f"{stat(rows, 'success', np.mean, **kw):.0%} | {stat(rows, 'bytes_per_uav', np.mean, **kw):.0f} |")
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
