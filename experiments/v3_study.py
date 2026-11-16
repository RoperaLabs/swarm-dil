"""v0.3 study: GRACE against flooding and silence on the A -> B navigation mission.

    python experiments/v3_study.py           # all experiments (~35 min on 6 cores)
    python experiments/v3_study.py --plot    # figures and tables from results/v3/*.csv

Writes results/v3/*.csv, results/v3/*.png, vector figures for the paper in
docs/theory/fig/*.pdf and one recorded mission for the interactive page in docs/replay.json.
"""
import argparse
import csv
import itertools
import json
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from swarm_dil.nav import theory
from swarm_dil.nav.world import Mission, run

ROOT = Path(__file__).resolve().parent.parent
OUT, FIG = ROOT / "results" / "v3", ROOT / "docs" / "theory" / "fig"
SEEDS = range(20)
BASE = Mission()
COL = {"none": "#7f8c8d", "flood_raw": "#e59866", "flood_tile": "#c0392b", "grace": "#2471a3",
       "grace_norel": "#76d7c4", "grace_noloss": "#8e44ad", "grace_fifo": "#f4d03f"}
INTERFERERS = ((16, 30), (30, 16))  # cells, near the middle of every A -> B route
INTERFERENCE_DBM = ["none", -30, -20, -10, 0]
MEMORY = [4, 8, 16, 32, "inf"]


def mission(protocol, knob, condition=None, value=None):
    m = replace(BASE, protocol=protocol)
    m = replace(m, price=knob) if protocol.startswith("grace") else replace(m, budget=int(knob or 64))
    if condition == "interference" and value != "none":
        m = replace(m, interferers=tuple((x, y, float(value)) for x, y in INTERFERERS))
    if condition == "memory" and value != "inf":
        m = replace(m, memory_tiles=int(value))
    return m


def jobs():
    yield from (("pareto", "none", 0, None, s) for s in SEEDS)
    yield from (("pareto", "grace", p, None, s) for p in (0.05, 0.1, 0.2, 0.5, 1, 2, 5) for s in SEEDS)
    yield from (("pareto", p, k, None, s) for p in ("grace_norel", "grace_noloss")
                for k in (0.02, 0.05, 0.2, 1) for s in SEEDS)
    yield from (("pareto", "flood_tile", b, None, s) for b in (8, 12, 16, 24, 32, 64) for s in SEEDS)
    yield from (("pareto", "flood_raw", b, None, s) for b in (8, 16, 32, 64) for s in SEEDS)
    for cond, values in (("interference", INTERFERENCE_DBM), ("memory", MEMORY)):
        protos = [("none", 0), ("grace", 0.2), ("flood_tile", 16), ("flood_tile", 64)]
        protos += [("grace_noloss", 0.2)] if cond == "interference" else [("grace_fifo", 0.2)]
        for (p, k), v, s in itertools.product(protos, values, SEEDS):
            yield cond, p, k, v, s


def one(job):
    exp, protocol, knob, value, seed = job
    o = run(mission(protocol, knob, exp, value), seed)
    return dict(experiment=exp, protocol=protocol, knob=knob, value=value, seed=seed,
                regret=round(o.regret, 5), arrival=o.arrival_rate, bytes=int(o.bytes_sent.sum()),
                last_arrival=float(np.nanmax(np.where(np.isfinite(o.arrived_at), o.arrived_at, BASE.t_max))))


def sweep():
    """Runs only the jobs missing from results/v3/study.csv, so the study can be extended."""
    path = OUT / "study.csv"
    done = set()
    if path.exists():
        with open(path) as f:
            done = {tuple(r[k] for k in ("experiment", "protocol", "knob", "value", "seed")) for r in csv.DictReader(f)}
    todo = [j for j in jobs() if tuple("" if x is None else str(x) for x in j) not in done]
    with Pool(6) as pool:  # 6 of 8 threads: inside the 85 % CPU budget
        rows = pool.map(one, todo, chunksize=2)
    OUT.mkdir(parents=True, exist_ok=True)
    if rows:
        new = not path.exists()
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=rows[0])
            if new:
                w.writeheader()
            w.writerows(rows)
    if (ROOT / "docs" / "replay.json").exists():
        return
    rec = run(replace(BASE, protocol="grace", price=0.2), 3, record=True)
    (ROOT / "docs").mkdir(exist_ok=True)
    rec.replay.update(regret=rec.regret, bytes=int(rec.bytes_sent.sum()), sensor_range=BASE.sensor_range,
                      half_angle=BASE.half_angle_deg)
    (ROOT / "docs" / "replay.json").write_text(json.dumps(rec.replay, separators=(",", ":")))


def load():
    with open(OUT / "study.csv") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in ("regret", "arrival", "bytes", "last_arrival"):
            r[k] = float(r[k])
    return rows


def agg(rows, **kw):
    sel = [r for r in rows if all(str(r[k]) == str(v) for k, v in kw.items())]
    reg = np.array([r["regret"] for r in sel])
    se = reg.std(ddof=1) / np.sqrt(len(reg))
    return reg.mean(), se, np.mean([r["bytes"] for r in sel]) / 1000, np.mean([r["arrival"] for r in sel])


def save(fig, name):
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / f"{name}.pdf")
    fig.savefig(FIG / f"{name}.png", dpi=150)  # for the interactive page (served from docs/)
    plt.close(fig)


def plot_pareto(rows):
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    r0, se0, _, _ = agg(rows, experiment="pareto", protocol="none")
    ax.axhline(r0, color=COL["none"], ls=":", label="no communication")
    for p, knobs, lab in (("grace", (0.05, 0.1, 0.2, 0.5, 1, 2, 5), "GRACE (price sweep)"),
                          ("flood_tile", (8, 12, 16, 24, 32, 64), "tile flooding (budget sweep)"),
                          ("flood_raw", (8, 16, 32, 64), "raw-cell flooding (budget sweep)")):
        pts = [agg(rows, experiment="pareto", protocol=p, knob=k) for k in knobs]
        ax.errorbar([b for _, _, b, _ in pts], [r for r, _, _, _ in pts], yerr=[s for _, s, _, _ in pts],
                    fmt="-o", color=COL[p], label=lab, ms=4, capsize=2)
    for p, lab in (("grace_norel", "ablation: no goal relevance"), ("grace_noloss", "ablation: no loss model")):
        pts = [agg(rows, experiment="pareto", protocol=p, knob=k) for k in (0.02, 0.05, 0.2, 1)]
        ax.plot([b for _, _, b, _ in pts], [r for r, _, _, _ in pts], "--s", color=COL[p], label=lab, ms=4)
    ax.set_xscale("log")
    ax.set_xticks([2, 3, 5, 10, 20, 30, 50, 80], ["2", "3", "5", "10", "20", "30", "50", "80"])
    ax.minorticks_off()
    ax.set_xlabel("total bytes sent by the team (kB, log scale)")
    ax.set_ylabel("regret: extra distance over the true shortest path")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=7.5)
    fig.tight_layout()
    save(fig, "pareto")


def plot_condition(rows, cond, values, labels, xlabel, name, extra):
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.8))
    for p, k in [("none", 0), ("grace", 0.2), ("flood_tile", 16), ("flood_tile", 64), (extra, 0.2)]:
        pts = [agg(rows, experiment=cond, protocol=p, knob=k, value=v) for v in values]
        lab = {"none": "no communication", "grace": "GRACE (price 0.2)",
               "flood_tile": f"tile flooding ({k} B/slot)", "grace_noloss": "ablation: GRACE without loss model",
               "grace_fifo": "ablation: GRACE with oldest-first forgetting"}[p]
        ls = "--" if (p == "flood_tile" and k == 64) else "-"
        axes[0].errorbar(range(len(values)), [r for r, _, _, _ in pts], yerr=[s for _, s, _, _ in pts],
                         fmt=ls + "o", color=COL[p], label=lab, ms=4, capsize=2)
        if p != "none":
            axes[1].plot(range(len(values)), [b for _, _, b, _ in pts], ls + "o", color=COL[p], ms=4)
    for ax in axes:
        ax.set_xticks(range(len(values)), labels)
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("regret")
    if cond == "memory":
        axes[0].set_yscale("log")
        axes[0].set_yticks([0.2, 0.5, 1, 2, 5], ["0.2", "0.5", "1", "2", "5"])
        axes[0].minorticks_off()
    axes[1].set_ylabel("total bytes (kB)")
    axes[1].set_yscale("log")
    axes[1].set_yticks([5, 10, 20, 50, 100], ["5", "10", "20", "50", "100"])
    axes[1].minorticks_off()
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    save(fig, name)


def plot_theory():
    fig, axes = plt.subplots(1, 2, figsize=(9.6, 3.6))
    p, q = 0.05, 0.15  # mean burst 1/q = 6.7 slots, pi_B = 0.25
    pi_b, lam = p / (p + q), 1 - p - q
    ks = np.arange(1, 9)
    rng = np.random.default_rng(1)
    T = 300_000
    u = rng.random(T)
    bad = np.empty(T, bool)
    bad[0] = False
    for t in range(1, T):
        bad[t] = u[t] >= q if bad[t - 1] else u[t] < p
    for tau, c in ((1, "#c0392b"), (3, "#e67e22"), (10, "#2471a3")):
        axes[0].semilogy(ks, [theory.p_all_lost([tau] * (k - 1), pi_b, lam) for k in ks], "-", color=c,
                         label=f"spacing {tau} slots")
        idx = np.arange(T - tau * 8)
        mc = [np.all([bad[idx + tau * m] for m in range(k)], axis=0).mean() for k in ks]
        axes[0].semilogy(ks, mc, "o", color=c, ms=3)
    axes[0].semilogy(ks, pi_b ** ks, "k:", label="independent losses")
    axes[0].set_xlabel("copies k")
    axes[0].set_ylabel("P(all k copies lost)")
    axes[0].set_title("Prop. 5: lines theory, dots Monte Carlo", fontsize=9)
    axes[0].legend(fontsize=7.5)
    s = np.linspace(1, 12, 200)
    axes[1].plot(s, (2 + s ** 2 / 8) / s, color="#2471a3", label="1-cell-thick boundary (what is sensed)")
    axes[1].plot(s, (2 + s ** 2 / 8) / s ** 2, color="#7f8c8d", label="filled area")
    axes[1].axhline(2, color="#c0392b", ls=":", label="raw cell list (2 B/cell)")
    axes[1].axvline(4, color="k", lw=0.8)
    axes[1].set_xlabel("tile side s (cells)")
    axes[1].set_ylabel("bytes per blocked cell")
    axes[1].set_title("Cor. 3: s* = sqrt(8 b_id) = 4", fontsize=9)
    axes[1].set_ylim(0, 3)
    axes[1].legend(fontsize=7.5)
    for ax in axes:
        ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    save(fig, "theory_checks")


def table(rows):
    lines = ["| experiment | protocol | knob | condition | regret (mean +- s.e.) | arrival | kB sent |",
             "|---|---|---|---|---|---|---|"]
    keys = sorted({(r["experiment"], r["protocol"], r["knob"], r["value"]) for r in rows}, key=str)
    for e, p, k, v in keys:
        r, s, b, a = agg(rows, experiment=e, protocol=p, knob=k, value=v)
        lines.append(f"| {e} | {p} | {k} | {v} | {r:.3f} +- {s:.3f} | {a:.0%} | {b:.1f} |")
    (OUT / "summary.md").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    if not ap.parse_args().plot:
        sweep()
    rows = load()
    plot_pareto(rows)
    plot_condition(rows, "interference", INTERFERENCE_DBM, ["none", "-30", "-20", "-10", "0"],
                   "power of two field interferers (dBm)", "interference", "grace_noloss")
    plot_condition(rows, "memory", MEMORY, ["4", "8", "16", "32", "inf"],
                   "map memory per agent (tiles of 4 B)", "memory", "grace_fifo")
    plot_theory()
    table(rows)
