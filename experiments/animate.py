"""Side-by-side GIF of the three strategies on the same DIL scenario and seed.

    python experiments/animate.py [--loss 0.5] [--burst 10] [--attrition 0.3] [--seed 0]
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import ListedColormap

from swarm_dil import STRATEGIES, Scenario, run

OUT = Path(__file__).resolve().parent.parent / "results"

ap = argparse.ArgumentParser()
ap.add_argument("--loss", type=float, default=0.5)
ap.add_argument("--burst", type=float, default=10)
ap.add_argument("--attrition", type=float, default=0.3)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--seconds", type=int, default=600)
ap.add_argument("--every", type=int, default=4, help="simulated seconds per frame")
a = ap.parse_args()

sc = Scenario(loss=a.loss, burst=a.burst, attrition=a.attrition)
runs = {s: run(s, sc, a.seed) for s in STRATEGIES}
n = round(sc.area / sc.cell)
cmap = ListedColormap(["#f4f6f7", "#aed6f1"])

fig, axes = plt.subplots(1, 3, figsize=(11, 4.2))
artists = []
for ax, (s, r) in zip(axes, runs.items()):
    img = ax.imshow(np.zeros((n, n)), cmap=cmap, vmin=0, vmax=1, origin="lower",
                    extent=(0, sc.area, 0, sc.area))
    hidden = ax.scatter(*r.targets_xy.T, s=30, facecolors="none", edgecolors="#7f8c8d")
    found = ax.scatter([], [], s=120, marker="*", color="#e67e22", zorder=3)
    live = ax.scatter([], [], s=14, color="#1b4f72", zorder=4)
    dead = ax.scatter([], [], s=24, marker="x", color="#c0392b", zorder=4)
    ax.set_xticks([])
    ax.set_yticks([])
    artists.append((s, r, img, found, live, dead, ax))
burst_txt = f"{a.burst:.0f} s bursts" if a.burst else "independent"
fig.suptitle(f"20 UAVs searching 1 km²: {a.loss:.0%} message loss ({burst_txt}), "
             f"{a.attrition:.0%} of UAVs fail", fontsize=11)
fig.tight_layout(rect=(0, 0, 1, 0.93))


def frame(t):
    for s, r, img, found, live, dead, ax in artists:
        img.set_data((r.covered_at <= t).reshape(n, n).T)
        hit = r.found <= t
        found.set_offsets(r.targets_xy[hit] if hit.any() else np.empty((0, 2)))
        al = r.alive[t]
        live.set_offsets(r.trace[t][al])
        dead.set_offsets(r.trace[t][~al] if (~al).any() else np.empty((0, 2)))
        ax.set_title(f"{s}\nt = {t:3d} s   targets {hit.sum()}/{len(hit)}", fontsize=10)


anim = FuncAnimation(fig, frame, frames=range(0, a.seconds, a.every))
OUT.mkdir(exist_ok=True)
anim.save(OUT / "search_dil.gif", writer=PillowWriter(fps=15), dpi=72)
