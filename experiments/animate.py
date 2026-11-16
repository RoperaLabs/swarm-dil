"""Side-by-side GIF of the three strategies on the reference terrain, same seed.

    python experiments/animate.py [--interferer 10] [--attrition 0.3] [--seed 0]
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Rectangle

from swarm_dil import STRATEGIES, run
from swarm_dil.sim import REFERENCE_INTERFERER_XY, reference

OUT = Path(__file__).resolve().parent.parent / "results"

ap = argparse.ArgumentParser()
ap.add_argument("--interferer", type=float, default=10.0, help="dBm, mid-area emitter")
ap.add_argument("--attrition", type=float, default=0.3)
ap.add_argument("--seed", type=int, default=0)
ap.add_argument("--seconds", type=int, default=600)
ap.add_argument("--every", type=int, default=4, help="simulated seconds per frame")
a = ap.parse_args()

sc = reference(a.interferer, attrition=a.attrition)
runs = {s: run(s, sc, a.seed) for s in STRATEGIES}
n = round(sc.area / sc.cell)
cmap = ListedColormap(["#f4f6f7", "#aed6f1"])

fig, axes = plt.subplots(1, 3, figsize=(11, 4.4))
artists = []
for ax, (s, r) in zip(axes, runs.items()):
    img = ax.imshow(np.zeros((n, n)), cmap=cmap, vmin=0, vmax=1, origin="lower",
                    extent=(0, sc.area, 0, sc.area))
    for x0, y0, x1, y1 in sc.obstacles:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color="#5d6d7e"))
    ax.add_patch(Circle(REFERENCE_INTERFERER_XY, 150, fill=False, ls=":", color="#e67e22"))
    ax.plot(*REFERENCE_INTERFERER_XY, marker="X", color="#e67e22", ms=9)
    ax.plot(*sc.base_xy, marker="^", color="#145a32", ms=10, zorder=5)
    ax.scatter(*r.targets_xy.T, s=30, facecolors="none", edgecolors="#7f8c8d")
    seen = ax.scatter([], [], s=120, marker="*", color="#f5b041", zorder=3)
    known = ax.scatter([], [], s=140, marker="*", color="#1e8449", zorder=3)
    live = ax.scatter([], [], s=14, color="#1b4f72", zorder=4)
    dead = ax.scatter([], [], s=24, marker="x", color="#c0392b", zorder=4)
    ax.set_xticks([])
    ax.set_yticks([])
    artists.append((s, r, img, seen, known, live, dead, ax))
fig.suptitle(f"20 UAVs, 1 km² with obstacles ▮, {a.interferer:.0f} dBm interferer ✕, "
             f"{a.attrition:.0%} of UAVs fail. Star: yellow = seen, green = C2 knows (C2 ▲)",
             fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.93))


def pts(xy, mask):
    return xy[mask] if mask.any() else np.empty((0, 2))


def frame(t):
    for s, r, img, seen, known, live, dead, ax in artists:
        img.set_data((r.covered_at <= t).reshape(n, n).T)
        seen.set_offsets(pts(r.targets_xy, (r.found <= t) & (r.reported > t)))
        known.set_offsets(pts(r.targets_xy, r.reported <= t))
        al = r.alive[t]
        live.set_offsets(r.trace[t][al])
        dead.set_offsets(pts(r.trace[t], ~al))
        ax.set_title(f"{s}\nt = {t:3d} s   C2 knows {(r.reported <= t).sum()}/{len(r.reported)}",
                     fontsize=10)


anim = FuncAnimation(fig, frame, frames=range(0, a.seconds, a.every))
OUT.mkdir(exist_ok=True)
anim.save(OUT / "search_dil.gif", writer=PillowWriter(fps=15), dpi=72)
