"""Media for GRACE-3D 2.0.0 (swarm-dil v2.0.0). Every frame is drawn from the simulator's record of a seeded mission.

    results/v2/grace3d_swarm.gif   the swarm at work, one drone followed: what it holds, how routes are chosen, what
                                   the team radios, how obstacles are avoided, and the futility escape (Sec. F4.5)
    results/v2/grace3d_vs.gif      the same terrain flown by GRACE-3D 2.0.0 and four other protocols, side by side
    docs/replay3d.json             the swarm mission for the 3-D replay of the explainer page (docs/index.html)
    paper/releases/fig/*.pdf|png   the figures of the technical note

    python experiments/v2_media.py            # seed 2: regret 0.88, near the 20-terrain mean (0.99); drone 1 followed

The agents are observed from outside: the futility escapes are read by wrapping the agent's escape method here.
"""
import argparse
import json
import math
import sys
import textwrap
from dataclasses import replace
from multiprocessing import Pool
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import to_rgb  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from sim3d import agent as grace3d  # noqa: E402
from sim3d.geom import cells_of, key_bit, segment  # noqa: E402
from sim3d.ops import BUDGET  # noqa: E402
from sim3d.world import Mission, run  # noqa: E402
from swarm_dil.nav3.world3 import make_world  # noqa: E402

FIG, OUT = ROOT / "paper" / "releases" / "fig", ROOT / "results" / "v2"
SURF, INK, INK2, MUTED, BASE, GRID = "#ffffff", "#0b0b0b", "#52514e", "#898781", "#c3c2b7", "#e1e0d9"
BLUE, ORANGE, RED = "#2a78d6", "#eb6834", "#d03b3b"     # the followed drone; map knowledge; a real obstacle hit
plt.rcParams.update({"font.family": "sans-serif", "font.size": 9, "figure.facecolor": SURF, "axes.facecolor": SURF,
                     "savefig.facecolor": SURF, "axes.edgecolor": BASE, "axes.labelcolor": INK2, "text.color": INK,
                     "xtick.color": MUTED, "ytick.color": MUTED, "axes.spines.top": False, "axes.spines.right": False})
PROTOCOLS = {"v2": ("GRACE-3D 2.0.0", {}), "grace": ("GRACE ported to 3-D", dict(price=2, memory_bytes=120)),
             "cairn": ("CAIRN v0.4", dict(price=2, memory_bytes=120)),
             "flood": ("Flooding", dict(budget=36, memory_bytes=120)), "none": ("Silence", dict(memory_bytes=120))}

FUTILE = []
_escape = grace3d.Agent._start_escape


def _watch_escape(self):
    if math.isfinite(self.phi) and self.phi < self.L - 1e-6 and self.plan:
        FUTILE.append(dict(id=self.id, t=self.t, phi=self.phi, L=self.L, prev=self.prev, k=self.k,
                           plan=list(self.plan), tiles=dict(self.tiles)))
    return _escape(self)


grace3d.Agent._start_escape = _watch_escape


def record(job):
    proto, seed = job
    FUTILE.clear()
    trace = []
    o = run(replace(Mission(), protocol=proto, **PROTOCOLS[proto][1]), seed, trace=trace)
    return proto, dict(trace=trace, regret=o.regret, cycles=o.cycles, bytes=int(o.bytes_sent.sum()),
                       arrived_at=o.arrived_at.tolist(), futile=list(FUTILE))


# ---------------------------------------------------------------------------------------------- 3-D scene
class Scene:
    """One 3-D view of a mission: the ground truth faint, then the agents, radio and (optionally) one followed drone."""

    def __init__(self, ax, blocked, start, goal, zoom=1.16):
        self.ax, self.dyn, self.shape = ax, [], blocked.shape
        nx, ny, nz = blocked.shape
        ax.voxels(blocked, facecolors=(*to_rgb(BASE), 0.16), edgecolor=None)
        ax.set_box_aspect((nx, ny, nz * 1.6), zoom=zoom)
        ax.set_xlim(0, nx), ax.set_ylim(0, ny), ax.set_zlim(0, nz)
        ax.set_axis_off()
        ax.plot([0, nx, nx, 0, 0], [0, 0, ny, ny, 0], [0, 0, 0, 0, 0], color=BASE, lw=0.8)
        for p, lab, mk in ((start, "A", "^"), (goal, "B", "*")):
            ax.scatter(*[[v + 0.5] for v in p], marker=mk, s=110 if mk == "*" else 70, color=INK, depthshade=False)
            ax.text(p[0] + 1.5, p[1] + 0.5, p[2] + 1.5, lab, color=INK, fontsize=11, weight="bold")

    def clear(self):
        for a in self.dyn:
            a.remove()
        self.dyn = []

    def _voxels(self, cells, color, alpha):
        if not cells:
            return
        grid = np.zeros(self.shape, bool)
        for c in cells:
            grid[c] = True
        self.dyn += list(self.ax.voxels(grid, facecolors=(*to_rgb(color), alpha), edgecolor=None).values())

    def label(self, c, text):
        """A red cell made unmissable: its voxel, a ring marker on top, and a text label in ink."""
        x, y, z = (v + 0.5 for v in c)
        self.dyn.append(self.ax.scatter([x], [y], [z + 0.6], s=320, facecolors="none", edgecolors=RED, linewidths=2.4,
                                        depthshade=False))
        self.dyn.append(self.ax.scatter([x], [y], [z + 0.6], s=60, color=RED, depthshade=False))
        self.dyn += self.ax.plot([x, x + 1.5], [y, y - 1.0], [z + 0.9, z + 3.0], color=INK2, lw=0.8)
        self.dyn.append(self.ax.text(x + 1.6, y - 1.0, z + 3.2, text, color=INK, fontsize=9, weight="bold"))

    def show(self, trace, u0, u1, hero=None, tail=18, plans="hero", memory=None, hit=None, hero_plan=None,
             radio=True, full_trails=False, mem_alpha=0.42):
        """Draw slot u1, with the radio deliveries of slots (u0, u1]. memory: tiles {key: mask} shown as orange voxels;
        hit: cells drawn red (a real obstacle on the followed drone's plan); hero_plan overrides the drawn plan."""
        self.clear()
        ax, s = self.ax, trace[u1]
        P = np.array(s["pos"]) + 0.5
        n = len(P)
        others = [i for i in range(n) if i != hero]
        for i in others:
            hist = np.array([trace[u]["pos"][i] for u in range(0 if full_trails else max(0, u1 - tail), u1 + 1)]) + 0.5
            self.dyn += ax.plot(hist[:, 0], hist[:, 1], hist[:, 2], color=MUTED, lw=0.9, alpha=0.6)
        if plans == "all":
            for i in others:
                if not s["arrived"][i] and len(s["plan"][i]) > 1:
                    q = np.array(s["plan"][i]) + 0.5
                    self.dyn += ax.plot(q[:, 0], q[:, 1], q[:, 2], color=MUTED, lw=0.9, ls=(0, (3, 2)))
        if radio:
            data, chat = [], []
            for u in range(u0 + 1, u1 + 1):
                pu = np.array(trace[u]["pos"]) + 0.5
                for i, j in trace[u]["edges"]:
                    (data if trace[u]["map_tx"][i] else chat).append([pu[i], pu[j]])
            for segs, col, lw, al in ((chat, MUTED, 0.6, 0.35), (data, ORANGE, 1.3, 0.85)):
                if segs:
                    lc = Line3DCollection(segs, colors=col, linewidths=lw, alpha=al)
                    ax.add_collection3d(lc, autolim=False)
                    self.dyn.append(lc)
        if memory:
            self._voxels([c for key, mask in memory.items() for c in cells_of(key, mask)], ORANGE, mem_alpha)
        if hit:
            self._voxels(hit, RED, 1.0)
            self.label(hit[0], "forgotten obstacle")
        self.dyn.append(ax.scatter(P[others, 0], P[others, 1], P[others, 2], s=20, color=MUTED, depthshade=False,
                                   edgecolors=SURF, linewidths=0.6))
        if hero is not None:
            hist = np.array([trace[u]["pos"][hero] for u in range(u1 + 1)]) + 0.5
            self.dyn += ax.plot(hist[:, 0], hist[:, 1], hist[:, 2], color=BLUE, lw=2)
            q = hero_plan if hero_plan is not None else (s["plan"][hero] if not s["arrived"][hero] else [])
            if len(q) > 1:
                q = np.array(q) + 0.5
                self.dyn += ax.plot(q[:, 0], q[:, 1], q[:, 2], color=BLUE, lw=1.6, ls=(0, (4, 2)))
            h = hist[-1]
            self.dyn.append(ax.scatter([h[0]], [h[1]], [h[2]], s=70, color=BLUE, depthshade=False, edgecolors=SURF,
                                       linewidths=1.4))


def grab(fig):
    fig.canvas.draw()
    return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())


def save_gif(frames, ms, path, colors=112):
    sample = Image.fromarray(np.concatenate([np.asarray(f) for f in frames[::max(1, len(frames) // 8)]], axis=1))
    pal = sample.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)
    gif = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    gif[0].save(path, save_all=True, append_images=gif[1:], duration=ms, loop=0, disposal=1)


def busiest(tr, a, b, w):
    """The slot in [a, b] whose last w slots carried the most obstacle-map deliveries."""
    def n(u):
        return sum(1 for v in range(max(0, u - w) + 1, u + 1) for i, _ in tr[v]["edges"] if tr[v]["map_tx"][i])
    return max(range(a, b + 1), key=n)


def plan_cells(ev):
    cells = segment(ev["prev"], ev["plan"][0])[ev["k"]:]
    for a, b in zip(ev["plan"], ev["plan"][1:]):
        cells += segment(a, b)
    return cells


# ---------------------------------------------------------------------------------------------- the swarm video
def swarm_video(rec, blocked, start, goal, hero, ev):
    tr = rec["trace"]
    T, tf, tarr = len(tr) - 1, ev["t"], int(rec["arrived_at"][hero])
    hit = [c for c in plan_cells(ev) if blocked[c]]
    fig = plt.figure(figsize=(9.6, 5.6), dpi=90)
    ax = fig.add_axes([0.0, 0.10, 1.0, 0.84], projection="3d", computed_zorder=False)
    sc = Scene(ax, blocked, start, goal, zoom=1.24)
    insets = [fig.add_axes([0.66, 0.63, 0.15, 0.27]), fig.add_axes([0.83, 0.63, 0.15, 0.27])]
    for a in insets:
        a.set_visible(False)
    hud = fig.text(0.02, 0.955, "", fontsize=10, color=INK2)
    cap = fig.text(0.5, 0.035, "", fontsize=12.5, color=INK, ha="center", va="bottom", linespacing=1.35)
    gauge = [fig.add_artist(Rectangle((0.775 + 0.016 * i, 0.93), 0.013, 0.026, transform=fig.transFigure, lw=0))
             for i in range(12)]
    glab = fig.text(0.775, 0.895, "", fontsize=9, color=INK2)
    story = [  # (first slot, last slot, slots per frame, ms per frame, caption, options)
        (0, 0, 1, 2800, "Ten drones must fly from A to B through an unknown 3-D world. None of them has a map.", {}),
        (1, 34, 2, 190, "They take off ten seconds apart. Each sees only a few metres ahead, and they talk by radio:\n"
                        "orange links carry obstacle maps, grey ones are short position messages.", {}),
        (35, 60, 1, 150, "Follow the blue drone. Its whole map is the 12 tiles it holds (orange):\n"
                         "with everything else it remembers, 228 bytes of memory.", dict(memory=True)),
        (61, 72, 1, 300, "Every drone plans its own shortest route through what it knows (dashed),\n"
                         "treating unknown space as free.", dict(memory=True, plans="all")),
        (73, tf - 1, 1, 300, "Teammates radio an obstacle only if it is worth its bytes to someone's route.\n"
                             "When the blue drone receives one, its map grows.", dict(memory=True)),
        ("futile",),
        (tf + 1, tarr, 2, 120, "It escapes, then re-plans around every obstacle it meets, and flies on.",
         dict(memory=True)),
        (tarr, tarr, 1, 1400, "The blue drone has arrived.", {}),
        (tarr + 1, T, 3, 110, "The others arrive too, each on a route of its own.", dict(full=True)),
        (T, T, 1, 4500, "All ten arrived. Every drone used 228 bytes of memory and under 1 ms of computing per second,\n"
                        "and no GPS coordinate ever went on the radio.", dict(full=True)),
    ]
    frames, ms = [], []

    def hud_for(u, memory):
        s = tr[u]
        hud.set_text(f"slot {u}  ·  {sum(s['arrived'])} of 10 arrived  ·  team has sent {s['bytes'] / 1000:.1f} kB")
        n = len(memory) if memory is not None else 0
        for i, r in enumerate(gauge):
            r.set_visible(memory is not None)
            r.set_facecolor(ORANGE if i < n else GRID)
        glab.set_text(f"blue drone's map: {n} of 12 tiles" if memory is not None else "")

    for k, scene in enumerate(story):
        if scene[0] == "futile":
            u = tf
            hud_for(u, None)
            sc.show(tr, u - 1, u - 1, hero=hero, memory=ev["tiles"], hit=hit, hero_plan=[tr[u - 1]["pos"][hero]] +
                    [list(w) for w in ev["plan"]], radio=False, mem_alpha=0.12)
            slices(insets, blocked, ev, tr[u - 1]["pos"][hero], ("what it holds", "the world"), size=7.5)
            for a in insets:
                a.set_visible(True)
            cap.set_text(f"Its plan costs Φ = {ev['phi']:.1f}, less than the lower bound L = {ev['L']:.1f} it has proved.\n"
                         "So the plan must cross a real obstacle: one it held a second ago and evicted (red). It escapes.")
            ax.view_init(elev=32, azim=-58 + 34 * k / len(story))
            frames.append(grab(fig)), ms.append(6500)
            for a in insets:
                a.clear(), a.set_visible(False)
            continue
        a, b, step, dur, text, opt = scene
        cap.set_text(text)
        for u in range(a, b + 1, step):
            mem = tr[u]["tiles"][hero] if opt.get("memory") and not tr[u]["arrived"][hero] else None
            hud_for(u, mem)
            sc.show(tr, max(0, u - step), u, hero=hero, memory=mem, plans=opt.get("plans", "hero"),
                    full_trails=opt.get("full", False))
            ax.view_init(elev=32, azim=-58 + 34 * (k + (u - a) / max(1, b - a + 1)) / len(story))
            frames.append(grab(fig)), ms.append(dur)
    plt.close(fig)
    OUT.mkdir(parents=True, exist_ok=True)
    save_gif(frames, ms, OUT / "grace3d_swarm.gif")
    return len(frames), sum(ms) / 1000


# ---------------------------------------------------------------------------------------------- the comparison video
def compare_video(recs, blocked, start, goal, step=4):
    fig = plt.figure(figsize=(12, 6.6), dpi=85)
    keys = list(PROTOCOLS)
    scenes, titles = {}, {}
    for n, key in enumerate(keys):
        r, c = divmod(n, 3)
        ax = fig.add_axes([c / 3, 0.50 - 0.47 * r, 1 / 3, 0.40], projection="3d", computed_zorder=False)
        scenes[key] = Scene(ax, blocked, start, goal)
        fig.text(c / 3 + 0.02, 0.935 - 0.47 * r, PROTOCOLS[key][0], fontsize=11, weight="bold",
                 color=BLUE if key == "v2" else INK)
        titles[key] = fig.text(c / 3 + 0.02, 0.905 - 0.47 * r, "", fontsize=9, color=INK2)
    prog = fig.add_axes([2 / 3 + 0.06, 0.1, 0.17, 0.30])
    fig.text(2 / 3 + 0.015, 0.43, "Radio traffic so far", fontsize=11, weight="bold")
    fig.text(0.5, 0.012, "Same terrain, radio and seed for all five protocols. Dots: drones, with their last 18 s of "
             "flight. Orange links carry obstacle maps; grey links, short position messages.", fontsize=10,
             color=INK2, ha="center")
    T = max(len(r["trace"]) for r in recs.values()) - 1
    frames, ms = [], []
    for u in list(range(0, T + 1, step)) + [T]:
        for key in keys:
            tr = recs[key]["trace"]
            v = min(u, len(tr) - 1)
            scenes[key].show(tr, max(0, v - step), v, full_trails=u == T)
            scenes[key].ax.view_init(elev=32, azim=-58 + 20 * u / T)
            titles[key].set_text(f"{sum(tr[v]['arrived'])} of 10 arrived  ·  {tr[v]['bytes'] / 1000:.1f} kB sent"
                                 + (f"  ·  regret {recs[key]['regret']:.2f}" if u == T else ""))
        kb_chart(prog, recs, upto=u)
        frames.append(grab(fig)), ms.append(110 if u < T else 6000)
    plt.close(fig)
    save_gif(frames, ms, OUT / "grace3d_vs.gif", colors=96)
    return len(frames)


# ---------------------------------------------------------------------------------------------- paper figures
def fig_strip(rec, blocked, start, goal, hero, ev):
    tr = rec["trace"]
    T, tf = len(tr) - 1, ev["t"]
    hit = [c for c in plan_cells(ev) if blocked[c]]
    panels = [("a", busiest(tr, 36, 56, 6), "Ten seconds apart, they take off. Orange links carry\n"
                                            "obstacle maps (last 6 s); grey links, position messages.", dict(window=6)),
              ("b", 66, "Each drone plans through what it knows (dashed).\nThe blue drone's whole map: 12 tiles (orange).",
               dict(plans="all", memory=True)),
              ("c", tf, f"Its plan costs Φ = {ev['phi']:.1f} < L = {ev['L']:.1f}, the bound it proved:\n"
                        "the plan crosses an obstacle it forgot (red). It escapes.", dict(futile=True)),
              ("d", T, "All ten arrive, each on a route of its own\n(blue: the drone followed in (a)–(c)).",
               dict(full=True))]
    fig = plt.figure(figsize=(7.2, 6.4))
    for n, (tag, u, text, opt) in enumerate(panels):
        r, c = divmod(n, 2)
        ax = fig.add_axes([c / 2, 0.5 - 0.5 * r + 0.0, 0.5, 0.40], projection="3d", computed_zorder=False)
        sc = Scene(ax, blocked, start, goal)
        if opt.get("futile"):
            sc.show(tr, u - 1, u - 1, hero=hero, memory=ev["tiles"], hit=hit, radio=False,
                    hero_plan=[tr[u - 1]["pos"][hero]] + [list(w) for w in ev["plan"]])
        else:
            mem = tr[u]["tiles"][hero] if opt.get("memory") else None
            sc.show(tr, max(0, u - opt.get("window", 3)), u, hero=hero, memory=mem, plans=opt.get("plans", "hero"),
                    full_trails=opt.get("full", False))
        ax.view_init(elev=32, azim=-58)
        fig.text(c / 2 + 0.02, 0.965 - 0.5 * r, f"({tag}) slot {u}", fontsize=9, weight="bold")
        fig.text(c / 2 + 0.02, 0.905 - 0.5 * r, text, fontsize=8, color=INK2, va="bottom")
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"swarm_strip.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def slices(axs, blocked, ev, pos, titles, size=8.5):
    """Top view at the height where a futile plan meets the obstacle: left, what the drone holds; right, the world."""
    cells = plan_cells(ev)
    hit = [c for c in cells if blocked[c]]
    z = hit[0][2]
    path = np.array([pos] + cells)
    held = {c for key, mask in ev["tiles"].items() for c in cells_of(key, mask)}
    xs, ys = path[:, 0], path[:, 1]
    x0, x1 = max(0, xs.min() - 7), min(blocked.shape[0], xs.max() + 4)
    y0, y1 = max(0, ys.min() - 9), min(blocked.shape[1], ys.max() + 4)
    for ax, title, truth in ((axs[0], titles[0], False), (axs[1], titles[1], True)):
        for x in range(x0, x1):
            for y in range(y0, y1):
                c = (x, y, z)
                col = (RED if c in hit else BASE) if truth and blocked[c] else (ORANGE if not truth and c in held
                                                                                  else None)
                if col:
                    ax.add_patch(Rectangle((x, y), 1, 1, color=col, lw=0))
        ax.plot(path[:, 0] + 0.5, path[:, 1] + 0.5, color=BLUE, lw=1.6, ls=(0, (4, 2)))
        ax.scatter([pos[0] + 0.5], [pos[1] + 0.5], s=50, color=BLUE, edgecolors=SURF, linewidths=1.2, zorder=3)
        ax.set_xlim(x0, x1), ax.set_ylim(y0, y1), ax.set_aspect("equal")
        ax.set_title(title, fontsize=size + 1, loc="left", color=INK)
        ax.set_xticks([]), ax.set_yticks([])
        for sp in ax.spines.values():
            sp.set_visible(True), sp.set_color(BASE)
    c = hit[0]
    axs[1].annotate("forgotten\nobstacle", (c[0] + 0.5, c[1] + 0.5), xytext=(c[0] - 7, c[1] - 6), fontsize=size,
                    color=INK, arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    return z, x0, y0


def fig_futility(rec, blocked, hero, ev):
    tr, tf = rec["trace"], ev["t"]
    pos = tr[tf - 1]["pos"][hero]
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.2))
    z, x0, y0 = slices(axs, blocked, ev, pos, ("What the blue drone holds (its map, orange)",
                                               "The world (ground truth, grey)"))
    axs[0].annotate(f"plan cost Φ = {ev['phi']:.1f}\nproved bound L = {ev['L']:.1f}\nΦ < L: the plan must cross a real "
                    "obstacle", (pos[0] + 0.5, pos[1] + 0.5), xytext=(x0 + 1, y0 + 1.2), fontsize=8.5, color=INK)
    fig.text(0.01, 0.01, f"Horizontal slice at height z = {z}, slot {tf}. Dashed: the plan, projected from above.",
             fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"futility.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def kb_chart(ax, recs, upto=None, labels=True):
    """Cumulative kB on the radio against time, one line per protocol, GRACE-3D emphasised, every line labelled."""
    ax.clear()
    for key in list(PROTOCOLS)[::-1]:
        tr = recs[key]["trace"]
        v = len(tr) - 1 if upto is None else min(upto, len(tr) - 1)
        kb = [s["bytes"] / 1000 for s in tr[:v + 1]]
        own = key == "v2"
        ax.plot(range(v + 1), kb, color=BLUE if own else MUTED, lw=2 if own else 1.2)
        if labels:
            short = {"v2": "GRACE-3D", "grace": "GRACE port", "cairn": "CAIRN", "flood": "Flooding", "none": "Silence"}
            ax.text(v + 10, max(kb[-1], 0.13), f"{short[key]} {kb[-1]:.0f}" if kb[-1] >= 10 else
                    f"{short[key]} {kb[-1]:.1f}", fontsize=7, va="center", color=INK if own else INK2,
                    weight="bold" if own else None, zorder=4,
                    bbox=dict(boxstyle="square,pad=0.15", fc=SURF, ec="none"))   # halo: lines pass behind labels
    ax.set_yscale("symlog", linthresh=1), ax.set_ylim(0, 400), ax.set_xlim(0, 500)
    ax.set_yticks([0, 1, 10, 100]), ax.set_yticklabels(["0", "1", "10", "100"])
    ax.set_xlabel("slot (1 s)", fontsize=8), ax.set_ylabel("kB sent by the team", fontsize=8)
    ax.tick_params(labelsize=7.5)
    ax.grid(axis="y", color=GRID, lw=0.6)


def fig_compare(recs, blocked, start, goal, u=120):
    keys = list(PROTOCOLS)
    fig = plt.figure(figsize=(7.2, 5.0))
    for n, key in enumerate(keys):
        r, c = divmod(n, 3)
        tr = recs[key]["trace"]
        v = min(u, len(tr) - 1)
        ax = fig.add_axes([c / 3, 0.47 - 0.47 * r, 1 / 3, 0.40], projection="3d", computed_zorder=False)
        Scene(ax, blocked, start, goal).show(tr, max(0, v - 5), v, tail=30)
        ax.view_init(elev=32, azim=-58)
        fig.text(c / 3 + 0.015, 0.955 - 0.47 * r, PROTOCOLS[key][0], fontsize=9, weight="bold",
                 color=BLUE if key == "v2" else INK)
        fig.text(c / 3 + 0.015, 0.925 - 0.47 * r, f"slot {u}: {sum(tr[v]['arrived'])}/10 arrived, {tr[v]['bytes'] / 1000:.1f} kB",
                 fontsize=7.5, color=INK2)
        fig.text(c / 3 + 0.015, 0.898 - 0.47 * r, f"end: {sum(tr[-1]['arrived'])}/10 arrived, regret {recs[key]['regret']:.2f}",
                 fontsize=7.5, color=INK2)
    ax = fig.add_axes([2 / 3 + 0.06, 0.1, 0.15, 0.30])
    kb_chart(ax, recs)
    fig.text(2 / 3 + 0.015, 0.425, "Radio traffic (kB), whole mission", fontsize=9, weight="bold")
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"compare.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def fig_results():
    res = json.loads((ROOT / "benchmarks" / "v2_results.json").read_text(encoding="utf-8"))
    d = res["v2_minus_other_regret"]
    fig, a2 = plt.subplots(figsize=(6.2, 3.0))
    rows = [("Other protocols", None), ("Silence", "none@120"), ("Flooding", "flood@120"),
            ("GRACE ported to 3-D", "grace@120"), ("CAIRN v0.4", "cairn@120"), ("Without one component", None),
            ("no radio", "v2-silent"), ("no random escape", "v2-noescape"), ("oldest-first forgetting", "v2-fifo"),
            ("box-visibility planner", "v2-bvp"), ("no futility (F4)", "v2-nofutile")]
    ys = np.arange(len(rows))[::-1]
    for y, (name, key) in zip(ys, rows):
        if key is not None:
            m, se = d[key]["mean"], d[key]["se"]
            a2.plot([m - 1.96 * se, m + 1.96 * se], [y, y], color=BLUE, lw=1.6)
            a2.scatter([m], [y], s=30, color=BLUE, edgecolors=SURF, linewidths=1, zorder=3)
    a2.axvline(0, color=BASE, lw=1)
    a2.set_yticks(ys, [name for name, _ in rows])
    for tick, (_, key) in zip(a2.get_yticklabels(), rows):
        tick.set_color(INK if key is None else INK2)
        tick.set_fontweight("bold" if key is None else "normal")
    a2.tick_params(axis="y", length=0, labelsize=8)
    a2.set_xlim(-4.0, 1.0), a2.spines["left"].set_visible(False)
    a2.set_xlabel("regret of GRACE-3D 2.0.0 minus regret of the other\n(paired over 20 terrains, ±1.96 s.e.)")
    a2.text(-3.95, len(rows) - 0.4, "← GRACE-3D better", fontsize=7.5, color=MUTED)
    a2.text(0.95, len(rows) - 0.4, "worse →", fontsize=7.5, color=MUTED, ha="right")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"results.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def fig_budget(cycles):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 3.6), gridspec_kw=dict(height_ratios=[0.8, 1.6]))
    parts = [("map: 12 tiles", 120), ("teammates", 36), ("walk record", 24), ("plan", 16), ("tile send ages", 12),
             ("bounds, pose, filter", 20)]
    cols = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6"]
    a1.barh([0], [256], color=GRID, height=0.5)
    x = 0
    for (name, b), col in zip(parts, cols):
        a1.barh([0], [b - 1], left=x, color=col, height=0.5)
        if b >= 20:
            a1.text(x + 2, 0, f"{b}", color="#0b0b0b" if col == "#eda100" else "#ffffff", va="center", fontsize=8)
        x += b
    a1.text(256, 0.42, "256 B limit", ha="right", fontsize=8, color=INK2)
    a1.text(228 + 2, 0, "28 B free", va="center", fontsize=8, color=INK2)
    a1.set_xlim(0, 256), a1.set_yticks([]), a1.set_xticks([0, 64, 128, 192, 256])
    a1.spines["left"].set_visible(False)
    a1.set_title("(a) Persistent memory of one drone: 228 of 256 bytes", fontsize=9, loc="left")
    a1.legend(handles=[Rectangle((0, 0), 1, 1, color=col) for col in cols],
              labels=[f"{n} ({b} B)" for n, b in parts], ncol=3, fontsize=7.5, frameon=False,
              loc="upper center", bbox_to_anchor=(0.5, -0.45))
    a2.hist(cycles / 1000, bins=np.linspace(0, BUDGET / 1000, 85), color=BLUE, log=True)
    a2.axvline(0.7 * BUDGET / 1000, color=INK2, lw=1, ls=(0, (2, 2)))
    a2.axvline(BUDGET / 1000, color=INK, lw=1.2, ls=(0, (5, 2)))
    a2.text(0.7 * BUDGET / 1000 - 2, a2.get_ylim()[1] * 0.3, "search stops\n(70 %)", ha="right", fontsize=8,
            color=INK2)
    a2.text(BUDGET / 1000 - 2, a2.get_ylim()[1] * 0.3, "1 ms budget", ha="right", fontsize=8, color=INK)
    a2.set_xlim(0, BUDGET / 1000 + 2)
    a2.set_xlabel("thousand cycles in one slot (operation-count model, 168 MHz Cortex-M4)")
    a2.set_ylabel("agent-slots")
    a2.set_title(f"(b) Computing per slot, 5 missions, {len(cycles):,} agent-slots: maximum {cycles.max():,.0f} cycles",
                 fontsize=9, loc="left")
    fig.tight_layout(h_pad=2.2)
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"budget.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def fig_planes():
    """The two main planes, from benchmarks/v2_sweep.json: every point is a mean over the same 20 terrains, with
    ±1 standard error over those paired seeds on each axis that varies."""
    pts = json.loads((ROOT / "benchmarks" / "v2_sweep.json").read_text(encoding="utf-8"))["points"]
    style = {"GRACE-3D 2.0.0": (BLUE, "o", 6.5), "GRACE ported to 3-D": (MUTED, "s", 5),
             "CAIRN v0.4": (MUTED, "D", 4.5), "Flooding": (MUTED, "^", 5.5), "Silence": (MUTED, "v", 5.5)}
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.3))
    for name, (col, mk, size) in style.items():
        own = name.startswith("GRACE-3D")
        for ax, study, xk, yk in ((a1, "front", "kB", "regret"), (a2, "memory", "memory_bytes", "arrival")):
            p = [q for q in pts if q["study"] == study and q["protocol"] == name]
            if not p:
                continue
            p.sort(key=lambda q: q[xk][0] if xk == "kB" else q[xk])
            x = np.array([q[xk][0] if xk == "kB" else q[xk] for q in p])
            xe = np.array([q[xk][1] for q in p]) if xk == "kB" else None
            y, ye = np.array([q[yk][0] for q in p]), np.array([q[yk][1] for q in p])
            ax.errorbar(x, y, xerr=xe, yerr=ye, color=col, marker=mk, ms=size, lw=2 if own else 1.1,
                        elinewidth=1, capsize=0, mfc=col if own or study == "front" else SURF, mec=col if not own
                        else SURF, mew=1 if own else 1.2, zorder=3 if own else 2, label=name)
            if study == "front" and not own:   # direct labels; in (b) the baselines end together: a legend
                ax.annotate(name, (x[-1], y[-1]), xytext=(6, 3), textcoords="offset points", fontsize=7.5,
                            color=INK2)
            if own and study == "front":       # name above the cheap end of the price sweep
                lo = min(range(len(p)), key=lambda k: x[k])
                ax.annotate(name, (x[lo], y[lo]), xytext=(-4, 14), textcoords="offset points", fontsize=7.5,
                            color=INK, weight="bold", ha="right")
            if own and study == "memory":
                for q, xi, yi in zip(p, x, y):
                    ax.annotate(f"K={q['value']}", (xi, yi), xytext=(7, -4), textcoords="offset points",
                                fontsize=6.5, color=MUTED, ha="left")
    a1.set_xscale("symlog", linthresh=1), a1.set_xlim(-0.3, 400)
    a1.set_xticks([0, 1, 10, 100]), a1.set_xticklabels(["0", "1", "10", "100"])
    a1.set_xlabel("kB on the radio per mission"), a1.set_ylabel("regret (mean ± s.e.)")
    a1.set_title("(a) Detour against bytes", fontsize=9, loc="left")
    a2.axvline(256, color=INK2, lw=1, ls=(0, (4, 2)))
    a2.text(252, 0.72, "256 B\nbudget", ha="right", fontsize=7.5, color=INK2)
    a2.legend(loc="lower right", fontsize=7, frameon=False)
    a2.set_xlabel("persistent bytes per drone")
    a2.set_ylabel("arrival within 500 slots (mean ± s.e.)")
    a2.set_ylim(0, 1.04), a2.set_xlim(40, 330)
    a2.set_title("(b) Arrival against memory", fontsize=9, loc="left")
    for ax in (a1, a2):
        ax.grid(axis="y", color=GRID, lw=0.6)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(FIG / f"planes.{ext}", dpi=170 if ext == "png" else None)
    plt.close(fig)


def replay_json(rec, blocked, start, goal, hero, ev, seed):
    tr = rec["trace"]
    steps = [dict(pos=s["pos"], edges=[[i, j, int(s["map_tx"][i] > 0)] for i, j in s["edges"]],
                  plan=s["plan"][hero] if not s["arrived"][hero] else [], tiles=sorted(list(k) for k in s["tiles"][hero]),
                  arrived=s["arrived"]) for s in tr]
    rep = dict(seed=seed, hero=hero, shape=list(blocked.shape), start=list(map(int, start)), goal=list(map(int, goal)),
               blocked=np.argwhere(blocked).tolist(), regret=round(rec["regret"], 4), bytes=rec["bytes"],
               futile=dict(t=ev["t"], phi=round(ev["phi"], 2), L=round(ev["L"], 2),
                           plan=[tr[ev["t"] - 1]["pos"][hero]] + [list(w) for w in ev["plan"]],
                           hit=[list(c) for c in plan_cells(ev) if blocked[c]]), steps=steps)
    (ROOT / "docs" / "replay3d.json").write_text(json.dumps(rep, separators=(",", ":")), encoding="utf-8")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--only", choices=["figures", "swarm", "vs", "planes"], default=None)
    ap.add_argument("--workers", type=int, default=5)
    a = ap.parse_args()
    FIG.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    if a.only == "planes":                   # needs benchmarks/v2_sweep.json only
        fig_planes()
        sys.exit()
    if a.only == "swarm":                    # one mission, one process
        recs = dict([record(("v2", a.seed))])
    else:
        with Pool(a.workers) as pool:
            recs = dict(pool.map(record, [(k, a.seed) for k in PROTOCOLS]))
            if a.only != "vs":
                cycles = np.concatenate([r["cycles"] for _, r in pool.map(record, [("v2", s) for s in range(5)])])
    blocked, start, goal, _ = make_world(Mission(), np.random.default_rng(a.seed))
    sw = recs["v2"]
    ev = next(e for e in sw["futile"] if math.isfinite(sw["arrived_at"][e["id"]]))
    hero = ev["id"]
    out = dict(seed=a.seed, hero=hero, futile_slot=ev["t"], regret=round(sw["regret"], 3))
    if a.only in (None, "figures"):
        fig_strip(sw, blocked, start, goal, hero, ev)
        fig_futility(sw, blocked, hero, ev)
        fig_compare(recs, blocked, start, goal)
        fig_results()
        fig_budget(cycles)
        if (ROOT / "benchmarks" / "v2_sweep.json").exists():
            fig_planes()
        replay_json(sw, blocked, start, goal, hero, ev, a.seed)
    if a.only in (None, "swarm"):
        out["swarm_frames"], out["swarm_seconds"] = swarm_video(sw, blocked, start, goal, hero, ev)
    if a.only in (None, "vs"):
        out["vs_frames"] = compare_video(recs, blocked, start, goal)
    for f in sorted(OUT.glob("*.gif")):
        out[f.name] = round(f.stat().st_size / 1e6, 2)
    print(json.dumps(out))
