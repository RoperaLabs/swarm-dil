# swarm-dil

**Coordination and communication of memory- and bandwidth-limited UAV teams over degraded (DIL: disconnected, intermittent, limited) links.**

| Version | Question | Main result |
|---|---|---|
| **v0.3** (this) | Which information should agents with tiny memories exchange, in how many bytes and how often, to cross unknown terrain from A to B? | **GRACE**, a protocol derived from the receivers' navigation decisions: same detour as flooding with **~3× fewer bytes**, and unlimited-memory performance with **64 bytes of map memory** |
| [v0.2](../../tree/v0.2.0) | Which coordination architecture survives a realistic radio, terrain, interference and bounded memory in an area search? | A single ridge makes central control 2.2× slower than a mesh |
| [v0.1](../../tree/v0.1.0) | How much does message loss cost a search mission? | Bursty loss of 50 % makes central control 3.6× slower |

# v0.3: GRACE, telling teammates only what changes their decision

📄 **[Technical report (PDF, 12 pages): model, theorems and proofs](docs/theory/theory.pdf)** · 🧪 **[Interactive explainer](https://roperalabs.github.io/swarm-dil/)** (try the value-of-information map, the burst-loss calculator and a recorded mission)

![Bytes against detour: GRACE, flooding and silence](docs/theory/fig/pareto.png)

## Problem

A team of 8 agents must go from **A** to **B** across terrain none of them knows, with obstacles, dead-end pockets and radio interference. Each agent has:

- a **vision cone** (5 cells, ±60°) with line of sight, so it only ever sees obstacle surfaces;
- a **map memory** of a few hundred bytes at most;
- a **short, fading, shared radio**: path loss, obstacle blockage, seconds-long fades, interferers and a 9.6 kbit/s channel.

Agents are launched one after another, so what the first ones discover can spare the others a detour. **Each agent runs the protocol on board.** The only things it ever receives are its own cone readings, whether its last move succeeded, and the raw bytes teammates broadcast. Its position, map, teammates' positions and knowledge, and link quality are all estimated internally ([`agent.py`](src/swarm_dil/nav/agent.py) never imports the world, and a test checks it).

**Goal:** the least team detour (*regret*: extra distance over the true shortest path) per byte sent.

## Design: derived, not copied

Instead of adapting an existing C2 protocol, every rule of GRACE comes from a result in the [report](docs/theory/theory.pdf):

| Result | Statement | What GRACE does with it |
|---|---|---|
| Lemma 1 | Under optimistic planning, learning that a cell is free never changes an action | Free space is never stored or sent |
| Prop. 1 | A report about an obstacle at path distance $s$ saves at most $2(s-r)^+$ ($r$ = sensor range), with equality in dead ends | Obstacles just ahead of a teammate are worthless; the bottom of a pocket far ahead is worth the most |
| Lemma 3 | Cells outside the octile ellipse $h(x,y)+h(y,B)\le d_K(x)$ cannot be on the current plan | Forget the tiles farthest outside the ellipse first (Cor. 1: forgetting them is safe) |
| Def. 3, Prop. 2–3 | Plan-equivalence closure, and an ellipse prior $e^{-m/\mu}$ with self-calibrated $\mu$ | Value tiles a teammate will hit after it reroutes, too |
| Props. 4–5 | Over a Gilbert–Elliott link, $k$ copies spaced $\tau_m$ are all lost with probability $\pi_B\prod_m(\pi_B+\pi_G\lambda^{\tau_m})$ | No acknowledgements: keep a belief that each teammate lacks each tile and update it per copy, using link reciprocity |
| Prop. 6 | The optimal number of copies grows with $\log v$ | Important tiles are repeated a few times, trivial ones never sent |
| Theorem 1 | The per-slot message with the top tiles by expected value above a price $\kappa$ is optimal | One knob, $\kappa$ (detour per byte), traces the bytes–detour front |
| Cor. 3 | For one-cell-thick sets (what a range sensor sees), the byte-optimal tile side is $\sqrt{8\,b_{\mathrm{id}}}=4$ | 4×4 tiles: 2-byte id plus 16-bit mask, 1 byte per sensed cell; merged by bitwise OR, so duplicates and reordering are harmless |

Each closed form is checked numerically in [`tests/test_nav.py`](tests/test_nav.py): Monte Carlo of the burst-loss chain, brute force for $k^\star$, exact tile counts, and the ellipse lemma and value bound on random terrains.

## Results (20 random terrains per point, error bars = 1 s.e.)

| Comparison | Result |
|---|---|
| Same detour as tile flooding | **2.9–3.3× fewer bytes** (paired difference in regret −0.001 ± 0.013 at 9.6 vs 27.3 kB) |
| Against flooding raw cells | **~6× fewer bytes** for the same detour |
| Detour removed per kB (vs silence) | GRACE 0.013–0.026, tile flooding 0.002–0.007 |
| Lowest detour | Flooding at 64 B/slot: 0.181 with 61 kB; GRACE: 0.199 with 12.7 kB (5.1× fewer bytes) |
| **64 bytes of map memory per agent** | GRACE keeps its unlimited-memory regret (0.220 vs 0.212). Flooding degrades to 0.59–2.13 (worse than silence, 18 % of agents lost) because irrelevant tiles evict useful ones; GRACE with oldest-first forgetting reaches 0.382 |
| Two interferers at 0 dBm | GRACE matches 16 B/slot flooding (0.286 vs 0.287) with 2.6× fewer bytes, and **spends less by itself** as links degrade (9.6 → 7.2 kB) |

![Bounded memory](docs/theory/fig/memory.png)

**What each component buys, honestly.**
- **Loss model:** without it the protocol never repairs a loss and stalls at a regret of 0.253, whatever its budget.
- **Relevance-aware forgetting:** decisive under tight memory.
- **Goal relevance for *transmission*:** in this scenario its front lies on GRACE's own front within error bars. A short convoy along similar routes makes most tiles relevant to someone, so we do not claim a gain there yet.

Full table: [`results/v3/summary.md`](results/v3/summary.md). Raw data: [`results/v3/study.csv`](results/v3/study.csv).

## Limitations

- **World model:** 2-D grid with static obstacles, perfect localisation and identical agents. The link memory $\lambda$ is a fixed design constant.
- **Theory assumptions:** the ellipse prior assumes an exponential cost rise, and the value bound assumes detection within path distance $r$ ahead. Bounded memory voids the completeness guarantee.
- **Simulated radio:** the next step is to run the same agent code over Zenoh with `tc netem` and in PX4 SITL.
- **On-board compute:** relevance prediction runs one shortest-path search per teammate per round. Fine on a 48×48 grid; it needs incremental updates to scale up.

## Reproduce

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[plot,test]"
pytest                                   # 31 tests
python experiments/v3_study.py           # 1,520 missions, ~45 min on 6 cores (resumable)
python experiments/v3_study.py --plot    # figures and tables only
tectonic docs/theory/theory.tex          # rebuild the PDF (any LaTeX engine works)
```

Every mission is seeded, so results are bit-for-bit reproducible.

```python
from swarm_dil.nav import Mission, run
o = run(Mission(protocol="grace", price=0.2, memory_tiles=16), seed=0)
o.regret, o.bytes_sent.sum(), o.arrival_rate
```

---

# v0.2: area search under a realistic radio

![Time until C2 knows all targets against interference, bandwidth and memory](results/fig_v02.png)

In this study, 20 UAVs search 1 km² with obstacles, a physical radio model ([`radio.py`](src/swarm_dil/radio.py): path loss, correlated shadowing, line-of-sight blockage, interferers, contention), byte budgets and bounded memory. They are compared under three coordination schemes ([`sim.py`](src/swarm_dil/sim.py)). Median time until C2 knows all 10 targets:

| Condition | centralised | decentralised | partition |
|---|---|---|---|
| Clear radio | 570 s | **260 s** | 268 s |
| + 30 % of UAVs lost | 674 s | **283 s** | 588 s |
| + 20 dBm field interferer | 702 s | 324 s | **282 s** |
| UAVs remember ≤ 25 % of the map | 570 s | **never** | 268 s |

Terrain, not packet loss, breaks central control. Losing vehicles breaks pre-planning. Bounded memory is a cliff for greedy mesh search, and that cliff motivated v0.3's relevance-aware memory. Reproduce with `python experiments/sweep.py`; tables in [`results/summary.md`](results/summary.md).

### Watch a run in TAK

`python experiments/stream_cot.py --strategy decentralised --speedup 10` replays a v0.2 mission as **Cursor on Target**. It sends to the TAK situational-awareness multicast group (`239.2.3.1:6969`), so ATAK, WinTAK or iTAK on the same LAN display it. Coordinates are synthetic.

## Responsible use

This is open, basic research by Ropera Labs on information-efficient coordination of autonomous systems over degraded networks, aimed at dual-use applications such as search and rescue, emergency response and infrastructure inspection. Interference is modelled as a generic raised noise floor, not as an attack technique. The project uses only synthetic data and contains no weapon, targeting or vehicle-specific control code. We believe it falls under the basic-research and public-domain exemptions of Regulation (EU) 2021/821. Please use it in line with international humanitarian law and your local regulations.

## Licence

MIT © 2026 Ropera Labs. Part of the Ropera Labs roadmap on distributed autonomy and C4ISR for degraded networks.
