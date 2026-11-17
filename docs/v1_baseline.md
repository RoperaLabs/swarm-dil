# GRACE baseline (version 1, gate F0)

Frozen reference for every later version. Source of truth: `docs/theory/theory.tex` (RL-TR-2026-03,
rendered `docs/theory/theory.pdf`), code at tag `v0.3.0` (commit `ef5815e`).

## 1. Physical and mission parameters

| Symbol | Value | Where |
|---|---|---|
| grid $\Omega_n$ | $48\times48$, 8-connected, octile costs $1,\sqrt2$, corner rule | `nav/world.py: Mission.n` |
| cell side $c$ | 5 m | `Mission.cell_m` |
| team $N$ | 8, staggered launch $\Delta=12$ slots (1 slot = 1 s) | `Mission.team, launch_every` |
| horizon | 600 slots | `Mission.t_max` |
| vision cone | $r=5$ cells, half-angle $\alpha=60^\circ$, ray-cast, stops at first blocked cell | `sensor_range, half_angle_deg` |
| terrain | 10 random blocks + 4 L-pockets opening towards $A$, regenerated until $B$ reachable | `n_blocks, n_traps` |
| radio | $P_{tx}=-16$ dBm, path-loss exponent 2.7, noise $-110$ dBm, shadowing $\sigma=4$ dB with $\rho=0.9$/s (reciprocal), 20 dB per obstacle, logistic PER over SINR (width 1.5 dB), 9.6 kbit/s shared channel with contention | `radio.py: Radio`, `Mission.radio` |
| per-slot budget $b$ | 64 B | `Spec.budget` |
| price $\kappa$ | 0.5 cells/B (swept) | `Spec.price` |
| link memory $\lambda$ | 0.9 (design constant) | `Spec.burst_lambda` |
| stale / heartbeat | 20 / 5 slots | `Spec.stale, heartbeat` |

## 2. Model

- **Def. 1 (on-device agent).** $\mathcal I_i(t)$ = own cone readings, own move outcomes, received bytes, plus the
  pre-launch order ($A,B,N$, launch slot, hardware limits). Every computed quantity is a function of $\mathcal I_i(t)$.
  Enforced by the AST test `tests/test_nav.py` (agent module imports neither world nor radio).
- **Objective.** $R=\frac1N\sum_i(L_i/d^\star-1)$, $\mathrm{Bytes}=\sum_{i,t}|\text{packet}_i(t)|$,
  efficiency $\eta=(R_\varnothing-R)/\mathrm{kB}$.
- **Policy.** Optimistic: $\pi_K(x)=\arg\min_{y}c(x,y)+d_K(y)$, unknown = free.
- **Link abstraction.** Gilbert–Elliott per directed link: $p,q$, $\pi_B=p/(p+q)$, $\lambda=1-p-q$.

## 3. Formal statements (all proved in theory.tex unless noted)

| # | Label | Statement (short) | Proof status | Test |
|---|---|---|---|---|
| L1 | `lem:blocked` | $\pi_K$ depends only on the blocked set $K$; free cells never matter | complete | implicit (code) |
| L2 | `lem:mono` | $K\subseteq K'\Rightarrow d_K\le d_{K'}$ | complete | — |
| L3 | `lem:ellipse` | cells on the plan satisfy $h(x,y)+h(y,B)\le d_K(x)$ | complete | random terrains |
| L4 | `lem:tiles` | $\mathbb E[\#\text{tiles}]=\frac{w+s-1}s\cdot\frac{h+s-1}s$ | complete | exact |
| P1 | `prop:voi` | $0\le W\le2\ell$, $W\le2(s_a-r)^+$; tight in a dead end | complete; assumes detection within path distance $r$ | random terrains |
| P2 | `prop:closure` | plan-equivalence closure stops in $\le|K_i\setminus K^0|$ rounds | complete (GRACE truncates at 4) | — |
| P3 | `prop:prior` | $\Pr[a\text{ on future plan}]\le e^{-m_j(a)^+/\mu}$ if $\Delta\sim\mathrm{Exp}(\mu)$ | complete **under the exponential assumption** (not justified) | — |
| P4 | `prop:cond` | $\Pr[B_{t+\tau}\mid B_t]=\pi_B+\pi_G\lambda^\tau$, $\Pr[B_{t+\tau}\mid G_t]=\pi_B(1-\lambda^\tau)$ | complete | closed form |
| P5 | `prop:allLost` | $\Pr[\text{all }k\text{ lost}]=\pi_B\prod_m(\pi_B+\pi_G\lambda^{\tau_m})$ | complete | Monte Carlo |
| P6 | `prop:kstar` | $k^\star$ closed form | **statement only, no written proof** | brute force |
| C1 | `cor:forget` | forgetting a tile of positive margin keeps the plan | complete | — |
| C2 | `cor:spacing` | spacing $\tau\ge\ln(\varepsilon\pi_B/\pi_G)/\ln\lambda$ | complete | — |
| C3 | `cor:tile` | surfaces: $s^\star=\sqrt{8b_{id}}=4$, 1 B/cell | complete | sweep |
| T1 | `thm:select` | per-slot optimal message = top-$g$ tiles with $g\ge4\kappa$ | complete (linear, separable, cardinality) | — |
| Alg. 1 | `alg:grace` | observe, receive, forget, select, send, move | — | integration |
| Tab. 1 | wire | header 4 B (id, seq, $x$, $y$); tile 4 B ($t_x,t_y$, 16-bit mask) | — | round trip |
| Tab. 2 | memory | $4M + MN + 6N$ B | — | memory test |

Known gaps stated in the paper: no completeness under bounded memory (Remark after L2); P3 rests on an
unjustified exponential tail; P1 needs a detection model; $\lambda$ is fixed, not learned.

## 4. Reported results (20 paired seeds, ±1 s.e.)

- Silence $R=0.371$. GRACE matches tile flooding with **2.9–3.3× fewer bytes** ($\kappa=0.2$ vs 24 B/slot:
  $\Delta R=-0.001\pm0.013$, 9.6 vs 27.3 kB); ~6× vs raw-cell flooding; $\eta$ 0.013–0.026 vs 0.002–0.007.
- Best GRACE $R=0.199$ vs flooding-64 $R=0.181$ at 5.1× the bytes.
- Ablations: no loss model stalls at 0.253; **goal relevance gives no measurable transmission benefit** in this scenario.
- Memory 16 tiles: GRACE 0.220 (vs 0.212 unbounded, full arrival); flooding up to 2.13 with 18 % non-arrival;
  FIFO forgetting in GRACE 0.382. At 8 tiles GRACE arrival 89 %.
- Interference to 0 dBm: GRACE 0.212→0.286 and spends less (9.6→7.2 kB) without being told.

## 5. Audit against the v2 targets

| v2 target | v1 status | Gap |
|---|---|---|
| 1. Encoding $O(r^2)$ in 3-D, $\eta=O(1)$ vs 2-D | 2-D only; tile code optimal for 1-cell surfaces | needs 3-D surface code; C3 generalises to $s^\star=(d\,b/(D-d))^{1/D}$ (v0.4 `theory3.best_tile_side`) |
| 2. Perception $O(|\Delta O_{vis}|)$, < 1 ms MCU | ray-casts the whole cone every slot | incremental frontier ingestion not designed |
| 3. Local repair, completeness, map ≤ 256 B | full Dijkstra on $n^2$ per teammate per slot (≈600 searches/agent/mission in 3-D); **no completeness under bounded memory** | needs incremental search + a liveness proof |
| 4. 0 B global coordinates | header carries absolute $(x,y)$, tiles carry absolute $(t_x,t_y)$ | needs frame-relative keys with shift-invariant merge |
| memory ≤ 256 B | Tab. 2 at $M=16,N=8$: $64+128+48=240$ B (fits); v0.4 3-D peak map ≈ 570–650 B | 3-D must be re-budgeted |

## 6. Work already done towards v2 (v0.4 draft, uncommitted)

`src/swarm_dil/nav3/` (3-D, 26-connected lattice, certificate exchange "CAIRN"), design notes and literature in
`research/` (local only). Pareto, 10 seeds, $40\times40\times10$ voxels, $N=10$:

| protocol | regret | kB | searches/agent |
|---|---|---|---|
| silence | 0.729 ± 0.160 | 0 | 44 |
| oracle (free bandwidth, instant) | 0.275 ± 0.031 | — | 19 |
| oracle over real links | 0.362 ± 0.070 | — | 24 |
| flooding 16 / 66 B | 0.699 / 0.416 | 27.3 / 98.4 | 38 / 25 |
| GRACE ported to 3-D, κ=0.5 / 2 | 0.480 / 0.587 | 29.2 / 16.1 | 599 / 665 |
| CAIRN κ=0.5 / 20 | 0.396 / 0.442 | 33.8 / 18.0 | 99 / 91 |

CAIRN reaches flooding-66 regret with ~5× fewer bytes and the best of GRACE ported to 3-D with ~6× fewer path searches.
It still violates targets 2, 3 (map ~600 B, global search) and 4 (6 B header with absolute $x,y,z$).

## 7. Toolchain

| Item | Status |
|---|---|
| Python | `.venv` (system Python 3.14 has a broken numpy; always use `.venv/Scripts/python.exe`) |
| v1 tests | **31 passed** in 29 s (`.venv/Scripts/python.exe -m pytest -q`) |
| Lean 4 | elan 4.2.4, Lean v4.36.0, Mathlib tag v4.36.0; project `formal/` (`lake build` OK, no `sorry`; theorems in `formal/v2_theorems.lean`) |
| SymPy / Z3 | SymPy 1.14.0, z3-solver 4.15.1 in `.venv` |

Lean lives outside the user PATH. Before `lake build` (PowerShell):

```powershell
$env:ELAN_HOME = "<toolchains>\elan"; $env:PATH = "$env:ELAN_HOME\bin;$env:PATH"
$env:MATHLIB_CACHE_DIR = "<toolchains>\mathlib-cache"; $env:XDG_CACHE_HOME = "<toolchains>\cache"
cd formal; lake exe cache get; lake build
```
| LaTeX | MiKTeX present |
