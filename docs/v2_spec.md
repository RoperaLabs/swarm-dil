# GRACE-3D 2.0.0 specification (gate F2A, revised in F2B)

v2 = CAIRN certificate exchange (v0.4 draft) re-engineered for the four v2 targets. Baseline: `docs/v1_baseline.md`.
Literature: `docs/v2_literature.md`. Proofs: `docs/v2_proofs.md`. Lean: `formal/v2_theorems.lean`.
Checks: `tests/math_verify_v2.py`. F2B changes are listed at the end.

## 1. Model and regime of validity

- **World.** Lattice $\Omega=\{0..n_x-1\}\times\{0..n_y-1\}\times\{0..n_z-1\}$, 26-connected, step cost
  $h_3(\Delta)=\delta_1+(\sqrt2-1)\delta_2+(\sqrt3-\sqrt2)\delta_3$ ($\delta_1\ge\delta_2\ge\delta_3$ sorted $|\Delta|$);
  a move succeeds iff the target voxel is free (v0.4 rule), and its outcome is observed. Static obstacles $O$.
  Lattice stretch vs Euclidean $\le1.128$. Default $40\times40\times10$, 5 m voxels, $N=10$; persistent waypoint
  packing limits worlds to $64\times64\times16$.
- **Frame.** Every coordinate is in the swarm frame $F_A$: origin at the launch point, axes from the launch heading
  and gravity. *Global coordinates* = any frame external to the swarm (geodetic, GNSS, ENU); none appears in the
  protocol. Odometry is exact (drift is v3).
- **Sensor.** Per slot, $P$ zones (default $P=128$: two $8\times8$ multizone ToF-class arrays, FoV $\pm30^\circ$,
  one forward, one rotating through 4 headings), each returning the range of its first hit $\le r=5$ voxels or
  "none" (MCU-class depth sensing, v2_literature §3). Ray marching is world-side. Sensing is exact (no false hits).
- **Hardware class.** Cortex-M4F, 168 MHz (STM32F405, Crazyflie 2.1). Per slot (1 s): CPU $\le$ 168 k cycles (1 ms);
  **persistent state** $\le 256$ B; **scratch** = one static 16 KiB arena, reused, never heap-allocated; mission
  order (A, B, N, launch slot, limits) in ROM.
- **Information constraint.** Def. 1 of v1, enforced by AST test on `src/sim3d/agent.py`.
- **Unchanged from v1/v0.4.** Optimistic policy (unknown = free), Lemmas 1–2 of v1, Gilbert–Elliott abstraction,
  CAIRN certificates (dual $d_K\le V^\star$, primal $V^\star\le U$), commit rule $U\le(1+\theta)L$.

## 2. Architecture (one slot of agent $i$)

An **acquisition** is a blocked voxel that enters memory: sensed, revealed by a failed move, or received.
It is *fresh* if its tile key misses the eviction Bloom filter (the tile was never evicted), *repeated* otherwise.
$\Phi$ = cost of the remaining plan ($\infty$ without a plan).

1. **Ingest** (target 2). For each zone with a hit: voxel $v=\mathrm{round}(p_i+\rho\,u_z)$; if bit $v$ is unset in its
   tile (or the tile is absent), set it and mark the tile dirty. Free space is never carved (v1 Lemma 1).
2. **Receive.** Decode packets (§4); unwrap poses; OR-merge tiles; PLAN re-anchors the sender, TEAM raises the
   team bound on $V^\star(A)$, VERIFIED chunks (target field = chunk index) assemble a route at $A$.
3. **Forget** (target 3). While $>K$ tiles: among the tiles not touched this slot (all, if every tile was), evict
   the one of largest margin $\min_{y\in T}h_3(y-p_i)+h_3(B-y)-\Phi$ ($h_3(B-p_i)$ without a plan); the key enters a
   32-bit Bloom filter. Keeping this slot's tiles prevents bumping into what was just seen.
4. **Plan** (when the first `horizon` = 2 pieces of the plan are blocked, or the plan is exhausted; the near pieces
   are re-checked whenever the map changes and whenever a waypoint is reached, so a piece is always checked before
   it is walked). Exact A* from $p_i$ on $G(K)$, heuristic $h_3$, in the scratch arena (2048 nodes), as many
   expansions per slot as fit in 70 % of the cycle budget (the rest covers the compression); the agent holds
   meanwhile (time, not distance) and stores no new acquisition, so the search runs on a frozen map and the episode
   ends. The path is compressed greedily (doubling, then bisection) into $\sigma$ segments believed free, which keeps
   its cost $d_K(p_i)$. Arena overflow (0.4 % of searches in F4) ⇒ from the next slot, the *box-visibility planner*
   (BVP), also spread over slots: nodes = $p_i$, $B$ and the free lattice points diagonally adjacent to the 8 corners
   of each stored tile's occupied box ($n\le8K+2$), lazy weighted A* on the complete graph with exact edge tests
   (T2.5a); none ⇒ no plan. A plan whose near pieces are clear is kept (receding horizon). The tile that blocks a
   plan joins the *pool* (step 7).
5. **Certify.** Every completed search gives $d_K(p_i)\le V^\star(p_i)$: $L_i\leftarrow\max(L_i,d_K(p_i))$ (an overflowed
   one gives its last $f_{\min}$). An agent still at $A$ runs the same search in slices of 40 expansions for $V^\star(A)$
   (TEAM claims, commit). $L_i$ is transported by moves: $L_i\leftarrow\max(L_i-c(\text{step}),h_3(B-p_i))$.
6. **Escape** (liveness). If a decision (a planning episode counts as one) did not lower $\Phi$ by at least 1 (no step
   along the plan, a failed move, a costlier replan) and no acquisition since the previous decision was fresh,
   start an escape with probability $p_{\rm esc}=0.05$; with no plan, or a *futile* plan ($\Phi<L_i$: it enters a
   blocked voxel, F45.1, which the agent forgot, F45.2), start one surely (F4.5). Normal plan following lowers
   $\Phi$ by the step cost $\ge1$ every slot. An escape is a correlated random walk: it draws a new direction at the
   start, with probability $\rho=0.25$ at each step, and whenever the current one is believed blocked; a draw is
   uniform over all 26 directions with probability 1/2 and over the directions believed free otherwise (so known
   obstacles are mostly avoided, while false map content cannot exclude a direction). It continues each slot with
   probability $1-1/\mu$, $\mu=4$. It travels about linearly in its length, every direction keeps probability
   $\ge\rho/52$ per step, and any finite distance is reachable in one escape (a fixed length would not do: the
   free-space distance to $B$ is unknown).
7. **Select and send.** Claims first: VERIFIED (an arrived agent whose walk fits, one chunk of $\le3$ waypoints per
   packet, value = walk cost rounded up), TEAM (an agent still at $A$ whose bound rose by $\ge0.5$, value rounded
   down), PLAN every 10 slots (first waypoint = the sender's cell). Then pool tiles not sent for 8 slots whose gain
   $\sum_j2(\mathrm{dist}_j-r)^+e^{-m_j^+/\hat\mu}$ over travelling teammates is $\ge10\kappa$, best first. ADVICE and
   RELAY are reserved. Heartbeat every 5 slots.
8. **Move** one step along $\sigma$ from the previous to the next waypoint (step index persistent, so the cells are
   those step 4 checked); a waypoint reached, a replan or an escape step appends to the walk record (§5) the
   walked cells as $\sigma$ pieces, then merges the last two pieces while the merged $\sigma$ enters only walked cells
   (so the record stays a verified route). A committed agent refills its plan buffer from the rest of its route.

**Canonical segment** $\sigma(p,q)$, $d=q-p$, axes sorted so $|d_{hi}|\ge|d_{mid}|\ge|d_{lo}|$: step $k=1..|d_{hi}|$
sets $hi_k=k$, $mid_k=\lfloor k|d_{mid}|/|d_{hi}|+\tfrac12\rfloor$, $lo_k=\lfloor mid_k|d_{lo}|/|d_{mid}|+\tfrac12\rfloor$
(signs restored). The minor axis moves only when the middle one does (nesting), so $\sigma$ is a geodesic of cost
exactly $h_3(d)$, and every cell is within $\ell_\infty$ distance 1 of the straight segment (T2.5a). v0.4's
diagonal-first segment is geodesic but can stray $\Theta(|d|)$ from the line and is not used.

## 3. Coding geometry (target 1)

Tile = $4\times4\times4$ voxels, 64-bit mask + 2 B key = 10 B. A planar surface of area $a$ costs
$(2+s^3/8)\,a/s^2=0.625\,a$ B in expectation over offsets ($s=4$); the real optimum $s^\star=32^{1/3}=3.17$ and the
integer one $s=3$ (0.597 B/voxel, 4.5 % better) are rejected for byte alignment. Per slot at most $P$ new voxels
⇒ $\le 0.625P$ B of new geometry: $O(r^2)$ at fixed angular resolution, never $O(r^3)$.

## 4. Wire format (bit-exact, big-endian)

| Field | Bits | Content |
|---|---|---|
| H0 | 8 | `[7:3]` sender id (0–31), `[2]` LOST, `[1]` ARRIVED, `[0]` COMMITTED |
| H1 | 8 | sequence number mod 256 |
| H2–H3 | 16 | `[15:11]` $x$ mod 32, `[10:6]` $y$ mod 32, `[5:1]` $z$ mod 32 (wrapped pose), `[0]` DIGEST |
| H4 | 8 | `[7:4]` tile count, `[3:0]` claim count |
| digest | 32 | present iff DIGEST: CAIRN plan digest (v0.4 T3) |
| claim | 32+24n | `[31:27]` target id, `[26:24]` kind (ADVICE TEAM VERIFIED RELAY PLAN PLAN_X = 1..6), `[23:16]` $n\le3$, `[15:0]` value in 1/8 voxel; then $n$ waypoints $(x,y,z)$, 8 bits each, in $F_A$ |
| tile | 80 | `[79:75]` $t_x$, `[74:70]` $t_y$, `[69:65]` $t_z$, `[64]` 0, `[63:0]` mask, bit $16d_x+4d_y+d_z$ |

Header 5 B (v0.4: 6 B); no global coordinate anywhere. Receiver unwraps the pose against its last estimate (T2.4);
a teammate unheard for $>15$ slots is stale and its pose is not trusted until a PLAN claim re-anchors it.

## 5. Persistent state layout (target 3, exact)

| Structure | Bytes |
|---|---|
| tiles $K=12$ × (2 B key incl. pool bit + 8 B mask) | 120 |
| plan: 8 waypoints × 2 B (packed $6{+}6{+}4$ bits) | 16 |
| bounds $L_i$, $\Phi$, team $V(A)$, verified $U$: 4 × 2 B | 8 |
| pose 2, heading 1, seq 1, escape flag and direction 1, RNG 2, segment step 1 | 8 |
| eviction Bloom filter | 4 |
| neighbour table 9 × (pose 2, last-heard 1, flags 1) | 36 |
| walk record: 12 $\sigma$-waypoints × 2 B (before launch: VERIFIED assembly; when committed: the route) | 24 |
| tile send age: 12 × 1 B | 12 |
| **total** | **228** |

Everything else (BVP graph, A* open/closed sets, packet buffers) lives in the scratch arena. `Agent.memory_used`
computes this table from the live state and the checked run asserts it at every slot.

## 6. Theorems (L = Lean, S = SymPy/Z3/exhaustive/Monte Carlo)

- **T2.0 (certificates, inherited).** Monotonicity, union fusion, sandwich, commit regret $\le\theta$, bound transport. [L]
- **T2.1 (code).** Any code injective on the $2^a$ height fields of $a$ voxels needs $\ge a$ bits for some field; the
  tile code uses 5 bits/voxel when aligned and $\le10(1+6/m)^2$ for an $m\times m$ field at any offset. [L, S]
- **T2.2 (ingestion).** Step 1 costs $P$ tests $+\ O(|\Delta O|)$ inserts; any agent correct on every world must read
  every zone whose ray holds an unknown voxel (adversary), so $\Omega(P)$ is tight on an empty map.
- **T2.3 (frame equivariance).** Translation commutes with union; re-tiling is a join homomorphism that decodes back
  to the map and commutes with translation; masks merge by OR. [L]
- **T2.4 (wrapped pose).** Unwrap is exact iff $-16\le p-\hat p<16$ per axis; a teammate decoded $\le15$ slots ago
  is decoded exactly. [L, S]
- **T2.5a (canonical segment).** Unit steps, cost exactly $h_3(q-p)$, every cell within $\ell_\infty$ distance 1 of
  $[p,q]$; so a segment clear of a box dilated by one voxel has no cell in the box. [S: exhaustive]
- **T2.5 (BVP).** Any returned plan is a walk of $G(K)$ with cost $\ge d_K(p_i)$; work $O(n^2+nK(1+s))$ per edge-test
  round, $n\le8K+2$, independent of $|\Omega|$. Not complete. Since F4 it is only the overflow fallback: its plans
  cost 5.7 % (median) to 36 % (p99) more than $d_K$, which tripled regret as the main planner. [S]
- **T2.6 (anytime certificate).** Every value read from a search is $\le d_K(p_i)\le V^\star(p_i)$; values are
  non-decreasing within a run and exact at completion; a search on a frozen map $K\subseteq O$ is no exception.
  [L core, S]
- **T2.7 (certified forgetting).** A tile outside $\mathcal E(x,B,U)$ meets no walk of cost $\le U$ from $x$;
  forgetting it changes no plan of cost $\le U$ at that slot. Not a completeness device (a later informed set
  can contain it after an optimistic detour). [L, S]
- **T2.8 (completeness from a verified route).** An agent holding a VERIFIED route ($\le12$ waypoints whose $\sigma$
  segments enter only cells the sender walked) reaches $B$ deterministically with no map memory; with the commit
  rule its regret is $\le\theta$. The merge rule of step 8 preserves the property. How often a walk fits is
  measured (C3 fit rate).
- **T2.9 (probabilistic completeness, any $K\ge1$, any received content).** In a finite world with $B$ reachable,
  steps 4, 6, 8 reach $B$ with probability 1; each escape succeeds with probability
  $\ge(\rho(1-1/\mu)/52)^{L}$. Planning episodes end because the search runs on a frozen map. [S: Monte Carlo,
  and every agent of 20 benchmark missions arrives within 20 000 slots]
- **T2.10 (memory lower bound).** A single deterministic agent with finite state cannot find a goal in every finite
  3-D world (Budach's theorem, embedded); $O(\log|\Omega|)$ bits suffice (Reingold). Teams and known-goal
  variants are not settled here.
- **T2.11 (memory).** Persistent layout 228 B $\le$ 256 B. [L, S, asserted at run time]
- **F45.1–F45.3 (futility, F4.5).** A plan cheaper than a sound bound, $\Phi<L_i$, enters a blocked voxel; with honest
  maps this needs an eviction; the futility trigger keeps T2.9. [L, S]

Target 3's "$O(k\log k)$ local repair" in the Ramalingam–Reps sense needs a stored distance map ($\Theta(|\Omega|)$
state), incompatible with 256 B. v2 instead re-plans exactly from scratch in the 16 KiB arena, spread over slots:
measured (6 missions, 3376 searches), a search expands 208 nodes at the median, 940 at p99, and fits the arena
99.6 % of the time; T2.6 makes every search a certificate as well. This deviation is part of the v2 claim.

## 7. Acceptance (gate F4)

| Constraint | Metric | Pass |
|---|---|---|
| C1 encoding | B per new surface voxel; $\eta_{3D}/\eta_{2D}$ (regret removed per kB, against v2's own silence) | $\le0.7$; $\ge0.5$ (no loss against 2-D) |
| C2 perception | cycles of step 1 per slot (op-count model) | p99 $<$ 20 k |
| C3 planning | cycles of steps 2–8 per slot | max $<$ 168 k (1 ms) |
| C3 memory | persistent / scratch bytes | 228 / $\le$ 16384 |
| C3 liveness | every agent arrives within 20 000 slots, $K=12$ | 100 % |
| C3 stress | $K=6$ (60 B of map, below one wall of a pocket): arrival within 5000 slots | report |
| C3 timeliness | arrival within the 500-slot mission; verified-route fit rate | report |
| C4 frame | global-coordinate bytes on wire | 0 |
| regret | vs silence, flooding, GRACE ported to 3-D, CAIRN v0.4, all at 120 B of map; ablations: no random escape trigger, FIFO forgetting, BVP planner, no futility trigger (= F4) | report, $\ge20$ paired seeds, mean, s.e., p95, p99 |

**Op-count model.** Each primitive is counted in the agent (bit test 4, hash/heap op 30, segment–box slab test 40,
mask lookup 8, Dijkstra relax 12 cycles), a declared Cortex-M4 estimate to be calibrated against a C port of
steps 1 and 4. Python wall time is never used as evidence.

## 8. Code layout

`src/sim3d/` = `world.py` (lattice, sensor, radio reuse), `agent.py` (steps 1–8, imports only `wire`, `geom`, `ops`),
`wire.py` (§4), `geom.py` ($h_3$, $\sigma$, segment test, BVP, sliced A*), `ops.py` (cycle counters).
`benchmarks/eval_v2.py` → `benchmarks/v2_results.json`; console prints a < 800-token JSON summary.
`benchmarks/sweep_v2.py` → `benchmarks/v2_sweep.json` (the price and memory fronts of the technical report, 20 paired
seeds per point); `benchmarks/f45_v2.py` → `v2_f45.json`; `experiments/v2_media.py` → the GIFs in `results/v2/`, the
figures in `paper/releases/fig/` and the explainer's `docs/replay3d.json`, all drawn from `run(trace=...)`.

## F2B changes to the F2A draft

Nested-Bresenham $\sigma$ replaces v0.4's segment (soundness of the box test); escape length geometric, trigger on
any slot without progress and without a fresh acquisition (covers radio re-learning and repeated failed moves); T2.8 restated (capacity version was false); walk record
16 B; stale poses re-anchored by PLAN claims; arena-overflow rule; T2.10 scoped to single agents.

## F3 changes

Lazy weighted A* replaces lazy Dijkstra in the BVP (one search; LazySP iterations ran out near walls); the pool no
longer pins memory and tiles touched this slot are protected (pinning 9 of 12 tiles made agents forget the voxel
they had just bumped into); reply queue replaced by per-tile send age (total 256 B); claim subset fixed to PLAN,
TEAM, VERIFIED.

## F4 changes

Diagnosis (6 missions): the BVP was the whole gap between v2 and v0.4 silence at equal memory (v2 silent: 2.42 with
the BVP, 1.38 with exact A*, v0.4 silence 1.42). Exact A* spread over slots on a frozen map replaces it; the BVP is
the overflow fallback. Validation of 2 plan pieces (receding horizon). Correlated escape walk ($\rho=0.25$; doubling
the length was tried and dropped: it cost time without improving liveness). Walk record 12 waypoints with the
merge rule, paid by dropping the unused neighbour lb byte (255 B). Liveness measured at 20 000 slots.

F4, second pass (after a review of slow missions): a piece entering the horizon at a waypoint is now validated
(before, a piece could be walked into a voxel the agent already knew blocked: 25-114 such bumps per agent at K=6, 8-28
after); escape draws avoid believed-blocked directions half of the time (bound $\rho/52$ instead of $\rho/26$); the
BVP fallback runs over slots and the search stops at 70 % of the budget (the two remaining 1 ms overruns, 240-276 k
cycles, came from the fallback starting in the slot where the search had used 80 %; 0 overruns after, 20 missions).
$K=6$ is a stress test, not a liveness target: below the obstacle-certificate size T2.9 holds but its bound is
exponentially slow (7 missions at 20 000 slots: 74 % of agents arrived).

## F4.5 changes

One post-empirical pass (`docs/v2_proofs.md` F4.5, `benchmarks/v2_f45.json`; 40 paired seeds, 20 of them unseen).
Adopted: the futility trigger of step 6 (F45.1–F45.3); against F4, −1 011 ± 362 bytes per mission ($p=0.008$),
regret −0.07 ± 0.04 (not significant), arrival +4.5 points. Rejected: the v1 loss model in step 7 (alone it saved
2 422 ± 681 bytes, but with futility it made regret significantly worse, +0.19 ± 0.06, $p=0.004$) and
$p_{\rm esc}=0.01$ (no Pareto metric separates it from 0.05). The link counters and the neighbour sequence byte, never
read, are deleted (228 B). Option `nofutile` gives back the F4 agent.
