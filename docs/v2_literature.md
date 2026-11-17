# v2 literature (gate F1)

Scope: what is known about the four v2 targets (3-D surface code, bounded perception, incremental planning with
completeness under bounded memory, frame-free protocol), and where the open ground is. Broader surveys behind this
note (CTP, certificates, channel models, systems; ~20 k words, BibTeX) live in `research/lit_*.md` (local).
**V** = verified from the primary source or its abstract in this or an earlier session; **U** = cited from
secondary sources only, statement not checked against the source.

## 1. Memory needed to be complete (target 3)

| Result | Statement | Status |
|---|---|---|
| Budach 1978 | no finite automaton searches all finite plane labyrinths | V (via Hoffmann 1981, Fraigniaud et al. 2005) |
| Blum, Sakoda, FOCS 1977 | capability of finite automata in 2- and 3-D space | U (statement not read) |
| Hoffmann, FCT 1981 | one pebble does not suffice to search plane labyrinths; restates Budach for finite (cofinite) labyrinths | V (abstract) |
| Blum, Kozen, FOCS 1978 ("On the power of the compass") | two cooperating automata search every finite planar labyrinth; why T2.10 is stated for single agents | U |
| Fraigniaud, Ilcinkas, Peer, Pelc, Peleg, TCS 345 (2005) | for every K-state automaton there is a planar trap of max degree 3 with ≤ K+1 nodes; exploring all graphs of diameter D, degree d needs Ω(D log d) bits, and O(D log d) suffices (depth-bounded DFS); perpetual exploration, no stop required | V |
| Reingold, JACM 55(4) 2008 | undirected s-t connectivity in deterministic log space | V |
| Aleliunas, Karp, Lipton, Lovász, Rackoff, FOCS 1979 | random walk covers a connected graph in O(\|V\|\|E\|) expected steps; log-space randomized connectivity | V (record); bound U |
| Kamon, Rimon, Rivlin, IJRR 20(1):6–25, 2001 (3DBug) | range-sensor bug in 3-D polyhedral worlds is globally convergent, but the robot "eventually possesses full knowledge of the obstacle" blocking it: memory grows with obstacle surface | V |
| Lumelsky, Stepanov, Algorithmica 1987 (Bug1/2) | 2-D complete with O(1) words given global position | U |
| Koenig, Tovey, Smirnov 2003; Mudgal et al. 2005 | freespace-assumption planning: travel Ω(log V/log log V)·d* to O(√V)·d* on graphs; grid Θ̃(n log n) | V (in lit_theory) |

**Consequences for v2.** (i) Uniform deterministic completeness with a fixed number of bits is impossible:
any M-bit agent has a trap world, so the right statement is Θ(log V) bits, not O(1). (ii) 2048 bits (256 B) far
exceed log₂V for any real scenario, so the binding constraint is not bits but *what* the optimistic planner
must remember: in 3-D the only known complete local method (3DBug) stores the whole blocking surface.
(iii) Randomisation removes the memory requirement for completeness (probability 1), at an expected cost
polynomial in V. Open: completeness of a *freespace-assumption* planner whose map is capped at K tiles. No source.

## 2. Incremental and informed search (target 3)

| Result | Statement | Status |
|---|---|---|
| Ramalingam, Reps, J. Algorithms 21:267–305, 1996 | incremental SSSP (DynamicSWSF-FP) in O(\|\|δ\|\| log \|\|δ\|\|), δ = vertices whose distance changes; the output is the full distance map, so it must keep Θ(V) state | V (record); bound V via Fan et al. 2022 survey |
| Koenig, Likhachev, D* Lite (AAAI 2002); LPA* (AIJ 2004) | Ramalingam–Reps applied to replanning; per-vertex g/rhs, Θ(V) memory | U |
| Hart, Nilsson, Raphael 1968; Dechter, Pearl, JACM 32(3) 1985 | A* with a consistent heuristic expands only nodes with f ≤ C*, and every admissible algorithm with the same heuristic must expand all nodes with f < C* (optimal efficiency) | U (classical) |
| Gammell, Srinivasa, Barfoot 2014 (informed sets) | all paths cheaper than U lie in the prolate spheroid with foci A, B and transverse diameter U | V (in lit_theory) |
| Canny, Reif, FOCS 1987 | Euclidean 3-D shortest path among polyhedra is NP-hard | V (record); statement via secondary |
| Lim, Srinivasa, Tsiotras 2021 (Lifelong-GLS) | LPA* + lazy edge evaluation | V |
| Barraquand, Latombe, IJRR 1991 (RPP) | random-walk escapes from potential-field local minima | U |

**Consequences.** The O(k log k) target in its Ramalingam–Reps sense *requires* a stored distance map, which
violates 256 B; the two halves of target 3 conflict by construction. What can be had without stored fields is
A* from the current state in a static scratch arena, whose work is bounded by the informed spheroid
E(x,B,d*) (volume (π/6)d*(d*²−h²), independent of world volume) and is optimal up to tie-breaking among admissible
algorithms (Dechter–Pearl). Its open-list minimum is a sound lower bound at every instant, i.e. an anytime
certificate in the CAIRN sense. Exact Euclidean 3-D is NP-hard, which justifies the 26-lattice (stretch ≤ 1.128).

## 3. Sensing on MCU-class vehicles (target 2)

| Source | Numbers | Status |
|---|---|---|
| Müller, Niculescu, Polonelli, Magno, Benini, arXiv:2208.12624 | Crazyflie 2.1, VL53L5CX 8×8 multizone ToF at 15 Hz (4×4 at 60 Hz), 0.2–4 m; obstacle avoidance on STM32F405 @168 MHz in 210 µs (0.3 % CPU) | V |
| ST VL53L5CX datasheet | 8×8 or 4×4 zones, I²C host interface | V |
| Hornung et al., OctoMap, Auton. Robots 2013 | ray insertion carves free space: cost O(range) per ray | U |

**Consequences.** Real depth sensors on this class deliver a *fixed* number P of first-hit ranges per frame
(64 per sensor), so ingestion is naturally O(P), not O(r³). By Lemma 1 of v1 free cells never change an action,
so no free-space carving is needed: only endpoints are inserted. Ω(P) per frame is unavoidable for any agent that
reads its sensor; O(|ΔO|) holds after an O(P) bit-test filter, or exactly if the sensor reports changes only.

## 4. Coding observed geometry (target 1)

| Source | Point | Status |
|---|---|---|
| MPEG G-PCC octree coding (overview; Mao et al. 2022) | lossless octree occupancy coding of surface point clouds; 8-bit occupancy symbols reducible to 2–4 bits with context | V (overview) |
| v1 Cor. 3 / v0.4 `theory3.best_tile_side` | for sets of dimension d in D-space, tile side s* = (d·b_id/(D−d))^{1/D} (bits) | ours |

No source gives a bytes-per-surface-voxel lower bound for *one-shot, loss-tolerant, idempotent* messages (G-PCC
codes are sequential/contextual and not mergeable). A counting bound is easy to prove directly (Θ(1) bits per
surface voxel, Section 3 of the spec) and makes the 4³ tile code constant-factor optimal.

## 5. Frames and coordinates (target 4)

CRDT joins (Shapiro et al. 2011, V) are invariant to delivery order and duplication; translation of the frame is a
lattice automorphism of the power set of voxels, so OR-merge commutes with re-anchoring. Multi-robot map fusion
without a shared frame (C-SLAM surveys in `lit_systems.md`) relies on loop closures and heavy descriptors; we found
no protocol that carries *no* absolute coordinates at a few bytes per packet. Open.

## 6. Open ground claimed by v2

1. Completeness of a bounded-map freespace planner, and how certificates from teammates restore it
   (certified forgetting) — no source.
2. A* informed-set work and anytime lower bounds used as the *communicated* certificate — no source joins search
   effort, certificates and messaging.
3. Constant-factor optimal, idempotent, frame-free 3-D surface code with a proved bit lower bound — no source.
4. A wrapped, frame-relative header with a proved decoding condition — no source.

## BibTeX (new entries; others in `research/lit_*.md`)

```bibtex
@article{FIPPP05, author={Fraigniaud, Pierre and Ilcinkas, David and Peer, Guy and Pelc, Andrzej and Peleg, David}, title={Graph exploration by a finite automaton}, journal={Theoretical Computer Science}, volume={345}, number={2--3}, pages={331--344}, year={2005}}
@article{Rei08, author={Reingold, Omer}, title={Undirected connectivity in log-space}, journal={Journal of the ACM}, volume={55}, number={4}, pages={17:1--17:24}, year={2008}, doi={10.1145/1391289.1391291}}
@inproceedings{AKLLR79, author={Aleliunas, Romas and Karp, Richard M. and Lipton, Richard J. and Lov{\'a}sz, L{\'a}szl{\'o} and Rackoff, Charles}, title={Random walks, universal traversal sequences, and the complexity of maze problems}, booktitle={FOCS}, pages={218--223}, year={1979}, doi={10.1109/SFCS.1979.34}}
@article{KRR01, author={Kamon, Ishay and Rimon, Elon and Rivlin, Ehud}, title={Range-sensor-based navigation in three-dimensional polyhedral environments}, journal={The International Journal of Robotics Research}, volume={20}, number={1}, pages={6--25}, year={2001}, doi={10.1177/02783640122067246}}
@article{RR96, author={Ramalingam, G. and Reps, Thomas}, title={An incremental algorithm for a generalization of the shortest-path problem}, journal={Journal of Algorithms}, volume={21}, number={2}, pages={267--305}, year={1996}, doi={10.1006/jagm.1996.0046}}
@article{DP85, author={Dechter, Rina and Pearl, Judea}, title={Generalized best-first search strategies and the optimality of {A*}}, journal={Journal of the ACM}, volume={32}, number={3}, pages={505--536}, year={1985}, note={UNVERIFIED in this session}}
@misc{MNPMB22, author={M{\"u}ller, Hanna and Niculescu, Vlad and Polonelli, Tommaso and Magno, Michele and Benini, Luca}, title={Robust and efficient depth-based obstacle avoidance for autonomous miniaturized {UAVs}}, howpublished={arXiv:2208.12624}, year={2022}}
```
