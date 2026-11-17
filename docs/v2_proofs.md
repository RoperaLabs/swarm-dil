# GRACE-3D 2.0.0: proofs (gate F2B, layer 3)

Complete proofs of the statements of `docs/v2_spec.md` §6. Every result lists its hypotheses, the v1 assumptions it
keeps or relaxes, and where it is checked:
`[Code | Formal (formal/v2_theorems.lean) | Math-Check (tests/math_verify_v2.py) | Benchmark]`.
Lean results compile with `lake build` and depend only on `propext`, `Classical.choice`, `Quot.sound`.

---

## 0. Standing model and notation

**(W) World.** $\Omega=\{0..n_x-1\}\times\{0..n_y-1\}\times\{0..n_z-1\}\subset\mathbb Z^3$; static blocked set
$O\subset\Omega$; start $A$ and goal $B$ free. Unit steps $S=\{-1,0,1\}^3\setminus\{0\}$. A move from $x$ by $s\in S$
succeeds iff $x+s\in\Omega\setminus O$, and the agent observes success or failure.

**(C) Costs.** $a_1=1$, $a_2=\sqrt2-1$, $a_3=\sqrt3-\sqrt2$;
$h_3(d)=a_1\delta_1+a_2\delta_2+a_3\delta_3$ with $\delta_1\ge\delta_2\ge\delta_3$ the sorted $|d_x|,|d_y|,|d_z|$.
A step $s$ costs $c(s)=h_3(s)$, which is $1,\sqrt2,\sqrt3$ for 1, 2, 3 non-zero coordinates.
A *walk* is a sequence $x_0,\dots,x_n$ with $x_k-x_{k-1}\in S$; its cost is $\sum_k c(x_k-x_{k-1})$; it *enters*
$x_1,\dots,x_n$.

**(G) Graphs.** For $K\subseteq\Omega$, $G(K)$ has an arc $x\to x+s$ iff $x+s\in\Omega\setminus K$. $d_K(x)$ is the least
cost of a walk from $x$ to $B$ in $G(K)$ ($\infty$ if none); $V^\star=d_O$. For $U\ge0$,
$\mathcal E(x,B,U)=\{y:h_3(y-x)+h_3(B-y)\le U\}$.

**(S) Sensing and messages.** Sensing is exact. Received obstacle data are true (honest teammates) wherever a result
says *honest*; T2.9 needs no such assumption.

**Kept from v1:** optimistic planning, static obstacles, exact odometry, identical agents.
**Relaxed:** 2-D grid → 3-D lattice; unbounded per-slot computation → 1 ms budget; unlimited map → 256 B state.

---

## H. Facts about $h_3$ (used throughout)

**H.1 (weights).** $a_1\ge a_2\ge a_3\ge0$.

*Proof.* $a_1\ge a_2\iff2\ge\sqrt2$, true since $4\ge2$. $a_2\ge a_3\iff2\sqrt2\ge1+\sqrt3$; both sides are positive, so
square: $8\ge4+2\sqrt3\iff2\ge\sqrt3\iff4\ge3$. $a_3\ge0\iff\sqrt3\ge\sqrt2\iff3\ge2$. $\square$

**H.2 (max form).** For every $d$, $h_3(d)=\max_\pi\big(a_1|d_{\pi1}|+a_2|d_{\pi2}|+a_3|d_{\pi3}|\big)$ over the six
permutations $\pi$ of the axes.

*Proof.* Let $x_1\ge x_2\ge x_3\ge0$ be the sorted magnitudes and $y=(y_1,y_2,y_3)$ any rearrangement of them.
By Abel summation,
$\sum_i a_iy_i=(a_1-a_2)y_1+(a_2-a_3)(y_1+y_2)+a_3(y_1+y_2+y_3)$.
Here $y_1\le x_1$ (the largest), $y_1+y_2\le x_1+x_2$ (the two largest) and $y_1+y_2+y_3=x_1+x_2+x_3$. The
coefficients $a_1-a_2$, $a_2-a_3$, $a_3$ are non-negative (H.1), so $\sum_ia_iy_i\le\sum_ia_ix_i=h_3(d)$, with equality
for the sorted arrangement. $\square$

**H.3 (norm).** $h_3(0)=0$ and $h_3(u+v)\le h_3(u)+h_3(v)$.

*Proof.* $h_3(0)=0$ from the definition. For a fixed $\pi$, $f_\pi(d)=\sum_ia_i|d_{\pi i}|$ satisfies
$f_\pi(u+v)\le f_\pi(u)+f_\pi(v)$ because $|u_j+v_j|\le|u_j|+|v_j|$ and $a_i\ge0$. For two functions with this property,
$\max(f,g)(u+v)\le\max(f(u)+f(v),g(u)+g(v))\le\max(f,g)(u)+\max(f,g)(v)$, since each of $f(u)+f(v)$ and $g(u)+g(v)$ is at
most the right-hand side. Iterating over the six forms and using H.2 gives the claim. $\square$

**H.4 (walk bound).** Every walk $x_0..x_n$ has cost $\ge h_3(x_n-x_0)$.

*Proof.* Induction on $n$. For $n=0$ both sides are 0 (H.3). If it holds for $n$, then
$x_{n+1}-x_0=(x_{n+1}-x_n)+(x_n-x_0)$ and H.3 give
$h_3(x_{n+1}-x_0)\le c(x_{n+1}-x_n)+h_3(x_n-x_0)\le c(x_{n+1}-x_n)+\mathrm{cost}(x_0..x_n)$. $\square$

**H.5 (consistency).** For all $x,y$: $h_3(B-x)\le h_3(y-x)+h_3(B-y)$; in particular $d_K(x)\ge h_3(B-x)$ for every $K$.

*Proof.* $B-x=(y-x)+(B-y)$ and H.3. The second claim is H.4 applied to any walk from $x$ to $B$. $\square$

Traceability: `[Code: src/sim3d/geom.py h3 | Formal: hmax_subadd, hmax_zero, h3_weights, consistent,
disp_le_walkCost | Math-Check: test_h3_is_lattice_distance, test_h3_weights_and_rearrangement | —]`

---

## T2.0 Certificates (inherited from CAIRN)

**Hypotheses.** (W), (C), (G). For (iii) and (vi): $K\subseteq O$ (honest, exact data). Valid in the full v2 regime.

**Statement.** (i) $K\subseteq K'\Rightarrow d_K\le d_{K'}$. (ii) $\max(d_K,d_{K'})\le d_{K\cup K'}$.
(iii) $K\subseteq O\Rightarrow d_K(x)\le V^\star(x)$. (iv) A walk from $x$ to $B$ that enters no cell of $O$ has cost
$\ge V^\star(x)$. (v) If $0\le\theta$, $L\le V^\star$ and $U\le(1+\theta)L$ then $U\le(1+\theta)V^\star$.
(vi) After a successful move $a\to a'$ of cost $c$, $L\le V^\star(a)$ implies
$\max(L-c,\,h_3(B-a'))\le V^\star(a')$. (vii) If $y$ lies on the agent's own walk at arc length $c$ before its
current cell $x$, then $V^\star(y)\ge V^\star(x)-c$.

*Proof.* (i) An arc $x\to y$ of $G(K')$ has $y\notin K'$, hence $y\notin K$: it is an arc of $G(K)$. Every walk of
$G(K')$ is a walk of $G(K)$, so the minimum over the larger set, $d_K$, is not larger. (ii) apply (i) to
$K\subseteq K\cup K'$ and $K'\subseteq K\cup K'$. (iii) is (i) with $K'=O$. (iv) Such a walk is a walk of $G(O)$, and
$V^\star(x)$ is the least cost over those. (v) $1+\theta\ge0$, so $(1+\theta)L\le(1+\theta)V^\star$; chain with
$U\le(1+\theta)L$. (vi) $a'$ is free (the move succeeded), so the arc $a\to a'$ is in $G(O)$; followed by an optimal
walk from $a'$ it gives $V^\star(a)\le c+V^\star(a')$, hence $L-c\le V^\star(a)-c\le V^\star(a')$; and
$h_3(B-a')\le V^\star(a')$ by H.5. (vii) every cell of the walk from $y$ to $x$ was occupied, hence free; reversed
(step costs are symmetric, $c(-s)=c(s)$) it is a walk from $x$ to $y$ in $G(O)$ of cost $c$, so
$V^\star(x)\le c+V^\star(y)$. $\square$

Traceability: `[Code: src/swarm_dil/nav3/agent3.py (v0.4), src/sim3d/agent.py (F3) | Formal: dist_mono, dist_union,
sandwich_lower, sandwich_upper, commit_regret, transport | Math-Check: test_informed_and_anytime | Benchmark:
benchmarks/eval_v2.py worst U/V*]`. In Lean the costs live in any complete lattice, and a path is any object with a
set of entered cells, so (i)–(iv) hold for every cost model with this structure.

---

## T2.1 Coding observed geometry

**Hypotheses.** Codes map voxel sets to bit strings and are injective on the family considered. Tiles
$4\times4\times4$, 2-byte key, 64-bit mask (10 B = 80 bits). Regime: any world; the family below is what a downward
sensor reports over terrain with unit bumps.

**Statement.** Let $\mathcal F_m=\{S_z:z\in\{0,1\}^{m\times m}\}$ with $S_z=\{(i,j,z_{ij})\}$, $a=m^2$.
(a) $|\mathcal F_m|=2^a$, each member has $a$ voxels, and any injective code gives some member at least $a$ bits.
(b) The tile code of $S_z$ uses exactly $5a$ bits when the tile grid is aligned with the field ($m\equiv0$ mod 4, the
field inside one tile layer), at most $10(1+6/m)^2a$ bits at any offset, and at most $6.25(1+3/m)^2a$ bits in
expectation over a uniformly random offset. Hence the tile code is within a constant factor (5 aligned, $\to10$ in the
worst case) of the information bound.
(c) For a box of $w_x\times w_y\times w_z$ voxels at a uniformly random offset, the expected number of tiles of side
$s$ touched is $\prod_a(w_a+s-1)/s$; a large planar patch costs $(2+s^3/8)/s^2$ bytes per voxel, minimised over
real $s$ at $s^\star=32^{1/3}\approx3.17$; $s=4$ gives 0.625 B/voxel.

*Proof.* (a) $z\mapsto S_z$ is injective: $z_{ij}$ is the unique $z$ with $(i,j,z)\in S_z$. So $|\mathcal F_m|=2^a$, and
$|S_z|=a$ since there is one voxel per column. The binary strings of length $<a$ number
$\sum_{\ell=0}^{a-1}2^\ell=2^a-1<2^a$, so an injective code cannot map all of $\mathcal F_m$ into them. (For
fixed-length codes, Lean `code_length`: an injection $\{0,1\}^a\to\{0,1\}^m$ forces $2^a\le2^m$, i.e. $a\le m$.)

(b) Along one axis the voxels occupy an interval of length $w$ starting at offset $o\in\{0,1,2,3\}$ inside its first
tile; it touches tiles $0..\lfloor(o+w-1)/4\rfloor$, i.e. $\lfloor(o+w-1)/4\rfloor+1$ tiles. Every column is occupied,
so the field touches every tile of the $x$- and $y$-intervals of length $m$, and in $z$ an interval of length at most
2. *Aligned:* $o_x=o_y=0$, $4\mid m$ gives $m/4$ tiles per axis, and $o_z\le2$ keeps $\{o_z,o_z+1\}$ in one tile:
$(m/4)^2$ tiles, $80(m/4)^2=5m^2$ bits. *Any offset:* per horizontal axis
$\lfloor(o+m-1)/4\rfloor+1\le(3+m-1)/4+1=(m+6)/4$, and at most 2 tiles in $z$; so at most $2(m+6)^2/16$ tiles and
$80\cdot2(m+6)^2/16=10(m+6)^2=10(1+6/m)^2a$ bits. *Random offset:* by (c) the mean number of tiles per horizontal
axis is $(m+3)/4$, and the number of $z$-tiles in a column stack is at most the number touched by an interval of
length 2, whose mean is $5/4$; the offsets are independent, so the mean tile count is at most $(m+3)^2/16\cdot5/4$,
i.e. $80\cdot5(m+3)^2/64=6.25(1+3/m)^2a$ bits. Divide by the lower bound $a$ from (a).

(c) For one axis, $\frac1s\sum_{o=0}^{s-1}\big(\lfloor\frac{o+w-1}s\rfloor+1\big)$. Hermite's identity
$\sum_{o=0}^{s-1}\lfloor x+\tfrac os\rfloor=\lfloor sx\rfloor$ with $x=(w-1)/s$ gives $\sum_o\lfloor\frac{w-1+o}s\rfloor=w-1$,
so the mean is $(w-1)/s+1=(w+s-1)/s$. The three offsets are independent and the count is the product of the
per-axis counts, so the mean of the product is the product of the means. For a $w\times h\times1$ patch the mean is
$(w+s-1)(h+s-1)s/s^3$ tiles of $2+s^3/8$ bytes over $wh$ voxels, which tends to $(2+s^3/8)/s^2$ as $w,h\to\infty$.
Its derivative $-4/s^3+1/8$ vanishes at $s^3=32$; the value at $s=4$ is $10/16=0.625$. $\square$

Traceability: `[Code: src/sim3d/wire.py tile | Formal: code_length | Math-Check: test_surface_code (SymPy optimum,
exact tiles-touched over all offsets, height-field family, bound over all offsets) | Benchmark: C1 bytes per new
surface voxel]`

---

## T2.2 Ingestion cost

**Hypotheses.** Sensor of $P$ zones per slot, each a first-hit range or "none"; at most $K$ stored tiles ($K$ is a
constant of the hardware class); dirty tiles and evictions are accounted in steps 3–4. Regime: v2.

**Statement.** (a) Step 1 reads $P$ zones, performs one tile lookup ($\le K$ key comparisons) and one bit test per
zone with a hit, and $|\Delta O|$ bit insertions. (b) On a map with no stored obstacle, every correct ingestion
procedure reads all $P$ zones when they all report "none". (c) More generally it reads every zone whose ray holds a
voxel not known to be blocked, on the input where all other zones report "none".

*Proof.* (a) by inspection of step 1: each zone is read once; a hit yields one voxel, one lookup among at most $K$
keys and one bit test; a voxel whose bit was unset is inserted once, and these voxels are exactly $\Delta O$.
(b), (c) A correct procedure outputs $\Delta O$ exactly. Let $\rho$ be the input in which every zone reports "none";
then $\Delta O(\rho)=\varnothing$. Suppose the procedure never reads zone $z$ on $\rho$, and let $u$ be a voxel of $z$'s
ray, within range, not known to be blocked. Let $\rho'$ equal $\rho$ except that $z$ reports a hit at $u$. A
deterministic procedure that does not read $z$ performs the same reads and produces the same output on $\rho$ and
$\rho'$, but $\Delta O(\rho')=\{u\}\ne\varnothing$: it is wrong on one of them. So it reads $z$. On an empty map every
zone has such a voxel, giving (b). $\square$

*Remark.* The reading term is $\Theta(P)$; the work beyond reading is $O(|\Delta O|)$ inserts. A sensor that reports
only changed zones would make the reading term the number of changed zones; no such sensor is assumed.

Traceability: `[Code: src/sim3d/agent.py step 1 (F3) | — | — | Benchmark: C2 cycles of step 1]`

---

## T2.3 Frame equivariance of merging

**Hypotheses.** Maps are sets of voxels of an additive group $V$ ($\mathbb Z^3$); a translation is
$\tau_\delta(v)=v+\delta$; a tiling is a family $(C_t)_t$ of cell sets covering $V$; a tile's content is
$K\cap C_t$, stored as a mask with one bit per cell. Regime: v2, and v3 re-anchoring once the offset $\delta$ between
frames is known.

**Statement.** (i) $\tau_\delta(K\cup K')=\tau_\delta K\cup\tau_\delta K'$; union is idempotent, commutative and
associative. (ii) $(K\cup K')\cap C_t=(K\cap C_t)\cup(K'\cap C_t)$, and $\bigcup_t(K\cap C_t)=K$.
(iii) $(\tau_\delta K)\cap(\tau_\delta C_t)=\tau_\delta(K\cap C_t)$. (iv) The mask of $(K\cup K')\cap C_t$ is the bitwise
OR of the masks of $K\cap C_t$ and $K'\cap C_t$, and OR is idempotent, commutative, associative.

*Proof.* (i) $v\in\tau_\delta(K\cup K')\iff v-\delta\in K\cup K'\iff v\in\tau_\delta K\cup\tau_\delta K'$; the union laws
are those of a Boolean algebra. (ii) distributivity of $\cap$ over $\cup$; for the cover, $v\in K$ lies in some $C_t$,
hence in $K\cap C_t$, and conversely $K\cap C_t\subseteq K$. (iii) $\tau_\delta$ is injective, and an injective map
preserves intersections: $v+\delta=w+\delta$ forces $v=w$. (iv) bit $i$ of the mask of $X\cap C_t$ is
$[c_i\in X]$, and $[c_i\in X\cup Y]=[c_i\in X]\lor[c_i\in Y]$; the OR laws hold bitwise.

*Consequence.* Merging received maps is independent of order, duplication and relaying (a join semilattice, as in
v1), and stays so after translating a whole map into another frame, including onto a tile grid that the
translation does not preserve: re-tiling onto the canonical grid is a join homomorphism by (ii). $\square$

Traceability: `[Code: src/sim3d/wire.py, agent merge (F3) | Formal: translate_merge, merge_idem, merge_comm,
merge_assoc, retile_merge, retile_cover, retile_translate, mask_or_idem, mask_or_comm, mask_or_assoc |
Math-Check: test_zero_global_coordinate_bytes | Benchmark: C4]`

---

## T2.4 Wrapped pose

**Hypotheses.** Each axis of the pose is sent as $w=p\bmod32\in[0,32)$; the receiver holds an estimate $e$ and decodes
$\hat p=e+((w-e+16)\bmod32)-16$. Agents move at most one voxel per slot per axis. Regime: v2.

**Statement.** (a) $\hat p=p\iff-16\le p-e<16$. (b) If $e$ is the exactly decoded position of $\tau\le15$ slots earlier,
then $\hat p=p$. (c) Consequently, by induction, a teammate heard at least once every 15 slots is decoded exactly at
every packet after its first exact decode.

*Proof.* (a) Let $r=(w-e+16)\bmod32\in[0,32)$; then $\hat p=e+r-16\in[e-16,e+16)$. Since $w\equiv p$ (mod 32),
$r\equiv p-e+16$ and $\hat p\equiv p$ (mod 32). Two integers congruent mod 32 that both lie in a half-open interval of
length 32 are equal (their difference is a multiple of 32 of absolute value $<32$). So if $p\in[e-16,e+16)$ then
$\hat p=p$; conversely $\hat p=p$ forces $p=\hat p\in[e-16,e+16)$. (b) $p-e$ is a sum of $\tau$ per-slot displacements
in $\{-1,0,1\}$, so $|p-e|\le\tau\le15$, and (a) applies. (c) induction on packets. $\square$

Stale teammates ($>15$ slots) are re-anchored by a PLAN claim, whose first waypoint is the sender's cell in full
swarm-frame coordinates (spec §4); the wrapped pose is not used before that.

Traceability: `[Code: src/sim3d/wire.py unwrap | Formal: unwrap, wrap_fits, unwrap_eq_iff, unwrap_of_recent |
Math-Check: test_unwrap (exhaustive and Z3 over all integers) | —]`

---

## T2.5a Canonical segment

**Hypotheses.** $p,q\in\mathbb Z^3$, $d=q-p$, axes relabelled $hi,mid,lo$ with $n_{hi}\ge n_{mid}\ge n_{lo}$ the
magnitudes, signs $\varsigma_a$. For $k=0..n_{hi}$: $m_k=\lfloor kn_{mid}/n_{hi}+\frac12\rfloor$,
$\ell_k=\lfloor m_kn_{lo}/n_{mid}+\frac12\rfloor$ ($\ell_k=0$ if $n_{mid}=0$), and
$c_k=p+\varsigma\odot(k,m_k,\ell_k)$ in $(hi,mid,lo)$ coordinates. $\sigma(p,q)=(c_1,\dots,c_{n_{hi}})$.

**Statement.** (a) $c_0=p$, $c_{n_{hi}}=q$, and $c_k-c_{k-1}\in S$. (b) $\sum_kc(c_k-c_{k-1})=h_3(d)$.
(c) $\|c_k-(p+\frac k{n_{hi}}d)\|_\infty\le\frac12(1+n_{lo}/n_{mid})\le1$. (d) If the closed segment $[p,q]$ misses
the closed box $[lo-\mathbf1,hi+\mathbf1]$, no $c_k$ lies in $[lo,hi]$. (e) Every $c_k$ lies in the bounding box of $p,q$.

*Proof.* (a) $m_0=\lfloor\frac12\rfloor=0$ and $m_{n_{hi}}=\lfloor n_{mid}+\frac12\rfloor=n_{mid}$. The argument of the
floor grows by $n_{mid}/n_{hi}\in[0,1]$ per step, and $\lfloor x+\epsilon\rfloor-\lfloor x\rfloor\in\{0,1\}$ for
$\epsilon\in[0,1]$, so $m$ moves by 0 or 1. Likewise $\ell=L(m)$ with $L(m)=\lfloor mn_{lo}/n_{mid}+\frac12\rfloor$,
$L(0)=0$, $L(n_{mid})=n_{lo}$, $L(m+1)-L(m)\in\{0,1\}$; since $\ell_k$ depends on $k$ only through $m_k$, $\ell$ changes
only at steps where $m$ changes. The $hi$ coordinate grows by exactly 1. So each step has $|s_{hi}|=1$ and
$|s_{mid}|,|s_{lo}|\le1$: it is in $S$. At $k=n_{hi}$ the coordinates are $(n_{hi},n_{mid},n_{lo})$, i.e. $c=q$.
(b) Let $M$ be the set of steps where $m$ changes and $\Lambda\subseteq M$ (nesting, from (a)) those where $\ell$ changes;
$|M|=n_{mid}$, $|\Lambda|=n_{lo}$. A step in $\Lambda$ changes three coordinates (cost $\sqrt3$), one in
$M\setminus\Lambda$ two ($\sqrt2$), any other one (1). Total:
$(n_{hi}-n_{mid})+\sqrt2(n_{mid}-n_{lo})+\sqrt3n_{lo}=n_{hi}+(\sqrt2-1)n_{mid}+(\sqrt3-\sqrt2)n_{lo}=h_3(d)$. (Without
nesting, $j$ coincident moves would cost $(\sqrt3-2\sqrt2+1)j$ relative to $n_{lo}$ coincident ones, and
$\sqrt3-2\sqrt2+1<0$, so nesting is exactly what makes $\sigma$ a geodesic.)
(c) On $hi$ the deviation is 0. For every real $x$, $|\lfloor x+\frac12\rfloor-x|\le\frac12$, so
$|m_k-kn_{mid}/n_{hi}|\le\frac12$. With $\rho=n_{lo}/n_{mid}\le1$:
$|\ell_k-kn_{lo}/n_{hi}|\le|\ell_k-\rho m_k|+\rho|m_k-kn_{mid}/n_{hi}|\le\frac12+\frac\rho2\le1$.
(d) If $c_k\in[lo,hi]$, the point $y=p+\frac k{n_{hi}}d$ lies on $[p,q]$ and, by (c), within $\ell_\infty$ distance 1 of
$c_k$, so $y\in[lo-\mathbf1,hi+\mathbf1]$: the segment meets the dilated box. Contrapositive. (e) each coordinate of
$c_k$ moves monotonically from that of $p$ to that of $q$. $\square$

Corollary (exactness of $h_3$). By (b), (e) and H.4, $h_3(q-p)$ is the cost of a shortest walk from $p$ to $q$ in
$G(\varnothing)$, also inside the box $\Omega$.

Traceability: `[Code: src/sim3d/geom.py segment | — | Math-Check: test_segment_exhaustive (all $|d|_\infty\le10$),
test_segment_box_corollary | —]`

---

## T2.5 Box-visibility planner

**Hypotheses.** $K$ = union of the set bits of the stored tiles; node set as in spec step 4 ($n\le8K+2$); every edge
used on a returned plan has passed the exact test of step 4. Regime: v2.

**Statement.** (a) A returned plan is a walk of $G(K)$ inside $\Omega$, so its cost $U_{\rm bvp}\ge d_K(p_i)$.
(b) The work is $O\big(I\,(n^2+n\,K(1+s))\big)$ with $I\le n(n-1)/2+1$ lazy iterations, independent of $|\Omega|$.
(c) BVP is not complete; the step-5 fallback is complete whenever the scratch arena holds the search.

*Proof.* (a) Let $u\to v$ be an accepted edge and $T$ a stored tile with occupied bounding box $[lo,hi]$. Either
$[u,v]$ misses $[lo-\mathbf1,hi+\mathbf1]$, and by T2.5a(d) no cell of $\sigma(u,v)$ lies in $[lo,hi]$, which contains
all set bits of $T$; or it meets it, and then the cells of $\sigma(u,v)$ in $[lo,hi]$ were tested one by one against
the mask. In both cases no cell of $\sigma(u,v)$ is in $K\cap T$. Over all tiles, $\sigma(u,v)$ enters no cell of $K$,
and by T2.5a(e) stays in the bounding box of $u,v\in\Omega$, hence in $\Omega$. The plan is the concatenation of such
segments from $p_i$ to $B$: a walk of $G(K)$, and $d_K(p_i)$ is the least cost of such walks.
(b) Each iteration runs array Dijkstra on $n$ nodes ($O(n^2)$) and tests the unchecked edges of one path ($\le n-1$
edges); an edge test costs $K$ slab tests plus at most $s$ mask lookups for each of at most $K$ box hits (the cells
of $\sigma$ inside a box have distinct $hi$ coordinates within the box's extent $\le s$). An iteration either returns
or removes at least one edge for good, so $I\le\binom n2+1$. No term depends on $|\Omega|$.
(c) Nodes are only corner-adjacent points; a passage visible from no pair of nodes (a one-voxel hole in a wall,
seen obliquely from every corner) is missed. A* on $G(K)$ is complete on a finite graph when it is allowed to finish
(T2.6(e)). $\square$

Traceability: `[Code: src/sim3d/geom.py BVP (F3) | — | Math-Check: F3 tests | Benchmark: C3 cycles, ablation
"field planner instead of BVP"]`

---

## T2.6 Anytime A* certificate

**Hypotheses.** A* on $G(K)$ from $a$ to $B$ with $H(x)=h_3(B-x)$; selection of an open node of least $f=g+H$; stale
heap entries discarded before reading; time slicing between expansions; on arena overflow the run stops and keeps the
value read before the overflowing expansion. $K\subseteq O$ for the last claim (honest, exact). Regime: v2.

**Statement.** (a) While $B$ is reachable in $G(K)$ and not yet selected, some open node $n$ on an optimal walk from
$a$ to $B$ has $g(n)=d_K(a,n)$. (b) A selected node $x$ has $g(x)=d_K(a,x)$. (c) Every value $f_{\min}$ read between
expansions satisfies $f_{\min}\le d_K(a)$. (d) Successive read values are non-decreasing. (e) The run ends with
$g(B)=d_K(a)$, or with an empty open set and $d_K(a)=\infty$. (f) With $K\subseteq O$, every read value is
$\le V^\star(a)$.

*Proof.* Consistency (H.5) gives, for each arc $x\to y$: $H(x)\le c(y-x)+H(y)$, and $H(B)=0$.
(b) by induction on the order of selection. Suppose all nodes selected before $x$ had optimal $g$, and
$g(x)>d_K(a,x)$. Take an optimal walk $a=y_0,..,y_m=x$ and the first $y_j$ not yet closed. If $j=0$ then $y_0=a$ is
open with $g=0$. Otherwise $y_{j-1}$ was closed with optimal $g$, and its expansion relaxed $y_j$:
$g(y_j)\le g(y_{j-1})+c=d_K(a,y_j)$ (prefixes of optimal walks are optimal), and $g\ge d_K$ always, since $g$ is the
cost of an actual walk. So $y_j$ is open with $g(y_j)=d_K(a,y_j)$. Summing consistency along $y_j..y_m$,
$H(y_j)\le d_K(y_j,x)+H(x)$, hence $f(y_j)\le d_K(a,y_j)+d_K(y_j,x)+H(x)=d_K(a,x)+H(x)<g(x)+H(x)=f(x)$, contradicting
the choice of $x$. ($y_j\ne x$, because $g(x)>d_K(a,x)$.)
(a) the same construction with $x=B$ and an optimal walk to $B$, using (b) for the closed prefix; the node found is
open because it is not closed and was inserted when its predecessor was expanded (or is $a$).
(c) With $n$ from (a): $f_{\min}\le f(n)=d_K(a,n)+H(n)\le d_K(a,n)+d_K(n,B)=d_K(a)$, by H.5 for $H(n)\le d_K(n,B)$.
This is Lean `fmin_sound`, with $g\le$ the prefix cost and $f_{\min}\le g+h(B-n)$. After an overflow the kept value
was read while (a) still held. Between slices the open set does not change, so (a) holds at every read.
(d) When $x$ with $f(x)=f_{\min}$ is expanded, every successor $y$ gets $f(y)=g(x)+c(y-x)+H(y)\ge g(x)+H(x)=f(x)$ by
consistency, and every other open node already had $f\ge f(x)$. So the minimum never decreases.
(e) when $B$ is selected, (b) gives $g(B)=d_K(a)$; if the open set empties, no walk to $B$ exists: by (a) an open
node would remain otherwise. The graph is finite, so one of the two happens.
(f) (c) and T2.0(iii). $\square$

*Use in v2 (since F4).* The planning search of step 4 runs on a frozen map: while it runs the agent holds and stores
nothing new, so the map $K$ is fixed for the whole search and $K\subseteq O$ (honest, exact data). (a)–(f) apply
verbatim; in particular the completed search returns $d_K(p_i)$, which is both the plan's cost and a sound bound
$L_i$. On arena overflow the run stops before the expansion that would overflow, so the last value read satisfies
(c).

Traceability: `[Code: src/sim3d/geom.py astar | Formal: consistent, disp_le_walkCost, walkCost_split, fmin_sound |
Math-Check: test_informed_and_anytime (150 worlds, slices 1/3/7) | Benchmark: certified bracket L/U]`

---

## T2.7 Certified forgetting

**Hypotheses.** Any $K$, any $U\ge0$, any $x$. Regime: v2.

**Statement.** (a) Every cell $y$ entered by a walk from $x$ to $B$ of cost $\le U$ lies in $\mathcal E(x,B,U)$.
(b) If a tile $T$ is disjoint from $\mathcal E(x,B,U)$, then the walks from $x$ to $B$ of cost $\le U$ in $G(K)$ and in
$G(K\setminus T)$ are the same; in particular, if $d_K(x)\le U$ then $d_{K\setminus T}(x)=d_K(x)$ with the same optimal
walks. (c) (a) and (b) say nothing about later slots: after the agent moves to $x'$, $\mathcal E(x',B,U')$ may contain
$T$.

*Proof.* (a) Split the walk at $y$: by H.4 the prefix costs $\ge h_3(y-x)$ and the suffix $\ge h_3(B-y)$; their sum is
the walk's cost, at most $U$. (Lean `informed`, `informed_h3`.) (b) $G(K)\subseteq G(K\setminus T)$ by T2.0(i). A walk of
$G(K\setminus T)$ of cost $\le U$ enters no cell of $T$ by (a), and no cell of $K\setminus T$ by definition, so it
enters no cell of $K$: it is a walk of $G(K)$. Both minima are then taken over the same set of walks of cost $\le U$,
which contains the optimal ones when $d_K(x)\le U$. (Lean `forget_safe`.) (c) Example: a tile beside the start, behind
the agent, has positive margin while the plan leads away; an optimistic detour that returns towards it brings it back
into the informed set of the new position. $\square$

Traceability: `[Code: src/sim3d/agent.py step 3 (F3) | Formal: informed, forget_safe, informed_h3 | Math-Check:
test_informed_and_anytime (37k cells over 150 worlds and three slacks) | Benchmark: ablation "no certified
forgetting"]`

---

## T2.8 Deterministic completeness from a verified route

**Hypotheses.** Honest teammates; a VERIFIED route $w_0..w_m$, $m\le11$, $w_m=B$, where every cell of every
$\sigma(w_k,w_{k+1})$ was occupied by its sender; the receiving agent stands at $w_0$, or holds a walk to $w_0$ through
cells it occupied (its own walk record, T2.0(vii) direction). Static world.

**Lemma 8.1 (the walk record is a verified route).** The record built by step 8 has this property. *Proof.* By
induction on appends. A walked stretch is split greedily into pieces $c_i\to c_j$ with
$\sigma(c_i,c_j)=(c_{i+1},..,c_j)$, the cells actually walked, so a new piece enters only walked cells. A merge of
the last two waypoints $a,b,c$ into $a,c$ happens only if every cell of $\sigma(a,c)$ lies in
$\sigma(a,b)\cup\sigma(b,c)$, whose cells were walked by the induction hypothesis. Both operations keep the property.
$\square$

**Statement.** Following the route reaches $B$ after
$\sum_k\|w_{k+1}-w_k\|_\infty$ steps from $w_0$, every move succeeding, using no map memory; its cost is
$\sum_kh_3(w_{k+1}-w_k)$, and if the commit rule held, $U\le(1+\theta)V^\star$.

*Proof.* A cell occupied by an agent is free, and obstacles are static, so every cell of every $\sigma(w_k,w_{k+1})$ is
free. Each step of $\sigma$ is a unit step (T2.5a(a)) into a free cell inside $\Omega$, so it succeeds; $\sigma(w_k,w_{k+1})$
has $\|w_{k+1}-w_k\|_\infty$ steps and costs $h_3(w_{k+1}-w_k)$ (T2.5a(b)). Moving uses only the waypoints and the step
index of the persistent state. The regret bound is T2.0(v). $\square$

*Scope.* The theorem is conditional on the route fitting 12 waypoints; the fraction of arrived agents whose walk
does is a benchmark quantity (C3 "verified-route fit rate"), not a theorem. Measured in F4 before the merge rule, a
walk needed 42 pieces at the median (p90 64-94).

Traceability: `[Code: src/sim3d/agent.py VERIFIED handling (F3) | Formal: commit_regret | — | Benchmark: C3 fit rate,
regret of committed agents]`

---

## T2.9 Probabilistic completeness under any memory bound

**Hypotheses.** Finite world; $B$ reachable from $A$ in $G(O)$. One agent, any $K\ge1$, any received content (true or
not). The agent follows its plan (a walk believed free); replaces it only by a believed-free plan; runs step 6 as
specified, with $p_{\rm esc}\in(0,1]$, $\mu>1$, $\rho\in(0,1]$, using fresh random bits; the Bloom filter has no false
negatives and is never cleared. A move into a blocked voxel fails and is an acquisition. A planning episode (step 4)
holds the agent for finitely many slots: the search runs on a frozen map in an arena of 2048 nodes, and at least one
expansion fits in every slot because step 1 uses far less than 80 % of the budget (C2). Regime: v2, single-agent
liveness; teammates only influence the run through received content.

**Definitions.** $R$ = free cells reachable from $A$ in $G(O)$; $\ell(x)$ = least number of steps from $x\in R$ to $B$;
$L=\max_{x\in R}\ell(x)<\infty$; $\epsilon=(\rho(1-1/\mu)/52)^L$. A *decision* is a slot in which the agent, holding a
plan, commits a step (a planning episode and the step that ends it form one decision). A decision is *productive* if
$\Phi$ dropped by at least 1 since the previous decision; *fresh* if some acquisition since the previous decision is
fresh; a *trigger* is a decision that is neither, or the end of an episode with no plan.

**Lemma 9.1.** The agent's position always lies in $R$. *Proof.* It starts at $A\in R$ and only enters free cells by
successful moves; a move $x\to y$ into a free $y$ is an arc of $G(O)$, so $y$ is reachable when $x$ is. $\square$

**Lemma 9.2.** Every run has at most $64\,|\mathcal K|$ fresh acquisitions, $\mathcal K$ the set of tile keys.
*Proof.* A fresh acquisition sets a bit of a tile whose key is absent from the Bloom filter; with no false negatives,
that key was never evicted. Before its first eviction a tile's bits are only set, never cleared, so it has at most 64
fresh acquisitions; after its first eviction its key is in the filter for ever (never cleared), so it has none. $\square$

**Lemma 9.3.** In a run that does not reach $B$, triggers occur infinitely often, almost surely.
*Proof.* Suppose finitely many triggers. Then finitely many escapes start; each lasts a geometric number of slots,
finite almost surely; and fresh decisions are finitely many (Lemma 9.2). Let $t_0$ follow all of them. From $t_0$ on,
every decision is productive, so $\Phi$ drops by at least 1 per decision, and decisions are at most $S$ slots apart, $S$
the finite bound on a planning episode. A non-empty plan has $\Phi\ge1$ (each step costs at least 1), and $\Phi\ge0$;
so within $\Phi_{t_0}$ decisions, i.e. $S\,\Phi_{t_0}$ slots, the plan is empty: the agent is at $B$. $\square$

**Lemma 9.4.** An escape started at $x\in R$ reaches $B$ with probability at least $\epsilon$.
*Proof.* Fix a shortest free walk from $x$ to $B$, of $\ell(x)\le L$ steps. A *draw* picks a direction uniformly among
all 26 with probability 1/2 and among those believed free otherwise, so it picks any given direction $d$ with
probability at least $1/52$, whatever the map holds. The escape draws at its start, and at each step it draws again
with probability $\rho$ (surely if the current direction is believed blocked), keeping the current direction
otherwise. So at every step $d$ is taken with probability at least $\rho/52$: at least $\rho\cdot\frac1{52}$ through a
redraw, plus the probability of keeping $d$ if it is current. The escape takes its first step in the slot it starts
and continues with probability $1-1/\mu$ per slot. It follows the fixed walk exactly with probability at least
$(\rho/52)^{\ell(x)}(1-1/\mu)^{\ell(x)-1}\ge(\rho(1-1/\mu)/52)^{L}=\epsilon$; those moves enter free cells and
succeed. $\square$

**Statement.** The agent reaches $B$ with probability 1.

*Proof.* Let $E_n$ be the event that at least $n$ triggers occur and none of the escapes started at the first $n$
triggers reaches $B$. Condition on the history up to the $n$-th trigger (a stopping time) on $E_{n-1}$: an escape
starts with probability at least $p_{\rm esc}$, using fresh bits, and then reaches $B$ with probability at least
$\epsilon$ by Lemma 9.4, since the position is in $R$ (Lemma 9.1). Hence
$P(E_n)\le(1-p_{\rm esc}\epsilon)P(E_{n-1})$ and $P(E_n)\le(1-p_{\rm esc}\epsilon)^n$. By Lemma 9.3, the event of never
reaching $B$ is contained, up to a null set, in every $E_n$, so its probability is at most
$\inf_n(1-p_{\rm esc}\epsilon)^n=0$. $\square$ (The recursion and the limit: Lean `fail_bound`, with a trigger-dependent
escape probability $q_n\ge p_{\rm esc}$, and `fail_tendsto`.)

*Remarks.* (1) The bound $\epsilon$ is exponentially small in $L$; it proves liveness, not speed. Measured: on 40
random $10\times10\times3$ worlds with $K=2$, 15 runs cycle forever without escapes and all 40 arrive with them
(Math-Check). (2) Received content enters only through acquisitions and plans, so false data cannot break liveness:
it can only cause failed moves (triggers) or an empty belief path (no plan, a sure escape). (3) A saturated Bloom filter
turns fresh acquisitions into apparent repeats, which only adds triggers; the price is efficiency, set by
$p_{\rm esc}$ and $\mu$ and measured in F4 (ablation "no escape"). (4) In the F4 benchmark (20 missions, 10 agents,
$K=12$) every agent arrives within 20 000 slots; before the frozen-map search, an agent could re-search for ever while
holding (an episode that never ended, outside the hypotheses), which is why the hypothesis is stated.

Traceability: `[Code: src/sim3d/agent.py step 6 (F3) | Formal: fail_bound, fail_tendsto (counting step) | Math-Check:
test_liveness (toy agent with the exact rule) | Benchmark: C3 arrival rate at K = 12 and K = 6]`

---

## T2.10 Memory lower bound for deterministic agents

**Hypotheses.** A single deterministic agent whose whole state takes finitely many values (any number $M$ of bits),
with any finite sensing range $r$, a compass and exact odometry, that must *find* $B$: $B$ is recognised on sight or on
arrival, and its position is not part of the mission order. Theorem used: **Budach (1978)**: for every finite automaton
with a compass moving in the 4-connected plane grid there is a finite connected labyrinth and a start cell from which
it does not visit every free cell (V, via Hoffmann 1981 and Fraigniaud et al. 2005).

**Statement.** (a) For every such agent there is a finite world $\Omega$ (one voxel thick), with $B$ reachable, in
which it never sees nor reaches $B$. (b) $O(\log|\Omega|)$ bits suffice to find $B$ in every finite world, given
$|\Omega|$. (c) Teams of communicating agents, and agents told $B$'s position, are not covered.

*Proof.* (a) *Embedding.* Map a 2-D labyrinth $\Lambda$ (free cells $F$) to a slab $\Omega$ of height 1: each cell $(i,j)$
becomes the $3\times3$ block $[3i,3i+2]\times[3j,3j+2]$; for $(i,j)\in F$ the centre $(3i+1,3j+1)$ is free, and so is the
arm cell towards each 4-neighbour in $F$ (e.g. $(3i+2,3j+1)$ towards $(i+1,j)$); every other voxel is blocked. Free
voxels have a coordinate $\equiv1$ (mod 3). A move changing both $x$ and $y$ joins a voxel with $x\equiv1$ to one with
$y\equiv1$; checking the cases, both lie in the same block, so diagonal moves never cross between blocks, and moves
with $z\ne0$ leave $\Omega$. Two blocks are joined by moves iff their cells are 4-neighbours in $F$: the embedding
preserves connectivity exactly.
*Simulation.* Let $\mathcal A$ be the agent. Build an automaton $\mathcal A'$ on $\Lambda$ whose state holds
$\mathcal A$'s state, $\mathcal A$'s position inside the current block (5 possibilities) and its heading. Every voxel
that $\mathcal A$ can see within range $r$ lies on a ray of free voxels from its position (successive voxels of a ray
are 26-adjacent, hence joined by moves into free cells) or is the blocked voxel ending such a ray, which lies in the
same block as the last free voxel or in a 4-neighbouring block (case check on the arm cells). So $\mathcal A$'s
observation is a function of the status of the cells of $\Lambda$ within $\ell_\infty$ radius $R=\lceil r/3\rceil+1$ that
are reachable inside that window, and of their 4-neighbours. $\mathcal A'$ obtains these by a depth-first search of
the window with a stack of at most $(2R+1)^2$ moves (finitely many states), returning to its cell, then applies
$\mathcal A$'s transition; when $\mathcal A$ crosses an arm into a neighbouring block, $\mathcal A'$ moves to the
corresponding 4-neighbour. $\mathcal A'$ is a finite automaton with a compass on $\Lambda$, and it visits every cell
whose block $\mathcal A$ enters or sees. By Budach's theorem there is a finite connected $\Lambda$ and a start from
which $\mathcal A'$ misses a free cell $c$. Put $B$ at the centre of $c$'s block: it is reachable, and $\mathcal A$
never sees it nor enters it.
(b) Make $\Omega$'s free graph 26-regular by treating a failed move in direction $s$ as a self-loop with port $s$ (a
successful move by $s$ enters through port $-s$, consistently). Reingold (2008) gives universal exploration
sequences for regular graphs on at most $n$ vertices, constructible in $O(\log n)$ space; an agent that knows
$n=|\Omega|$ stores the index into the sequence and the construction workspace in $O(\log|\Omega|)$ bits and visits
every free cell reachable from $A$, $B$ among them. (Statement of the UXS construction taken from the paper's
abstract.) (c) Two cooperating automata can search every finite planar labyrinth (Blum and Kozen 1978, unverified
here), so (a) does not extend to teams by this argument. $\square$

*Consequence.* "$O(1)$ memory and deterministic completeness on every world" is impossible for a single agent; the
right scale is $\Theta(\log|\Omega|)$ bits, met many times over by 256 B. v2 therefore claims T2.8 (with a certificate)
and T2.9 (randomised), and not uniform deterministic completeness.

Traceability: `[— | — | — | —]` (a reduction to cited theorems).

---

## T2.11 Persistent memory

**Statement.** The layout of spec §5 totals $120+16+8+8+4+36+24+12=228\le256$ bytes (255 until F4.5 deleted the
unread link counters, F4.5.3). *Proof.* Addition. The checked
simulation (`run(check=True)`) recomputes it from the live state of every agent at every slot.

Traceability: `[Code: src/sim3d/agent.py state (F3, asserted at run time) | Formal: memory_budget | Math-Check:
test_memory_layout | Benchmark: C3 memory]`

---

## F4.5 Post-empirical consolidation (one pass)

The F4 agent is the reference. Each candidate below is a refinement derived from a mechanism or constant that F3/F4
left empirical or unused; it is proved here, mechanised in `formal/v2_theorems.lean` (section F4.5), checked in
`tests/math_verify_v2.py`, and evaluated by `benchmarks/f45_v2.py` on the 20 main seeds and 20 unseen seeds. It is
adopted only if it improves a Pareto metric significantly without degrading any (rule of the roadmap).

### F4.5.0 Audit of the empirical constants and mechanisms of F3/F4

| Item | Value | Where it came from | Analytic status | Decision |
|---|---|---|---|---|
| receding horizon | 2 pieces | F4 sweep (2 better than 8) | since the second F4 pass every piece is validated before it is walked (T2.5a test), so the value trades CPU for early replans; no closed form for the distance effect | keep |
| search share | 70 % of the slot | F4 overrun diagnosis | atomic steps are bounded in closed form: an A* expansion costs $\le30+26(12+30+4+30+20)=2526$ cycles, a BVP pop $\le30+98(12+20+30)+12\cdot40+4\cdot12=6634$, a compression test $\le12\cdot40+40\cdot12=960$; the 30 % reserve covers the measured compressions (0 overruns in 20 missions) but not the worst case ($8\cdot23\cdot960$ cycles) | keep, flagged: an analytic guarantee needs a resumable compression |
| $p_{\rm esc},\mu,\rho$ | 0.05, 4, 0.25 | F4 ablation | T2.9 needs only $>0$; the bound $(\rho(1-1/\mu)/52)^L$ has no maximiser independent of the trap geometry | keep |
| tile resend spacing | 8 slots | F3 | v1 derives spacing from the link model (Cor. 2) | **candidate 2** |
| eviction Bloom filter | 32 bits, 2 bits/key | F2B | false-positive rate $(1-(1-1/32)^{2n})^2$ after $n$ evictions (checked: `test_bloom_saturation`); above $\sim$30 evictions most fresh acquisitions look repeated, which only adds triggers (T2.9 remark 3) | keep, no RAM left for more bits |
| link counters $n_{rx},n_{exp}$ | 18 B | v1 | stored and updated, **never read** in F4 | **candidate 2** uses them; removed if it is rejected |
| in-flight lower bound $L_i$ | 2 B | T2.6 | produced by every search, read only at $A$ (TEAM) | **candidate 1** uses it |
| walk record | 12 waypoints, 24 B | F4 | fit rate 17/177 (9.6 %); no commit ever happens: the routes never reach $A$ before the last launch (Prop. F45.6) | keep (other schedules), report |
| ellipse prior scale $\hat\mu$ | own inflation | v1 Prop. 3 | rests on an exponential tail assumption, not proved | keep |
| stale / heartbeat / announce | 15 / 5 / 10 slots | T2.4 | stale = 15 is the exact decode limit of T2.4; announce 10 < 15 keeps poses decodable | already analytic |

**Proposition F45.6 (commit opportunities in a launch schedule).** *Hypotheses:* agents launch at $i\Delta$,
$i=0..N-1$, and a move changes each coordinate by at most 1; a commit (T2.8) needs, at $A$ and before launch, every
chunk of a VERIFIED route; chunks travel one hop from an agent that has arrived and whose walk record holds (v2 does
not relay them). *Statement:* (a) the first arrival time satisfies $T_1\ge\|B-A\|_\infty$. (b) If the chunks reach $A$
by slot $m=T_1+\delta$, $\delta\ge1$, agent $i$ can commit only if $i\Delta>m$, so at most $N-(\lfloor m/\Delta\rfloor+1)$
agents can, and none when $m\ge(N-1)\Delta$. (c) None can commit if no packet from an arrived agent reaches $A$
before $(N-1)\Delta$. *Proof.* (a) Along any coordinate the position after $t$ moves differs from the start by at most
$t$ (induction on $t$, Lean `reach_time`), and reaching $B$ needs a difference of $\|B-A\|_\infty$ along some
coordinate. (b) The route exists only after an arrival; an agent that has launched has left $A$; the $i<N$ with
$i\Delta>m$ are those with $i>\lfloor m/\Delta\rfloor$ (Lean `commit_count`). (c) is the one-hop hypothesis. $\square$

*Measured* (F4.5 runs, 40 missions; $N=10$, $\Delta=10$, $\|B-A\|_\infty=35$). $T_1\ge60$ always, so by (b) at most
agents 7–9 could ever commit. $T_1\le88$, which (b) requires for any commit, in 5 missions (seeds 5, 9, 108, 111, 113:
$T_1$ = 67, 60, 79, 85, 65), identical for F4 and the adopted agent, and walks that fit the record exist there (9, 5,
1, 1 and 2 of the 10 agents arrived by the end). No agent committed: arrived agents sent 63–734 VERIFIED claims over
each of these missions, but no packet from an arrived agent reached an agent still at $A$ before its launch, for either agent
(`benchmarks/f45_commit_reach.py` → `v2_f45_commit.json`; $B$ lies 49 voxels, about 250 m, from $A$ across the
terrain). The binding condition is (c). The commit
path of T2.8 is sound but idle in this schedule; using it needs relayed routes or in-flight adoption, which is how
CAIRN v0.4 transfers routes (v3 item). The verified-route fit rate over the 40 missions is 21/336 for F4 and 21/354
for the adopted agent.

### F4.5.1 Candidate 1: certified futility

*Mechanism.* At a decision, if the plan costs $\Phi<L_i$, start an escape surely.

**Theorem F45.1 (a futile plan is blocked).** *Hypotheses:* $L\le V^\star(p)$ (sound, T2.6 and T2.0(vi)); $P$ a walk from
$p$ to $B$ of cost $\Phi$. *Statement:* if $\Phi<L$ then $P$ enters a cell of $O$. *Proof.* If $P$ entered no cell of $O$ it
would be a walk of $G(O)$, so $V^\star(p)\le\Phi$ (T2.0(iv)); then $\Phi<L\le V^\star(p)\le\Phi$, a contradiction. $\square$
(Lean `futile_crosses`, over any complete lattice of costs.)

**Theorem F45.2 (futility needs forgetting).** *Hypotheses:* honest maps ($K_t\subseteq O$); the agent's bound evolves
only by searches $L\leftarrow\max(L,d_{K}(p))$, by moves along its walk $L\leftarrow L-c$, and the map only by
acquisitions (no eviction) over an interval. *Statement:* throughout the interval $L\le d_{K}(p)$, so every plan of the
current map costs at least $L$: no futility. *Proof.* By induction over events. Initially $L=h_3(B-A)\le d_K(A)$ (H.5).
A search: $\max(L,d_K(p))\le d_K(p)$ because $L\le d_K(p)$ (Lean `inv_search`). A move $x\to y$ of cost $c$: $y$ was
entered, so it is free and, the map being honest, $y\notin K$; the arc $x\to y$ is in $G(K)$ and
$d_K(x)\le c+d_K(y)$, hence $L-c\le d_K(x)-c\le d_K(y)$ (`inv_move`). An acquisition $K\subseteq K'$:
$d_K\le d_{K'}$ (T2.0(i)), so $L\le d_{K'}(p)$ (`inv_learn`). A plan of the current map is a walk of $G(K)$, so its cost
is at least $d_K(p)\ge L$ (`no_futility`). $\square$

*Consequence.* Futility is a certificate of harmful forgetting: it can only appear after an eviction removed an
obstacle that an earlier search had used (or after false received content), and when it appears the plan is
certainly blocked (F45.1). Without the rule the agent follows that plan until it bumps or a random trigger fires.

**Proposition F45.3 (liveness kept).** T2.9 holds with the futility rule. *Proof.* The rule only adds triggers, each
starting an escape with probability $1\ge p_{\rm esc}$; Lemma 9.3 needs infinitely many triggers on runs that do not
arrive, which extra triggers cannot remove, and the bound of the theorem's proof uses only $p_{\rm esc}$ as a lower
bound on the probability of starting an escape at a trigger. $\square$ (Lean `fail_bound`: $q_n\in\{p_{\rm esc},1\}$.)

### F4.5.2 Candidate 2: the v1 loss model in v2

*Mechanism.* Pool tile $a$ carries a byte $u(a)\in[0,1]$ (it replaces the send age): $u\leftarrow1$ when the tile changes.
Its gain is $g(a)=u(a)\sum_jV_j(a)(1-\beta_j)$ with $\beta_j=\hat\pi_{B,j}(1-\lambda^{t-t_j})$, $t_j$ the slot teammate $j$ was
last heard, $\hat\pi_{B,j}=1-(n_{rx,j}+1)/(n_{exp,j}+2)$ and $\lambda=0.9$ (v1). After a copy,
$u\leftarrow u\,\bar\beta$, $\bar\beta=\sum_jV_j\beta_j/\sum_jV_j$.

**Proposition F45.4 (loss after hearing).** *Hypotheses:* the directed link is a two-state Markov chain,
$P(G\to B)=p$, $P(B\to G)=q$, $p+q>0$, reciprocal (v1 model); the receiver was heard at $t_j$, so the link was good then.
*Statement:* $P(B_{t_j+\tau}\mid G_{t_j})=\pi_B(1-\lambda^\tau)$, $\pi_B=p/(p+q)$, $\lambda=1-p-q$. *Proof.* Let
$x_\tau=P(B_{t_j+\tau}\mid G_{t_j})$. Then $x_0=0$ and, conditioning on the state at $\tau$,
$x_{\tau+1}=x_\tau(1-q)+(1-x_\tau)p=p+\lambda x_\tau$. The sequence $y_\tau=\pi_B(1-\lambda^\tau)$ has $y_0=0$ and
$p+\lambda y_\tau=p+\lambda\pi_B-\pi_B\lambda^{\tau+1}=\pi_B(p+q)+\pi_B(1-p-q)-\pi_B\lambda^{\tau+1}=\pi_B(1-\lambda^{\tau+1})
=y_{\tau+1}$, using $p=\pi_B(p+q)$. Both sequences satisfy the same first-order recurrence with the same start, so they
are equal (induction on $\tau$). $\square$ (Lean `badAfter_eq`; SymPy and Monte Carlo in `test_ge_closed_form`.)

**Proposition F45.5 (scalar belief).** *Hypotheses:* before a copy every receiver lacks the tile with probability $u$;
a copy reaches $j$ with probability $1-\beta_j$, independently of whether $j$ lacked it. *Statement:* (a) the copy's
expected delivered value is $u\sum_jV_j(1-\beta_j)$; (b) after it, $j$ lacks the tile with probability $u\beta_j$, and
the scalar update $u'=u\bar\beta$ keeps the expected value still to deliver, $\sum_jV_ju\beta_j=(\sum_jV_j)u'$, exactly.
*Proof.* (a) $j$ gains $V_j$ iff it lacked the tile and the copy arrived: probability $u(1-\beta_j)$; sum over $j$.
(b) $j$ still lacks it iff it lacked it and the copy was lost: $u\beta_j$. Then
$(\sum_jV_j)u\bar\beta=(\sum_jV_j)u\frac{\sum_jV_j\beta_j}{\sum_jV_j}=\sum_jV_ju\beta_j$. $\square$ (Lean `belief_update`;
SymPy in `test_belief_update_identity`.) The collapse is exact for the expected remaining value at the current
weights; it is an approximation of the per-receiver beliefs, which 256 B cannot hold ($K\times(N-1)$ bytes).

*Per-slot optimality.* With all tiles of 10 B, maximising expected delivered value minus $\kappa$ per byte selects the
tiles of largest $g(a)\ge10\kappa$ up to the room left: v1 Theorem 1, whose proof (a linear, separable objective under
a cardinality constraint) applies verbatim with the new $g$.

### F4.5.3 Evaluation and decision

*Run 1: each candidate against F4* (`benchmarks/f45_v2.py`, 40 paired seeds: main 0–19, unseen 100–119;
$\alpha=0.05/3$). Candidate minus F4, mean ± s.e., paired $t$-test $p$, then the means on the main and unseen halves:

| Candidate | regret | bytes per mission | p99 slot cycles | arrival | rule |
|---|---|---|---|---|---|
| loss | +0.081 ± 0.048, $p=0.10$ (+0.17 / −0.01) | −2 422 ± 681, $p=0.001$ (−1 406 / −3 438) | +17.8 ± 18.9, $p=0.35$ | +0.018 | passes (bytes) |
| futile | −0.072 ± 0.044, $p=0.11$ (−0.03 / −0.11) | −1 011 ± 362, $p=0.008$ (−561 / −1 460) | −17.5 ± 16.7, $p=0.30$ | +0.045 | passes (bytes) |
| futile, $p_{\rm esc}=0.01$ | −0.125 ± 0.060, $p=0.044$ (−0.12 / −0.12) | −1 156 ± 435, $p=0.011$ (−1 457 / −855) | −11.1 ± 21.0, $p=0.60$ | +0.050 | passes (bytes) |

Every constraint holds in all 160 missions (slot cycles at most 127 040 < 168 000, memory 255 B in every slot), and no
metric is significantly worse.

*Choice between the futility variants* (exclusive). Paired, $p_{\rm esc}=0.01$ minus $0.05$: regret −0.053 ± 0.033
($p=0.12$), bytes −146 ± 338 ($p=0.67$), arrival +0.005 ($p=0.71$); only the number of escapes differs (−8.0 ± 1.6 per
mission, $p<10^{-4}$). No Pareto metric separates them, so the variant that changes no design constant is kept
($p_{\rm esc}=0.05$, as audit row 3 decided before the run): T2.9's rate and the F4 liveness evidence for the random
trigger stay as they are.

*Rule for the composition, fixed before run 2.* Loss (step 7) and futility (step 6) act on different steps and both
pass, so they are adopted together only if their composite $C$ passes a confirmatory run on the same 40 seeds:
(i) $C$ against F4 passes the F4.5 rule; (ii) $C$ is not significantly worse ($p<\alpha$) than either component alone on
any Pareto metric. If (i) and (ii) hold, $C$ is adopted. Otherwise one component is adopted alone: if $C$ is worse than
exactly one component $X$ alone, adding the other one hurt, so $X$ is adopted; in every other case, the component with
the smaller $p$-value in run 1 (loss). `f45_v2.py` applies this rule and writes the outcome to `v2_f45.json`
(`decision`).

*Run 2: the composite* $C$ = loss + futility, same 40 seeds.

| $C$ minus | regret | bytes per mission | p99 slot cycles | arrival |
|---|---|---|---|---|
| F4 | +0.115 ± 0.065, $p=0.087$ (+0.25 / −0.02) | −2 428 ± 742, $p=0.002$ (−1 025 / −3 831) | −11.7 ± 19.7, $p=0.56$ | +0.040 |
| futility alone | **+0.186 ± 0.060, $p=0.004$** (+0.28 / +0.09) | −1 418 ± 584, $p=0.020$ | +5.8 ± 16.7, $p=0.73$ | −0.005 |
| loss alone | +0.034 ± 0.040, $p=0.40$ | −6 ± 269, $p=0.98$ | −29.5 ± 15.2, $p=0.060$ | +0.023 |

(i) holds (bytes improve on both halves, nothing significantly worse, constraints kept); (ii) fails against futility
alone: adding the loss model to futility makes regret significantly worse.

*Decision.* By the rule, **futility is adopted alone** (`decision`: F4+futile) and the loss model is rejected.
(a) Futility is part of step 6 (spec §2); the protocol option `nofutile` gives back the F4 agent. (b) The loss model's
code is removed (its specification stays in F4.5.2); its rows in `v2_f45_runs.csv` come from the evaluation run.
(c) The link counters it would have read go with it, together with the neighbour sequence byte, which was read only
to update $n_{exp}$: 3 B for each of 9 teammates, so the persistent state drops from 255 to 228 B (T2.11), a gain in
RAM margin with unchanged behaviour. (d) Check of (a)–(c): the three configurations the final code can run (F4 as
`v2-nofutile`, futility, futility with $p_{\rm esc}=0.01$) were re-run on it, 120 missions; regret, arrival, bytes, p99
and maximum slot cycles, escapes and first arrival equal the evaluation rows exactly, and only the memory peak changes
(255 → 228 B).

*Where futility's byte saving comes from.* It cuts the agent-slots spent before arrival from 2 339 to 2 229 per
mission (−110 ± 43, $p=0.015$) and the bytes per such agent-slot from 6.33 to 6.19 (pooled over missions; paired
per-mission difference −0.107 ± 0.052, $p=0.048$). At the F4 rate the shorter travel alone accounts for 69 % of the saving: a tile is worth nothing to an
arrived teammate (step 7), so the swarm stops paying for it sooner. Escapes of all kinds rise from 13.2 to 23.8 per
mission.

*Why the loss model fails here (a hypothesis, not tested).* Proposition F45.5 assumes that a receiver keeps a tile
once it has it. With $K=12$ tiles receivers evict (step 3), so the chance that the team lacks a tile grows again after
a copy; the F4 rule resends a pool tile every 8 slots and so re-supplies evicted tiles, while $u\leftarrow u\bar\beta$
only decays. A belief that models eviction at the receiver is left to v3.

---

## Deviations from the roadmap targets, with the reason

| Target | Status after F4.5 (final agent, 20 paired seeds, `benchmarks/v2_results.json`) | Why |
|---|---|---|
| 1. encoding $O(r^2)$, $\eta=O(1)$ vs 2-D | proved for the code (T2.1, §3); measured 0.641 B per surface voxel, $\eta_{3D}/\eta_{2D}=1.05$ | — |
| 2. perception $O(\lvert\Delta O\rvert)$, < 1 ms | $\Theta(P)$ reading is a lower bound (T2.2); work beyond reading is $O(\lvert\Delta O\rvert)$; measured p99 1 456 cycles | adversary argument |
| 3. $O(k\log k)$ repair with ≤ 256 B and completeness | exact A* spread over slots on a frozen map, every search a certificate (T2.6); 228 B; at most 124 954 cycles per slot over 694 326 agent-slots of every bench configuration, 0 overruns; all 200 agents arrive within 20 000 slots at $K=12$, 87 % within 5000 at $K=6$; probabilistic completeness proved (T2.9, F45.3), deterministic with a verified route (T2.8) | $O(k\log k)$ needs a $\Theta(\lvert\Omega\rvert)$ map; deterministic completeness with $O(1)$ memory is impossible (T2.10) |
| 4. 0 B global coordinates, merge invariant | proved (T2.3, T2.4, wire test) | — |

Regret at the 500-slot horizon (mean ± s.e.): v2 0.991 ± 0.144 with 13.3 kB (F4 agent 1.020 ± 0.162 with 13.9 kB);
GRACE ported to 3-D at 120 B of map 1.03 ± 0.24 with 14.3 kB; v0.4 silence at 120 B 1.42 ± 0.39; flooding at 120 B
3.73 ± 0.56 with 146 kB; CAIRN v0.4 at 120 B 0.62 ± 0.12 with 29.5 kB (in-flight route transfer, see Prop. F45.6).
