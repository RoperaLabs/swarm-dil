module

public import Mathlib.Analysis.Real.Sqrt
public import Mathlib.Algebra.BigOperators.Group.Finset.Basic
public import Mathlib.Analysis.SpecificLimits.Basic
public import Mathlib.Order.Interval.Finset.Nat
public import Mathlib.Data.Fintype.BigOperators
public import Mathlib.Data.Set.Lattice.Indexed
public import Mathlib.Tactic.Abel
public import Mathlib.Tactic.FieldSimp
public import Mathlib.Tactic.Linarith
public import Mathlib.Tactic.NormNum
public import Mathlib.Tactic.Ring

/-!
# GRACE-3D 2.0.0: structural theorems (gate F2B, layer 1)

Mechanised statements of `docs/v2_spec.md` §6. Paper proofs: `docs/v2_proofs.md`.
Numeric and bit-level checks: `tests/math_verify_v2.py`.

| Spec  | Here |
|-------|------|
| T2.0  | `dist_mono`, `dist_union`, `sandwich_lower`, `sandwich_upper`, `commit_regret`, `transport` |
| T2.1  | `code_length` |
| T2.3  | `translate_merge`, `retile_merge`, `retile_cover`, `retile_translate`, `mask_or_*` |
| T2.4  | `unwrap_eq_iff`, `unwrap_of_recent` |
| T2.6  | `consistent`, `disp_le_walkCost`, `fmin_sound` |
| T2.7  | `informed`, `forget_safe`, `informed_h3` |
| T2.9  | `fail_bound`, `fail_tendsto` (the counting step; the escape bound `ε` is Lemma 9.4 on paper) |
| T2.11 | `memory_budget` |
| F4.5  | `futile_crosses`, `inv_search`, `inv_move`, `inv_learn`, `no_futility`, `fail_bound` (F45.3), `badAfter_eq`, `belief_update`, `reach_time`, `commit_count` (F45.6) |
-/

@[expose] public section

namespace GraceV2

open Finset

/-! ## T2.0 Certificates (CAIRN, inherited): monotonicity, fusion, sandwich, commitment -/

section Certificates

variable {Cell Path C : Type*} [CompleteLattice C] (cost : Path → C) (cells : Path → Set Cell)

/-- Cost-to-go when the cells of `K` are known to be blocked: the cheapest path (between the
fixed start and goal) whose cells avoid `K`, with costs in any complete lattice (e.g. `ℝ≥0∞`,
where an empty infimum is `∞`). With `K = O` (the true obstacles) this is `V⋆`. -/
def dist (K : Set Cell) : C :=
  ⨅ (p : Path) (_ : Disjoint (cells p) K), cost p

/-- Knowing more obstacles never lowers the cost-to-go (v1 Lemma 2). -/
theorem dist_mono {K K' : Set Cell} (h : K ⊆ K') : dist cost cells K ≤ dist cost cells K' :=
  iInf_mono fun _ => iInf_mono' fun hp => ⟨hp.mono_right h, le_rfl⟩

/-- Fusing two dual certificates by union proves at least the larger bound. -/
theorem dist_union (K K' : Set Cell) :
    dist cost cells K ⊔ dist cost cells K' ≤ dist cost cells (K ∪ K') :=
  sup_le (dist_mono cost cells Set.subset_union_left) (dist_mono cost cells Set.subset_union_right)

/-- Dual side of the sandwich: any set of true obstacles gives a lower bound on `V⋆`. -/
theorem sandwich_lower {K O : Set Cell} (h : K ⊆ O) : dist cost cells K ≤ dist cost cells O :=
  dist_mono cost cells h

/-- Primal side: any path that avoids every true obstacle bounds `V⋆` from above. -/
theorem sandwich_upper {O : Set Cell} (p : Path) (hp : Disjoint (cells p) O) :
    dist cost cells O ≤ cost p :=
  iInf₂_le p hp

/-- Certified commitment (CAIRN T6): if `L ≤ V⋆` and the route has cost `U ≤ (1+θ)L`,
then `U ≤ (1+θ)V⋆`, i.e. the regret of committing is at most `θ`. -/
theorem commit_regret {L V U θ : ℝ} (hθ : 0 ≤ θ) (hL : L ≤ V) (hU : U ≤ (1 + θ) * L) :
    U ≤ (1 + θ) * V :=
  hU.trans (mul_le_mul_of_nonneg_left hL (by linarith))

/-- Bound transport (CAIRN T4): after a successful move `a → a'` of cost `c` (reversible, since
both cells are free), `V⋆(a) ≤ c + V⋆(a')`, so a bound `L ≤ V⋆(a)` becomes `L - c ≤ V⋆(a')`. -/
theorem transport {L c Va Va' : ℝ} (hV : Va ≤ c + Va') (hL : L ≤ Va) : L - c ≤ Va' := by
  linarith

end Certificates

/-! ## Walks with subadditive step cost (T2.6, T2.7) -/

section Walks

variable {V : Type*} [AddCommGroup V] (h : V → ℝ)

/-- Cost of the `n` steps of the walk `p` that start at index `k`; each step costs `h` of its
displacement (on the 26-lattice a unit step costs exactly `h₃` of the step). -/
def walkCost (p : ℕ → V) (k n : ℕ) : ℝ :=
  ∑ i ∈ range n, h (p (k + i + 1) - p (k + i))

variable {h}

/-- Consistency of the heuristic `x ↦ h (b - x)`: one step never lowers `f = g + h`. -/
theorem consistent (hsub : ∀ a b, h (a + b) ≤ h a + h b) (x y b : V) :
    h (b - x) ≤ h (y - x) + h (b - y) := by
  have e : b - x = (y - x) + (b - y) := by abel
  rw [e]
  exact hsub _ _

/-- Admissibility: the displacement cost never exceeds the walk cost. -/
theorem disp_le_walkCost (h0 : h 0 = 0) (hsub : ∀ a b, h (a + b) ≤ h a + h b)
    (p : ℕ → V) (k n : ℕ) : h (p (k + n) - p k) ≤ walkCost h p k n := by
  induction n with
  | zero => simp [walkCost, h0]
  | succ n ih =>
    have e : p (k + (n + 1)) - p k = (p (k + n + 1) - p (k + n)) + (p (k + n) - p k) := by
      rw [show k + (n + 1) = k + n + 1 by omega]
      abel
    rw [e, walkCost, sum_range_succ]
    have hs := hsub (p (k + n + 1) - p (k + n)) (p (k + n) - p k)
    simp only [walkCost] at ih
    linarith

theorem walkCost_split (p : ℕ → V) (k m : ℕ) :
    walkCost h p 0 (k + m) = walkCost h p 0 k + walkCost h p k m := by
  simp only [walkCost, sum_range_add, zero_add]

/-- T2.7. A walk from `x = p 0` to `B = p n` of cost at most `U` through `y = p k` keeps `y` in
the informed set `h (y - x) + h (B - y) ≤ U`. -/
theorem informed (h0 : h 0 = 0) (hsub : ∀ a b, h (a + b) ≤ h a + h b) (p : ℕ → V)
    {k n : ℕ} (hk : k ≤ n) {U : ℝ} (hU : walkCost h p 0 n ≤ U) :
    h (p k - p 0) + h (p n - p k) ≤ U := by
  obtain ⟨m, rfl⟩ := Nat.exists_eq_add_of_le hk
  have h1 : h (p k - p 0) ≤ walkCost h p 0 k := by
    simpa using disp_le_walkCost h0 hsub p 0 k
  have h2 := disp_le_walkCost h0 hsub p k m
  have hs := walkCost_split (h := h) p k m
  linarith

/-- T2.7 (certified forgetting). A set of cells outside the informed set of `U` holds no cell
of any walk of cost at most `U` from `x` to `B`; forgetting it changes no plan of cost `≤ U`. -/
theorem forget_safe (h0 : h 0 = 0) (hsub : ∀ a b, h (a + b) ≤ h a + h b) (p : ℕ → V)
    {n : ℕ} {U : ℝ} (hU : walkCost h p 0 n ≤ U) (T : Set V)
    (hT : ∀ y ∈ T, U < h (y - p 0) + h (p n - y)) : ∀ k ≤ n, p k ∉ T :=
  fun _ hk hy => absurd (informed h0 hsub p hk hU) (not_le.mpr (hT _ hy))

/-- T2.6 (anytime certificate). Hart–Nilsson–Raphael invariant: some open node `p k` of an
optimal walk has `g ≤` the cost of the walk's prefix. Then the open-list minimum `f_min` is a
sound lower bound on the optimal cost `C⋆ = walkCost h p 0 n`. -/
theorem fmin_sound (h0 : h 0 = 0) (hsub : ∀ a b, h (a + b) ≤ h a + h b) (p : ℕ → V)
    {k n : ℕ} (hk : k ≤ n) {g fmin : ℝ} (hg : g ≤ walkCost h p 0 k)
    (hmin : fmin ≤ g + h (p n - p k)) : fmin ≤ walkCost h p 0 n := by
  obtain ⟨m, rfl⟩ := Nat.exists_eq_add_of_le hk
  have h2 := disp_le_walkCost h0 hsub p k m
  have hs := walkCost_split (h := h) p k m
  linarith

end Walks

/-! ## `h₃` is subadditive (instantiates T2.6 and T2.7 on the lattice) -/

section H3

/-- Weighted ℓ¹ form; `h₃` uses weights `1 ≥ √2-1 ≥ √3-√2 ≥ 0`. -/
def form (a₁ a₂ a₃ u v w : ℝ) : ℝ := a₁ * |u| + a₂ * |v| + a₃ * |w|

/-- `h₃` as the maximum over the six assignments of the weights to the axes. With
`a₁ ≥ a₂ ≥ a₃ ≥ 0` the maximum is the sorted closed form of the spec (rearrangement
inequality); `tests/math_verify_v2.py` checks that identity with Z3. -/
def hmax (a₁ a₂ a₃ : ℝ) (d : ℝ × ℝ × ℝ) : ℝ :=
  max (max (max (form a₁ a₂ a₃ d.1 d.2.1 d.2.2) (form a₁ a₂ a₃ d.1 d.2.2 d.2.1))
           (max (form a₁ a₂ a₃ d.2.1 d.1 d.2.2) (form a₁ a₂ a₃ d.2.1 d.2.2 d.1)))
      (max (form a₁ a₂ a₃ d.2.2 d.1 d.2.1) (form a₁ a₂ a₃ d.2.2 d.2.1 d.1))

def Subadd (f : ℝ × ℝ × ℝ → ℝ) : Prop := ∀ d e, f (d + e) ≤ f d + f e

theorem Subadd.max {f g : ℝ × ℝ × ℝ → ℝ} (hf : Subadd f) (hg : Subadd g) :
    Subadd fun d => max (f d) (g d) := fun d e =>
  max_le ((hf d e).trans (add_le_add (le_max_left _ _) (le_max_left _ _)))
    ((hg d e).trans (add_le_add (le_max_right _ _) (le_max_right _ _)))

variable {a₁ a₂ a₃ : ℝ}

theorem form_sub (h1 : 0 ≤ a₁) (h2 : 0 ≤ a₂) (h3 : 0 ≤ a₃) (u v w u' v' w' : ℝ) :
    form a₁ a₂ a₃ (u + u') (v + v') (w + w') ≤ form a₁ a₂ a₃ u v w + form a₁ a₂ a₃ u' v' w' := by
  have hu := mul_le_mul_of_nonneg_left (abs_add_le u u') h1
  have hv := mul_le_mul_of_nonneg_left (abs_add_le v v') h2
  have hw := mul_le_mul_of_nonneg_left (abs_add_le w w') h3
  rw [mul_add] at hu hv hw
  unfold form
  linarith

theorem hmax_subadd (h1 : 0 ≤ a₁) (h2 : 0 ≤ a₂) (h3 : 0 ≤ a₃) : Subadd (hmax a₁ a₂ a₃) := by
  have s : ∀ (π : ℝ × ℝ × ℝ → ℝ × ℝ × ℝ), (∀ d e, π (d + e) = π d + π e) →
      Subadd fun d => form a₁ a₂ a₃ (π d).1 (π d).2.1 (π d).2.2 := by
    intro π hπ d e
    dsimp only
    rw [hπ]
    exact form_sub h1 h2 h3 _ _ _ _ _ _
  exact ((((s id fun _ _ => rfl).max (s (fun d => (d.1, d.2.2, d.2.1)) fun _ _ => rfl)).max
      ((s (fun d => (d.2.1, d.1, d.2.2)) fun _ _ => rfl).max
        (s (fun d => (d.2.1, d.2.2, d.1)) fun _ _ => rfl))).max
    ((s (fun d => (d.2.2, d.1, d.2.1)) fun _ _ => rfl).max
      (s (fun d => (d.2.2, d.2.1, d.1)) fun _ _ => rfl)))

theorem hmax_zero : hmax a₁ a₂ a₃ 0 = 0 := by
  simp [hmax, form]

/-- The lattice weights are non-negative. -/
theorem h3_weights : (0 : ℝ) ≤ 1 ∧ 0 ≤ Real.sqrt 2 - 1 ∧ 0 ≤ Real.sqrt 3 - Real.sqrt 2 := by
  refine ⟨zero_le_one, ?_, ?_⟩
  · have := Real.sqrt_le_sqrt (show (1 : ℝ) ≤ 2 by norm_num)
    rw [Real.sqrt_one] at this
    linarith
  · have := Real.sqrt_le_sqrt (show (2 : ℝ) ≤ 3 by norm_num)
    linarith

/-- `h₃` with the lattice weights. -/
noncomputable def h3 : ℝ × ℝ × ℝ → ℝ := hmax 1 (Real.sqrt 2 - 1) (Real.sqrt 3 - Real.sqrt 2)

/-- T2.7 on the 26-lattice: cells of any walk of cost `≤ U` lie in the `h₃` informed set. -/
theorem informed_h3 (p : ℕ → ℝ × ℝ × ℝ) {k n : ℕ} (hk : k ≤ n) {U : ℝ}
    (hU : walkCost h3 p 0 n ≤ U) : h3 (p k - p 0) + h3 (p n - p k) ≤ U :=
  informed hmax_zero (hmax_subadd h3_weights.1 h3_weights.2.1 h3_weights.2.2) p hk hU

end H3

/-! ## T2.3 Frame equivariance of the merge -/

section Frames

variable {V T : Type*}

/-- Re-anchoring the frame by `δ` commutes with merging (union) of maps. -/
theorem translate_merge [AddCommGroup V] (δ : V) (K K' : Set V) :
    (· + δ) '' (K ∪ K') = (· + δ) '' K ∪ (· + δ) '' K' :=
  Set.image_union _ _ _

theorem merge_idem (K : Set V) : K ∪ K = K := Set.union_self K
theorem merge_comm (K K' : Set V) : K ∪ K' = K' ∪ K := Set.union_comm K K'
theorem merge_assoc (K K' K'' : Set V) : K ∪ K' ∪ K'' = K ∪ (K' ∪ K'') := Set.union_assoc _ _ _

/-- Tile `t` of a map: the voxels of `K` in the cell set of `t`. -/
def retile (cellsOf : T → Set V) (K : Set V) (t : T) : Set V := K ∩ cellsOf t

/-- Re-tiling is a join homomorphism, tile by tile. -/
theorem retile_merge (cellsOf : T → Set V) (K K' : Set V) (t : T) :
    retile cellsOf (K ∪ K') t = retile cellsOf K t ∪ retile cellsOf K' t :=
  Set.union_inter_distrib_right _ _ _

/-- If tiles cover space, the tiles of a map decode back to the map. -/
theorem retile_cover (cellsOf : T → Set V) (hcover : ∀ v, ∃ t, v ∈ cellsOf t) (K : Set V) :
    (⋃ t, retile cellsOf K t) = K := by
  ext v
  simp only [retile, Set.mem_iUnion, Set.mem_inter_iff]
  exact ⟨fun ⟨_, hv, _⟩ => hv, fun hv => (hcover v).imp fun _ ht => ⟨hv, ht⟩⟩

/-- Translating map and tiling together translates each tile. -/
theorem retile_translate [AddCommGroup V] (cellsOf : T → Set V) (δ : V) (K : Set V) (t : T) :
    retile (fun t => (· + δ) '' cellsOf t) ((· + δ) '' K) t = (· + δ) '' retile cellsOf K t :=
  (Set.image_inter (add_left_injective δ)).symm

/-- On the wire a tile's voxels are a 64-bit mask, merged by bitwise OR. -/
theorem mask_or_idem (x : BitVec 64) : x ||| x = x := BitVec.or_self
theorem mask_or_comm (x y : BitVec 64) : x ||| y = y ||| x := BitVec.or_comm x y
theorem mask_or_assoc (x y z : BitVec 64) : x ||| y ||| z = x ||| (y ||| z) := BitVec.or_assoc x y z

end Frames

/-! ## T2.4 Wrapped pose (5 bits per axis) -/

/-- Receiver-side decoding of a coordinate sent modulo 32, against the estimate `e`. -/
def unwrap (w e : ℤ) : ℤ := e + (w - e + 16) % 32 - 16

theorem wrap_fits (p : ℤ) : 0 ≤ p % 32 ∧ p % 32 < 32 := by omega

/-- Decoding is exact iff the true coordinate is within `[-16, 16)` of the estimate. -/
theorem unwrap_eq_iff (p e : ℤ) : unwrap (p % 32) e = p ↔ -16 ≤ p - e ∧ p - e < 16 := by
  unfold unwrap
  omega

/-- At most one voxel per slot per axis: a teammate decoded `τ ≤ 15` slots ago is decoded exactly. -/
theorem unwrap_of_recent {p e τ : ℤ} (hτ : τ ≤ 15) (hmove : |p - e| ≤ τ) :
    unwrap (p % 32) e = p := by
  rw [unwrap_eq_iff]
  have := abs_le.mp hmove
  omega

/-! ## T2.1 Coding: a code injective on `2^a` configurations needs `a` bits -/

/-- Height-field surfaces of `a` voxels with heights in `{0,1}` are `2^a` distinct voxel sets
(`docs/v2_proofs.md`); any fixed-length code injective on them uses at least `a` bits. -/
theorem code_length {a m : ℕ} (f : (Fin a → Bool) → (Fin m → Bool)) (hf : Function.Injective f) :
    a ≤ m := by
  have hc := Fintype.card_le_of_injective f hf
  simp only [Fintype.card_fun, Fintype.card_bool, Fintype.card_fin] at hc
  exact (Nat.pow_le_pow_iff_right (by norm_num)).mp hc

/-! ## T2.11 Persistent memory budget (spec §5) -/

/-- Bytes of each persistent structure, in the order of the spec table. -/
def layout : List ℕ := [120, 16, 8, 8, 4, 36, 24, 12]

theorem memory_budget : layout.sum = 228 ∧ layout.sum ≤ 256 := by decide

/-! ## F4.5 candidate 1: certified futility (`docs/v2_proofs.md`, F4.5) -/

section Futility

variable {Cell Path C : Type*} [CompleteLattice C] (cost : Path → C) (cells : Path → Set Cell)

/-- F45.1. A believed plan cheaper than a sound lower bound on `V⋆` enters a truly blocked cell. -/
theorem futile_crosses {O : Set Cell} {L : C} (hL : L ≤ dist cost cells O) (p : Path) (hp : cost p < L) :
    ¬ Disjoint (cells p) O := fun hd =>
  lt_irrefl _ (hp.trans_le (hL.trans (sandwich_upper cost cells p hd)))

end Futility

/-! F45.2. The invariant `L ≤ d_K(p)`: kept by a search (`L ← max L d_K(p)`), by a move of cost `c` along an
arc that is free in the current map (triangle inequality `d_K(x) ≤ c + d_K(y)`), and by learning obstacles
(monotonicity, `dist_mono`). Hence, without forgetting, every plan of the current map costs at least `L`. -/

theorem inv_search {L d : ℝ} (h : L ≤ d) : max L d ≤ d := max_le h le_rfl

theorem inv_move {L dx dy c : ℝ} (h : L ≤ dx) (tri : dx ≤ c + dy) : L - c ≤ dy := by linarith

theorem inv_learn {L d d' : ℝ} (h : L ≤ d) (mono : d ≤ d') : L ≤ d' := h.trans mono

theorem no_futility {L d Φ : ℝ} (h : L ≤ d) (opt : d ≤ Φ) : ¬ Φ < L := not_lt.mpr (h.trans opt)

/-! ## F4.5 candidate 2: the v1 loss model in v2 -/

/-- Probability of the bad state `τ` slots after a good one, for the two-state chain with `P(G→B) = p`,
`P(B→G) = q`: `x₀ = 0`, `x_{τ+1} = x_τ (1 - q) + (1 - x_τ) p = p + (1 - p - q) x_τ`. -/
noncomputable def badAfter (p q : ℝ) : ℕ → ℝ
  | 0 => 0
  | τ + 1 => p + (1 - p - q) * badAfter p q τ

/-- F45.4 (v1 Prop. 4, second identity). `P(B_{t+τ} | G_t) = π_B (1 - λ^τ)`, `π_B = p / (p + q)`,
`λ = 1 - p - q`. -/
theorem badAfter_eq (p q : ℝ) (h : p + q ≠ 0) (τ : ℕ) :
    badAfter p q τ = p / (p + q) * (1 - (1 - p - q) ^ τ) := by
  induction τ with
  | zero => simp [badAfter]
  | succ n ih =>
    rw [badAfter, ih, pow_succ]
    field_simp
    ring

/-- F45.5. Collapsing the per-receiver beliefs `u β_j` after a copy into one scalar `u β̄`, with `β̄` the
value-weighted loss, keeps the expected value still to deliver exactly. -/
theorem belief_update {ι : Type*} (s : Finset ι) (V β : ι → ℝ) (u : ℝ) (hV : ∑ j ∈ s, V j ≠ 0) :
    ∑ j ∈ s, V j * (u * β j) = (∑ j ∈ s, V j) * (u * ((∑ j ∈ s, V j * β j) / ∑ j ∈ s, V j)) := by
  have h1 : ∑ j ∈ s, V j * (u * β j) = u * ∑ j ∈ s, V j * β j := by
    rw [Finset.mul_sum]
    exact Finset.sum_congr rfl fun j _ => by ring
  rw [h1]
  field_simp

/-! ## T2.9 and F45.3: the counting step of probabilistic completeness -/

/-- T2.9, last step, with F45.3: the `n`-th trigger starts an escape with probability `q n ≥ p` (`p = p_esc` for a
random trigger, `1` for a futility trigger) and an escape reaches `B` with probability at least `ε`, so
`P (n + 1) ≤ (1 - q n ε) P n` for the probability `P n` that the first `n` triggers all fail. Extra sure triggers
only lower the bound. -/
theorem fail_bound {p ε : ℝ} (q P : ℕ → ℝ) (hp : 0 ≤ p) (hε : 0 ≤ ε) (hε1 : ε ≤ 1) (hq : ∀ n, p ≤ q n)
    (hq1 : ∀ n, q n ≤ 1) (hP0 : P 0 ≤ 1) (hP : ∀ n, 0 ≤ P n)
    (hstep : ∀ n, P (n + 1) ≤ (1 - q n * ε) * P n) (n : ℕ) : P n ≤ (1 - p * ε) ^ n := by
  have hp1 : p * ε ≤ 1 := by nlinarith [(hq 0).trans (hq1 0)]
  induction n with
  | zero => simpa using hP0
  | succ k ih =>
    calc P (k + 1) ≤ (1 - q k * ε) * P k := hstep k
      _ ≤ (1 - p * ε) * P k := mul_le_mul_of_nonneg_right (by nlinarith [hq k]) (hP k)
      _ ≤ (1 - p * ε) * (1 - p * ε) ^ k := mul_le_mul_of_nonneg_left ih (by linarith)
      _ = (1 - p * ε) ^ (k + 1) := by ring

/-- T2.9: the bound of `fail_bound` tends to 0 when `p ε > 0`, so the agent arrives with probability 1. -/
theorem fail_tendsto {p ε : ℝ} (hpε : 0 < p * ε) (hpε1 : p * ε ≤ 1) :
    Filter.Tendsto (fun n : ℕ => (1 - p * ε) ^ n) Filter.atTop (nhds 0) :=
  tendsto_pow_atTop_nhds_zero_of_lt_one (by linarith) (by linarith)

/-! ## F45.6: commit opportunities in a launch schedule -/

/-- F45.6 (a): a walk of unit moves changes a coordinate by at most one per slot, so the first arrival takes
`T₁ ≥ ‖B - A‖∞` slots (apply it to each coordinate). -/
theorem reach_time (x : ℕ → ℤ) (hstep : ∀ k, |x (k + 1) - x k| ≤ 1) (t : ℕ) : |x t - x 0| ≤ t := by
  induction t with
  | zero => simp
  | succ n ih =>
    have := abs_sub_le (x (n + 1)) (x n) (x 0)
    push_cast
    linarith [hstep n]

/-- F45.6 (b): with launches at `i Δ`, the agents that launch after a route can reach `A` (slot `m = T₁ + δ`) are
exactly `i ∈ (m / Δ, N)`: `N - (m / Δ + 1)` of them. -/
theorem commit_count (N Δ m : ℕ) (hΔ : 0 < Δ) :
    ((Finset.range N).filter (fun i => m < i * Δ)).card = N - (m / Δ + 1) := by
  have h : (Finset.range N).filter (fun i => m < i * Δ) = Finset.Ico (m / Δ + 1) N := by
    ext i
    simp only [Finset.mem_filter, Finset.mem_range, Finset.mem_Ico]
    rw [← Nat.div_lt_iff_lt_mul hΔ]
    omega
  rw [h, Nat.card_Ico]

end GraceV2

end
