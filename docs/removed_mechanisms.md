# Removed Mechanisms: The Lookahead Gain-Scheduling Family

This doc is the single home for a whole family of mechanisms that **no
longer exist in the code**, removed in the 2026-08-13 corner-factor rewrite.
They're preserved here, in one place, for two reasons:

- **The elimination reasoning is the direct motivation for the nonlinear MPC
  (NMPC).** Understanding why this family had to fail structurally is the
  fastest way to understand why `nmpc_core.py` exists and what it does
  differently — see [Section 1](#1-the-structural-limit-the-argument-that-motivates-nmpc).
- **Several of these were tried, measured, and rejected for specific,
  non-obvious reasons.** Without this doc, it's easy to reinvent one of them
  from scratch and rediscover the same failure the hard way.

**None of the fields, functions, or flags described below exist on
`MPCParams`, `mpc_core.py`, or `controller/model_utils.py` today.** If
you're looking for what tuning knobs currently exist, see
[`tuning.md`](tuning.md) instead — it links back here only for history.

---

## Table of contents

- [1. The structural limit: the argument that motivates NMPC](#1-the-structural-limit-the-argument-that-motivates-nmpc)
- [2. What the whole family did, at a glance](#2-what-the-whole-family-did-at-a-glance)
- [3. Lookahead corner anticipation](#3-lookahead-corner-anticipation)
- [4. Demand normalisation](#4-demand-normalisation)
- [5. U-turn detection](#5-u-turn-detection)
- [6. Straight-line adjustments](#6-straight-line-adjustments)
- [7. Precomputed corner segmentation (`CornerMap`)](#7-precomputed-corner-segmentation-cornermap)
- [8. Curvature forcing: the closest attempt, and why it still failed](#8-curvature-forcing-the-closest-attempt-and-why-it-still-failed)
- [9. Low-speed steering-rate boost](#9-low-speed-steering-rate-boost-removed)
- [10. FSDS lateral-acceleration ceiling as a lookahead input](#10-fsds-lateral-acceleration-ceiling-as-a-lookahead-input)
- [11. What replaced all of this](#11-what-replaced-all-of-this)

---

## 1. The structural limit: the argument that motivates NMPC

It's tempting to read every mechanism below as "the MPC looks ahead at the
path, notices it curves, and plans to turn early." **That is not what
happens**, and the distinction is the single most important thing to take
from this doc.

- Every mechanism here computes something real — a genuine forward scan of
  the path ahead, finding real upcoming curvature (`kappa_max_abs`).
- But that number only ever reaches the solver as a **reweighting** of the
  cost function (`Q[0,0]`, `Q[2,2]`, `R[0,0]`, ...). It changes how
  *expensive* an existing tracking error is. It never changes what the
  solver's own prediction of the future *looks like*.
- The solver's internal model of "what happens over the next 35 steps"
  (`Ad`/`Bd`, see [`architecture.md`'s "Building the prediction
  model"](architecture.md#building-the-prediction-model-modelbicycle_modelpy))
  has **no path-curvature term at all**. Given the car dead on-line
  (`e_y ≈ e_psi ≈ 0`, exactly the situation on the straight approach to a
  corner) and no other input, that model predicts `e_y`/`e_psi` staying at
  ≈0 for the whole horizon — corner or no corner — because nothing in the
  model represents the road bending away from the car.

**Consequence:** every boost/relaxation below only helps once *some* real
tracking error already exists to reweight. None of them can make the
controller *start* turning while `e_y ≈ e_psi ≈ 0`, no matter how cheap
steering is made. This was measured directly on 2026-08-12: `kappa_max_abs`
and the derived boosts moved correctly and over a full second early
(`m_R_steer_relax` falling to ~0.55, `Q_ey_eff` climbing from 2.5 to 4.5+),
while `steer_deg` stayed at ≈0° the entire time — because `e_y`/`e_psi`
stayed at ≈0 throughout. Turn-in didn't begin until the car had already
entered the curved section and *current-position* curvature started feeding
real state error.

**The actual fix needed a model that predicts the road bending, not a
cheaper cost on a road that already bent.** That's what the nonlinear MPC
(`nmpc_core.py`, `use_nmpc`) does — see
[`architecture.md`'s "Second controller: nonlinear MPC"
section](architecture.md#second-controller-nonlinear-mpc-use_nmpc)
or, for the full derivation with worked numbers,
[`error_state_reference.md`](error_state_reference.md).

---

## 2. What the whole family did, at a glance

| Mechanism | What it scanned for | What it reweighted |
|---|---|---|
| Lookahead corner anticipation | Peak curvature in a speed-scaled window ahead | `Q[0,0]`/`Q[2,2]` (up), `Q[3,3]` (down) approaching/exiting a corner |
| Demand normalisation | Same, scaled by how much of available grip the corner needs at the current speed | Made the boosts above scale-free across corner types/speeds |
| U-turn detector | Accumulated heading change in the window (not just peak curvature) | Extra `Q[0,0]`/`Q[2,2]`/`Q[3,3]` boost for long, gradual turns |
| Straight-line adjustments | Window genuinely clear of curvature | `Q[0,0]` down, `Q[2,2]`/`Q[3,3]`/`R[0,0]` up |
| Precomputed corner segmentation (`CornerMap`) | Same scan, but precomputed once per static path instead of live | Replaced the live scan with an exact index lookup — same reweighting, cheaper |
| Curvature forcing | Curvature at each *horizon step's* predicted arc length | Injected directly into the predicted dynamics (not a cost reweight) — the one attempt that tried to fix Section 1's actual limit |
| Low-speed steering-rate boost | Car speed alone (no curvature gate) | `R_rate[0,0]` (steering smoothness) |
| alat-ceiling as lookahead input | — (a measured plant constant, not a scan) | Fed `_corner_demand` above; the ceiling law itself lives on |

---

## 3. Lookahead corner anticipation

**`adaptive_Q_lookahead` / `ADAPTIVE_Q_LOOKAHEAD_ENABLED`**

Every *current-state* mechanism reacts to curvature the car is at right
now. This one instead scanned a speed-scaled window of path ahead —
`lookahead_curvature_profile(path, base_idx, lookahead_dist)`, with
`lookahead_dist = clip(vx · 1.13 s, 3 m, 17 m)` — for the sharpest curvature
coming up (`kappa_max_abs`) and the total accumulated heading change over
that window.

- **Approaching a corner:** boosted `Q[0,0]` (lateral error, ceiling 2.0×)
  and `Q[2,2]` (heading error, ceiling 1.5×) so the controller committed
  steering authority before drifting, and relaxed `Q[3,3]` (yaw rate, floor
  0.5×) so a high straight-line yaw-rate penalty didn't itself make turn-in
  feel slow. Added after a live log showed steering only starting to ramp
  ~0.6 s before saturating, by which point `e_y` had already grown to
  -1.86 m.
- **Exiting a corner:** continued boosting `Q[2,2]` for a short distance
  afterward (`decay_dist`, 5 m), linear decay, scaled by how sharp the
  corner was. Used a rising-edge-after-a-clear-peak detector rather than a
  running maximum, since a running max would silently fail to re-trigger on
  a second corner of equal or lesser curvature — an ordinary case on tracks
  that reuse corner radii.
- **On a clear straight** (`kappa_max_abs → 0`): softened `Q[0,0]` (floor
  0.7×) and mildly boosted `Q[2,2]`/`Q[3,3]` (ceilings 1.1×/1.5×). Added
  after residual hunting persisted despite the boosts above. `Q[2,2]`'s
  ceiling was kept small deliberately — a stronger heading-error weight on
  a straight amplifies the QP's reaction to ordinary heading noise, the
  exact small-error hunting `adaptive_Q_scaling` (still live today) exists
  to fight.

**Later addition, also removed: `lookahead_steer_effort_relax`.** Neither
`adaptive_R_scaling`'s speed-based steering penalty nor the straight-line
`R[0,0]` boost (Section 6) ever pushed `R[0,0]` *below* baseline for an
approaching corner — so a car entering a corner hot still paid the full
speed-based steering-effort cost right when it most needed to commit. This
relaxed `R[0,0]` toward a floor as corner demand rose, mirroring the
yaw-rate relaxation above.

**Known constraints, historically:** never validated against
`VALIDATION_SUITE`/a recorded map as a whole mechanism — treat as
experimental even in hindsight. And per Section 1: this whole mechanism
only reweights the cost of an *existing* error; it cannot manufacture one.

---

## 4. Demand normalisation

**`ADAPTIVE_Q_DEMAND_NORMALISED`, on by default while it existed**

The boosts in Section 3 were driven by corner **demand**, not raw
curvature:

```
kappa_limit(v) = a_lat_ceiling(v) / v²    # tightest curvature holdable before FSDS's lateral-accel ceiling binds
demand         = kappa_max_abs / kappa_limit(v)
```

`demand ≈ 0` means straight, `≈ 1` means "this corner needs everything
available at this speed," `> 1` means it cannot be held (must slow).

**Why not raw curvature:** the raw-curvature curve turned out badly
mis-scaled against real corner radii — the whole range of curvatures the
car actually drives sat in the flat, low-response part of the curve, so
raising the boost ceiling barely changed anything in practice:

| Corner | Raw curvature | Boost reached (raw curve) |
|---|---|---|
| Gradual sweeper (R = 40 m) | κ = 0.025 | 17% |
| Typical corner (R = 12 m) | κ = 0.083 | 40% |

**Why demand fixed it:** scale-free and speed-aware, so a gradual sweeper
taken fast and a tight corner taken slow are judged by the same criterion —
one set of constants covers both instead of a hand-tuned threshold per
corner type.

---

## 5. U-turn detection

Every boost in Section 3 keyed off `kappa_max_abs` — peak curvature
*magnitude* — which under-scores a long, gradual U-turn: a large radius
means unremarkable peak curvature even though the turn demands a huge total
rotation.

- **Mechanism:** the same lookahead scan also returned accumulated
  `|heading change|` over the window. Past 60° of accumulated turning, an
  extra multiplicative boost applied to `Q[0,0]`/`Q[2,2]` (ceiling 1.6×
  each) and `Q[3,3]` (floor 0.6×), scaling to full strength by 120°.
- **Why 60°, not the 90° "U-turn" might suggest:** the threshold was
  measured *within* the lookahead window, not over the whole corner — 17 m
  of arc at a 12 m corner radius only subtends ~81°, so a 90° threshold
  could never fire on approach.
- **Scope limit:** this only ever helped *before* a corner, while steering
  was still unsaturated. On the log that motivated it, the controller was
  already at the full steering stop for over a second mid-corner, with
  achieved curvature varying 6× at constant steering input due to speed
  alone — the binding constraint there was FSDS's lateral-acceleration
  ceiling, not steering angle, and no `Q` boost can add steering that's
  already saturated.

---

## 6. Straight-line adjustments

Three independent mechanisms, active only when the lookahead window was
genuinely clear of curvature, each fading back to baseline sharply as a
corner entered the window:

- **`adaptive_q_straight_ey_floor`** — reduced `Q[0,0]` on a clear straight
  (nothing to track hard against), fading back to full authority as a
  corner appeared. Its fade sharpness was lowered from 20.0 to 8.0 after
  the old, sharper fade left the car still mid-recovery from the
  relaxation exactly when turn-in needed full lateral authority.
- **`adaptive_q_straight_epsi_boost_max`** / **`_r_boost_max`** — boosted
  `Q[2,2]`/`Q[3,3]` on a straight to keep the car pointed straight and damp
  yaw wander. Kept deliberately small — a strong straight-line heading
  weight amplifies the QP's reaction to ordinary heading noise, which can
  itself introduce oscillation, the opposite of the intent.
- **`steer_effort_straight_boost`** — the `R[0,0]` (steering *effort*, not
  its rate of change) counterpart: 1.5× on a clear straight, fading toward
  1.0 as a corner entered the window, sharper than the `Q`-side fades so it
  collapsed to baseline almost as soon as a real turn was needed.

Composition order mattered: the lookahead boost applied to `Q` first, then
`adaptive_Q_scaling`'s centred-softening (still live today) multiplied on
top — so a corner boost was never silently cancelled by the centred
softening.

---

## 7. Precomputed corner segmentation (`CornerMap`)

**`use_precomputed_corner_map`, added 2026-08-12, removed the next day**

A `CornerMap` dataclass, built once per static path (`mpc_core.py`'s
`_segment_corners`), replaced the live per-tick forward scan with an exact
index lookup — same reweighting as Sections 3-6, computed once instead of
scanned every tick. It existed for barely a day before the whole family it
served was deleted in the corner-factor rewrite.

Offline-validated (regression-checked bit-for-bit identical when disabled,
measurably different when enabled), never live-tested before removal. See
[`docs/reference/`](`docs/reference/`) for what replaced
it.

---

## 8. Curvature forcing: the closest attempt, and why it still failed

**`curvature_forcing_enabled` / `CURVATURE_FORCING_ENABLED`, added and
disabled 2026-08-12, fully removed 2026-08-13**

Every mechanism above only reweighted the *cost* of an existing tracking
error. This one tried something structurally different: injecting the
path's curvature directly into the **predicted dynamics**, not the cost.

**Mechanism:** look up the reference path's curvature at each horizon
step's own predicted arc-length position, and add a forcing term to the
predicted `e_psi` there:

```
predicted_e_psi[k+1] += -v_x * kappa(s_k) * dt * curvature_forcing_gain
```

This comes directly from `path_yaw_rate = v_x·κ` — a physically-motivated
term, not another cost reweight.

**Why it still failed — a gain sweep found no working operating point:**

- At `gain = 1.0` (physically exact), the QP's own `e_psi` decay
  (`Ad[2,2] ≈ 0.946` per step) bled off the forcing almost as fast as it
  accumulated — the resulting steering response was under 1°, noise-scale,
  too weak to matter.
- Raising the gain to compensate made it worse in a *new* way: past
  `gain ≈ 6`, the QP's cheapest predicted trajectory involved steering hard
  **away** from the corner first, then reversing.
- Only `gain ≈ 20` restored the correct net direction, by which point
  steering was saturated the entire time anyway.

**Why this happened, mechanically:** the forcing term was added to the same
dynamics recursion the QP minimises total cost over — giving the solver
complete freedom to choose the *cheapest way to absorb it* across the whole
horizon, which is not the same thing as "track the bend." A brief
wrong-direction dip can be mathematically cheaper in total squared cost
than committing immediately. No gain was simultaneously large enough to
produce meaningful anticipation and free of this wrong-direction transient
— **a structural property of forcing terms inside the dynamics constraint,
not a tuning gap.**

This is the single most important lesson from the whole family: getting
curvature into the *dynamics* (rather than just the cost) was the right
instinct, but injecting it as **external data the solver is free to defer**
doesn't work. The fix that actually worked (NMPC) makes curvature a
function of a **state the solver is actively choosing** (arc length `s`),
so there's no separate slot to schedule around — see
[`error_state_reference.md` Section 3](error_state_reference.md#3-why-the-ltv-qp-cant-just-re-project-at-every-future-step)
for the direct comparison.

Full numeric gain-sweep trace and the anti-hunt interaction found alongside
this: [`docs/reference/README.md`'s "Curvature-forcing
term"](`docs/reference/`) section.

---

## 9. Low-speed steering-rate boost — removed

`low_speed_steer_rate_boost` no longer exists in either codebase — the
function, its `MPCParams`/`settings.py` fields, and its telemetry column
have all been deleted along with the rest of the lookahead
gain-scheduling family.

- **Purpose:** damp a low-speed (3-4 m/s) post-corner-exit steering wobble
  that neither the anti-hunt nor corner-softening mechanisms touched, since
  both gate on curvature/tracking-error rather than speed.
- **Shape:** deliberately inverted from a Stanley-style `k/(v+eps)` curve —
  it made *fast* steering-rate changes **more** expensive at low speed,
  not cheaper.
- **Why it was disabled, then removed:** live-tested and found to also
  suppress fast turn-in, which also needs a fast steering-rate change at
  low speed. Speed alone can't distinguish "post-exit overcorrection" from
  "turn-in," so it taxed both identically.
- **If a future rework revisits this idea, it needs a curvature/lookahead
  gate** so it only fires when the car is *not* approaching or inside a
  corner. The tuned values from the original attempt, kept here as a
  starting point: `boost_max = 2.5, k = 0.35`.

See `docs/logs/late_turn_in_investigation.md`'s "Appendix — Low-speed
steering-rate boost" for the full incident, and
[`docs/reference/superseded_mechanisms.md`'s "Low-speed steering-rate boost"](`docs/reference/`)
for the current-state pointer.

---

## 10. FSDS lateral-acceleration ceiling as a lookahead input

The measured ceiling law (`a_lat_max(v) = max(FLAT, SLOPE·|v| + INTERCEPT)`)
used to live on `MPCParams` as three fields
(`alat_ceiling_flat`/`_slope`/`_intercept`) so it could feed the demand
normalisation in Section 4. **The ceiling law itself is not removed** — it
moved to `nmpc_core.py`'s `_Plant` class (hardcoded there, since only the
NMPC path uses it now) and `model/vehicle_physics.py`'s `alat_ceiling_at()`.
Only its role as a *lookahead-scan input* is gone, along with the rest of
this family.

This is a measured property of the simulator, not a tuning knob — see
`docs/reference/simulator_fidelity.md`'s "The sim-to-real gap" section for
the measurement, and re-measure with `ros2/run_steering_sysid.sh` /
`ros2/run_steering_step.sh` if it's ever suspected wrong, rather than
guessing.

---

## 11. What replaced all of this

A single, much simpler mechanism: `_corner_factor` / `_low_speed_corner_boost`
/ `_blend` — one continuous **current-curvature-only** fraction blending
four `Q`/`R_rate` weights between a straight endpoint and a corner endpoint,
plus an independent, always-on heading-error-driven accel/brake asymmetry.

This deliberately does **not** attempt to replicate the lookahead scanning
above — per Section 1's argument, a forward scan can only reweight costs,
never fix the underlying blindness, so the rewrite stopped trying to patch
around it with current-state-only logic instead, and left the *actual* fix
(seeing the corner in the prediction itself) to the NMPC.

Full formulas: [`architecture.md`'s "Corner-factor
scheduler"](architecture.md#corner-factor-scheduler)
section. Tuning-surface reference: [`tuning.md` §4.3b](tuning.md#43b-corner-factor-scheduler).
