# The Nonlinear MPC Controller (NMPC)

Full technical reference for the second, separately selectable controller,
`nmpc_core.NMPCController`, chosen by the node parameter `use_nmpc` (default
false). Split out of `architecture.md` because this material is large enough
to be its own document; that file now only summarises and links here.

For the default linear controller, see [`lmpc.md`](lmpc.md). For the
worked-by-hand arithmetic behind `e_y`/`e_psi`, see
[`error_state_reference.md`](error_state_reference.md).

Everything in [`lmpc.md`](lmpc.md) describes `mpc_core.MPCController`:
a linear time-varying MPC solved as one convex QP per tick. The
live workspace carries a **second, separately selectable** controller,
`nmpc_core.NMPCController`, described here. This repo has its own offline
port, `controller/nmpc_optimiser.py`,
selected by `settings.USE_NMPC`; this doc is a pointer to the live design,
not a mirror of the offline code, see `docs/reference/control_mechanisms.md`'s
"Nonlinear MPC (`use_nmpc`)" section for the offline port's specifics.

## The structural difference, in one line

In plain terms: the LTV-QP
plans ahead as though the road stays pointed the same direction for the
whole horizon, even if a corner is coming up; the NMPC's internal model
actually knows the road bends, and where. The LTV-QP predicts how the car's
current error (`e_y`, `e_psi`) drifts under its own dynamics, against a
reference direction it treats as fixed for the whole horizon. The NMPC
predicts that same error's evolution **relative to a path whose bend is
itself part of the prediction**, the model knows the reference direction
changes with `s`, not just the car's state.

**Both controllers measure their current-tick error the same way**: the
Frenet-frame projection described in
[`lmpc.md`'s "How the error vector is measured"](lmpc.md#how-the-error-vector-is-measured-frenet-frame-projection)
(`_error_state()` in `mpc_core.py`, `PathReference.project()` in
`nmpc_core.py`, same nearest-point-plus-perpendicular-offset arithmetic).
Frenet-frame measurement is not what tells them apart. What differs is
what happens to that error **over the prediction horizon**, after this
tick's measurement:

The LTV-QP takes its one Frenet measurement at the current tick, then
predicts forward in fixed error coordinates with the reference frame's
rotation dropped, so `e_psi_dot = r` instead of
`e_psi_dot = r - kappa(s)*s_dot`. Arc length `s` never appears as a
predicted state, curvature is sampled once (the ~1 m preview lookup in
`_error_state()`) and held fixed for the whole horizon. With the car on
line and a corner ahead, its 35-step rollout predicts staying on line
forever (measured: exactly 0.000 deg commanded at 8 dead-on-line states),
which is why the "structural limit" callout in `removed_mechanisms.md`
exists and why the adaptive lookahead layer had to be invented.

The NMPC instead carries `s` itself as a horizon *state*: at every one of
its 20 predicted steps, `kappa(s)`/`psi_ref(s)` are looked up fresh at that
step's predicted `s`, not sampled once at the current tick. So the road's
bend is re-evaluated at every future point along the plan, not frozen at
one lookahead distance the way the LTV-QP's preview curvature is. As `s`
advances along the predicted horizon, `kappa(s)` changes with it, so a
bend 10 steps out is already shaping the plan today, not just once the
car arrives there.

## Structure and solve method

**Structure**: states `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs
`[delta_cmd, a_cmd]`, linear-tyre bicycle dynamics with the same constants and
the same low-speed kinematic blend as the LTV-QP, plus a `tanh` saturation of
the predicted lateral force at FSDS's measured `a_lat` ceiling.

**How it's solved (Gauss-Newton SQP)**, step by step each tick:

1. **Roll the nonlinear model forward** from the car's actually-measured
   state, not an approximation, the real equations from
   [`lmpc.md`'s "Building the prediction model"](lmpc.md#building-the-prediction-model-modelbicycle_modelpy).
   Starting from a real state means the rollout is always physically
   consistent with where the car actually is (no "dynamics defect" to
   correct for later).
2. **Linearise around that rollout**: compute how a small change in each
   input would change the predicted trajectory, via finite-difference
   Jacobians (the same "how much does nudging this affect that" idea the
   `A`/`B` matrices in `lmpc.md` capture, just recomputed fresh around
   this tick's specific rollout instead of a fixed formula). Done for every
   horizon stage at once (vectorised), not one at a time.
3. **Condense into a QP**: fold the whole 20-step problem down into one
   solved for input *changes* only (not full trajectories), which OSQP can
   solve the same way it solves the LTV-QP's problem.
4. **Solve with a trust region**: cap how large a step OSQP is allowed to
   take from this tick's rollout, since the linearisation from step 2 is
   only accurate near it.
5. **One iteration per tick, warm-started from last tick's answer**
   ("real-time iteration"), rather than looping steps 1-4 until full
   convergence within a single tick, which would risk missing the 50 ms
   deadline.

Horizon 20 steps (1.0 s); measured solve time mean 8.9 ms, p95 11.6 ms.

**Consequences for the rest of the architecture**: when `use_nmpc=true` the
entire adaptive gain schedule, the precomputed corner map and the shaped
heading-lead profile are all inactive (each was a workaround for the missing
curvature term), and the telemetry CSV's `m_*` columns are empty while eight
`nmpc_*` columns carry solver/prediction diagnostics instead. The composite
score, the scoring pipeline, the path/speed-profile plumbing and the delay
compensation are unchanged.

## Feature comparison: LTV-QP vs. NMPC, at a glance

Every feature below is verified against actual read-sites in the code, not
inferred from a docstring or field name, see
`docs/reference/README.md`'s "Which settings affect which controller" map
for the exhaustive, field-by-field version this table summarises.

| Feature | LTV-QP (`mpc_core.py`) | NMPC (`nmpc_core.py`) | Why |
|---|---|---|---|
| Adaptive gain scheduling (`_corner_factor`, anti-hunt, `adaptive_Q_scaling`, `adaptive_R_scaling`, `adaptive_R_rate`) | **Yes** | **No** (inert, none of these fields have any read site in `nmpc_core.py`) | Every one of these mechanisms exists to compensate for the LTV-QP's blind spot (it can't predict the path curving). NMPC's model has that built in structurally, so reweighting the cost on top would double-count an effect that's now already handled, see [`removed_mechanisms.md` §1](removed_mechanisms.md#1-the-structural-limit-the-argument-that-motivates-nmpc). |
| `steer_rate_anti_hunt` (steering-rate damping when centred/aligned/uncurving) | **Yes**, on by default | **Opt-in**, off by default (`nmpc_steer_rate_anti_hunt_enabled`) | The one exception to the row above: it only ever makes steering *more* damped in a specific narrow case, the opposite direction from anticipation, so it doesn't fight NMPC's structural fix the way the rest of the gain schedule would. Reuses the LTV-QP's own function verbatim (imported, not reimplemented). |
| Precomputed corner map (`use_precomputed_corner_map`) | Removed from both | Removed from both | Served the deleted lookahead gain-scheduling family, gone from both controllers, not an LMPC/NMPC difference. See [`removed_mechanisms.md` §7](removed_mechanisms.md#7-precomputed-corner-segmentation-cornermap). |
| Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`) | **Yes** | **Accepted but ignored** (`set_heading_profile()` exists so the node needs no branch, logs a one-time warning) | Same reasoning as gain scheduling: the shaped lead is a workaround for the same missing curvature term NMPC closes structurally. Applying both would double-count the anticipation. |
| Delay/latency compensation (rolling `x0` forward through recently-issued commands) | **Yes** (`predict_ahead()`, linear rollforward) | **Yes** (rolls `x0` forward through the nonlinear model instead) | Both need this, it's about *sensor/actuation lag*, a problem that exists regardless of which prediction model is used. Different implementation, same four gating fields (`delay_compensation_enabled`, `max_delay_compensation_steps`, `pose_age_lp_alpha`, `n_delay_hysteresis`), shared `MPCParams` fields, read by both. One exception: `predict_epsi_clip` is LTV-QP only (a small-angle bound specific to the *linear* rollforward; NMPC's nonlinear rollforward has no such bound to set). |
| Tracking-error speed gate (slow down when `e_y`/`e_psi` are large) | **Yes** | **Yes** | This lives in `control_utils.py`, called by the **node** (`mpc_controller.py`) *before* either controller's `.compute()` is invoked, neither `MPCController` nor `NMPCController` is even aware it exists. Controller-agnostic by construction. |
| Curvature-based speed profile (`curvature_speed()`) | **Yes** | **Yes** | Same reason as the row above: computed by the node, handed to whichever controller is selected as `desired_speed`. |
| Cone-proximity emergency braking, GO-gating, stale-path fail-safes | **Yes** | **Yes** | All node-level (`mpc_controller.py`'s `_control_step` phases, `standalone_output=true` only), not part of either controller class. `NMPCController` exposes the same `compute()`/`reset()`/`set_static_path()` surface as `MPCController` specifically so the node doesn't need a branch. |
| FSDS lateral-acceleration ceiling | **Yes**, as a plain speed-profile input (`curvature_speed()`'s friction-circle cap) | **Yes**, AND inside the prediction itself (`tanh` saturation on predicted tyre force) | NMPC's version is strictly more: the ceiling shapes what the *solver itself* believes is achievable, not just the requested speed. Without it, NMPC's linear-tyre model believes it can hold any corner at any speed and the car spins (measured). |
| Horizon length | 35 steps (1.75 s) | 20 steps (1.0 s) | Independent tuning choices, not a structural requirement, NMPC's shorter horizon reflects its per-tick solve cost (Gauss-Newton SQP is more expensive per step than one convex QP). |
| Solve method | One convex QP per tick (OSQP) | Real-time-iteration SQP: one Gauss-Newton step per tick, warm-started, condensed dense QP (OSQP) | See [`lmpc.md`'s "The solver"](lmpc.md#the-solver) for what a QP is; NMPC needs the extra linearize-and-resolve step because its own model is nonlinear (curvature is now a function of a state, not a fixed matrix entry). |

**Three further, NMPC-only additions**, assessed against
Alexander Liniger's Model Predictive Contouring Control (MPCC) but narrower
than it: full MPCC's progress-maximisation apparatus was considered and
rejected as too close to a failure mode already eliminated here (see
`docs/reference/README.md`'s writeup for why). One is on by default, two are
off:

- `nmpc_spline_reference_enabled` (default **true**): `PathReference`'s
  `kappa(s)`/`psi_ref(s)` come from an analytic cubic-spline fit to the
  waypoints instead of moving-average-smoothed finite differences. A
  numerical-quality fix, not a new coupling to the solver.
- `nmpc_horizon_speed_profile_enabled` (default **false**, experimental):
  samples a precomputed speed profile at each horizon stage's own predicted
  arc length, the same state-keyed pattern `kappa(s)` already uses, instead
  of holding one frozen speed target across the horizon.
- `nmpc_friction_circle_enabled` (default **false**, experimental): a hard
  per-axle tyre-force bound in the QP, additional to (not replacing) the
  existing soft `alat_ceiling` saturation.

All three are implemented identically in `nmpc_core.py` and the offline
`controller/nmpc_optimiser.py`; none touch `mpc_core.py` (the LTV-QP).

Full detail: `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section
(what it is, what it reuses, what is inactive, offline A/B numbers, the offline
port, a matched same-day LIVE A/B (steering saturation 6.45% → 0.58%,
lap 54.72s → 52.35s) and the "Which settings affect which controller" map
and the three MPCC-inspired additions above), `tuning.md` §4.5d (tuning
surface), and `late_turn_in_investigation.md` Part 16 (research survey,
formulation choice, validation, the four bugs found in testing).
