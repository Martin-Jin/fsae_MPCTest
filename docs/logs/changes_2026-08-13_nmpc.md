# Change log: 2026-08-13 (nonlinear MPC only)

**Scope note, read first:** this entry covers only the nonlinear-MPC (NMPC)
work described below. It does **not** continue from
[`changes_2026-08-07_to_2026-08-09.md`](changes_2026-08-07_to_2026-08-09.md)
(which ends at Stage 27, 2026-08-09) — there is an unlogged gap from
2026-08-10 through 2026-08-12 covering the precomputed corner-map feature,
the shaped heading-lead profile, and a base-weight retune, none of which this
entry describes. Those are documented in `planning_control_sync.md`'s own
dated sections ("Precomputed corner segmentation", "Precomputed shaped
heading-lead profile") rather than in this changelog series. Numbering below
restarts at Stage 1 rather than continuing at 28, to avoid implying this
entry picks up where Stage 27 left off.

## Index

| Stage | What happened |
|---|---|
| [1](#stage-1-the-standing-structural-limit-a-linear-model-cant-see-the-road-bend) | The standing structural limit: a linear model can't see the road bend |
| [2](#stage-2-why-not-just-bolt-curvature-onto-the-existing-model-three-prior-attempts-all-failed-the-same-way) | Why not just bolt curvature onto the existing model — three prior attempts, one shared failure mode |
| [3](#stage-3-decision-a-second-controller-frenet-frame-nonlinear-mpc) | Decision: a second controller, Frenet-frame nonlinear MPC |
| [4](#stage-4-building-it-what-a-nonlinear-model-actually-costs-per-tick) | Building it: what a nonlinear model actually costs per tick |
| [5](#stage-5-four-bugs-that-only-showed-up-in-closed-loop) | Four bugs that only showed up in closed loop |
| [6](#stage-6-the-honest-numbers) | The honest numbers |
| [7](#stage-7-what-this-does-not-fix) | What this does not fix |

---

## Stage 1: the standing structural limit — a linear model can't see the road bend

`mpc_core.MPCController`'s prediction model (`_discrete_model`) is the bicycle
model in error coordinates, and the reference path's own rotation is entirely
absent from it: `e_psi_dot` in the model is just `r` (the car's own yaw rate),
never `r - kappa(s)*s_dot`. Consequence, checked directly rather than assumed
here: place the car exactly on the centreline, exactly on heading
(`e_y = e_psi = 0`), with a real corner ahead — the QP's own 35-step rollout
predicts staying at zero forever, regardless of how the cost weights are set,
because nothing in the dynamics represents the path bending. Measured: the
LTV-QP commands **exactly 0.000 degrees** at 8 separate dead-on-line states
approaching a known bend, at two different speeds. Not "small" — exactly
zero, matching what the model implies.

This is the same "late turn-in" problem the live repo's
`late_turn_in_investigation.md` has been chasing across 15 prior Parts via
every lever available to a linear model: reweighting costs, gating adaptive
gain schedules, precomputing corner geometry, shaping the heading reference
ahead of time. Each of those helped at the margins but none could close the
structural gap, because none of them changes what the model's own rollout
predicts.

## Stage 2: why not just bolt curvature onto the existing model — three prior attempts, one shared failure mode

Documented in `late_turn_in_investigation.md` Parts 2, 7, and 15: feeding path
curvature into the QP's dynamics as an extra disturbance term, shifting the
cost function's per-step target ahead of time, and extending the shaped
heading-lead profile to a full per-horizon-step reference. All three produced
a **wrong-direction transient** — the solver steering briefly the wrong way
before committing to the correct direction — confirmed via isolated synthetic
QP tests in each case, not inferred from a live log.

The shared root cause: in every one of those three designs, `kappa` (or the
target it feeds) is exogenous data known at solve time and indexed by horizon
step — i.e. "you'll owe this correction at step k." That gives the solver
freedom to choose *when* within the plan to pay for it, and an early
wrong-direction dip can integrate to a lower total quadratic cost than
committing immediately. This applies identically whether the future
obligation is phrased as a dynamics disturbance or a cost target — the
mechanism, not the specific implementation, is what's unsound.

## Stage 3: decision — a second controller, Frenet-frame nonlinear MPC

Surveyed candidate formulations (Frenet-frame tracking NMPC, Model Predictive
Contouring Control, Cartesian NMPC with online projection, an outer SQP loop
around the existing error-coordinate model) before choosing. Recommendation:
**Frenet-frame nonlinear MPC**, because it's the only one that removes the
structural gap by construction (arc length `s` becomes a state, `kappa(s)` is
looked up from it — no longer horizon-indexed exogenous data, so there's no
"pay it later" freedom to exploit), it preserves the reference/score/weights
this whole project is built around (unlike MPCC, which abandons the tracking
objective for progress-maximisation), and it needs no new runtime dependency
(`osqp` is already required via `cvxpy`).

Built as `nmpc_core.NMPCController` — a genuinely separate class, selected by
a single flag (`use_nmpc`, default **false**). `mpc_core.py` was not touched
at all (confirmed by file mtime predating this work). No CasADi/acados in the
shipped code: CasADi isn't installable into the ROS interpreter on Ubuntu
24.04 without `--break-system-packages`; the SQP and its Jacobians are plain
numpy. CasADi + IPOPT were used once, from a private install, purely as an
independent cross-check (Stage 6).

## Stage 4: building it — what a nonlinear model actually costs per tick

First working version cost 42 ms/tick — too slow, mostly the horizon rollout
(16.7 ms) going through a numpy-vectorised model where per-call overhead
dominates at this array size. Fixed with a hand-mirrored **scalar** fast path
for the sequential rollout (verified to agree with the vectorised form to
7.2e-16 relative difference over 300 randomised states), cutting the rollout
to ~1 ms. Similarly, the QP's own sensitivity Jacobians deliberately use
coarser RK substeps than the rollout itself — they only set a step
*direction*, never the predicted trajectory, so a slightly coarser
sensitivity costs at most a slightly worse step rather than a wrong
prediction.

A horizon/iteration sweep (closed-loop, identical weights, same track) found
two results that both contradict the obvious assumption: a **longer**
horizon (35 steps, matching the LTV-QP's own) tracked *worse* than a shorter
one (20 steps), because the prediction model is optimistic (linear tyres, no
suspension) and that mismatch compounds over a longer horizon; and **more**
SQP iterations per tick measured slightly worse as well as ~2x more
expensive, consistent with the real-time-iteration literature (the warm
start already carries convergence across ticks). Shipped defaults: 20 steps
(1.0 s), 1 iteration/tick, ~9 ms mean solve time.

## Stage 5: four bugs that only showed up in closed loop

None of these were visible in isolated synthetic tests — each only appeared
once the controller was driven around the real `comp_test_map_3` raceline
against `fsae_MPCTest`'s own 25-state Pacejka plant:

1. **An infeasible QP subproblem was accepted as a step direction.** OSQP
   returns a finite-but-meaningless answer when the subproblem is
   primal-infeasible; the first version checked only for NaN. Fixed by
   checking the solver status explicitly, and — the actual fix — by
   projecting the warm-start input trajectory onto the slew-feasible set
   first, so the subproblem is unconditionally feasible by construction.
2. **A non-improving SQP step was force-accepted** ("so a tick always makes
   progress"), which wrote a bad direction into the next tick's warm start.
   Combined with (1), this built a divergent wrong-way full-lock steering
   ramp over roughly 20 ticks before the fix. Now only genuine cost
   improvements are kept; otherwise the tick keeps the previous solution.
3. **The reference heading was quantised.** Measuring `e_psi` against the
   raw waypoint-to-waypoint tangent (as the LTV-QP does) steps by
   `ds/R` — 5.7 degrees per 0.5 m waypoint on a 5 m-radius corner. The NMPC
   read each of those steps as real tracking error and corrected it within a
   tick or two, producing a hard period-2 ±25 degree steering oscillation.
   Fixed by deriving the reference heading from the same smoothed samples the
   curvature signal already comes from, so the measured state and the
   predicted state describe one consistent reference.
4. **The prediction had no grip limit.** Linear tyres produce unbounded
   lateral force, so without a cap the model believed it could hold any
   corner at any speed; the simulated car cannot, and the car spun mid-lap
   (heading error past 90 degrees). Fixed by saturating the predicted tyre
   forces at FSDS's own measured sustained lateral-acceleration ceiling —
   reusing the existing `alat_ceiling_flat`/`_slope`/`_intercept` constants
   already used elsewhere in this repo for exactly that ceiling, not a new
   number. This single change is what turned a spin into the Stage 6 result.

## Stage 6: the honest numbers

Closed loop, `comp_test_map_3/raceline.csv`, identical cost weights and the
same simulated plant for both controllers (a genuine single-variable A/B,
unlike comparing separate live logs that may carry different weight sets):

| | \|e_y\| mean / p90 / max | \|e_psi\| mean | steer saturation | lap time | solve p95 |
|---|---|---|---|---|---|
| LTV-QP (unchanged) | 0.400 / 1.451 / 2.323 m | 5.92 deg | 12.5% | 43.1 s | 9.9 ms |
| Nonlinear MPC | 0.277 / 0.686 / 1.150 m | 5.84 deg | **0.8%** | 42.0 s | 11.6 ms |

Turn-in point (arc length where steering first reaches 25% of a corner's own
peak, relative to the corner's geometric start): **earlier on all 7 corners
tested, median 25.6 metres**, and before the corner starts on 7/7 versus
after it starts on 6/7 for the LTV-QP.

Independently cross-checked against CasADi + IPOPT (exact automatic
differentiation, interior-point solver) at several operating points: costs
agree to within 0.22%, and the one genuinely wrong-direction value found
during testing (a single-step -0.33 degree dip) was reproduced by IPOPT too
(-0.327 degrees) — confirming it's the true optimum of this cost function,
not a solver artifact, and that it's a single step rather than the
multi-step transient that killed the three earlier attempts in Stage 2.

## Stage 7: what this does not fix

- **Not yet tested on the live/simulated FSDS car** — every number above is
  from this offline pipeline against the Pacejka plant. `use_nmpc` defaults
  to false; the LTV-QP path is unchanged.
- **The whole adaptive gain schedule is inactive** when `use_nmpc=true` — it
  exists to fake the anticipation this model now does structurally, and
  `use_precomputed_heading_profile` is likewise a no-op (it approximates the
  same curvature this model carries exactly).
- **The known longitudinal problem is untouched.** A separate, already-known
  issue (severe speed-tracking lag causing a near-spin, unrelated to turn-in
  timing) is not addressed here — the NMPC still holds a single scalar speed
  target across its whole horizon rather than a full speed profile. Feeding
  the profile over the horizon is the obvious next step.
- **Not mirrored anywhere on the offline side of this repo** — `nmpc_core.py`
  and its parameters live only in the ROS 2 workspace's own `fsds_simulator`
  staging mirror. There is no `settings.py` equivalent and none of the
  weights above have a numeric-parity obligation the way the LTV-QP's do.
  **Superseded later the same day:** an offline port,
  `controller/nmpc_optimiser.py`, was added (`settings.USE_NMPC`), giving the
  NMPC structural/solver constants a real `settings.py` parity obligation —
  see `planning_control_sync.md`'s "Nonlinear MPC (`use_nmpc`)" section for
  the current, authoritative description.
