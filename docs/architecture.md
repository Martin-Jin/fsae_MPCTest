# Architecture

Deep technical reference for how the simulator, MPC, and offline tuner work.
For quick-start usage, tuning workflow, and FSDS integration steps, see
[Developer Guide](developer_guide.md) instead, this document explains the
system, that one explains how to operate/extend it.

**Two MPC implementations exist**, selected by one flag (`use_nmpc`): the
default linear time-varying MPC (LTV-QP, `mpc_core.MPCController`), full
reference in **[`lmpc.md`](lmpc.md)**; and the alternative Frenet-frame
nonlinear MPC (`nmpc_core.NMPCController`), full reference in
**[`nmpc.md`](nmpc.md)**. Both moved out of this file because they had grown
too large for an overview document; sections 4 and 8 below are now short
pointers into those two docs, not the full material.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Configuring the Project (`settings.py`)](#configuring-the-project-settingspy)
3. [Configuring the Vehicle (`model/vehicle_physics.py`)](#configuring-the-vehicle-modelvehicle_physicspy)
4. [How the MPC Works](#how-the-mpc-works) (full reference: [`lmpc.md`](lmpc.md))
5. [How the Offline Tuner Works](#how-the-offline-tuner-works)
6. [The Composite Score](#the-composite-score)
7. [Module Reference](#module-reference)
8. [Second Controller: Nonlinear MPC (`use_nmpc`)](#second-controller-nonlinear-mpc-use_nmpc) (full reference: [`nmpc.md`](nmpc.md))

---

## Architecture Overview

### Full System Flow

This is the closed loop the simulator runs at 20 Hz. The same loop runs
headless (no plotting) thousands of times during tuning in
`tuner/offline_tuner.py`, and also runs live against the real/FSDS vehicle
as `mpc_controller.py` (in its `standalone_output=true` mode; mirrored
under `fsds_simulator/`, pasted into `fsae_planning`, see
[`docs/reference/`](docs/reference/)). All three share one implementation:
`sim/rollout_core.run_core_rollout()` for the first two, and
`mpc_core.MPCController` for the live node, kept in numeric parity with
`rollout_core`.

Note: the diagram below shows the case where `USE_PLANNER = True` (the
simulator/tuner reconstructs the track from cones, like the real car would).
When `USE_PLANNER = False`, the Perception/Planner boxes are skipped and the
true reference path is used directly for tracking error.

```
USER INPUT (draw path / load synthetic path)
        │
        ▼
  path_X, path_Y, path_Psi
  speed_profile.compute_speed_profile()
  sim_track.place_cones()
        │
        ▼
┌─────────────────────────────────────────────────────────┐
│                     SIMULATION LOOP (20 Hz)             │
│                                                         │
│  ┌──────────────┐     visible      ┌─────────────────┐  │
│  │ SimPerception│◄─── cones ───────│  Static cone    │  │
│  │ (FOV filter) │                  │  map (full      │  │
│  └──────┬───────┘                  │  track layout)  │  │
│         │ blue[], yellow[]         └─────────────────┘  │
│         ▼                                               │
│  ┌──────────────┐     centreline   ┌─────────────────┐  │
│  │  SimPlanner  │─────────────────►│  ConeMap        │  │
│  │  (boundary + │  + speed profile │  (accumulates   │  │
│  │   ConeMap +  │                  │  observations)  │  │
│  │   speed prof)│                  └─────────────────┘  │
│  └──────┬───────┘                                       │
│         │ waypoints[], v_target                         │
│         ▼                                               │
│  ┌──────────────┐     x0 (8-state  ┌─────────────────┐  │
│  │ Error State  │─────error vec)──►│   MPC Solver    │  │
│  │ Extraction   │                  │   (OSQP /       │  │
│  │ + Adaptive   │                  │   Clarabel)     │  │
│  │ Gain Scaling │                  └────────┬────────┘  │
│  └──────────────┘                           │ u=[δ, a]  │
│         ▲                                   |           |
|         |                          ▼        |           |
│  ┌──────────────┐                           |           │
│  │ 24-State     │◄──────────────────────────┘           │
│  │ Nonlinear    │  step_nonlinear_plant(state, u, dt)   │
│  │ Plant        │                                       │
│  └──────────────┘                                       │
└─────────────────────────────────────────────────────────┘
        │
        ▼
  history dict → scrub viewer + tuner/performance_stats.py (Show Metrics / Benchmark All Paths)
```

### Controller / Plant Architecture

```
                    ┌──────────────────────────────────────────┐
                    │   rollout_core.run_core_rollout()         │
                    │   (sim/rollout_core.py — see below)       │
                    │                                          │
  path waypoints ──►│  bicycle_model.get_8state_discrete_model │
  car state      ──►│  → Ad, Bd  (ZOH linearised bicycle model)│
                    │                                          │
                    │  model_utils.adaptive_R_scaling(vx, R)   │
                    │  model_utils.adaptive_R_rate(κ, R_rate)  │
                    │  → speed- and curvature-adjusted weights │
                    │                                          │
                    │  optimiser.solve_mpc()                   │
                    │  → OSQP QP → u* = [δ_cmd, a_cmd]         │
                    │                                          │
                    │  scoring.RolloutMetrics.add_step()       │
                    │  → accumulates the 12 score metrics      │
                    └──────────────┬───────────────────────────┘
                                   │
                    ┌──────────────▼───────────────────────────┐
                    │            Plant (truth layer)           │
                    │                                          │
                    │  vehicle_physics.step_nonlinear_plant    │
                    │  24 states: X, Y, ψ, vx, vy, r,          │
                    │  δ_act, a_act, ω×4, z×4, dz×4,           │
                    │  Fy_rlx×4, ω_FL, ω_FR                    │
                    │  4 sub-steps per control tick            │
                    └──────────────────────────────────────────┘
```

Both `offline_tuner.run_headless_rollout()` and `simulation.simulate_closed_loop()`
are thin wrappers around `rollout_core.run_core_rollout()` (`sim/rollout_core.py`),
the single implementation of the tracking-error computation, progress tracking,
MPC solve, delay queue, termination checks, and metric accumulation.
`gui/simulation.py` calls it with `want_history=True` to get a full
step-by-step history dict for the GUI; `tuner/offline_tuner.py` calls it with
`want_history=False` for a fast, scoring-only path. This guarantees a path run
in the live simulator and the same path benchmarked offline produce
(near-)identical composite scores.

### ROS 2 vs Simulator Mapping

How each component maps to its ROS 2 equivalent in the `fsae_planning` package:

```
ROS 2 Node (fsae_planning)      │  Simulator Equivalent
─────────────────────────────────┼─────────────────────────────────────
sim_perception.py               │  sim_track.SimPerception  (active when USE_PLANNER=True)
centerline_planner.py           │  sim_track.SimPlanner     (active when USE_PLANNER=True)
cone_map.py                     │  planning/cone_map.ConeMap        (shared)
boundary.py                     │  planning/boundary.py             (shared)
path_utils.py                   │  planning/path_utils.py           (shared)
cone_sorting.py                 │  planning/cone_sorting.py         (shared)
mpc_core.py                     │  controller/optimiser.py + model/bicycle_model.py + controller/model_utils.py  (shared design, same QP)
mpc_controller.py (standalone_output=true) │  gui/simulation.py's rollout loop   (shared design — see `docs/reference/`)
cone_recorder.py                │  sim/track_io.py + gui/simulation.py's Load Recorded Track  (recorder writes what the loader reads)
```

`fsds_simulator/control/fsae_control/fsae_control/stanley_controller.py` is
the actual current Stanley controller (mirrored from upstream, kept in sync
like everything else under `fsds_simulator/`, see
[`docs/reference/`](docs/reference/)), not just a
structural reference. This project's tuner and offline simulator only ever
drive against the MPC (`mpc_controller.py`'s `standalone_output=true` mode /
`mpc_core.py`, same directory). Stanley is mirrored purely so `fsds_simulator/` can stand
up the full live stack, not because this repo's own simulator exercises it.

---
## Configuring the Project (`settings.py`)

`settings.py` is the single place to change tuning knobs, cost weights, and
DNF/validation configuration shared by `gui/simulation.py`,
`tuner/offline_tuner.py`, `sim/scoring.py`, `sim/rollout_core.py`, and
`tuner/performance_stats.py`. It has no vehicle physics in it, that lives
in `model/vehicle_physics.py` (see next section). Every setting has a
detailed, plain-language explanation directly above it in the file itself.

For what every weight/gain/flag in `settings.py` does, how to tune it, and
known constraints (including `N_HORIZON`, `DELAY_STEPS`/`DELAY_JITTER_STEPS`,
`SLAM_NOISE_ENABLED` and the rest of the simulator-fidelity settings, the
`Q_diag`/`R_diag`/`R_rate_diag` cost weights, and `SCORE_WEIGHTS`/
`METRIC_SCALES`), see [tuning.md](tuning.md); this section instead covers
the parts of `settings.py` that are about tuner *mechanics* (DNF detection,
solver settings, the pose-feed-hold sim-to-real model) rather than tuning
values themselves.

### DNF penalty configuration

`DNF_PENALTY` and `DNF_OFFTRACK_PENALTY` are flat score penalties added when a
tuning rollout doesn't finish the track, and an additional penalty
specifically when the reason was leaving the track boundary. These exist so
the tuner can't find a deceptively good score by having the car crawl
slowly and carefully without ever finishing.

### Solver settings for headless rollouts

`ROLLOUT_EPS` / `ROLLOUT_MAX_ITER` are OSQP convergence tolerance and iteration
cap used only during offline tuning rollouts (looser than the live
simulator's defaults for faster mass evaluation, at negligible accuracy
cost). `MAX_EVALS` is the total true-rollout budget for one tuning run.
`PATH_N_POINTS` is how many points each synthetic test track is resampled to.
`USE_OPTUNA_PRESEARCH` / `OPTUNA_PRE_PASS_EVALS` configure an optional TPE
pre-search that seeds CMA-ES's starting point; see
[Optional Optuna TPE pre-search](#optional-optuna-tpe-pre-search).

### Scoring weights

`SCORE_WEIGHTS`/`METRIC_SCALES` define what "good driving" means to the
tuner. See [tuning.md](tuning.md#6-scoring-metric_scales-and-score_weights)
for how to tune these; see [The Composite Score](#the-composite-score) below
for exactly what each of the 13 metrics measures and how they combine into
one score.

`VALIDATION_SUITE` is which of the synthetic corner-shape paths (defined in
`tuner/offline_tuner.build_synthetic_paths()`) the tuner actually evaluates
candidates against. Commented-out paths are available but excluded by
default to keep each tuning run faster.

### Pose-feed hold (sim-to-real)

`PoseFeedHold` in `sim/rollout_core.py` models the live pose feed **repeating**
its last measurement instead of delivering a fresh one. Measured on live
telemetry 2026-08-06 (two runs, same track, same tuned weights, differing only
in how badly the feed stalled):

| | normal run | failed run |
|---|---|---|
| fresh-pose rate | 18.9 Hz | 6.4 Hz |
| repeated ticks | 5.3% | 60.7% |
| longest hold | 5 ticks (0.25 s) | 20 ticks (0.99 s) |
| peak `pose_age_s` | 347 ms | 1242 ms |

In the failed run the pose froze for ~1 s at 14 m/s, about 17 m travelled blind,
and the car spun on resume with 105° of heading error.

This is distinct from the two existing delay knobs, and none of them substitute
for it:

- `DELAY_STEPS` delays a pose that is still **fresh** every tick.
- `DELAY_JITTER_STEPS` perturbs only the controller's **belief** about the lag.
- `PoseFeedHold` repeats the **data**, so `pose_age` genuinely ramps and the
  controller is briefly blind.

While a hold is active the rollout also **skips perception and planning**, since
on the car the planner is triggered by `car_position`, and a stalled pose stalls
the whole chain. Without that, re-planning from a frozen pose still yields a
slightly different centreline each tick and the controller is never blind
(measured: `e_y` repeated on 0.0% of ticks instead of the intended ~5%).

Tuned to the normal run: `POSE_HOLD_PROB = 0.05`, `MEAN_TICKS = 2.1`,
`MAX_TICKS = 5` reproduces 5.8% repeated ticks / mean hold 2.10 against the
measured 5.3% / 2.08.

> **This does NOT close the sim-to-real gap.**
>
> - With the model on and firing correctly, steering saturation moves only
>   3.4% → 4.4% against a live 21.1%, and heading error 6.0° → 6.3° against
>   a live 15.9°.
> - The pose hold is real and now faithfully reproduced, but it is **not**
>   the cause of the gap.
> - Also tested and eliminated: plant grip, corner entry speed, planner
>   centreline quality, SLAM pose noise, extra actuation delay, and planner
>   update rate.
> - The cause remains open. Do not treat offline scores as predictive of
>   live behaviour until it is found.

---

### Bonus weights

`TIME_BONUS_WEIGHT` is a legacy weight, no longer used by the score itself.
Time is now the *primary objective* (tier 2), scaled by
`TIME_OBJECTIVE_WEIGHT`, not a bonus subtracted from a metric sum.

`COMPLETION_BONUS_WEIGHT`: **no longer used by the score.** Completion is a
hard constraint (tier 1), not something rewarded: a run that doesn't finish
is scored above `CONSTRAINT_FLOOR` regardless of how well it drove. Both
constants are retained only so the live copy's CSV header and
`tuning history.txt` logging keep their existing fields.

---

## Configuring the Vehicle (`model/vehicle_physics.py`)

The single source of truth for all vehicle physics (mass, geometry, tyre
grip, suspension, aerodynamics, actuator limits) is the `VehicleParams`
class in `model/vehicle_physics.py`. This is what the nonlinear 24-state
plant (the "truth" simulation) uses, and several of these values (`Cf`,
`Cr`, `tau_delta`, `tau_a`, `lf`, `lr`, `m`, `Iz`) also feed directly into
the MPC's own internal linear model in `model/bicycle_model.py`, see
[`lmpc.md`](lmpc.md) for how those specific values are
used mathematically.

### Global scaling knobs

Three constants at the top of `VehicleParams.__init__` proportionally scale
groups of related parameters, removing the need to hand-tune every
individual tyre/inertia constant to make the car noticeably grippier,
heavier-feeling, or coast further:

```python
GRIP_SCALE     = 1.1   # Scales tyre stiffness (B) and peak grip (D) together
INERTIA_SCALE  = 0.8   # Scales yaw inertia and wheel rotational mass together
COASTING_SCALE = 3.0   # Scales rolling resistance / drivetrain drag only, NOT aero drag (Cd_A is a fixed physical value) — < 1.0 = rolls further, > 1.0 = stops faster
```

These three should generally be adjusted in preference to individual
Pacejka/inertia constants, unless real tyre test data (TTC) or measured
chassis inertia is available to plug in directly.

### Importing new tyre data

The plant uses a Pacejka **MF94** tyre model (`B`, `C`, `D`, `E`, `Sv`, `Sh`
per axle, see [The Pacejka Tyre Model](#the-pacejka-tyre-model) below for
what each coefficient physically means). Replacing these with real TTC
data requires one additional step:

> **You must also recompute `Cf` and `Cr`**, the *linear* cornering
> stiffnesses used by the MPC's internal bicycle model in
> `model/bicycle_model.py`, a completely separate pair of constants from
> the Pacejka coefficients above.
>
> - **Why:** `Cf`/`Cr` need to match the new Pacejka curve's initial slope
>   near zero slip angle, via `C_eff ≈ mu * Fz_nominal * B * C * D`.
> - **What happens if you skip this:** the MPC's internal prediction model
>   quietly stops matching the plant it's controlling. It doesn't
>   error out, it just produces degraded tracking with no obvious cause,
>   since nothing flags the mismatch directly.

### Actuator limits

`max_steer`, `max_accel`, `max_accel_brake`: changing these automatically
propagates to the MPC's hard QP constraints in `controller/optimiser.py` and
`mpc_core.py` (both read `VehicleParams` directly), so the controller
will never be asked to command something the (simulated) vehicle physically
can't do.

### The Pacejka Tyre Model

The plant computes tyre grip using the Pacejka **MF94** "Magic Formula",
an empirical curve fit to real tyre test data, rather than a physics-derived
equation:
Fy = mu · Fz · sin(C · atan(B·α − E·(B·α − atan(B·α))))

Where `α` is slip angle (lateral) or slip ratio (longitudinal), and `Fz` is
the tyre's current normal load. See
[vehicle_physics_guide.md §4](vehicle_physics_guide.md#4-what-is-full-mf94-pacejka-and-what-is-a-tyre-model-at-all)
for what each coefficient (`B`/`C`/`D`/`E`/`Sv`/`Sh`), `mu`/`k_sens`, tyre
relaxation, and the friction ellipse physically mean, not repeated here.

This curve is where the plant's nonlinearity shows up numerically.
Near `α = 0` it's *approximately* a straight line through the origin, and
that local slope is exactly the linear cornering-stiffness `Cf`/`Cr` the
MPC's internal model assumes holds everywhere (see "Linear vs nonlinear" in
[`lmpc.md`](lmpc.md#linear-vs-nonlinear-in-plain-english)). Push `α` out past roughly 5-8° of
slip, though, and the real curve visibly bends over: each extra degree of
slip buys noticeably less extra force than the last, until it saturates at
`D` and can even fall past that (a tyre that's broken traction). Doubling
the slip angle out here does **not** double the force: it might only add
20% more, or none at all, which is exactly the behaviour a fixed-multiplier
linear model cannot represent.

---
## How the MPC Works

Full technical reference (state vector, every matrix entry, the cost
function, the solver, and the two runtime adaptive features layered on top)
has moved to its own document, **[`lmpc.md`](lmpc.md)**, split out because
it had grown too large for this overview. The implementation is split
across three files that must be kept in numeric agreement:
`model/bicycle_model.py` (the prediction model), `controller/optimiser.py`
(the QP formulation, used by the simulator/tuner), and `mpc_core.py` (a
self-contained duplicate of both, used by the live ROS 2 node so it has no
simulator dependencies).

In one line: at every 20 Hz tick, the controller measures tracking error
relative to the path, predicts how that error evolves over a 1.75 s horizon
under a linear bicycle model, solves a Quadratic Program (QP) for the
steering/throttle sequence that minimises tracking error plus control
effort plus smoothness, and applies only the first command before
re-solving next tick (the *receding horizon* principle). See
[`lmpc.md`](lmpc.md) for the full derivation, including how error is
measured (Frenet-frame projection), the kinematic/dynamic model blend, the
cost function and constraints, the OSQP/Clarabel solver, and the adaptive
gain-scheduling layer on top.

---
## How the Offline Tuner Works

`tuner/offline_tuner.py` searches for `Q`, `R`, `R_rate` cost weights automatically
rather than requiring hand-tuning, by running many closed-loop rollouts and
minimising a single scalar score. This section covers the search algorithm;
see [The Composite Score](#the-composite-score) for exactly what's being
minimised.

```
settings.py: Q/R/R_rate templates, VALIDATION_SUITE, INITIAL_CONDITIONS
        │
        ▼
┌───────────────────────────────────────────────────────────┐
│  USE_OPTUNA_PRESEARCH (optional, default True)             │
│  Optuna TPE search, OPTUNA_PRE_PASS_EVALS trials            │
│  → cheap, coarse scan of the 9-dim scale-factor space       │
└──────────────────────────┬───────────────────────────────┘
                            │ seeds x0 (else x0 = geometric
                            │ midpoint of [0.1, 10.0] per dim)
                            ▼
┌───────────────────────────────────────────────────────────┐
│  CMA-ES (cma.fmin_lq_surr2), BIPOP restarts,                │
│  local-quadratic surrogate assistance                       │
│                                                             │
│   for each generation:                                     │
│     sample a population of candidate weight-scale vectors  │
│         │                                                   │
│         ▼                                                   │
│   ┌─────────────────────────────────────────────────────┐  │
│   │ per candidate: parallel_evaluate_candidate()          │  │
│   │                                                       │  │
│   │  EVAL_TASKS = VALIDATION_SUITE × INITIAL_CONDITIONS   │  │
│   │  → one rollout_core.run_core_rollout() per task,      │  │
│   │    fanned out across cpu_count-1 worker processes     │  │
│   │         │                                             │  │
│   │         ▼                                             │  │
│   │  scoring.compute_composite_score() per task           │  │
│   │         │                                             │  │
│   │         ▼                                             │  │
│   │  objective = 0.7·weighted_mean(scores)                │  │
│   │            + 0.3·quantile(scores, TAIL_QUANTILE)      │  │
│   └─────────────────────────────────────────────────────┘  │
│         │                                                   │
│         ▼                                                   │
│   adapt distribution mean/covariance toward better regions │
│   (surrogate model filters which candidates get a real     │
│   rollout vs. a predicted score)                            │
│         │                                                   │
│         └──── repeat until MAX_EVALS budget exhausted, or   │
│               Ctrl+C ─────────────────────────────────────►┘
└──────────────────────────┬───────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────┐
│  Post-optimisation: clean serial re-evaluation              │
│  xbest (best single candidate) vs.                          │
│  xfavorite (mean of final search distribution)               │
│  → lower-scoring one is the result                          │
└──────────────────────────┬───────────────────────────────┘
                            ▼
              printed result + appended to tuning_history.txt
```

### Search space

Rather than searching over raw weight values directly, CMA-ES searches over
9 **multiplicative scale factors**, one per tunable diagonal entry
(`TUNABLE_Q_IDX = [0,1,2,3,4]`, `TUNABLE_R_IDX = [0,1]`,
`TUNABLE_R_RATE_IDX = [0,1]`):

```
Q[i,i]      = vec[j] · Q_template[i,i]
R[i,i]      = vec[j] · R_template[i,i]
R_rate[i,i] = vec[j] · R_rate_template[i,i]
```

Each factor is bounded to `[0.1, 10.0]`, one decade of adjustment in either
direction from the template. Searching in multiplicative (rather than
absolute) space keeps the problem dimensionally consistent regardless of
the template's starting magnitude, and the `0.1` floor (rather than `1.0`)
specifically allows the tuner to discover that a weight should be *reduced*
below its starting point, not only increased.

The starting point `x0 = sqrt(lower · upper) = 1.0` for every parameter is
the geometric (log-scale) midpoint of `[0.1, 10.0]`, i.e. "start the search
exactly at the current template weights, unscaled," which is the natural
neutral point for a multiplicative search space (the arithmetic mean would
be biased toward the larger bound). This fixed midpoint is CMA-ES's default
starting point; if `USE_OPTUNA_PRESEARCH` is enabled (see below), `x0` is
replaced by the Optuna pre-pass's best result instead.

### Optional Optuna TPE pre-search

`USE_OPTUNA_PRESEARCH` in `settings.py` (default `True`) runs a short
Optuna TPE (Tree-structured Parzen Estimator) search *before* CMA-ES starts,
using `OPTUNA_PRE_PASS_EVALS` true rollouts (default 10% of `MAX_EVALS`) out
of a separate mini-budget. This phase's cost is in addition to, not carved
out of, the main `MAX_EVALS` budget. TPE is a cheaper, less precise
global search method than CMA-ES; the idea is to spend a small budget
finding a promising general region of the 9-dimensional search space, then
start CMA-ES there instead of at the fixed geometric midpoint, so more of
CMA-ES's own budget goes toward local refinement instead of coarse search.

The pre-pass reuses the exact same objective (`parallel_evaluate_candidate`)
and worker pool as the CMA-ES phase (no rollout logic is duplicated),
running trials sequentially (`n_jobs=1`) since each trial already fans a
single candidate out across every core via the pool; a second layer of
Optuna-level parallelism would only oversubscribe the same cores. It
respects the same Ctrl+C graceful-shutdown flag (`_stop_requested`) as the
CMA-ES phase, and its result (trial count, best score, seeded x0) is logged
to `tuning history.txt` alongside the run's weights so it's traceable which
runs used it.

> **Closed book before 2026-08-06.** The Optuna pre-pass is one of several
> things that changed partway through the recorded tuning history (alongside
> `SCORE_WEIGHTS` edits and the scoring/simulation unification), which is why
> entries above the `COMPARABLE HISTORY RESUMES HERE` marker in
> `tuning history.txt` are not comparable to each other or to later runs.
> See the header of that file for the full list and consequences.

Requires the optional `optuna` package (see
[Dependencies](developer_guide.md#dependencies)), only needed if this flag
is enabled.

### CMA-ES: what it's doing and why

CMA-ES (Covariance Matrix Adaptation Evolution Strategy) is a
**derivative-free black-box optimiser**, it doesn't need a formula for how
the score changes as a weight changes, only the ability to run a rollout
and read off a score.

**Why that matters here:** the objective (drive N corners well) is noisy,
two rollouts with identical weights can score slightly differently, and
has no clean formula connecting a weight to the score, the way fitting a
straight line to data does. There's no calculus shortcut available, so any
optimiser that needs one is off the table.

**How CMA-ES actually searches**, each generation:

1. Maintain a multivariate Gaussian distribution over candidate solutions
   (think: a fuzzy cloud centred on the current best guess).
2. Sample a population of candidates from that cloud, and run a real
   rollout to score each one.
3. Adapt the cloud's centre and shape toward the better-scoring region,
   learning, over generations, not just *where* good solutions are but
   which *directions* in parameter space matter and which don't.

This project specifically uses `cma.fmin_lq_surr2`, which layers two
additional techniques on top of plain CMA-ES:

**BIPOP (bi-population) restarts.** Rather than one long single run, the
optimiser interleaves "large" restarts (population size doubles each time
via `incpopsize=2`, broader exploration, better at escaping local minima)
with "small" restarts (reduced population, faster local refinement around
the current best candidate). `max_restarts = 7` caps how many restarts the
whole session gets.

**Surrogate assistance (the "lq" in `fmin_lq_surr2` = local quadratic).** A
cheap quadratic model is fitted to recently-evaluated candidates and used to
*predict* the score of new candidates without running a full rollout. Only
candidates the surrogate predicts are promising (or a periodic sample, to
keep the surrogate honest) get a real rollout. This is what lets `MAX_EVALS`
"true" rollouts produce roughly 3-10× as much effective search coverage.

**Initial step size (`sigma0 = 0.65`) and per-dimension spread
(`CMA_stds = 0.23 · log(upper/lower)`)** control how large a jump CMA-ES
takes when sampling new candidates early in the search. Since
`log(10/0.1) ≈ 4.6`, this gives an initial per-dimension standard deviation
of roughly `1.06` in log-space, large enough to explore meaningfully across
the full decade of allowed adjustment, without being so large that early
generations are mostly wasted on wildly implausible weight combinations.

### Parallel + serial evaluation

Every CMA-ES candidate is evaluated across all tasks in
`EVAL_TASKS`, the cross-product of `VALIDATION_SUITE` (the corner shapes
from `settings.py`) and `INITIAL_CONDITIONS` (a nominal on-path start, plus
a perturbed start with `ey0=0.2 m, epsi0=0.05 rad`, to force the tuner to
find weights that also recover from imperfect starting position). Each
task's rollout runs in parallel across `cpu_count - 1` worker processes.

The per-candidate objective combines all task scores as:

```
objective = 0.7 · weighted_mean(scores) + 0.3 · quantile(scores, TAIL_QUANTILE)
```

The 30% tail term exists specifically so CMA-ES can't find a weight set that
scores well *on average* by driving one corner shape perfectly and another
one badly, every task in the suite has to be reasonably good, not just the
average.

`TAIL_QUANTILE` (in `settings.py`, default `0.8`) replaced a hard `max()`.
With the flat `DNF_PENALTY` of +3.0 (+6.0 off-track), the old `max()` let
**one** unlucky task out of ten shift the objective by ~0.9 and swamp all
twelve continuous quality metrics. Measured, a plausible hand-picked gain set
ranked 3rd-worst of six (below two deliberately pathological sets) purely
because a single one of its ten tasks DNF'd. That is a discontinuous,
high-variance signal for CMA-ES and a likely contributor to the ~10× spread in
tuned gains across historical runs. A high quantile keeps the intent, punish
weights that fail badly *somewhere*, while requiring more than one bad task
before it dominates. Set `TAIL_QUANTILE = 1.0` to recover the old behaviour
exactly.

### DNF conditions (offline tuner, tighter than the live simulator)

A rollout inside the tuner is marked "did not finish" if any of:

- `|e_y| ≥ 3.50 m` (left the track, matches `OFFTRACK_LIMIT`)
- 5 consecutive MPC solver failures (matches `MAX_FAILS`)
- **Rolling stall check**: less than 3.0 m of forward progress in any
  rolling 60-step (3 s) window, catches a car that hasn't technically left
  the track or failed to solve, but also isn't actually driving anywhere
  (e.g. stuck oscillating in place).

On a DNF, `DNF_PENALTY` is added to the score, plus `DNF_OFFTRACK_PENALTY`
specifically if the DNF was caused by leaving the track (see
[Configuring the Project](#configuring-the-project-settingspy) for both
values).

### Post-optimisation: picking the final answer

After the search budget is exhausted (or `Ctrl+C` is pressed), two
candidates are freshly evaluated **serially** (outside the noisy parallel
pool, for a clean comparison):

- **`xbest`** is the single best individual candidate observed across the
  entire search.
- **`xfavorite`** is the mean of CMA-ES's final search distribution, which
  tends to be more robust/averaged than any one lucky sample.

Whichever scores lower in this final clean evaluation is printed as the
result and appended to `tuning_history.txt`.

---

## The Composite Score

Both the offline tuner and the simulator's **Show Metrics**/**Benchmark All
Paths** buttons score a rollout through the exact same code path
(`scoring.RolloutMetrics`), which is what guarantees a path scored live in
the GUI and the same path scored offline produce matching numbers, there
is exactly one implementation of the scoring maths, not two independently
maintained copies.

### The 13 metrics

Accumulated once per simulation step via `RolloutMetrics.add_step()`, then
normalised (mostly to RMS values) at the end via `.finalize()`:

| # | Metric | What it measures |
|---|---|---|
| 0 | `rmse` | Combined tracking error: `1.2·e_y² + 0.4·e_psi²`, root-mean-squared over the run. The primary quality signal. |
| 1 | `yaw_rms` | RMS of the true yaw rate, penalises a car whose heading oscillates/wobbles. |
| 2 | `smooth_rms` | RMS of step-to-step control change (`Δu`), penalises jerky command sequences. A failed solver step adds a flat +5.0 penalty here. |
| 3 | `steer_rms` | RMS steering command magnitude, overall steering effort. |
| 4 | `accel_rms` | RMS acceleration/brake command magnitude, overall longitudinal effort. |
| 5 | `max_steering` | The single largest steering command issued during the run. |
| 6 | `steering_sat_ratio` | Fraction of steps where steering was within 95% of `max_steer`, how often the controller is pinned at its limit. |
| 7 | `jerk_rms` | RMS of the *second* difference of control (`Δ²u`), smoothness of the smoothness, catches abrupt changes in how fast commands are changing. |
| 8 | `max_yaw_rate` | The single fastest yaw rate reached, cornering aggressiveness ceiling. |
| 9 | `steering_reversal_rms` | Magnitude-weighted RMS of steering sign-flip swings (beyond a 0.02 rad noise gate): `sqrt(Σ swing² / n steps)`, where `swing = |u_steer| + |u_steer_prev|` at the moment of the flip. A tiny back-and-forth trim wiggle contributes almost nothing while a large aggressive swing dominates (squared), which is what distinguishes controller hunting/dithering from a twisty path (S-bends, slaloms) legitimately demanding more frequent-but-small direction changes; a flat per-flip count couldn't tell those apart. The raw reversal count and its per-step rate are still reported separately as informational-only fields (`steering_reversals`, `steering_reversal_rate` in the returned dict) alongside it. |
| 10 | `peak_lateral_error` | The single worst `|e_y|` reached at any point, a safety-margin measure independent of the average. |
| 11 | `speed_rmse` | RMS of `v_actual - v_target`, how well the car tracks the planner's requested speed. |
| 12 | `accel_reversal_rms` | The same magnitude-weighted reversal construction as `steering_reversal_rms` (metric 9), applied to `u_opt[1]` (`a_cmd`) instead of `u_opt[0]` (`delta_cmd`), with a 0.02 m/s² noise gate in place of the steering metric's 0.02 rad. `steering_reversal_rms` only ever looks at the steering command, so without this nothing in the score discourages `a_cmd` oscillating across zero even though the same accel/brake chatter concern applies. Keyword-only with a default value so callers written before this metric existed keep working unmodified. |

### Combining into one score

```python
quality = SCORE_WEIGHTS @ (metrics / METRIC_SCALES)             # normalised weighted sum

# TIER 1 — hard constraints: infeasible runs land above CONSTRAINT_FLOOR
if dnf or offtrack:
    return CONSTRAINT_FLOOR + (DNF_PENALTY + offtrack*DNF_OFFTRACK_PENALTY) * (1 - progress)
if not reached_end:
    return CONSTRAINT_FLOOR + DNF_PENALTY * (1 - progress)

# TIER 2 — primary objective: how much slower than physically possible
time_cost = 1.0 - time_bonus            # time_bonus = optimal_lap_time / actual_time

# TIER 3 — quality group, shapes rather than drives
score = TIME_OBJECTIVE_WEIGHT * time_cost + QUALITY_WEIGHT * quality
if inaccurate_count > 0:
    score += abs(score) * min(5, inaccurate_count) * 0.1        # capped at 50%
```

**Why three tiers instead of one sum.** A weighted sum is
linear scalarisation, and can only reach solutions on the *convex hull* of the
trade-off surface. Where that surface is non-convex (normal for vehicle
dynamics), whole regions of good behaviour are unreachable by **any** weight
vector. Measured: a deliberately-hunting gain set outscored a sane one purely
by tracking the line more tightly, and kept winning even after `METRIC_SCALES`
made the smoothness terms bite (normalisation amplifies the tracking terms
too). Re-weighting cannot fix that, because the hunting set is genuinely better
on the dominant term.

- **Constraints are no longer prices.** Previously a DNF added a flat `+3.0` on
  the same axis as the metrics, so a sufficiently tight-tracking run could
  *buy its way out of a crash*. Now infeasible runs occupy a band strictly
  above `CONSTRAINT_FLOOR` and no quality score can promote them. Ordering
  *within* the band still improves with `progress`, so the optimiser keeps a
  gradient rather than hitting a flat wall.
- **The objective is time, in real units.** `time_bonus` is
  `optimal_lap_time / actual_time` (see `speed_profile.optimal_lap_time()`), so
  `time_cost = 0.15` means the lap took ~18% longer than physically possible.
  This is what kills the hunting exploit: hunting cannot buy lap time, so it
  only ever costs.
- **`reached_end`, not `progress`, decides completion.** `progress` comes from
  a bounded nearest-index search that stops short of the final path point, so a
  fully-completed run reports ~0.90. Thresholding on it marked every successful
  run infeasible. `COMPLETION_THRESHOLD` remains only as a fallback for callers
  that cannot supply `reached_end`, that no longer includes
  the live car when it's running against a precomputed speed profile (see
  `LapProgressTracker` in `docs/reference/README.md`'s "Live/offline score
  parity" section); a run against the live planner topic instead still has no
  known path end and falls back to this threshold.
- `COMPLETION_BONUS_WEIGHT` is now unused by the score, completion is a
  precondition, not a reward. The constant is retained for the live copy's
  header compatibility.

`METRIC_SCALES` divides each metric by a reference magnitude
*before* weighting, so `SCORE_WEIGHTS` expresses priority rather than silently
doing unit conversion as well. Without it a metric's influence is
`weight × typical magnitude`: measured, that left all ten non-tracking metrics
contributing a combined +0.0064 against a −0.2649 tracking term, i.e. the score
was effectively single-objective and the smoothness/oscillation terms could not
bite no matter how their weights were set. `tuner/performance_stats.py` now
prints each metric's **effective contribution** (`weight × metric / scale`) and
percentage share, so this is visible directly in a benchmark report.

Consequence: post-2026-08-06 scores are on a different scale (a run with every
metric at its reference scores exactly 1.0 before bonuses) and are **not**
comparable to earlier logged scores.

**Lower is always better.** A good finishing run typically scores in
`[-0.5, -0.3]`, negative because the completion/time bonuses usually
outweigh the (small, well-tuned) metric costs. See
[tuning.md](tuning.md#6-scoring-metric_scales-and-score_weights) for how to
tune `SCORE_WEIGHTS`/`METRIC_SCALES`.

The inaccurate-solver penalty (up to +50% at 5 or more
`OPTIMAL_INACCURATE` occurrences in one rollout) uses
`score + abs(score)·factor` rather than a flat addition specifically so it
scales with, and preserves the sign of, an already-good (negative) score:
a run that finished well but had a few marginally-converged solves is
penalised proportionally, not knocked into DNF-penalty territory outright.

---
## Module Reference

Detailed explanations of the core algorithms live in
[`lmpc.md`](lmpc.md), [`nmpc.md`](nmpc.md), and
[How the Offline Tuner Works](#how-the-offline-tuner-works) above. This section is
a short per-file index: what each module is for, and where its logic is
documented in depth (either there, or in the file's own docstrings/comments,
which are kept in sync with this README).

Note: this covers the simulator/tuner files only. The shared planning code
in `planning/` is copied from the `fsae_planning` repo and documented there,
not here.

| File | Purpose |
|---|---|
| `gui/simulation.py` | Interactive matplotlib GUI: draw/load a path, run one closed-loop rollout, scrub through history, view metrics. Thin wrapper around `rollout_core.run_core_rollout(want_history=True)`. |
| `sim/rollout_core.py` | The single shared closed-loop rollout loop used by both `gui/simulation.py` and `tuner/offline_tuner.py`. Not GUI-safe to import from `gui/simulation.py`'s multiprocessing workers, so it's split out into its own dependency-light module. |
| `sim/scoring.py` | The single implementation of the 13-metric accumulation and composite score. See [The Composite Score](#the-composite-score). |
| `model/bicycle_model.py` | Builds the MPC's linear 8-state prediction model. See [`lmpc.md`](lmpc.md). |
| `controller/model_utils.py` | Runtime curvature/speed-based rescaling of `R`/`R_rate`. See [`lmpc.md`'s Adaptive gain scheduling](lmpc.md#adaptive-gain-scheduling-controllermodel_utilspy). |
| `controller/optimiser.py` | The parameterised CVXPY/OSQP QP formulation and solve. See [`lmpc.md`'s The cost function and QP](lmpc.md#the-cost-function-and-qp-controlleroptimiserpy). |
| `model/vehicle_physics.py` | The 24-state nonlinear "truth" plant (Pacejka tyres, suspension, aero) that the MPC never observes directly, only through tracking error. See [Configuring the Vehicle](#configuring-the-vehicle-modelvehicle_physicspy). |
| `tuner/offline_tuner.py` | Headless CMA-ES weight search. See [How the Offline Tuner Works](#how-the-offline-tuner-works). Also exports the synthetic path library (`SYNTHETIC_PATHS`, `PATH_NAMES`) and the speed-keyed model cache (`get_cached_model`) used by both the tuner and the simulator. |
| `sim/speed_profile.py` | Curvature-based per-point target speed (`compute_speed_profile`), with a moving-average smoothing pass (`smooth_profile`). Uses the friction-circle approximation `v = sqrt(a_lat_max / κ)` over a forward look-ahead window. |
| `sim/sim_track.py` | Simulator-side mirrors of the real perception/planner nodes: `place_cones()` (static track layout), `SimPerception` (FOV filter), `SimPlanner` (cone accumulation → centreline + speed profile). |
| `sim/track_io.py` | Loads a `fsae_planning` `cone_recorder` JSON cone map into the same `(path_X, path_Y, path_Psi, path_v, blue, yellow)` tuple shape as a synthetic path, see [Recording, exporting and driving a track](developer_guide.md#recording-exporting-and-driving-a-track). |
| `tuner/performance_stats.py` | Scores a completed simulator run for the **Show Metrics** button by replaying its stored history through the exact same `scoring.RolloutMetrics` accumulator the tuner uses. Also exposes `benchmark_weights()` for **Benchmark All Paths**. |
| `gui/manual_drive.py` | Standalone WASD/mouse drive mode against the 24-state nonlinear plant, no MPC, no scoring, purely open-loop human control for building intuition or sanity-checking a track. See [Manual Drive Mode](developer_guide.md#manual-drive-mode). |
| `settings.py` | All project-level tuning/scoring/DNF configuration. See [Configuring the Project](#configuring-the-project-settingspy). |
| `mpc_controller.py` / `mpc_core.py` / `control_utils.py` (staged under `fsds_simulator/control/fsae_control/fsae_control/mpc/` and `.../fsae_control/`) | The live ROS 2 MPC controller for FSDS, `mpc_controller.py`'s `standalone_output` parameter selects its output mode. See [Simulator Integration](developer_guide.md#simulator-integration). |
| `fsds_simulator/` (whole tree) | Full staging mirror of upstream's ROS 2 workspace, every package, not just control, so a clone of this repo plus FSDS can build and run the complete stack (`stanley` or `mpc`, either `standalone_output` mode) with no separate `fsae_planning` checkout. See [`docs/reference/`](`docs/reference/`) and [fsds_simulator/README.md](../fsds_simulator/README.md). |

---
<a id="second-controller-nonlinear-mpc-use_nmpc"></a>
## Second controller: nonlinear MPC (`use_nmpc`)

Full technical reference (the structural difference from the LTV-QP, how
it's solved via Gauss-Newton SQP, the full feature-comparison table, and the
three MPCC-inspired additions) has moved to its own document,
**[`nmpc.md`](nmpc.md)**, split out because it had grown too large for this
overview.

In one line: the live workspace carries a second, separately selectable
controller, `nmpc_core.NMPCController` (chosen by the node parameter
`use_nmpc`, default false), which replaces the LTV-QP's fixed-error-frame
prediction with one that carries arc length `s` itself as a horizon state,
so the road's curvature ahead is part of the prediction rather than
something bolted onto the cost via the adaptive gain schedule. See
[`nmpc.md`](nmpc.md) for the full derivation, the solve procedure, and the
side-by-side feature table against the LTV-QP.
