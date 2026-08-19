# Architecture

Deep technical reference for how the simulator, MPC, and offline tuner work.
For quick-start usage, tuning workflow, and FSDS integration steps, see
[Developer Guide](developer_guide.md) instead — this document explains the
system, that one explains how to operate/extend it.

**Two MPC implementations exist**, selected by one flag (`use_nmpc`):
sections 2-7 below describe the default linear time-varying MPC (LTV-QP,
`mpc_core.MPCController`); section 8, [Second Controller: Nonlinear
MPC](#second-controller-nonlinear-mpc-use_nmpc--live-only-2026-08-13),
describes the alternative Frenet-frame nonlinear MPC
(`nmpc_core.NMPCController`) and how it differs.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Configuring the Project (`settings.py`)](#configuring-the-project-settingspy)
3. [Configuring the Vehicle (`model/vehicle_physics.py`)](#configuring-the-vehicle-modelvehicle_physicspy)
4. [How the MPC Works](#how-the-mpc-works)
5. [How the Offline Tuner Works](#how-the-offline-tuner-works)
6. [The Composite Score](#the-composite-score)
7. [Module Reference](#module-reference)
8. [Second Controller: Nonlinear MPC (`use_nmpc`)](#second-controller-nonlinear-mpc-use_nmpc--live-only-2026-08-13)

---

## Architecture Overview

### Full System Flow

This is the closed loop the simulator runs at 20 Hz. It's the same loop
`tuner/offline_tuner.py` runs headless (no plotting) thousands of times
during tuning, and the same loop `mpc_controller_standalone.py` (staged
under `fsds_simulator/`, pasted into `fsae_planning` — see
[docs/planning_control_sync.md](planning_control_sync.md)) runs live against
the real/FSDS vehicle. All three share one implementation
(`sim/rollout_core.run_core_rollout()` for the first two;
`mpc_core.MPCController` for the live node, kept in numeric parity with
`rollout_core`).

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
are thin wrappers around `rollout_core.run_core_rollout()` (`sim/rollout_core.py`)
— the single implementation of the tracking-error computation, progress tracking,
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
mpc_core.py                     │  gui/simulation.py / mpc_core.py               (shared)
mpc_controller_standalone.py    │  gui/simulation.py's rollout loop              (shared design — see docs/planning_control_sync.md)
cone_recorder.py                │  sim/track_io.py + gui/simulation.py's Load Recorded Track  (recorder writes what the loader reads)
```

`fsds_simulator/control/fsae_control/fsae_control/stanley_controller.py` is
the actual current Stanley controller (mirrored from upstream, kept in sync
like everything else under `fsds_simulator/` — see
[docs/planning_control_sync.md](planning_control_sync.md)), not just a
structural reference. This project's tuner and offline simulator only ever
drive against the MPC (`mpc_controller_standalone.py` / `mpc_core.py`,
same directory) — Stanley is mirrored purely so `fsds_simulator/` can stand
up the full live stack, not because this repo's own simulator exercises it.

---
## Configuring the Project (`settings.py`)

`settings.py` is the single place to change tuning knobs, cost weights, and
DNF/validation configuration shared by `gui/simulation.py`,
`tuner/offline_tuner.py`, `sim/scoring.py`, `sim/rollout_core.py`, and
`tuner/performance_stats.py`. It has no vehicle physics in it — that lives
in `model/vehicle_physics.py` (see next section). Every setting has a
detailed, plain-language explanation directly above it in the file itself.

For what every weight/gain/flag in `settings.py` does, how to tune it, and
known constraints (including `N_HORIZON`, `DELAY_STEPS`/`DELAY_JITTER_STEPS`,
`SLAM_NOISE_ENABLED` and the rest of the simulator-fidelity settings, the
`Q_diag`/`R_diag`/`R_rate_diag` cost weights, and `SCORE_WEIGHTS`/
`METRIC_SCALES`), see [tuning.md](tuning.md) — this section instead covers
the parts of `settings.py` that are about tuner *mechanics* (DNF detection,
solver settings, the pose-feed-hold sim-to-real model) rather than tuning
values themselves.

### DNF penalty configuration

`DNF_PENALTY` and `DNF_OFFTRACK_PENALTY` — flat score penalties added when a
tuning rollout doesn't finish the track, and an additional penalty
specifically when the reason was leaving the track boundary. These exist so
the tuner can't find a deceptively good score by having the car crawl
slowly and carefully without ever finishing.

### Solver settings for headless rollouts

`ROLLOUT_EPS` / `ROLLOUT_MAX_ITER` — OSQP convergence tolerance and iteration
cap used only during offline tuning rollouts (looser than the live
simulator's defaults for faster mass evaluation, at negligible accuracy
cost). `MAX_EVALS` — total true-rollout budget for one tuning run.
`PATH_N_POINTS` — how many points each synthetic test track is resampled to.
`USE_OPTUNA_PRESEARCH` / `OPTUNA_PRE_PASS_EVALS` — optional TPE pre-search
that seeds CMA-ES's starting point; see
[Optional Optuna TPE pre-search](#optional-optuna-tpe-pre-search).

### Scoring weights

`SCORE_WEIGHTS`/`METRIC_SCALES` define what "good driving" means to the
tuner. See [tuning.md](tuning.md#6-scoring-metric_scales-and-score_weights)
for how to tune these; see [The Composite Score](#the-composite-score) below
for exactly what each of the 13 metrics measures and how they combine into
one score.

`VALIDATION_SUITE` — which of the synthetic corner-shape paths (defined in
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

In the failed run the pose froze for ~1 s at 14 m/s — ~17 m travelled blind —
and the car spun on resume with 105° of heading error.

This is distinct from the two existing delay knobs, and none of them substitute
for it:

- `DELAY_STEPS` delays a pose that is still **fresh** every tick.
- `DELAY_JITTER_STEPS` perturbs only the controller's **belief** about the lag.
- `PoseFeedHold` repeats the **data**, so `pose_age` genuinely ramps and the
  controller is briefly blind.

While a hold is active the rollout also **skips perception and planning**, since
on the car the planner is triggered by `car_position` — a stalled pose stalls
the whole chain. Without that, re-planning from a frozen pose still yields a
slightly different centreline each tick and the controller is never actually
blind (measured: `e_y` repeated on 0.0% of ticks instead of the intended ~5%).

Tuned to the normal run: `POSE_HOLD_PROB = 0.05`, `MEAN_TICKS = 2.1`,
`MAX_TICKS = 5` reproduces 5.8% repeated ticks / mean hold 2.10 against the
measured 5.3% / 2.08.

> **This does NOT close the sim-to-real gap.** With the model on and firing
> correctly, steering saturation moves only 3.4% → 4.4% against a live 21.1%,
> and heading error 6.0° → 6.3° against a live 15.9°. The pose hold is real and
> now faithfully reproduced, but it is **not** the cause of the gap. Plant grip,
> corner entry speed, planner centreline quality, SLAM pose noise, extra
> actuation delay and planner update rate have each also been tested and
> eliminated. The cause remains open — do not treat offline scores as
> predictive of live behaviour until it is found.

---

### Bonus weights

`TIME_BONUS_WEIGHT` — legacy weight, no longer used by the score itself.
Time is now the *primary objective* (tier 2), scaled by
`TIME_OBJECTIVE_WEIGHT`, not a bonus subtracted from a metric sum.

`COMPLETION_BONUS_WEIGHT` — **no longer used by the score.** Completion is a
hard constraint (tier 1), not something rewarded: a run that doesn't finish
is scored above `CONSTRAINT_FLOOR` regardless of how well it drove. Both
constants are retained only so the live copy's CSV header and
`tuning history.txt` logging keep their existing fields.

---

## Configuring the Vehicle (`model/vehicle_physics.py`)

The single source of truth for all vehicle physics — mass, geometry, tyre
grip, suspension, aerodynamics, actuator limits — is the `VehicleParams`
class in `model/vehicle_physics.py`. This is what the nonlinear 24-state
plant (the "truth" simulation) uses, and several of these values (`Cf`,
`Cr`, `tau_delta`, `tau_a`, `lf`, `lr`, `m`, `Iz`) also feed directly into
the MPC's own internal linear model in `model/bicycle_model.py` — see
[How the MPC Works](#how-the-mpc-works) for how those specific values are
used mathematically.

### Global scaling knobs

Three constants at the top of `VehicleParams.__init__` proportionally scale
groups of related parameters, so you don't have to hand-tune every
individual tyre/inertia constant to make the car noticeably grippier,
heavier-feeling, or coast further:

```python
GRIP_SCALE     = 1.1   # Scales tyre stiffness (B) and peak grip (D) together
INERTIA_SCALE  = 0.8   # Scales yaw inertia and wheel rotational mass together
COASTING_SCALE = 3.0   # Scales rolling resistance / drivetrain drag only, NOT aero drag (Cd_A is a fixed physical value) — < 1.0 = rolls further, > 1.0 = stops faster
```

Prefer adjusting these three over individual Pacejka/inertia constants
unless you have real tyre test data (TTC) or measured chassis inertia to
plug in directly.

### If you import new tyre data

The plant uses a Pacejka **MF94** tyre model (`B`, `C`, `D`, `E`, `Sv`, `Sh`
per axle — see [The Pacejka Tyre Model](#the-pacejka-tyre-model) below for
what each coefficient physically means). If you replace these with real TTC
data:

> **You must also recompute `Cf` and `Cr`** (the *linear* cornering
> stiffnesses used by the MPC's internal bicycle model in
> `model/bicycle_model.py`) to match the new Pacejka curve's initial slope near
> zero slip angle: `C_eff ≈ mu * Fz_nominal * B * C * D`. If `Cf`/`Cr` don't
> match the new Pacejka peak (`D`) and stiffness (`B`), the MPC's internal
> prediction model will diverge from the plant it's actually controlling,
> degrading tracking performance in ways that are hard to diagnose from the
> symptoms alone.

### Actuator limits

`max_steer`, `max_accel`, `max_accel_brake` — changing these automatically
propagates to the MPC's hard QP constraints in `controller/optimiser.py` and
`mpc_core.py` (both read `VehicleParams` directly), so the controller
will never be asked to command something the (simulated) vehicle physically
can't do.

### The Pacejka Tyre Model

The plant computes tyre grip using the Pacejka **MF94** "Magic Formula" —
an empirical curve fit to real tyre test data, rather than a physics-derived
equation:
Fy = mu · Fz · sin(C · atan(B·α − E·(B·α − atan(B·α))))

Where `α` is slip angle (lateral) or slip ratio (longitudinal), and `Fz` is
the tyre's current normal load. See
[vehicle_physics_guide.md §4](vehicle_physics_guide.md#4-what-is-full-mf94-pacejka-and-what-is-a-tyre-model-at-all)
for what each coefficient (`B`/`C`/`D`/`E`/`Sv`/`Sh`), `mu`/`k_sens`, tyre
relaxation, and the friction ellipse physically mean — not repeated here.

This curve is where the plant's nonlinearity actually shows up numerically.
Near `α = 0` it's *approximately* a straight line through the origin —
that local slope is exactly the linear cornering-stiffness `Cf`/`Cr` the
MPC's internal model assumes holds everywhere (see "Linear vs nonlinear" in
[How the MPC Works](#how-the-mpc-works)). Push `α` out past roughly 5-8° of
slip, though, and the real curve visibly bends over: each extra degree of
slip buys noticeably less extra force than the last, until it saturates at
`D` and can even fall past that (a tyre that's broken traction). Doubling
the slip angle out here does **not** double the force — it might only add
20% more, or none at all — which is exactly the behaviour a fixed-multiplier
linear model cannot represent.

---
## How the MPC Works

This section explains the controller in full: the state vector, where every
entry of every matrix comes from, the cost function, the solver, and the two
runtime adaptive features layered on top. The implementation is split across
three files that must be kept in numeric agreement — `model/bicycle_model.py`
(the prediction model), `controller/optimiser.py` (the QP formulation, used
by the simulator/tuner), and `mpc_core.py` (a self-contained duplicate of
both, used by the live ROS 2 node so it has no simulator dependencies).

This section assumes no prior background — every term below (model, state,
input, cost function, solver) is defined in plain English the first time it
appears.

### Four terms you need before anything else makes sense

- **State**: the set of numbers that describe "where things stand right
  now." For this controller, state doesn't mean the car's raw (X, Y)
  position — it means *how far off the intended path the car currently is*
  (see [The 8-state error vector](#the-8-state-error-vector) below for the
  actual list). Written as a vector, `x`.
- **Input** (also called a **command**): the numbers the controller
  actually gets to choose each tick — steering angle and
  acceleration/braking. Written as a vector, `u`.
- **Model**: a mathematical rule that answers *"given the current state `x`
  and a chosen input `u`, what will the state be a moment later?"* It's the
  controller's internal, simplified stand-in for "how the car behaves,"
  used purely for planning — not the real car itself. This project's model
  is a **bicycle model**: instead of simulating all four wheels, the car is
  approximated as one wheel on the front axle and one on the rear, both on
  the centreline. That's a standard simplification in vehicle control — it
  captures the two things that matter most for path tracking (how the
  front wheel steers, and how the whole car rotates/slides) while staying
  simple enough to evaluate thousands of times a second.
- **Cost function**: a single number that scores "how bad" a candidate plan
  is — bigger tracking error, more control effort, or jerkier commands all
  make this number bigger. The controller's whole job each tick is to
  search for the sequence of inputs that makes this number as small as
  possible. The thing doing that search is called the **solver** (see [The
  solver](#the-solver) below) — you don't write a search algorithm
  yourself, you hand the cost function and the model to an existing,
  purpose-built solver library and it finds the best answer.

With those four ideas in hand, the rest of this section builds up exactly
what this controller's state, input, model, and cost function actually are,
in full detail.

### What "MPC" means here

At every control tick (20 Hz), the controller:

1. Measures the current tracking error (`x0`).
2. Predicts, using a simplified **linear** model, what the tracking error
   would do over the next `N_HORIZON` steps (1.75 s) for every possible
   sequence of steering/throttle commands.
3. Solves for the sequence that minimises a cost (tracking error + control
   effort + smoothness), subject to hard limits (max steering angle, max
   acceleration, a soft lane boundary).
4. Applies **only the first command** in that sequence to the real
   (nonlinear) plant.
5. Throws the rest of the plan away and repeats from measurement at the next
   tick.

This "solve a plan, use only the first step, replan" pattern is the
*receding horizon* principle, and it's what makes MPC robust to the fact
that its internal model (linear, 8-state) is not a perfect match for the
real vehicle (nonlinear, 24-state, Pacejka tyres, suspension, aero). Any
mismatch between what the model predicted and what the plant actually did
shows up as tracking error at the next measurement, and gets corrected on
the next solve — the controller never needs its internal model to be
perfectly accurate, only good enough to plan a *reasonable* next step.

### The 8-state error vector

The MPC does not track the car's raw position (X, Y). It tracks **error
relative to the path** — how far off, and in what way, the car currently is.
This keeps the model's behaviour independent of where on the map the car
happens to be.

```
x = [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
```

| # | Symbol | Meaning | Units |
|---|---|---|---|
| 0 | `e_y` | Lateral (sideways) distance from the path centreline | m |
| 1 | `e_y_dot` | Rate of change of `e_y` | m/s |
| 2 | `e_psi` | Heading error — car's yaw minus the path's tangent direction | rad |
| 3 | `e_psi_dot` | Yaw rate (how fast the car's heading is currently changing) | rad/s |
| 4 | `e_v` | Speed error — current speed minus the planner's target speed | m/s |
| 5 | `e_a` | Unused acceleration-error placeholder, always driven toward 0 | m/s² |
| 6 | `delta_act` | The steering angle the actuator has *actually* reached so far (after lag) | rad |
| 7 | `a_act` | The acceleration command the actuator has *actually* reached so far (after lag) | m/s² |

States 6 and 7 exist because a real steering rack / throttle doesn't jump
instantly to a commanded value — there's a first-order lag (see
`tau_delta`, `tau_a` in `model/vehicle_physics.py`). Tracking the *actual*
(lagged) actuator state, not just the commanded value, lets the model
correctly predict how the car will really move over the horizon.
State 5 is purely for consistency, there is a rate of change for each state.

**`e_v`'s target speed is frozen for the whole horizon, not a per-step
profile.** `desired_speed` is looked up/computed once per solve (from
`speed_profile.curvature_speed()` live, or the offline `v_profile` array
previously — see "Planner/Speed-Profile Architecture" below) and baked into
`x0[4]` as a single scalar; nothing in `Ad`/`Bd` re-references it at later
horizon steps, so the cost function penalises deviation from *the same*
target speed across all `N` steps (1.75 s), not the true curvature-limited
speed at each predicted future position. This is a deliberate receding-horizon
simplification — the controller re-solves every 50 ms with a freshly
recomputed `desired_speed`, so a stale in-horizon reference self-corrects
within one tick — not a bug, but worth knowing if you're debugging
speed-tracking behaviour approaching a corner whose onset falls inside the
current horizon.
Currently there is no acceleration profile so there is no acceleration error.

#### How the error vector is actually measured (Frenet-frame projection)

The error states above (`e_y`, `e_psi`, etc.) aren't things the car can
read off a sensor directly — they only make sense *relative to a point on
the path*. Every control tick, `vehicle_physics.plant_to_tracking_error()`
(`model/vehicle_physics.py`) has to answer: "of all the points along the
reference path, which one is
the car currently 'at', and how far off is it from that point?"

This is a **Frenet-frame** conversion: instead of describing the car's
position in the usual global (X, Y) map coordinates, it's re-described
relative to the path itself — as a longitudinal position *along* the path
plus a lateral offset *perpendicular* to it. Concretely, the code:

1. Finds the nearest reference point on the path to the car's current
   (X, Y) position (`get_interpolated_ref_point()`), giving a reference
   `(ref_x, ref_y, ref_psi)` — the path's position and tangent heading at
   that point.
2. Projects the car's offset from that point onto the direction
   perpendicular to the path's tangent, which gives the signed lateral
   error `e_y` (positive/negative = left/right of the centreline).
3. Takes the difference between the car's heading and the path's tangent
   heading at that point, giving `e_psi`.

This is the same idea used throughout path-tracking control (and in the
planner's centreline/curvature calculations — see
[Architecture Overview](#architecture-overview)): re-expressing "where am
I" as "how far along the path, and how far off to the side," which is a
much more useful frame for a controller whose whole job is to stay close
to a curve, rather than reaching a specific (X, Y) point.

### The 2-input control vector

```
u = [delta_cmd, a_cmd]
```

`delta_cmd` (rad) and `a_cmd` (m/s²) are the raw commands sent to the
actuator lag filters — not the actual steering angle / acceleration
themselves (those are states 6 and 7 above, which lag behind `u`).

### Building the prediction model (`model/bicycle_model.py`)

Before the MPC can plan anything, it needs a way to answer the question:
*"if the car is currently in error state `x`, and I apply steering/throttle
command `u`, what will the error state be a tiny fraction of a second
later?"* That question, answered mathematically, is the **prediction
model**. This section builds it up from scratch — the general form, the two
physical models that get blended into it, and finally how it's converted
into the exact numbers the solver uses.

The car itself is approximated as a **bicycle model** — instead of four
separate wheels, it's treated as one wheel on the front axle and one wheel
on the rear axle, both sitting on the car's centreline. This is a standard
simplification in vehicle control: it captures the two things that matter
most for path tracking (how the front wheel steers, and how the whole car
rotates and slides sideways) while staying simple enough to solve fast,
20 times a second.

#### The general continuous-time form

Every linear model in control theory is written the same way:

```
ẋ = A·x + B·u
```

Read this as: **"the rate of change of the state vector (ẋ) is some fixed
mixture of the current state (x) plus some fixed mixture of the current
command (u)."** `A` and `B` are just tables of numbers (matrices) that say
*how much* of each state and each command feeds into the rate of change of
every other state. This is "continuous-time" because `ẋ` is a true
instantaneous rate of change (like a speedometer reading), not a per-tick
step — that comes later.

Recall the 8-state error vector and 2-input command vector from above:

```
x = [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]ᵀ
u = [delta_cmd, a_cmd]ᵀ
```

So `A` is an **8×8** grid of numbers and `B` is an **8×2** grid of numbers.
Reading the grid: **entry `A[row, col]` is a multiplier saying "how much
does the current value of state `col` contribute to the rate of change of
state `row`."** Most entries are zero, because most states have no direct
physical influence on most other states — only a handful of meaningful
physical relationships exist, and those are the only non-zero numbers in
the grid. Two different physical assumptions produce two different sets of
numbers for `A` (`B` turns out to be the same in both), described next.

#### 1. Kinematic model (used below ~1 m/s)

At very low speed, the tyres haven't built up any real sideways
(cornering) grip yet — the car turns purely by geometry, the same way
pushing a shopping trolley by its handle makes it pivot. The physical
relationships are:

```
ė_y   = v_x · e_psi
ė_psi = v_x · delta_act / L        (L = wheelbase = lf + lr)
```

In plain words: *"how fast the car drifts sideways off the path depends on
how much it's currently pointing the wrong way, scaled by speed"* (turn
your wheels while stationary and nothing happens — sideways drift needs
forward motion to convert into it), and *"how fast the car's heading is
changing depends on the current steering angle and speed, via the
wheelbase"* (standard Ackermann steering geometry — a longer car turns more
slowly for the same steering angle).

Every other state either isn't affected in this simple model, or follows
the same "shared" behaviour described in section 3 below (actuator lag,
speed error). Written out as the full 8×8 matrix `A_kin` (blank cells are
zero):

```
        e_y   e_y_dot  e_psi  e_psi_dot   e_v    e_a   delta_act  a_act
e_y   [  0      0      v_x       0         0      0        0        0   ]
e_y_dot[ 0      0       0        0         0      0        0        0   ]
e_psi [  0      0       0        0         0      0      v_x/L      0   ]
e_psi_dot[0     0       0        0         0      0        0        0   ]
e_v   [  0      0       0        0         0      1        0        0   ]
e_a   [  0      0       0        0         0      0        0        1   ]
delta_act[0     0       0        0         0      0     -1/tau_δ    0   ]
a_act [  0      0       0        0         0      0        0    -1/tau_a]
```

In code:

```python
A_kin[0, 2] = v_x_safe          # ė_y = v_x * e_psi
A_kin[2, 6] = v_x_safe / L      # ė_psi = v_x/L * delta_act  (Ackermann geometry)
```

(rows 4-7 are the shared rows, covered in section 3.)

#### 2. Dynamic model (used above ~2.5 m/s)

At higher speed, tyre grip (cornering stiffness × slip angle) dominates
over pure geometry — this is the regime a real car spends most of its time
in. It's the standard linearised bicycle model, derived from Newton's laws
for a rigid body sliding and rotating in a plane, assuming small slip
angles:

```
ë_y   = -(2Cf+2Cr)/(m·vx) · ė_y  +  (2Cf+2Cr)/m · e_psi
        + (-2Cf·lf+2Cr·lr)/(m·vx) · e_psi_dot  +  (2Cf)/m · delta_act

ë_psi = (-2Cf·lf+2Cr·lr)/(Iz·vx) · ė_y  +  (2Cf·lf-2Cr·lr)/Iz · e_psi
        - (2Cf·lf²+2Cr·lr²)/(Iz·vx) · e_psi_dot  +  (2Cf·lf)/Iz · delta_act
```

`Cf`/`Cr` are the front/rear cornering stiffnesses (N/rad, from
`VehicleParams` — how much sideways force a tyre generates per radian of
slip angle), `lf`/`lr` are the distances from the car's centre of mass to
each axle, `m` is mass, and `Iz` is yaw inertia (how hard it is to make the
car spin, similar to how a figure skater with arms out spins slower). The
`1/vx` terms exist because at higher speed, the same sideways drift
produces a *smaller* slip angle — the tyre has rolled further forward for
the same amount of sideways motion, so it "notices" the slide less, and
grip builds up more gradually rather than instantly.

As the full 8×8 matrix `A_dyn`:

```
         e_y  e_y_dot          e_psi           e_psi_dot         e_v  e_a  delta_act    a_act
e_y     [ 0     1                0                 0              0   0       0           0   ]
e_y_dot [ 0  -(2Cf+2Cr)/(m·vx) (2Cf+2Cr)/m  (-2Cf·lf+2Cr·lr)/(m·vx) 0   0    (2Cf)/m        0   ]
e_psi   [ 0     0                0                 1              0   0       0           0   ]
e_psi_dot[0 (-2Cf·lf+2Cr·lr)/(Iz·vx) (2Cf·lf-2Cr·lr)/Iz -(2Cf·lf²+2Cr·lr²)/(Iz·vx) 0 0  (2Cf·lf)/Iz  0 ]
e_v     [ 0     0                0                 0              0   1       0           0   ]
e_a     [ 0     0                0                 0              0   0       0           1   ]
delta_act[0     0                0                 0              0   0    -1/tau_δ       0   ]
a_act   [ 0     0                0                 0              0   0       0        -1/tau_a]
```

In code:

```python
A_dyn[0, 1] = 1.0                                          # ė_y = e_y_dot
A_dyn[1, 1] = -(2*Cf + 2*Cr) / (m * v_x_safe)              # Lateral damping
A_dyn[1, 2] = (2*Cf + 2*Cr) / m                             # Heading error → lateral accel
A_dyn[1, 3] = (-2*Cf*lf + 2*Cr*lr) / (m * v_x_safe)         # Yaw rate → lateral accel
A_dyn[1, 6] = (2*Cf) / m                                    # Steering → lateral force
A_dyn[2, 3] = 1.0                                           # ė_psi = e_psi_dot
A_dyn[3, 1] = (-2*Cf*lf + 2*Cr*lr) / (Iz * v_x_safe)        # Lateral velocity → yaw moment
A_dyn[3, 2] = (2*Cf*lf - 2*Cr*lr) / Iz                      # Heading error → yaw moment
A_dyn[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x_safe)  # Yaw damping (both axles)
A_dyn[3, 6] = (2*Cf * lf) / Iz                               # Steering → yaw moment
```

Notice row 1 (`e_y_dot`) here isn't just `ė_y = ...` like the kinematic
model — it's a *second-order* relationship (`ë_y`, acceleration of lateral
error), so the state `e_y_dot` itself needs its own row saying `ė_y = 
e_y_dot` (row 0, entry `[0,1] = 1`) before row 1 can describe how
`e_y_dot` itself accelerates. This is the standard trick for turning a
second-order physical equation into two coupled first-order ones, which is
why the dynamic model needs both `e_y` *and* `e_y_dot` as genuinely
separate states, while the kinematic model above barely used `e_y_dot` at
all.

#### 3. Shared rows (identical in both models)

Four rows don't depend on which physical regime is active — they're either
structural bookkeeping or simple decay behaviour, so both `A_kin` and
`A_dyn` set them identically:

```python
A_kin[4, 5] = A_dyn[4, 5] = 1.0             # ė_v = e_a
A_kin[5, 7] = A_dyn[5, 7] = 1.0             # ė_a = a_act (structural; e_a itself is unused)
A_kin[6, 6] = A_dyn[6, 6] = -1.0 / tau_delta  # dδ_act/dt = -δ_act/tau_delta (decays toward 0 with no input)
A_kin[7, 7] = A_dyn[7, 7] = -1.0 / tau_a      # da_act/dt = -a_act/tau_a
```

The last two rows describe **actuator lag**: a real steering rack or
throttle doesn't jump instantly to a commanded value, it eases toward it.
Left alone (no new command), `delta_act` and `a_act` naturally decay back
toward zero over a time constant `tau_delta`/`tau_a` — like a stretched
spring relaxing. What actually *drives* them toward the commanded value is
the input matrix `B` (8×2 — one column per command, `delta_cmd` and
`a_cmd`), which is identical for both the kinematic and dynamic models:

```
           delta_cmd   a_cmd
e_y       [   0          0   ]
e_y_dot   [   0          0   ]
e_psi     [   0          0   ]
e_psi_dot [   0          0   ]
e_v       [   0          0   ]
e_a       [   0          0   ]
delta_act [ 1/tau_δ       0   ]
a_act     [   0        1/tau_a]
```

```python
B[6, 0] = 1.0 / tau_delta   # delta_cmd drives the steering lag integrator
B[7, 1] = 1.0 / tau_a       # a_cmd drives the acceleration lag integrator
```

Together, row 6 of `A` and row 6 of `B` combine into the classic
first-order lag equation `dδ_act/dt = (delta_cmd − δ_act) / tau_delta` —
the actuator moves toward the command, at a rate proportional to how far
away it still is (the `-δ_act/tau_delta` self-decay term lives in `A`,
the `+delta_cmd/tau_delta` "pull toward the target" term lives in `B`).

#### What the matrix multiplication actually produces

Putting `A` and `B` together, `ẋ = A·x + B·u` means: multiply each row of
`A` by the entire state vector `x` (a dot product), add the matching row of
`B` multiplied by `u`, and that gives you the rate of change of that one
state. Spelling out just the two most important rows — using the dynamic
model's `e_y_dot` row and the kinematic model's `e_psi` row as concrete
examples — the matrix multiplication `A·x` expands into exactly the
physical equations from sections 1 and 2:

```
Row 1 (e_y_dot) of A_dyn · x  =
    0·e_y + [-(2Cf+2Cr)/(m·vx)]·e_y_dot + [(2Cf+2Cr)/m]·e_psi
    + [(-2Cf·lf+2Cr·lr)/(m·vx)]·e_psi_dot + 0·e_v + 0·e_a
    + [(2Cf)/m]·delta_act + 0·a_act

  = -(2Cf+2Cr)/(m·vx)·e_y_dot + (2Cf+2Cr)/m·e_psi
    + (-2Cf·lf+2Cr·lr)/(m·vx)·e_psi_dot + (2Cf)/m·delta_act

  = ë_y      ← exactly the dynamic-model equation from section 2
```

```
Row 2 (e_psi) of A_kin · x  =  [v_x/L]·delta_act  =  ė_psi
  ← exactly the kinematic-model equation from section 1
```

Every zero entry in the row simply means "this state has no effect here" —
the dot product just drops those terms out. This is the whole point of
writing the physics as a matrix: instead of writing eight separate
equations by hand, `Ad @ x + Bd @ u` (one line of code) computes all eight
rates of change at once, which is exactly what lets the solver evaluate the
model quickly, thousands of times, while searching for the best control
sequence.

#### 4. Blending kinematic and dynamic models

A single linear model can't represent the car well across its whole speed
range — the kinematic model breaks down once tyres start sliding, and the
dynamic model's `1/vx` terms blow up as speed approaches zero. Rather than
switching abruptly between the two (which would cause a visible jump/jerk
in the car's predicted behaviour right at the switch-over speed), the two
matrices are blended smoothly:

```python
alpha = clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
A_c   = (1.0 - alpha) * A_kin + alpha * A_dyn
```

`alpha` ramps linearly from 0 to 1 as speed goes from 1 m/s to 2.5 m/s:
pure kinematic model below 1 m/s, pure dynamic model above 2.5 m/s, and a
proportional mix of the two matrices' numbers in between (e.g. at
`alpha = 0.5`, every entry of `A_c` is exactly halfway between the
matching entry of `A_kin` and `A_dyn`). `B` is identical in both models, so
it doesn't need blending — it's used unchanged regardless of `alpha`.

#### 5. From continuous to discrete: Zero-Order Hold (ZOH)

Everything above describes `ẋ = A_c·x + B_c·u` — an instantaneous,
continuous-time rate of change. But the MPC doesn't operate continuously;
it makes one decision every `dt = 0.05 s` and holds that decision fixed
until the next tick. What it actually needs is a **discrete** one-step
prediction:

```
x[k+1] = Ad·x[k] + Bd·u[k]
```

— "given the state right now (`x[k]`) and the command I'm about to hold for
the next 0.05 s (`u[k]`), what will the state be exactly one tick later
(`x[k+1]`)?" Converting the continuous equation into this discrete one is
called **discretisation**, and the method used here is **Zero-Order Hold
(ZOH)** — the exact, mathematically correct discretisation for a system
where the input is held constant between updates (a "zero-order hold" on
the input), which is precisely how MPC applies its commands. This is more
accurate than a simpler method like Euler's approximation, which introduces
compounding error at every step.

Both `Ad` and `Bd` are computed together via one matrix exponential (`expm`
— the matrix equivalent of `e^x`) on an augmented matrix, which sidesteps
having to directly invert `A_c` (a numerically risky operation if `A_c` is
close to singular):

```
exp( [A_c  B_c] · dt )  =  [Ad  Bd]
     [ 0    0 ]            [ 0  I ]
```

```python
M[:8, :8] = A_c
M[:8, 8:] = B_c
Md = scipy.linalg.expm(M * dt)
Ad, Bd = Md[:8, :8], Md[:8, 8:]
```

`Ad` and `Bd` are what actually get handed to the solver — the continuous
matrices `A_c`/`B_c` above exist only as an intermediate step to build them
correctly.

#### Linear vs nonlinear, in plain English

Now that `Ad`/`Bd` exist concretely, it's worth being precise about what
"linear" actually means here, since the word gets used constantly below. A
model is **linear** if every output is just a fixed multiple of each input,
added together — double an input and its contribution exactly doubles, and
no input's effect depends on the current value of another input. Every
matrix built above (`A`, `B`, `Ad`, `Bd`) is exactly this: a table of fixed
multipliers, so `x[k+1] = Ad·x[k] + Bd·u[k]` is always "this state times a
fixed number, plus that state times a fixed number, ...", never anything
that bends or saturates depending on where the car currently is.

That rigid structure is what lets the solver treat "find the best control
sequence" as a **Quadratic Program (QP)** — see [The cost function and
QP](#the-cost-function-and-qp-controlleroptimiserpy) and [The
solver](#the-solver) below — with a fast, predictable solve and a
guaranteed global optimum every tick.

The real plant (`model/vehicle_physics.py`) has no such structure — its
tyre forces, weight transfer, and heading kinematics genuinely curve and
saturate (see the tyre section above for concrete numbers). If the MPC
tried to plan against those equations directly, the relationship between
`x[k+1]` and `(x[k], u[k])` would no longer reduce to a fixed multiplier
table, and the problem would stop being a QP and become a much harder
nonlinear program (NLP) — no guaranteed optimum, no fast off-the-shelf
solver, and solve times that can balloon unpredictably.

That's exactly why the MPC doesn't plan against the real nonlinear plant
directly: it builds the much simpler **linear** approximation derived above
(good near the car's *current* operating point, thanks to the
kinematic/dynamic blend) and re-linearises it fresh every single tick as
speed and conditions change. The nonlinear plant is reserved for simulating
what actually happens to the "real" car in response to a command — see
[Configuring the Vehicle](#configuring-the-vehicle-modelvehicle_physicspy)
above.

#### Note on OSQP sparsity

`Ad` and `Bd` are consumed a few sections down by **OSQP**, the QP
(Quadratic Program — see [The solver](#the-solver) below) solver that
actually computes the steering/throttle command every tick. OSQP has a
quirk that affects how these matrices must be initialised, explained here
since it's decided at model-construction time even though it only matters
once the solver is involved.

All matrices start as `1e-12` (not exact `0.0`) rather than `np.zeros(...)`.
OSQP analyses which matrix entries are nonzero on its *first* solve and
caches that pattern (the "sparsity pattern" — the *set* of matrix
positions holding a nonzero value) for speed. If a later solve produces an
entry that rounds exactly to zero where it was previously nonzero (which
can happen as `vx` changes and terms like `1/vx` shrink), OSQP's cached
factorisation becomes invalid and it throws a reallocation error. Filling
every entry with a tiny nonzero epsilon keeps the sparsity pattern
identical at every speed, so OSQP never needs to re-analyse it mid-run. See
[The solver](#the-solver) for what OSQP is doing with these matrices and
why sparsity matters to it in the first place.

### The cost function and QP (`controller/optimiser.py`)

The cost function is the concrete answer to "how bad is this candidate
plan" (see the plain-English definition at the top of this section). Each
solve minimises, over the predicted `N`-step horizon:

```
min  Σᵢ ‖√Q ⊙ x[:,i]‖²   (state/tracking cost, all N+1 predicted states)
   + Σᵢ ‖√R ⊙ u[:,i]‖²   (control effort cost, all N inputs)
   + Σᵢ ‖√R_rate ⊙ Δu[:,i]‖²   (smoothness cost, penalises step-to-step change)
   + W_slack · ‖slack‖²   (soft lane-boundary violation penalty)

subject to:
   x[:,0] = x0                           (must start at the measured state)
   x[:,k+1] = Ad·x[:,k] + Bd·u[:,k]       (obey the linear model, all N steps)
   u_min ≤ u[:,k] ≤ u_max                 (hard actuator limits)
   -3.5 - slack ≤ x[0,k] ≤ 3.5 + slack    (soft ±3.5 m lane corridor on e_y)
```

`Q`, `R`, `R_rate` are diagonal weight matrices — one number per state/input
dimension, controlling how much the solver cares about minimising that
particular quantity relative to the others (see the
[Manual Tuning Guide](junior_project_mpc_docs.md#26-manual-tuning-guide) and
the comments in `settings.py` for what each entry means practically). They're
expressed and injected as
square roots (`sqrtQ`, `sqrtR`, `sqrtR_rate`) so the cost can be written with
`cp.sum_squares`, which CVXPY maps efficiently onto OSQP's internal
quadratic-cost matrix — this is a numerical-stability/implementation choice,
not a change in what's being penalised (`‖√w·x‖² = w·x²`).

**Why states 5-7 (`e_a`, `delta_act`, `a_act`) are never tuned:** only the
first 5 diagonal entries of `Q` (`e_y` through `e_v`) and both entries of
`R`/`R_rate` are exposed to the offline tuner (`TUNABLE_Q_IDX = [0,1,2,3,4]`
in `tuner/offline_tuner.py`). `Q[5,5]` (`e_a`) stays at 0 because that state is a
structural placeholder with no independent target — penalising it would
just add noise to the cost with no corresponding control lever. `Q[6,6]`
and `Q[7,7]` (`delta_act`, `a_act`) also stay at 0 because those are
*measurements* of where the actuator currently is, not tracking errors —
there's no "correct" value for them to be pulled toward; the actual
steering/acceleration commands are already penalised directly through `R`
and `R_rate` instead.

**The rate-of-change (smoothness) cost is split into two pieces** because
the first horizon step needs a different "previous command" than every
step after it:

```python
# Step 0: compare against the last command actually sent to the real plant
cost += sum_squares(sqrtR_rate * u[:,0] - sqrtR_rate * u_prev)

# Steps 1..N-1: compare each step against the previous *predicted* step
du = cp.diff(u, axis=1)
cost += sum(sum_squares(sqrtR_rate * du))
```

**The soft lane boundary** (`±3.5 m` on `e_y`, matching `TRACK_HALF_WIDTH`)
uses a slack variable rather than a hard constraint. `W_slack = 10000.0` is
large enough that the solver will essentially never choose to violate the
corridor when a compliant solution exists — but because it's *soft*
(penalised, not forbidden), the QP stays solvable even when the car is
already outside the corridor (e.g. mid-recovery from an off-track excursion),
where a hard constraint would make the problem infeasible and the solver
would return nothing at all.

**The "parameterised" trick:** the QP's variables, constraints, and cost
expression are built **once** using `cp.Parameter` placeholders rather than
plain numbers. Every subsequent solve only updates the parameter *values*
(`Ad`, `Bd`, `x0`, weights, etc.) and re-invokes the same compiled problem.
This lets OSQP reuse its cached factorisation and warm-start from the
previous solution — rebuilding the whole CVXPY expression graph from scratch
every tick would be roughly 10× slower and is unnecessary since the
problem's *structure* (which variables relate to which) never changes,
only the numbers plugged into it.

### The solver

**What kind of problem is being solved?** The cost function above (state
error + control effort + smoothness, all squared) is a **quadratic**
function of the unknowns (`x` and `u` over the whole horizon), and every
constraint (dynamics, actuator limits, lane boundary) is **linear**. A
quadratic cost with linear constraints is called a **Quadratic Program
(QP)** — a well-studied category of optimisation problem for which fast,
reliable, purpose-built solvers exist. This is precisely why the cost
function was built the way it was (squared errors, not e.g. absolute
values or something more exotic) — it's what keeps the whole problem inside
this fast-to-solve category rather than needing a slower, more general
optimiser.

**What does "solving" it actually mean?** The solver is handed the fully
built-out cost expression and constraint list from the previous section,
and searches for the one sequence of steering/throttle values (`u[0]`
through `u[N-1]`) that makes the total cost as small as possible, while
never violating a hard constraint (actuator limits) and only softly
violating the lane boundary if truly necessary. It does this by starting
from a guess, checking whether nudging that guess in some direction reduces
the cost while respecting the constraints, and repeating until no further
nudge helps — this iterative process is what OSQP's `max_iter`/`eps_abs`
settings control (how many nudges it's allowed, and how small a nudge
counts as "close enough" to stop).

**Primary: OSQP.** Exploits the QP's sparsity (most matrix entries are
zero, so the solver skips work on them) and supports warm-starting —
reusing the *previous* tick's solution as this tick's starting guess. Since
consecutive MPC solves in a receding horizon differ by only one step (the
horizon just slides forward by 0.05 s each time), the previous answer is
already an excellent starting guess, so warm-started solves typically
converge in ~50-200 nudges instead of 500-2000 from a cold start — this is
what makes solving a QP fast enough to happen 20 times per second. Typical
solve time is 1-5 ms at `N=25`.

**Fallback: Clarabel.** A different (interior-point) solving strategy that
is generally slower per solve but more numerically robust on
poorly-behaved problems. It's only invoked if OSQP itself fails to reach a
usable answer — returning infeasible, unbounded, or hitting numerical
trouble or its iteration cap.

**If both fail**, the simulator/tuner returns `None` and the caller holds
the previous command; the live `mpc_core.MPCController` instead
returns a full-brake command (`[u_prev[0], -a_max_brake]`) — braking is the
safer default for a real vehicle than continuing to coast on a stale plan.

**`OPTIMAL_INACCURATE`** (OSQP found an answer, but not to its full
precision tolerance) is still accepted and used — refusing it and holding
the previous command would generally be worse than using a
slightly-under-converged-but-still-reasonable solution at 20 Hz. The
offline tuner counts these occurrences and applies a scoring penalty (see
[The Composite Score](#the-composite-score)) so weight sets that cause
frequent `OPTIMAL_INACCURATE` are still discouraged, without discarding the
run outright.

### Adaptive gain scheduling (`controller/model_utils.py`)

The tuned `Q`, `R`, `R_rate` weights are optimised as if for a single
"average" operating point. A handful of functions rescale `Q`/`R`/`R_rate`
*every tick* to compensate for known, predictable ways the required control
authority changes with speed and curvature — without needing a separate
tuned weight set for every regime.

**This section describes two generations of that idea.** Through
2026-08-12 the mechanism was a family of ~15 interacting functions that
scanned *forward* along the path (a "lookahead" scan producing a peak
curvature ahead, `kappa_max_abs`) and reweighted the cost matrices in
anticipation of a corner not yet reached. On 2026-08-13 that whole family
was deleted and replaced by three simpler, **current-state-only** factors —
no forward scan at all. **"Current-state gain scheduling" below documents
what runs today; "Historical: the lookahead gain-scheduling family"
documents what it replaced, kept for the reasoning (why it was tried, what
it got right, and the structural argument for why reweighting *today's*
cost based on a *future* corner cannot substitute for a prediction model
that actually represents the road bending — see the [Second controller:
nonlinear MPC](#second-controller-nonlinear-mpc-use_nmpc-2026-08-13) section
for how that argument plays out fully.**

#### Current-state gain scheduling (current, since 2026-08-13)

Every function below reacts only to curvature/error the car is measuring
*right now* — none of them scan the path ahead. Two of these
(`adaptive_R_scaling`, `adaptive_R_rate`) predate the 2026-08-13 rewrite and
were carried over unchanged; the corner-factor blend and the heading-error
accel/brake asymmetry are new in that rewrite.

**`adaptive_R_scaling(vx, R)`** — increases steering cost with speed:

```
steer_scale = 1 + (1.5 · vx) / (6.0 + vx)      # → 1.0 at vx=0, → 2.5 as vx→∞
accel_scale = 1 + 0.05 · vx                     # gentler linear scale
```

At higher speed, the same steering angle produces much more lateral
acceleration (`a_lat ≈ vx² · κ`), so the same-magnitude steering command is
more destabilising. This Hill-function form was chosen over a straight
linear ramp because it *saturates* — steering cost approaches but never
exceeds 2.5× base, so the controller is never effectively locked out of
steering at very high speed. The half-saturation point (`vx_half = 6.0`)
sits in the same speed range where the kinematic→dynamic model blend
transitions (1-2.5 m/s), so extra steering conservatism ramps up exactly
where the internal prediction model itself becomes less certain.

**`adaptive_R_rate(kappa, R_rate, enable_in_corners=True)`** — softens the
steering *jerk* penalty in tight corners, via a floor on the current-position
curvature alone:

```
during_scale = max(0.625, 1 / (1 + 3·κ))     # current-position curvature only
```

`κ` (curvature) is estimated causally from the plant's own current yaw rate
and speed (`curvature_estimate()`: `κ = |yaw_rate| / vx`) — it reflects the
curvature the car is *currently experiencing*. In a straight, the full
smoothness penalty applies. In a tight corner, the penalty is floored rather
than removed entirely — enough softening to let the controller make the fast
steering changes a tight corner demands, without ever allowing the rate cost
to vanish completely (which would permit arbitrarily rapid, oscillatory
steering). (This function used to also combine a second, lookahead-driven
floor via `min()` — see "Historical" below; that half was removed in the
2026-08-13 rewrite, leaving only the current-position floor shown above.)

Both functions return a **copy** of the base matrix — the tuned weights in
`settings.py` are never mutated, only scaled per-tick on top of.

#### Corner-factor scheduler (current, since 2026-08-13)

Everything from here through `steer_rate_anti_hunt` above reacts to
curvature the car is *at* right now — none of it anticipates a corner ahead.
`mpc_core.py`'s `_corner_factor`/`_low_speed_corner_boost`/`_blend` (and the
offline mirror, `controller/model_utils.py`'s functions of the same name)
turn that same current-position `κ` into a single continuous 0→1 "how much
in a corner am I" fraction, then blend several weights between a straight
endpoint and a corner endpoint on that fraction — replacing the entire
forward-scanning lookahead family documented under "Historical" below.

```
corner_factor = 1 - 1 / (1 + corner_factor_k · |κ|)      # 0 (straight) -> 1 (full corner)
low_speed_boost = corner_factor · max_extra · v_half / (v_half + |car_speed|)
corner_frac = clip(corner_factor + low_speed_boost, 0, 1)
```

- **`corner_factor`** is a saturating curve of current curvature only (no
  forward scan, no decay-distance timer, no hysteresis state) — entry and
  exit are exactly symmetric, driven purely by how `κ` itself rises and
  falls tick to tick. `corner_factor_k` (default 8.0) sets the curve's
  sharpness.
- **`low_speed_corner_boost`** adds extra weight toward the "full corner"
  endpoint specifically at low speed, but **only while `corner_factor > 0`**
  — the multiplicative gate on `corner_factor` makes this an exact no-op on
  a straight regardless of speed. That gate is what makes it safe where a
  since-deleted mechanism (`low_speed_steer_rate_boost`, gated on speed
  alone) was not: with no curvature signal to distinguish the two, that
  older mechanism suppressed a car's own turn-in exactly as often as it
  damped the post-exit wobble it was built for.
- **`corner_frac`** (the two combined, clipped to `[0,1]`) drives a shared
  linear blend (`_blend(straight_val, corner_val, corner_frac)`) across four
  weights, each with its own straight/corner endpoint pair in `MPCParams`:
  `Q[0,0]` (`q_ey_straight`/`q_ey_corner`), `Q[2,2]`
  (`q_epsi_straight`/`q_epsi_corner`), `Q[3,3]`
  (`q_r_straight`/`q_r_corner`, which *relaxes* in-corner, unlike the other
  two), and `R_rate[0,0]` (`rrate_steer_straight`/`rrate_steer_corner`,
  likewise relaxing). `R[0,0]` (steering effort) blends toward a **middle**
  value instead (`r_steer_corner_mid`) rather than either extreme, so
  turn-in isn't made maximally cheap right when saturation risk is highest.
- Every quantity above is written into the per-tick telemetry trace
  (`corner_factor`, `low_speed_corner_boost`, `corner_frac`, `Q_ey_base`,
  `Q_ey_eff`, `Rrate_steer_corner_blend`, `R_steer_corner_blend`, ...) so a
  log shows exactly where each blend landed, not just the final QP weights.

**Heading-error-driven accel/brake asymmetry (current, since 2026-08-13,
always-on)** — independent of `corner_frac` above, a continuous fraction of
current `|e_psi|` scales `r_a_accel` (acceleration effort) toward a boost
ceiling — making the MPC less willing to keep accelerating through a
heading error it should be correcting — and scales `r_a_brake` toward a
floor, freeing up braking authority specifically when heading error is
large:

```
frac_epsi = |e_psi| / (|e_psi| + epsi_ra_half_rad)
r_a_accel_eff = r_a_accel · (1 + (epsi_ra_accel_boost_max - 1) · frac_epsi)
r_a_brake_eff = r_a_brake · (1 - (1 - epsi_ra_brake_floor) · frac_epsi)
```

Not a replacement for `adaptive_R_scaling`'s current-speed-driven `R[0,0]`
scaling above, which this leaves untouched — the two compose.

`adaptive_R_rate`'s current-position floor above used to combine with a
second, lookahead-driven floor via `min()` (whichever was more aggressive
won) — see "Historical" below for that half.

**`adaptive_Q_scaling(e_y, Q, enabled)`** — softens the lateral-error cost
`Q[0,0]` when the car is already close to the centreline, to reduce
small-error hunting/chatter:

```
scale = floor                                            |e_y| <= ey_lo (0.05 m)
scale = floor + (1-floor)*(|e_y|-ey_lo)/(ey_hi-ey_lo)     ey_lo < |e_y| < ey_hi (0.3 m)
scale = 1.0                                               |e_y| >= ey_hi
```

- **Why:** steering sign-reversal rate was observed rising as `|e_y|` gets
  *smaller* live — the car darting across the centreline rather than
  settling onto it. A quadratic cost pulls toward zero error with the same
  proportional strength no matter how small the error already is, a
  plausible contributor to a correct-overcorrect cycle right where the
  controller should be settling, not correcting.
- **Status:** `ADAPTIVE_Q_SCALING_ENABLED = True` in `settings.py` (**enabled
  by default since 2026-08-09**, to match the live controller). Still not
  reproduced on the offline recorded-map rollout — there, reversal rate
  rises *with* `|e_y|`, the opposite direction — so it may be a live-only
  symptom of sensor noise, delay-compensation dynamics, or the plant
  behaving differently from the linear model near zero slip. Re-validate
  against `VALIDATION_SUITE`/the recorded map before any further re-tuning
  around it.

**`enable_in_corners` (an `adaptive_R_rate` parameter, on by default)** —
renamed from `disable_in_corners`, whose `True`/`False` polarity was inverted
from what the name suggested. Setting it `False` *undoes* `adaptive_R_rate`'s
softening once estimated curvature exceeds a small "cornering" threshold
(`kappa_straight = 0.03`), restoring the full unscaled `R_rate[0,0]` baseline
instead. Tried disabled and reverted the same day: it caused severe lag
specifically in corners, most likely because the discontinuous cost jump at
the threshold crossing spikes QP solver iterations and invalidates
warm-starts on ticks straddling it. Kept in the code, gated on (softening
active, the setting that avoids the discontinuity), as a documented dead end
rather than deleted, so it isn't accidentally re-tried without this context.

**`steer_rate_anti_hunt(kappa, e_y, R_rate, enabled, e_psi=0.0)`** — stacks on
top of `adaptive_R_rate` (not a replacement): multiplies `R_rate[0,0]` **up**
by a fixed boost ceiling (6.0×) instead of softening it, strongest when the
car is simultaneously straight (`κ` near zero), centred (`|e_y|` small), *and*
well-aligned (`|e_psi|` small):

```
boost_kappa = 1 / (1 + 60·|κ|)
boost_ey    = 1 / (1 + 30·|e_y|)
boost_epsi  = 1 / (1 + 23·|e_psi|)
scale = 1 + (6.0 - 1) · boost_kappa · boost_ey · boost_epsi
```

- Each factor saturates independently toward 1.0 as its input shrinks, so
  the full ceiling only applies when all three are near their "straight,
  centred, aligned" ideal, fading smoothly (never snapping) as any one of
  them grows.
- **Why `e_psi`:** guards against a car that enters a straight *misaligned*
  (large `|e_psi|`, small `|e_y|` — e.g. just exited a corner still pointed
  the wrong way). Without it, `κ`/`e_y` alone can't distinguish "straight
  and correctly aligned" from "straight but needs to yaw back into line",
  making exactly the correction it needs artificially expensive.
- **What it covers:** the "already on the line, not cornering" regime
  `adaptive_R_rate` alone doesn't address — that function only ever softens
  the rate cost for corners, never stiffens it for straights.
- **Status:** `STEER_RATE_ANTI_HUNT_ENABLED = True` in `settings.py`
  (**enabled by default since 2026-08-09**). Experimental, not validated.

#### Historical: the lookahead gain-scheduling family (removed 2026-08-13)

An entire family of mechanisms — lookahead corner anticipation, demand
normalisation, U-turn detection, straight-line adjustments, precomputed
corner segmentation (`CornerMap`), curvature forcing, and the low-speed
steering-rate boost — scanned forward along the path every tick and
reweighted `Q`/`R`/`R_rate` in anticipation of a corner not yet reached.
All of it was deleted in the 2026-08-13 corner-factor rewrite.

It's moved to its own doc, **[`removed_mechanisms.md`](removed_mechanisms.md)**,
because the elimination reasoning (see that doc's "structural limit"
section) is the direct motivation for the nonlinear MPC below, and because
several of the ideas were tried, measured, and rejected for specific,
non-obvious reasons worth not re-discovering by accident.

**Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`,
added 2026-08-12, current)** — a related but distinct idea, unaffected by
the corner-factor rewrite above: instead of reweighting Q/R given an
existing tracking error, precompute a heading REFERENCE (`psi_target`, a
new `raceline.csv` column) that already leads the geometric path tangent by
however much yaw the car can achieve at its planned speed between here and
the next station. `_error_state` measures `e_psi` against this instead of
the geometric tangent (only `e_psi` — `e_y` is unaffected). Structurally
different from curvature-forcing above: it changes what's true at `k=0` (a
real, current error) rather than telling the QP about a future obligation
it's free to satisfy however is cheapest — synthetic testing found this
avoids curvature-forcing's wrong-direction transient entirely. See
`planning_control_sync.md`'s "Precomputed shaped heading-lead profile"
section for the full design, the fixed-lookahead version that was tried and
rejected first (immediate full-lock steering), and an important caveat
about this track's lack of true straights. NOT YET LIVE-TESTED (multiple
live attempts were made — see `tuning.md` §4.5c for the current, more
nuanced status than a flat "not yet tested").

**Delay compensation (`mpc_core.py`'s `predict_ahead()` / `_update_n_delay()`,
live controller only)** — the live car's perception→planning→control→actuation
latency is unknown and time-varying, unlike the offline simulator's fixed
`DELAY_STEPS`.

- **Mechanism:** each solve is told how old the pose it's using actually is
  (`pose_age_s`, from the pose message's own timestamp), converts that into
  an integer step count, and rolls the error state forward through that
  many recently-issued commands before solving — so the QP plans against
  the state the car will actually be in, not a stale measurement.
- **Why filtered first:** the raw step count is low-pass filtered and
  hysteresis-gated before use. Ordinary control-loop jitter would otherwise
  flip the raw `round(pose_age_s / dt)` between adjacent integers tick to
  tick, and each flip discontinuously changes how far the state gets rolled
  forward — injecting a step disturbance into the QP at the control rate
  purely from measurement noise, independent of any real latency change.

**Tracking-error speed gate (`control_utils.tracking_error_speed_gate()`,
both live nodes)** — `curvature_speed()`'s target only looks at path shape,
with no way to know whether the car is actually near that path right now.

- **Mechanism:** scales the speed target down (linear ramp, floored so the
  car always retains enough speed to steer) once lateral or heading
  tracking error grows large.
- **Rise-rate limiter:** paired with a cap on how fast the resulting speed
  target may *rise* — braking is never delayed, only the "speed up"
  direction is capped — so the target doesn't jump around tick to tick.

### Where this is duplicated, and why

`mpc_core.py`'s `MPCController` re-implements `_discrete_model`
(mirrors `model/bicycle_model.py`), `_adaptive_R_scaling`/`_adaptive_R_rate`
(mirrors `controller/model_utils.py`), and `_build_qp` (mirrors `controller/optimiser.py`'s
`init_parameterized_mpc`, including the same `±3.5 m` soft boundary,
`W_slack=10000`, and step-0/subsequent rate-cost split) as self-contained
local copies, rather than importing the shared modules. This is deliberate:
`mpc_core.py` runs inside a ROS 2 node on the real/FSDS vehicle and
must have zero simulator dependencies. **Any change to the cost/constraint
structure in one location must be mirrored in the other**, or weights tuned
by `tuner/offline_tuner.py` will not transfer faithfully to the live controller.

Both QPs enforce a hard per-step slew-rate limit (`du_max`) on top of the soft
`R_rate` cost. This used to be a live-only constraint, which meant the tuner
was optimising against a plant that could change steering arbitrarily fast
while the real car was clamped — a silent parity break independent of any
weight choice.

`controller/optimiser.py` now takes a `du_max` too (baked into
the cached QP alongside `u_min`/`u_max`, and participating in the same
cache-staleness check), and `sim/rollout_core.py` derives it from
`VehicleParams.max_steer_rate * DT` so both sides agree. See
[`planning_control_sync.md`](planning_control_sync.md)'s "Slew-rate limit"
section for the measurement behind the current 180 deg/s value and why the
previous 80 deg/s was the direct cause of live steering chatter.

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
9 **multiplicative scale factors** — one per tunable diagonal entry
(`TUNABLE_Q_IDX = [0,1,2,3,4]`, `TUNABLE_R_IDX = [0,1]`,
`TUNABLE_R_RATE_IDX = [0,1]`):

```
Q[i,i]      = vec[j] · Q_template[i,i]
R[i,i]      = vec[j] · R_template[i,i]
R_rate[i,i] = vec[j] · R_rate_template[i,i]
```

Each factor is bounded to `[0.1, 10.0]` — one decade of adjustment in either
direction from the template. Searching in multiplicative (rather than
absolute) space keeps the problem dimensionally consistent regardless of
the template's starting magnitude, and the `0.1` floor (rather than `1.0`)
specifically allows the tuner to discover that a weight should be *reduced*
below its starting point, not only increased.

The starting point `x0 = sqrt(lower · upper) = 1.0` for every parameter is
the geometric (log-scale) midpoint of `[0.1, 10.0]` — i.e. "start the search
exactly at the current template weights, unscaled," which is the natural
neutral point for a multiplicative search space (the arithmetic mean would
be biased toward the larger bound). This fixed midpoint is CMA-ES's default
starting point; if `USE_OPTUNA_PRESEARCH` is enabled (see below), `x0` is
replaced by the Optuna pre-pass's best result instead.

### Optional Optuna TPE pre-search

`USE_OPTUNA_PRESEARCH` in `settings.py` (default `True` as of 2026-08-05) runs a short
Optuna TPE (Tree-structured Parzen Estimator) search *before* CMA-ES starts,
using `OPTUNA_PRE_PASS_EVALS` true rollouts (default 10% of `MAX_EVALS`) out
of a separate mini-budget — this phase's cost is in addition to, not carved
out of, the main `MAX_EVALS` budget. TPE is a cheaper, less precise
global search method than CMA-ES; the idea is to spend a small budget
finding a promising general region of the 9-dimensional search space, then
start CMA-ES there instead of at the fixed geometric midpoint, so more of
CMA-ES's own budget goes toward local refinement instead of coarse search.

The pre-pass reuses the exact same objective (`parallel_evaluate_candidate`)
and worker pool as the CMA-ES phase — no rollout logic is duplicated —
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
[Dependencies](developer_guide.md#dependencies)) — only needed if this flag
is enabled.

### CMA-ES: what it's doing and why

CMA-ES (Covariance Matrix Adaptation Evolution Strategy) is a
derivative-free black-box optimiser well suited to this problem because the
objective (drive N corners well) is noisy, non-convex, and has no usable
gradient — you can't analytically differentiate "how smooth did the
steering feel" with respect to a cost weight. CMA-ES instead maintains a
multivariate Gaussian distribution over candidate solutions, samples a
population from it each generation, evaluates them, and adapts the
distribution's mean and covariance toward better-scoring regions —
learning, over generations, not just *where* good solutions are but which
*directions* in parameter space matter and which don't.

This project specifically uses `cma.fmin_lq_surr2`, which layers two
additional techniques on top of plain CMA-ES:

**BIPOP (bi-population) restarts.** Rather than one long single run, the
optimiser interleaves "large" restarts (population size doubles each time
via `incpopsize=2` — broader exploration, better at escaping local minima)
with "small" restarts (reduced population — faster local refinement around
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
of roughly `1.06` in log-space — large enough to explore meaningfully across
the full decade of allowed adjustment, without being so large that early
generations are mostly wasted on wildly implausible weight combinations.

### Parallel + serial evaluation

Every CMA-ES candidate is evaluated across all tasks in
`EVAL_TASKS` — the cross-product of `VALIDATION_SUITE` (the corner shapes
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
one badly — every task in the suite has to be reasonably good, not just the
average.

`TAIL_QUANTILE` (in `settings.py`, default `0.8`) replaced a hard `max()` on
2026-08-06. With the flat `DNF_PENALTY` of +3.0 (+6.0 off-track), `max()` let
**one** unlucky task out of ten shift the objective by ~0.9 and swamp all
twelve continuous quality metrics — measured, a plausible hand-picked gain set
ranked 3rd-worst of six (below two deliberately pathological sets) purely
because a single one of its ten tasks DNF'd. That is a discontinuous,
high-variance signal for CMA-ES and a likely contributor to the ~10× spread in
tuned gains across historical runs. A high quantile keeps the intent — punish
weights that fail badly *somewhere* — while requiring more than one bad task
before it dominates. Set `TAIL_QUANTILE = 1.0` to recover the old behaviour
exactly.

### DNF conditions (offline tuner — tighter than the live simulator)

A rollout inside the tuner is marked "did not finish" if any of:

- `|e_y| ≥ 3.50 m` (left the track — matches `OFFTRACK_LIMIT`)
- 5 consecutive MPC solver failures (matches `MAX_FAILS`)
- **Rolling stall check**: less than 3.0 m of forward progress in any
  rolling 60-step (3 s) window — catches a car that hasn't technically left
  the track or failed to solve, but also isn't actually driving anywhere
  (e.g. stuck oscillating in place).

On a DNF, `DNF_PENALTY` is added to the score, plus `DNF_OFFTRACK_PENALTY`
specifically if the DNF was caused by leaving the track (see
[Configuring the Project](#configuring-the-project-settingspy) for both
values).

### Post-optimisation: picking the final answer

After the search budget is exhausted (or you `Ctrl+C`), two candidates are
freshly evaluated **serially** (outside the noisy parallel pool, for a
clean comparison):

- **`xbest`** — the single best individual candidate observed across the
  entire search.
- **`xfavorite`** — the mean of CMA-ES's final search distribution, which
  tends to be more robust/averaged than any one lucky sample.

Whichever scores lower in this final clean evaluation is printed as the
result and appended to `tuning_history.txt`.

---

## The Composite Score

Both the offline tuner and the simulator's **Show Metrics**/**Benchmark All
Paths** buttons score a rollout through the exact same code path
(`scoring.RolloutMetrics`), which is what guarantees a path scored live in
the GUI and the same path scored offline produce matching numbers — there
is exactly one implementation of the scoring maths, not two independently
maintained copies.

### The 13 metrics

Accumulated once per simulation step via `RolloutMetrics.add_step()`, then
normalised (mostly to RMS values) at the end via `.finalize()`:

| # | Metric | What it measures |
|---|---|---|
| 0 | `rmse` | Combined tracking error: `1.2·e_y² + 0.4·e_psi²`, root-mean-squared over the run. The primary quality signal. |
| 1 | `yaw_rms` | RMS of the true yaw rate — penalises a car whose heading oscillates/wobbles. |
| 2 | `smooth_rms` | RMS of step-to-step control change (`Δu`) — penalises jerky command sequences. A failed solver step adds a flat +5.0 penalty here. |
| 3 | `steer_rms` | RMS steering command magnitude — overall steering effort. |
| 4 | `accel_rms` | RMS acceleration/brake command magnitude — overall longitudinal effort. |
| 5 | `max_steering` | The single largest steering command issued during the run. |
| 6 | `steering_sat_ratio` | Fraction of steps where steering was within 95% of `max_steer` — how often the controller is pinned at its limit. |
| 7 | `jerk_rms` | RMS of the *second* difference of control (`Δ²u`) — smoothness of the smoothness, catches abrupt changes in how fast commands are changing. |
| 8 | `max_yaw_rate` | The single fastest yaw rate reached — cornering aggressiveness ceiling. |
| 9 | `steering_reversal_rms` | Magnitude-weighted RMS of steering sign-flip swings (beyond a 0.02 rad noise gate): `sqrt(Σ swing² / n steps)`, where `swing = |u_steer| + |u_steer_prev|` at the moment of the flip. A tiny back-and-forth trim wiggle contributes almost nothing while a large aggressive swing dominates (squared), which is what distinguishes controller hunting/dithering from a twisty path (S-bends, slaloms) legitimately demanding more frequent-but-small direction changes — a flat per-flip count couldn't tell those apart. The raw reversal count and its per-step rate are still reported separately as informational-only fields (`steering_reversals`, `steering_reversal_rate` in the returned dict) alongside it. |
| 10 | `peak_lateral_error` | The single worst `|e_y|` reached at any point — a safety-margin measure independent of the average. |
| 11 | `speed_rmse` | RMS of `v_actual - v_target` — how well the car tracks the planner's requested speed. |
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

**Why three tiers instead of one sum (changed 2026-08-06).** A weighted sum is
linear scalarisation, and can only reach solutions on the *convex hull* of the
trade-off surface. Where that surface is non-convex — normal for vehicle
dynamics — whole regions of good behaviour are unreachable by **any** weight
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
  that cannot supply `reached_end` — as of 2026-08-11 that no longer includes
  the live car when it's running against a precomputed speed profile (see
  `LapProgressTracker` in `planning_control_sync.md`'s "Live/offline score
  parity" section); a run against the live planner topic instead still has no
  known path end and falls back to this threshold.
- `COMPLETION_BONUS_WEIGHT` is now unused by the score — completion is a
  precondition, not a reward. The constant is retained for the live copy's
  header compatibility.

`METRIC_SCALES` (added 2026-08-06) divides each metric by a reference magnitude
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
`[-0.5, -0.3]` — negative because the completion/time bonuses usually
outweigh the (small, well-tuned) metric costs. See
[tuning.md](tuning.md#6-scoring-metric_scales-and-score_weights) for how to
tune `SCORE_WEIGHTS`/`METRIC_SCALES`.

The inaccurate-solver penalty (up to +50% at 5 or more
`OPTIMAL_INACCURATE` occurrences in one rollout) uses
`score + abs(score)·factor` rather than a flat addition specifically so it
scales with, and preserves the sign of, an already-good (negative) score —
a run that finished well but had a few marginally-converged solves is
penalised proportionally, not knocked into DNF-penalty territory outright.

---
## Module Reference

Detailed explanations of the core algorithms live in the sections above —
[How the MPC Works](#how-the-mpc-works) and
[How the Offline Tuner Works](#how-the-offline-tuner-works). This section is
a short per-file index: what each module is for, and where its logic is
documented in depth (either above, or in the file's own docstrings/comments,
which are kept in sync with this README).

Note: this covers the simulator/tuner files only. The shared planning code
in `planning/` is copied from the `fsae_planning` repo and documented there,
not here.

| File | Purpose |
|---|---|
| `gui/simulation.py` | Interactive matplotlib GUI — draw/load a path, run one closed-loop rollout, scrub through history, view metrics. Thin wrapper around `rollout_core.run_core_rollout(want_history=True)`. |
| `sim/rollout_core.py` | The single shared closed-loop rollout loop used by both `gui/simulation.py` and `tuner/offline_tuner.py`. Not GUI-safe to import from `gui/simulation.py`'s multiprocessing workers, so it's split out into its own dependency-light module. |
| `sim/scoring.py` | The single implementation of the 13-metric accumulation and composite score. See [The Composite Score](#the-composite-score). |
| `model/bicycle_model.py` | Builds the MPC's linear 8-state prediction model. See [How the MPC Works](#how-the-mpc-works). |
| `controller/model_utils.py` | Runtime curvature/speed-based rescaling of `R`/`R_rate`. See [Adaptive gain scheduling](#adaptive-gain-scheduling-controllermodel_utilspy). |
| `controller/optimiser.py` | The parameterised CVXPY/OSQP QP formulation and solve. See [The cost function and QP](#the-cost-function-and-qp-controlleroptimiserpy). |
| `model/vehicle_physics.py` | The 24-state nonlinear "truth" plant (Pacejka tyres, suspension, aero) that the MPC never observes directly — only through tracking error. See [Configuring the Vehicle](#configuring-the-vehicle-modelvehicle_physicspy). |
| `tuner/offline_tuner.py` | Headless CMA-ES weight search. See [How the Offline Tuner Works](#how-the-offline-tuner-works). Also exports the synthetic path library (`SYNTHETIC_PATHS`, `PATH_NAMES`) and the speed-keyed model cache (`get_cached_model`) used by both the tuner and the simulator. |
| `sim/speed_profile.py` | Curvature-based per-point target speed (`compute_speed_profile`), with a moving-average smoothing pass (`smooth_profile`). Uses the friction-circle approximation `v = sqrt(a_lat_max / κ)` over a forward look-ahead window. |
| `sim/sim_track.py` | Simulator-side mirrors of the real perception/planner nodes: `place_cones()` (static track layout), `SimPerception` (FOV filter), `SimPlanner` (cone accumulation → centreline + speed profile). |
| `sim/track_io.py` | Loads a `fsae_planning` `cone_recorder` JSON cone map into the same `(path_X, path_Y, path_Psi, path_v, blue, yellow)` tuple shape as a synthetic path — see [Recording, exporting and driving a track](developer_guide.md#recording-exporting-and-driving-a-track). |
| `tuner/performance_stats.py` | Scores a completed simulator run for the **Show Metrics** button by replaying its stored history through the exact same `scoring.RolloutMetrics` accumulator the tuner uses. Also exposes `benchmark_weights()` for **Benchmark All Paths**. |
| `gui/manual_drive.py` | Standalone WASD/mouse drive mode against the 24-state nonlinear plant — no MPC, no scoring, purely open-loop human control for building intuition or sanity-checking a track. See [Manual Drive Mode](developer_guide.md#manual-drive-mode). |
| `settings.py` | All project-level tuning/scoring/DNF configuration. See [Configuring the Project](#configuring-the-project-settingspy). |
| `mpc_controller_standalone.py` / `mpc_core.py` / `control_utils.py` (staged under `fsds_simulator/control/fsae_control/fsae_control/`) | The live ROS 2 MPC controller for FSDS. See [Simulator Integration](developer_guide.md#simulator-integration). |
| `fsds_simulator/` (whole tree) | Full staging mirror of upstream's ROS 2 workspace — every package, not just control — so a clone of this repo plus FSDS can build and run the complete stack (`stanley`, `mpc`, or `mpc_standalone`) with no separate `fsae_planning` checkout. See [docs/planning_control_sync.md](planning_control_sync.md) and [fsds_simulator/README.md](../fsds_simulator/README.md). |


## Second controller: nonlinear MPC (`use_nmpc`) — 2026-08-13

Everything under "How the MPC Works" above describes `mpc_core.MPCController`:
a linear time-varying MPC solved as one convex QP per tick. As of 2026-08-13 the
live workspace carries a **second, separately selectable** controller,
`nmpc_core.NMPCController`, chosen by the node parameter `use_nmpc` (default
false). This repo has its own offline port, `controller/nmpc_optimiser.py`,
selected by `settings.USE_NMPC`; this section is a pointer to the live design,
not a mirror of the offline code — see `planning_control_sync.md`'s "Nonlinear
MPC (`use_nmpc`)" section for the offline port's specifics.

**The structural difference, in one line.** The LTV-QP predicts how the car's
current error (`e_y`, `e_psi`) drifts under its own dynamics, against a
reference direction it treats as fixed for the whole horizon. The NMPC
predicts that same error's evolution **relative to a path whose bend is
itself part of the prediction** — the model knows the reference direction
changes with `s`, not just the car's state. Concretely: the LTV-QP's
prediction is written in error coordinates with the reference frame's
rotation dropped, so `e_psi_dot = r` instead of
`e_psi_dot = r - kappa(s)*s_dot`. With the car on line and a corner ahead, its
35-step rollout predicts staying on line forever (measured: exactly 0.000 deg
commanded at 8 dead-on-line states), which is why the "structural limit"
callout elsewhere in this doc exists and why the adaptive lookahead layer had
to be invented. The NMPC works in curvilinear (Frenet) coordinates where arc
length `s` is a state and `kappa(s)` is looked up from the path, so the road
bending ahead is part of the dynamics rather than something bolted onto the
cost — as `s` advances along the predicted horizon, `kappa(s)` changes with
it, so a bend 10 steps out is already shaping the plan today, not just once
the car arrives there.

**Structure**: states `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs
`[delta_cmd, a_cmd]`, linear-tyre bicycle dynamics with the same constants and
the same low-speed kinematic blend as the LTV-QP, plus a `tanh` saturation of
the predicted lateral force at FSDS's measured `a_lat` ceiling. Solved by
Gauss-Newton SQP: nonlinear rollout from the measured state (so the
linearisation point is always dynamically feasible and the QP's dynamics defect
is zero), finite-difference Jacobians vectorised across horizon stages,
condensed into a dense QP in the input deviations only, solved by OSQP with a
box trust region, one iteration per tick warm-started from the previous tick
(real-time iteration). Horizon 20 steps (1.0 s); measured solve time mean
8.9 ms, p95 11.6 ms.

**Consequences for the rest of the architecture**: when `use_nmpc=true` the
entire adaptive gain schedule, the precomputed corner map and the shaped
heading-lead profile are all inactive (each was a workaround for the missing
curvature term), and the telemetry CSV's `m_*` columns are empty while eight
`nmpc_*` columns carry solver/prediction diagnostics instead. The composite
score, the scoring pipeline, the path/speed-profile plumbing and the delay
compensation are unchanged.

#### Feature comparison: LTV-QP vs. NMPC, at a glance

Every feature below is verified against actual read-sites in the code, not
inferred from a docstring or field name — see
`planning_control_sync.md`'s "Which settings affect which controller" map
for the exhaustive, field-by-field version this table summarises.

| Feature | LTV-QP (`mpc_core.py`) | NMPC (`nmpc_core.py`) | Why |
|---|---|---|---|
| Adaptive gain scheduling (`_corner_factor`, anti-hunt, `adaptive_Q_scaling`, `adaptive_R_scaling`, `adaptive_R_rate`) | **Yes** | **No** (inert — none of these fields have any read site in `nmpc_core.py`) | Every one of these mechanisms exists to compensate for the LTV-QP's blind spot (it can't predict the path curving). NMPC's model has that built in structurally, so reweighting the cost on top would double-count an effect that's now already handled — see [`removed_mechanisms.md` §1](removed_mechanisms.md#1-the-structural-limit-the-argument-that-motivates-nmpc). |
| `steer_rate_anti_hunt` (steering-rate damping when centred/aligned/uncurving) | **Yes**, on by default | **Opt-in**, off by default (`nmpc_steer_rate_anti_hunt_enabled`) | The one exception to the row above — it only ever makes steering *more* damped in a specific narrow case, the opposite direction from anticipation, so it doesn't fight NMPC's structural fix the way the rest of the gain schedule would. Reuses the LTV-QP's own function verbatim (imported, not reimplemented). |
| Precomputed corner map (`use_precomputed_corner_map`) | Removed from both (2026-08-13) | Removed from both (2026-08-13) | Served the deleted lookahead gain-scheduling family — gone from both controllers, not an LMPC/NMPC difference. See [`removed_mechanisms.md` §7](removed_mechanisms.md#7-precomputed-corner-segmentation-cornermap). |
| Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`) | **Yes** | **Accepted but ignored** (`set_heading_profile()` exists so the node needs no branch, logs a one-time warning) | Same reasoning as gain scheduling — the shaped lead is a workaround for the same missing curvature term NMPC closes structurally. Applying both would double-count the anticipation. |
| Delay/latency compensation (rolling `x0` forward through recently-issued commands) | **Yes** (`predict_ahead()`, linear rollforward) | **Yes** (rolls `x0` forward through the nonlinear model instead) | Both need this — it's about *sensor/actuation lag*, a problem that exists regardless of which prediction model is used. Different implementation, same four gating fields (`delay_compensation_enabled`, `max_delay_compensation_steps`, `pose_age_lp_alpha`, `n_delay_hysteresis`) — shared `MPCParams` fields, read by both. One exception: `predict_epsi_clip` is LTV-QP only (a small-angle bound specific to the *linear* rollforward; NMPC's nonlinear rollforward has no such bound to set). |
| Tracking-error speed gate (slow down when `e_y`/`e_psi` are large) | **Yes** | **Yes** | This lives in `control_utils.py`, called by the **node** (`mpc_controller.py`/`mpc_controller_standalone.py`) *before* either controller's `.compute()` is invoked — neither `MPCController` nor `NMPCController` is even aware it exists. Controller-agnostic by construction. |
| Curvature-based speed profile (`curvature_speed()`) | **Yes** | **Yes** | Same reason as the row above — computed by the node, handed to whichever controller is selected as `desired_speed`. |
| Cone-proximity emergency braking, GO-gating, stale-path fail-safes | **Yes** | **Yes** | All node-level (`mpc_controller_standalone.py`'s `_control_loop` phases), not part of either controller class. `NMPCController` exposes the same `compute()`/`reset()`/`set_static_path()` surface as `MPCController` specifically so the node doesn't need a branch. |
| FSDS lateral-acceleration ceiling | **Yes**, as a plain speed-profile input (`curvature_speed()`'s friction-circle cap) | **Yes**, AND inside the prediction itself (`tanh` saturation on predicted tyre force) | NMPC's version is strictly more — the ceiling shapes what the *solver itself* believes is achievable, not just the requested speed. Without it, NMPC's linear-tyre model believes it can hold any corner at any speed and the car spins (measured). |
| Horizon length | 35 steps (1.75 s) | 20 steps (1.0 s) | Independent tuning choices, not a structural requirement — NMPC's shorter horizon reflects its per-tick solve cost (Gauss-Newton SQP is more expensive per step than one convex QP). |
| Solve method | One convex QP per tick (OSQP) | Real-time-iteration SQP: one Gauss-Newton step per tick, warm-started, condensed dense QP (OSQP) | See [The solver](#the-solver) above for what a QP is; NMPC needs the extra linearize-and-resolve step because its own model is nonlinear (curvature is now a function of a state, not a fixed matrix entry). |

**Three further, NMPC-only additions (2026-08-13)**, assessed against
Alexander Liniger's Model Predictive Contouring Control (MPCC) but narrower
than it — full MPCC's progress-maximisation apparatus was considered and
rejected as too close to a failure mode already eliminated here (see
`planning_control_sync.md`'s writeup for why). One is on by default, two are
off:

- `nmpc_spline_reference_enabled` (default **true**) — `PathReference`'s
  `kappa(s)`/`psi_ref(s)` come from an analytic cubic-spline fit to the
  waypoints instead of moving-average-smoothed finite differences. A
  numerical-quality fix, not a new coupling to the solver.
- `nmpc_horizon_speed_profile_enabled` (default **false**, experimental) —
  samples a precomputed speed profile at each horizon stage's own predicted
  arc length, the same state-keyed pattern `kappa(s)` already uses, instead
  of holding one frozen speed target across the horizon.
- `nmpc_friction_circle_enabled` (default **false**, experimental) — a hard
  per-axle tyre-force bound in the QP, additional to (not replacing) the
  existing soft `alat_ceiling` saturation.

All three are implemented identically in `nmpc_core.py` and the offline
`controller/nmpc_optimiser.py`; none touch `mpc_core.py` (the LTV-QP).

Full detail: `planning_control_sync.md`'s "Nonlinear MPC (`use_nmpc`)" section
(what it is, what it reuses, what is inactive, offline A/B numbers, the offline
port, a matched same-day LIVE A/B — steering saturation 6.45% → 0.58%,
lap 54.72s → 52.35s — and the "Which settings affect which controller" map
and the three MPCC-inspired additions above), `tuning.md` §4.5d (tuning
surface), and `late_turn_in_investigation.md` Part 16 (research survey,
formulation choice, validation, the four bugs found in testing).
