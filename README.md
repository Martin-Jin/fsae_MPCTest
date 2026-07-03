# FSAE MPC Path Tracking Simulator

A high-fidelity 2D closed-loop simulator and offline weight tuner for a Formula Student
autonomous vehicle. The system pairs a nonlinear 22-state vehicle plant with a linear
time-varying MPC controller, and provides CMA-ES based automated weight optimisation.

This includes an implementation of a control node that is combined with the fsae_planning repo (just replaces the cooresponding controller node file in the repo) 
to run the MPC controller in the fsds simulator, which simulates the car using unreal engine 4.

The 2D simulator simulates perception and planning for an autonomous vehicle, rather than just using a predefined path. 
This is implemented by placing cones to define the borders of a provided path in `sim_track` with the help of some cone functions (in the `planning` folder) from the fsae_planning repo. 

fsds simulator repo: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator (current implementation uses commit 59f03fa, and the V2.20 release)
fsae planning repo: https://github.com/UOA-FSAE/fsae_planning (current implementation uses commit 28dcd4d)

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Module Reference](#module-reference)
3. [Simulator Deep-Dive](#simulator-deep-dive)
4. [Offline Tuner Deep-Dive](#offline-tuner-deep-dive)
5. [Running the Simulator](#running-the-simulator)
6. [Running the Offline Tuner](#running-the-offline-tuner)
7. [Tuning Guide](#tuning-guide)
8. [ROS 2 Integration (fsds)](#ros-2-integration-fsds)
9. [Dependencies](#dependencies)

---

## Architecture Overview

### Full System Flow

```
USER INPUT (draw path / load synthetic path)
        │
        ▼
  path_X, path_Y, path_Psi
  speed_profile.compute_speed_profile()
        │
        ▼
┌─────────────────────────────────────────────────────────┐
│                     SIMULATION LOOP                     │
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
│  │ Extraction   │                  │   (OSQP/        │  │
│  └──────────────┘                  │   Clarabel)     │  │
│         ▲                          └────────┬────────┘  │
│         │                                   │ u=[δ, a]  │
│  ┌──────────────┐                           ▼           │
│  │ 22-State     │◄──────────────────────────┘           │
│  │ Nonlinear    │  step_nonlinear_plant(state, u, dt)   │
│  │ Plant        │                                       │
│  └──────────────┘                                       │
└─────────────────────────────────────────────────────────┘
        │
        ▼
  history dict → visualisation + performance_stats
```

### Controller / Plant Architecture

```
                    ┌─────────────────────────────────────┐
                    │         MPC (control layer)         │
                    │                                     │
  path waypoints ──►│  get_8state_discrete_model(vx, dt)  │
  car state      ──►│  → Ad, Bd (ZOH linearisation)       │
                    │                                     │
                    │  optimiser.solve_mpc()              │
                    │  → OSQP QP → u* = [δ_cmd, a_cmd]    │
                    └──────────────┬──────────────────────┘
                                   │
                    ┌──────────────▼───────────────────────┐
                    │     Plant (truth layer)              │
                    │                                      │
                    │  vehicle_physics.step_nonlinear_plant│
                    │  22 states: X, Y, ψ, vx, vy, r,      │
                    │  δ_act, a_act, ω×4, z×4, dz×4, Fy×4  │
                    │  4 sub-steps per control tick        │ 
                    └──────────────────────────────────────┘
```

### ROS 2 vs FSDS Simulator Mapping
How each component matches the ros2 equilvalent in the planning node from fsae_planning used for the fsds simulator.

```
ROS 2 Node              │  Simulator Equivalent
────────────────────────┼───────────────────────────────────
perception_node.py      │  sim_track.SimPerception
planner_node.py         │  sim_track.SimPlanner
cone_map.py             │  cone_map.ConeMap  (shared)
boundary.py             │  boundary.py       (shared)
path_utils.py           │  path_utils.py     (shared)
speed_profile.py        │  speed_profile.py  (shared)
control_utils.py        │  simulation.py (MPC solve inline)
vehicle_physics.py      │  vehicle_physics.py (shared)
model.py                │  model.py           (shared)
optimiser.py            │  optimiser.py       (shared)
```

---

## Module Reference
Note this only for the main simulator files, the shared planning code in the `planning` folder is not included. Refer to the fsae_planning repo for more details.

### `simulation.py`
**Purpose:** Interactive GUI simulator. Draw a path, configure initial errors, run
closed-loop MPC, scrub through history, view metrics.

**Inputs:** User mouse draw events or synthetic path loaded from `offline_tuner`.

**Key outputs:** `sim_history` dict with `X, Y, psi, v, v_target, u_steer, u_accel,
e_y, e_psi, pred_X, pred_Y, failed, completion_frac`.

**Dependencies:** `model`, `optimiser`, `vehicle_physics`, `performance_stats`,
`speed_profile`, `offline_tuner` (for synthetic paths and adaptive helpers).

---

### `offline_tuner.py`
**Purpose:** Headless CMA-ES weight optimiser. Runs many closed-loop rollouts across
synthetic paths and initial conditions to minimise a composite score.

**Inputs:** Template `Q`, `R`, `R_rate` matrices; bound ratios per weight index.

**Key outputs:** Printed `Q_diag`, `R_diag`, `R_rate_diag` to paste into `simulation.py`
and `control_utils.py`.

**Dependencies:** `vehicle_physics`, `model`, `optimiser`, `speed_profile`, `cma`.

---

### `model.py`
**Purpose:** Builds the 8-state discrete-time linear bicycle model used by the MPC
for internal prediction.

**Function:** `get_8state_discrete_model(v_x, dt) → (Ad, Bd)`

**States:** `[e_y, ė_y, e_ψ, ė_ψ, e_v, e_a, δ_act, a_act]`

**Method:** Exact ZOH via `scipy.linalg.expm` on the augmented `[A_c, B_c; 0, 0]`
block. Forward-Euler is not used because at low speed `|λ(A_c)| * dt` exceeds the
stability margin and the QP goes infeasible.

**Inputs:** `v_x` (m/s, clamped ≥ 0.5), `dt` (s).

**Returns:** `Ad (8×8)`, `Bd (8×2)` discrete matrices.

**Dependencies:** `numpy`, `scipy.linalg`.

---

### `optimiser.py`
**Purpose:** CVXPY parameterised QP. Built once, solved each tick by injecting new
parameter values—avoids CVXPY graph rebuild overhead at 20 Hz.

**Function:** `solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max, ...) → u_opt or None`

**Returns:** `u_opt (2,)` = `[δ_cmd, a_cmd]` for the first horizon step, or `None`
on solver failure.

**Solver chain:** OSQP primary → Clarabel fallback. `OPTIMAL_INACCURATE` is accepted
with a warning (solution still used at 20 Hz). Returns `None` for any other failure,
caller should hold previous command.

**Key parameters:**
- `N` — horizon length (default 25 steps = 1.25 s at 50 ms)
- `eps_abs/rel` — OSQP tolerances (`1e-5` live, `1e-4` offline tuner for speed)
- `warm_start` — reuse prior solution (True in live sim, False on first tuner step)
- `R_rate` — slew-rate penalty on `Δu`; injected as `sqrtR_rate` parameter
- Slack variable on lateral error with penalty `10000` to keep QP feasible

**Dependencies:** `cvxpy`, `numpy`.

---

### `vehicle_physics.py`
**Purpose:** 22-state high-fidelity nonlinear plant. Truth model the MPC never sees
directly—it only receives tracking errors from the plant state each tick.

**Function:** `step_nonlinear_plant(state, u_cmd, dt, params, road_mu, tv_gain) → state`

**State vector (22 elements):**
```
[0]  X          global position (m)
[1]  Y          global position (m)
[2]  ψ          yaw angle (rad)
[3]  vx         longitudinal velocity (m/s)
[4]  vy         lateral velocity (m/s)
[5]  r          yaw rate (rad/s)
[6]  δ_act      actual steering after lag (rad)
[7]  a_act      actual acceleration after lag (m/s²)
[8-9]  ω_RL, ω_RR   rear wheel spin (rad/s)
[10-13] z_FL..z_RR  suspension deflection from equilibrium (m)
[14-17] dz_FL..dz_RR suspension velocity (m/s)
[18-21] Fy_FL..Fy_RR relaxed lateral tyre forces (N)
[22-23] ω_FL, ω_FR  front wheel spin (rad/s)
```

**Physics included:**
- Full Pacejka MF94 lateral and longitudinal tyre model with load sensitivity,
  camber gain, and tyre relaxation length
- Per-wheel spring/damper/ARB suspension with dynamic normal loads
- Split front/rear aerodynamic downforce with pitch sensitivity
- Friction ellipse coupling (combined slip)
- Torque vectoring (disabled by default, `tv_gain=0`)
- Road surface µ scaling via `road_mu`
- 4 sub-steps per control tick (Euler inside, semi-implicit for suspension)

**`VehicleParams` key values:**
```
m=255 kg, lf=0.9 m, lr=0.6 m, Iz=110 kg·m²
Cf=11500 N/rad, Cr=12500 N/rad
tau_delta= 0.08 s, tau_a=0.05 s
mu=1.6 (peak, racing slick)
max_steer=35°, max_accel=±5 m/s²
```

**Dependencies:** `numpy`.

---

### `speed_profile.py`
**Purpose:** Computes a physically achievable per-point target speed along any path
using curvature, forward acceleration, and braking limits.

**Functions:**
- `compute_path_curvature(path_X, path_Y) → kappa[]` — finite-difference curvature
- `compute_speed_profile(...) → v_profile[]` — 3-pass: corner limit → forward pass → backward pass
- `smooth_profile(v_profile, window=9) → v_profile[]` — moving-average smoothing

**Key parameters:**
```
v_max=18.0     m/s   absolute top speed cap
mu=0.6               planning friction (~70% of peak; leaves MPC margin)
a_accel_max=5.0 m/s² matches MPC actuator bound
a_brake_max=-5.0 m/s²
v_min=2.5      m/s   floor speed
safety=1.0           multiplier on corner speed (reduce to 0.85–0.95 on tight paths)
scan_end=14.0  m     short-path cap reference length
```

**Dependencies:** `numpy`.

---

### `sim_track.py`  *(new, after upgrade)*
**Purpose:** Provides the simulator equivalents of `perception_node.py` and
`planner_node.py`. Shares all the real planning code; only the ROS 2 message
transport layer is replaced.

**`place_cones(path_X, path_Y) → (blue, yellow)`**
Places cones along both boundaries at `CONE_SPACING=3.0 m` intervals,
`TRACK_HALF_WIDTH=1.75 m` either side of the centreline (3.5 m total, FSG spec).
Uses the path tangent to compute left/right normals.

**`SimPerception(blue_all, yellow_all)`**
- `visible_cones(car_x, car_y, car_yaw) → (blue_vis, yellow_vis)`
  Filters to the car's FOV: `MIN_AHEAD=0.5 m`, `LOOK_AHEAD=25 m`, `LOOK_WIDE=10 m`.
  Directly mirrors `perception_node._publish_visible_cones()`.

**`SimPlanner(v_max, v_min, lookahead_dist)`**
- `update(blue_obs, yellow_obs, car_pos, car_yaw)` — ingest new cones, rebuild path + speed profile
- `get_target(car_pos, car_yaw) → (target_xy, v_desired)` — lookahead point and speed

**Dependencies:** `cone_map`, `boundary`, `path_utils`, `speed_profile`, `numpy`.

---

### `performance_stats.py`
**Purpose:** Scores a completed `sim_history` dict using the identical cost formula
as `offline_tuner.run_headless_rollout`. Imports `SCORE_WEIGHTS`, `DNF_PENALTY`,
etc. directly from `offline_tuner` — single source of truth.

**Function:** `report_performance_metrics(history, log_fn=print) → metrics dict`

**Returns dict keys:** `composite_score`, `lateral_rmse_m`, `heading_rmse_deg`,
`speed_rmse_mps`, `yaw_rms_radps`, `control_smooth_rms`, `steering_rms_deg`,
`accel_rms_mps2`, `jerk_rms`, `max_steering_deg`, `steering_sat_ratio`,
`steering_reversals`, `peak_lateral_error_m`, `completion_pct`, `failed`, `n_steps`.

**Dependencies:** `offline_tuner` (for weights), `numpy`.

---

### ROS 2 Nodes (not used by simulator directly)

| File | Node name | Role |
|---|---|---|
| `perception_node.py` | `perception` | Oracle FOV filter → `/FusionCones` |
| `planner_node.py` | `centreline_planner` | Cone map + path + speed → `/fsds/planned_path`, `/fsds/desired_speed` |
| `control_node.py` | `controller` | MPC → `/fsds/control_command` |
| `control_utils.py` | — | `MPCController` class (QP, error state, adaptive gains) |
| `integration_node.py` | `fsds_mock_pipeline` | Simple fallback mock node |

---

## Simulator Deep-Dive

### Startup and Path Input

The simulator opens a `matplotlib` figure with a 600×920 px layout. The user either
draws a path by clicking and dragging on the map axes, or clicks **Load Test Path**
to cycle through the 10 synthetic paths defined in `offline_tuner.build_synthetic_paths()`.

**Drawn paths:** On mouse release, raw points are deduplicated (min 0.5 m between
samples) and fitted with a clamped `CubicSpline` (first and last derivatives pinned
to the chord direction to prevent endpoint overshoot). The spline is resampled at
600+ points. Heading `path_Psi` is derived from `arctan2(dy/dt, dx/dt)`.
`speed_profile.compute_speed_profile()` runs immediately to produce `path_v_profile`.

**Synthetic paths:** Pre-computed at import time by `build_synthetic_paths()` which
stores `(path_X, path_Y, path_Psi, path_v)` per path. Geometry is FS-spec:
straights 5–10 m, corner radii 5–12 m.

After the upgrade, `sim_track.place_cones()` is called at this stage to populate
the static cone arrays that `SimPerception` will filter each step.

### Simulation Loop (`simulate_closed_loop`)

Runs for up to `max_steps=400` steps at `dt=0.05 s` (20 Hz). Each step:

**1. Record state** — append `X, Y, psi, v` to history before any computation.

**2. Perception update** — `SimPerception.visible_cones()` filters the static cone
map to the car's forward FOV. `SimPlanner.update()` merges observations into
`ConeMap`, runs `build_path_walls()` (falling back to `build_local_path()`), and
recomputes the speed profile via `speed_profile.compute_speed_profile()`.

**3. Reference extraction** — the car is projected onto the planner's centreline.
The nearest waypoint gives the reference heading `rpsi`. Lateral error `e_y` is
the signed perpendicular distance (positive = left of path). While the planner
warms up (no centreline yet), this falls back to the original drawn path.

**4. Error state assembly** — the 8-element MPC state vector:
```
x = [e_y, ė_y, e_ψ, ψ̇, e_v, 0, δ_act, a_act]
     ↑ lateral  ↑ heading  ↑ speed   ↑ actuator states
```

**5. Early exit checks:**
- `|e_y| > 8.0 m` → off-track, `failed = True`
- `consecutive_solver_failures ≥ 5` → `failed = True`
- `idx ≥ len(path) - 2` → reached end

**6. MPC solve** — `get_8state_discrete_model(vx, dt)` linearises the bicycle model
at the current speed. `curvature_estimate()` and the two adaptive helpers modify
`R` and `R_rate` before the QP:
- `adaptive_R_scaling(vx, R)` — increases steering cost with speed (saturating,
  ~2× at 10 m/s) to prevent aggressive high-speed inputs
- `adaptive_R_rate(kappa, R_rate)` — reduces steering jerk penalty in sharp corners
  to prevent understeering from excessive damping

`optimiser.solve_mpc()` runs OSQP, falls back to Clarabel, returns `u*=[δ,a]` or
`None`. On `None`, previous command is held and `consecutive_solver_failures` incremented.

**7. Horizon prediction** — linear model rollout of `N=25` steps starting from current
plant state, for visualisation only.

**8. Plant advance** — `step_nonlinear_plant(plant_state, u_opt, dt, vehicle_params)`
advances the 22-state truth model. The plant uses Pacejka tyre forces, suspension
dynamics, and aerodynamic downforce—none of which the MPC's linear model sees.
The model-plant mismatch is intentional: tracking error closes the loop every step.

### Completion and History

`history["reached_end"] = True` if the car reaches within 2 path indices of the end.
`completion_frac = 1.0` if reached, else `n_steps / max_steps`. On failure,
`history["failed"] = True` and `fail_reason` describes the cause.

### Scrub Viewer

After simulation, the time-scrub slider replays history frame by frame: trail,
MPC horizon prediction, car triangle marker, and telemetry panel all update
synchronously.

### Show Metrics

Calls `performance_stats.report_performance_metrics(sim_history)` which produces a
full console breakdown and updates the plot title with a summary. The composite
score uses the same `SCORE_WEIGHTS` vector as the offline tuner.

---

## Offline Tuner Deep-Dive

### Purpose

Automatically find `Q`, `R`, `R_rate` weight matrices that minimise a composite
performance score across multiple synthetic paths and initial conditions, without
running the GUI.

### CMA-ES Strategy

Uses `cma.fmin_lq_surr2`: BIPOP (bi-population) restart strategy with a quadratic
surrogate model. The surrogate predicts which candidates are worth truly evaluating,
reducing the number of actual simulator rollouts by ~3–10×.

**Parameter space:** 9 multipliers (5 Q, 2 R, 2 R_rate), each bounded `[0.1, 10.0]`
relative to the template values. The multiplier is applied to the template diagonal
entry, so a value of `1.0` means no change.

**Initial point:** midpoint of bounds `(0.1+10.0)/2 = 5.05` per parameter.

**`sigma0=0.75`, `CMA_stds = 0.23 × (upper - lower)`** — 23% of the parameter range
as initial step size per dimension.

### Objective Function

`evaluate_candidate(vec)` runs `run_headless_rollout()` for each task in
`EVAL_TASKS` (9 paths × 2 initial conditions = 18 tasks) and aggregates:
```
score = 0.7 × weighted_mean + 0.3 × worst_case
```
The worst-case term prevents the optimiser from finding weights that do well on
average but catastrophically fail one path type.

`parallel_evaluate_candidate(vec)` distributes these 18 tasks across a
`multiprocessing.Pool` so all CPU cores are used.

### Headless Rollout (`run_headless_rollout`)

Functionally identical to `simulate_closed_loop` but without GUI, matplotlib, or
history recording. Uses looser OSQP tolerances (`eps=1e-4`, `max_iter=5000`) and
a model cache keyed by `round(vx, 1)` to avoid rebuilding the ZOH matrices at
every step.

**DNF (did-not-finish) conditions:**
- `|e_y| > 2.5 m` (tighter than simulator's 8 m — penalises poor tracking harder)
- `consecutive_fails ≥ 5`
- Progress `< 3 m` after 60 steps (stuck detection)

**DNF penalty:** `3.0 × (1 - progress) + 1.0 × offtrack_excess²`
The graded form gives CMA-ES a gradient slope toward "get further around the track"
rather than a cliff that collapses the covariance update.

### Composite Score (`SCORE_WEIGHTS`)

```
Index  Metric               Weight  Notes
  0    rmse                  0.40   primary tracking signal
  1    yaw_rms               0.06
  2    smooth_rms (Δu)       0.07   control smoothness
  3    steer_rms             0.04
  4    accel_rms             0.03
  5    max_steering          0.03
  6    steering_sat_ratio    0.09   fraction of steps at saturation
  7    jerk_rms (Δ²u)        0.10
  8    max_yaw_rate          0.03
  9    steering_reversals    0.02
 10    peak_lateral_error    0.13
```

Bonuses (subtracted from score):
- `COMPLETION_BONUS_WEIGHT=0.30` × `completion_frac`
- `TIME_BONUS_WEIGHT=0.05` × `time_bonus`

Lower composite score is better. A typical finishing score is in the range `[-0.4, 1.0]`.

### Post-Optimisation

After all evaluations, both `xbest` (lowest observed) and `xfavorite` (distribution
mean, often more robust) are freshly evaluated. Whichever scores lower is selected
and printed as copy-paste weight arrays.

---

## Running the Simulator

### Requirements

```
pip install numpy scipy matplotlib cvxpy cma
# OSQP and Clarabel are installed as CVXPY optional solvers:
pip install cvxpy[osqp] cvxpy[clarabel]
```

### Launch

```bash
cd /path/to/project
python simulation.py
```

### Drawing a path

1. Click and drag on the map to draw a reference path.
2. The path is splined, heading computed, and speed profile generated automatically.
3. Optionally adjust **Initial Lat Error** (±4 m) and **Initial Yaw Error** (±30°)
   sliders to place the car off-centre.
4. Use **Flip Heading (180°)** to start the car facing the wrong way — tests
   recovery behaviour.

### Loading a synthetic path

Click **Load Test Path** to cycle through the 10 built-in paths:
`PATH_SUDDEN_TURN`, `PATH_S_BEND`, `PATH_SKIDPAD`, `PATH_SPIRAL`,
`PATH_MICRO_SLALOM`, `PATH_OFFSET_CHICANE`, `PATH_HAIRPIN`, `PATH_CHICANE`,
`PATH_FS_CORNER`, `PATH_MIXED`.
Each click advances to the next path. The camera frames automatically.

### Running a simulation

Click **Start Sim**. The loop runs synchronously (no animation during solve).
On completion:
- The plot title turns green with a summary.
- The time-scrub slider appears. Drag it to replay history.
- The telemetry panel shows speed, errors, and commands at each frame.

### Viewing metrics

Click **Show Metrics** after a simulation. The console prints a full breakdown.
The composite score, lateral RMSE, and heading RMSE appear in the plot title.

### Resetting

Click **Reset Environment** to clear everything and draw or load a new path.

---

## Running the Offline Tuner

### Launch

```bash
cd /path/to/project
python offline_tuner.py
```

This uses all available CPU cores minus one. A typical run takes 20–60 minutes
depending on core count and `MAX_EVALS`.

### Key constants to adjust

In `offline_tuner.py`:

```python
MAX_EVALS    = 1000   # total true rollouts budget
sigma0       = 0.75   # CMA-ES initial step size
max_restarts = 9      # BIPOP restart count
```

### Reading the output

Each generation prints:
```
[lq-CMA-ES] gen   5 | true_evals    90 | gen_best 0.2341 | overall_best 0.1892 | sigma 0.6123e+00
```

On completion:
```
Replace your simulation.py weights with:
Q_diag      = [45.3, 16.3, 45.5, 13.2, 11.0, 0.0, 0.0, 0.0]
R_diag      = [5.2, 2.8]
R_rate_diag = [0.64, 15.2]
```

### Applying optimised weights

Copy these three arrays into:
- `simulation.py` — the `Q_diag`, `R_diag`, `R_rate_diag` assignments near the top
- `control_utils.py` — the same three arrays inside `MPCController.__init__`

The two files must stay in sync. The tuner was designed against the same plant and
horizon used by both.

---

## Tuning Guide

### Weight matrices

`Q` penalises tracking error states. `R` penalises control effort. `R_rate` penalises
control rate-of-change (smoothness).

State order for `Q_diag`: `[e_y, ė_y, e_ψ, ė_ψ, e_v, e_a, δ_act, a_act]`

- `Q[0]` (`e_y`) and `Q[2]` (`e_ψ`) are the dominant tracking terms. Raising them
  tightens path following but increases control effort and saturation risk.
- `Q[4]` (`e_v`) controls how aggressively the car follows the speed profile.
  Setting it to 0 disables speed tracking (pure lateral control).
- `Q[5..7]` are zero — actuator states are not penalised directly; `R_rate` handles
  their smoothness implicitly.
- `R[0]` (steering effort) and `R_rate[0]` (steering jerk) are the primary
  trade-off knobs. Increasing either will reduce overshoot at the cost of slower
  response.
- `R_rate[1]` (acceleration jerk) controls longitudinal smoothness. High values
  prevent jerky throttle/brake transitions.

### Adaptive scaling

Two functions in `offline_tuner.py` and `control_utils.py` modify `R` and `R_rate`
at runtime:

**`adaptive_R_scaling(vx, R)`** — multiplies `R[0,0]` by `1 + (1.5·vx)/(6+vx)`.
At 10 m/s this is ~2×. Prevents the MPC from commanding large steering angles
at high speed where the linear model is less accurate.

**`adaptive_R_rate(kappa, R_rate)`** — multiplies `R_rate[0,0]` by
`max(0.35, 1/(1+3κ))`. In tight corners (high κ) the jerk penalty drops to 35%,
letting the controller steer more aggressively through the corner entry.

To disable adaptive scaling, set these functions to return their inputs unchanged.

### Horizon length `N`

Currently `N=25` (1.25 s at 20 Hz). Increasing N gives more look-ahead—useful at
high speed—but increases QP solve time quadratically. At `v_max=20 m/s` the car
travels 25 m per horizon; this is well-matched to the 25 m perception window.

### Speed profile

`speed_profile.compute_speed_profile` parameters most worth adjusting:

- `mu` — planning friction. Currently `0.6` (~70% of peak `1.6`). Reduce to slow
  down corners further; increase to allow faster cornering.
- `safety` — multiplier on corner speed before forward/backward passes. Values
  `0.85–0.95` compensate for spline smoothing underestimating true curvature.
- `v_max` — absolute top speed. Currently `18 m/s`. The MPC actuator bound is
  `±5 m/s²` so acceleration from standstill is limited separately.

### DNF thresholds (tuner only)

`OFFTRACK_LIMIT = 2.5 m` in `run_headless_rollout`. Tightening this makes the
tuner penalise any lateral deviation more harshly, producing more conservative
weights. The live simulator uses `8.0 m` (physical track width limit).

### Synthetic path diversity

`VALIDATION_SUITE` in `offline_tuner.py` lists which paths are used for scoring.
Adding a new path to both `build_synthetic_paths()` and `VALIDATION_SUITE` forces
the optimiser to generalise to it. Removing paths from the suite speeds up
evaluation at the cost of specialisation.

---

## ROS 2 Integration (fsds)
To use the controller in the fsds simulator, we first need the fsae_planning repo.
Simply paste the contents of the `control_node.py` and `control_utils.py` into the matching files in track_utils of the planning package. 
(Assuming you already have the fsds repo cloned and set up. If not, you can refer to the windows set up below.)

**Topic map for the control node:**
For reference, here is how the control node recives data and controls the car.
```
/fsds/testing_only/track   → perception_node  → /FusionCones
/fsds/testing_only/odom    → perception_node
                             planner_node
                             control_node

/FusionCones               → planner_node     → /fsds/planned_path
                                              → /fsds/desired_speed
                                              → /fsds/lookahead_target

/fsds/planned_path         → control_node     → /fsds/control_command
/fsds/desired_speed        → control_node
/fsds/testing_only/odom    → control_node
/FusionCones               → control_node  (cone proximity brake)

/fsds/signal/go            → planner_node  (unlock)
                           → control_node  (unlock)
```

**Launching nodes with fsds on windows:**
To run the fsds simulator on windows, use wsl and clone the repo according to the instructions in documentation listed on the read me.
However, download the simulator release (exe file) on windows, not on WSL for better performance.
Note when cloning you may need to skip the larger files, otherwise it may not let you clone. 
You can run: `GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules`

To connect the simulator to the ros2 bridge, you need to configure the default IP in the ros2 bridge launch file to your WSL IP.
```
# Get WSL network interface IP: 
ip route | grep default | awk '{print $3}'

# In fsds_ros2_bridge.launch.py, find the host launch argument and configure as shown below with your IP.
launch.actions.DeclareLaunchArgument(
    'host',
    default_value='xxx.xx.xxx.x',
    description='IP address of the Windows host running the simulator'
),
```
Run the simulator first, then launch the ros2 bridge. Alternatively you can just use the launch script provided in the `fsds simulator` folder. Place this in the ros2 folder before running.
```
cd /home/Formula-Student-Driverless-Simulator/ros2/
chmod +x launch_all.sh
./launch_all.sh
```

**Building the ros2 nodes:**
To build the ros2 nodes (such as when running the simulator for the first time), you can use the custom docker script provided in the `fsds simulator` folder.
Then on wsl, run the below commands after cloning the fsds repo.
```
cd /root/Formula-Student-Driverless-Simulator/ros2
colcon build --packages-select fsae_planning --symlink-install

docker run -it \
    --name fsds_ros2_bridge \
    --net=host \
    --privileged \
    -v "$(pwd)":/root/Formula-Student-Driverless-Simulator \
    fsds_ros2_custom \
    bash

cd /root/Formula-Student-Driverless-Simulator/ros2
colcon build --packages-select fsae_planning --symlink-install
```
This should install all dependencies needed, then build the node.

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | ≥1.24 | All numerical computation |
| `scipy` | ≥1.10 | ZOH discretisation (`expm`), spline fitting |
| `matplotlib` | ≥3.7 | Simulator GUI and visualiser |
| `cvxpy` | ≥1.4 | MPC QP formulation |
| `osqp` | ≥0.6 | Primary QP solver (via CVXPY) |
| `clarabel` | ≥0.6 | Fallback QP solver (via CVXPY) |
| `cma` | ≥3.3 | CMA-ES optimiser (`fmin_lq_surr2`) |
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `Track`, `ControlCommand`, `GoSignal` message types |
| `nav_msgs` | ROS 2 | `Odometry`, `Path` |
| `geometry_msgs` | ROS 2 | `PoseStamped`, `PointStamped` |
