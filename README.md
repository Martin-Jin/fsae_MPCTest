# FSAE MPC Path Tracking Simulator

A high-fidelity 2D closed-loop simulator and offline weight tuner for a Formula Student
autonomous vehicle. The system pairs a nonlinear 24-state vehicle plant with a linear
time-varying MPC controller, and provides CMA-ES-based automated weight optimisation.

This includes an implementation of a control node that is combined with the fsae_planning repo (replacing the corresponding controller node file in the repo) to run the MPC controller in the fsds simulator, which simulates the car using Unreal Engine 4.

The simulator is also capable of simulating perception and planning via `USE_PLANNER`. In `simulation.py` this defaults to `False` (uses true reference path); in `offline_tuner.py` it defaults to `True` (full planner pipeline).
The 2D simulator does this by "placing" cones (setting coordinates of cones) to define the borders of a provided path in `sim_track.py` with the help of cone functions (in the `planning` folder) from the fsae_planning repo. These cones are then used for perception and planning in `sim_track.py`.
This is also togglable by a `USE_PLANNER` constant in the simulation file and tuner file. 


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
8. [Tuning History](#tuning-history)
9. [ROS 2 Integration (fsds)](#ros-2-integration-fsds)
10. [Dependencies](#dependencies)

---

## Architecture Overview

### Full System Flow
Note that this is when `USE_PLANNER` is toggled to be true and perception and planning is simulated.
When `USE_PLANNER` is false, the only difference is that true lateral error is calculated with the true center line rather than with the planner center line.
Also, note that there is input delay in the simulation which can be adjusted by `DELAY_STEPS` in `simulation.py`.

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
  history dict → scrub viewer + performance_stats (Show Metrics / Benchmark All Paths)
```

### Controller / Plant Architecture

```
                    ┌──────────────────────────────────────────┐
                    │              MPC (control layer)         │
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

### ROS 2 vs Simulator Mapping

How each component maps to its ROS2 equivalent, when compared to the fase planning package.

```
ROS 2 Node              │  Simulator Equivalent
────────────────────────┼─────────────────────────────────────
perception_node.py      │  sim_track.SimPerception  (active when USE_PLANNER=True)
planner_node.py         │  sim_track.SimPlanner     (active when USE_PLANNER=True)
cone_map.py             │  planning/cone_map.ConeMap        (shared)
boundary.py             │  planning/boundary.py             (shared)
path_utils.py           │  planning/path_utils.py           (shared)
cone_sorting.py         │  planning/cone_sorting.py         (shared)
control_utils.py        │  simulation.py / control_utils.py (shared)
control_node.py         │  simulation.py / control_node.py  (shared)
```

---

## Module Reference

Note: this covers the main simulator files only. The shared planning code in the `planning` folder is not documented here — refer to the fsae_planning repo.

---

### `simulation.py`
**Purpose:** Interactive matplotlib GUI simulator. Draw or load a path, configure initial conditions, run a closed-loop MPC simulation, scrub through history, view performance metrics. The main integration point for the entire codebase.

**Inputs:** User mouse draw events, or synthetic path loaded via the "Load Test Path" button cycling through `offline_tuner.PATH_NAMES`.

**Key outputs:** `sim_history` dict populated after each run, containing `X, Y, psi, v, v_target, u_steer, u_accel, e_y, e_psi, pred_X, pred_Y, failed, completion_frac, time_bonus, peak_lateral_error`.

**Key constants:**
```
dt         = 0.05 s     (20 Hz control rate)
N_horizon  = 25 steps   (1.25 s look-ahead)
V_MAX      = 20.0 m/s   (planner + profiler cap)
V_MIN      = 1.5 m/s    (planner floor speed)
OFFTRACK_LIMIT = 2.5 m  (lateral error failure threshold)
MAX_CONSECUTIVE_FAILURES = 5  (solver failures before DNF)
```

**Dependencies:** `bicycle_model`, `optimiser`, `vehicle_physics`, `performance_stats`, `speed_profile`, `offline_tuner` (SYNTHETIC_PATHS / PATH_NAMES only), `sim_track`, `model_utils`.

---

### `bicycle_model.py`
**Purpose:** Builds the 8-state discrete-time linear bicycle model used by the MPC for its N-step horizon predictions. Renamed from `model.py` to clarify its role as a bicycle model specifically.

**Function:** `get_8state_discrete_model(v_x, dt) → (Ad, Bd)`

**States:** `[e_y, ė_y, e_ψ, ψ̇, e_v, e_a, δ_act, a_act]`

**Method:** Blends kinematic (low speed, <1 m/s) and dynamic (high speed, >2.5 m/s) bicycle models linearly by speed, then discretises via exact ZOH using `scipy.linalg.expm` on the augmented `[A_c B_c; 0 0]` block.

All matrices are initialised with ε=1e-12 rather than exact zeros to maintain consistent sparsity patterns and prevent OSQP reallocation crashes across speed changes.

**Dependencies:** `numpy`, `scipy.linalg`, `vehicle_physics` (VehicleParams).

---

### `model_utils.py`
**Purpose:** Runtime adaptive gain-scheduling helpers that modify the MPC's R and R_rate weight matrices each step based on the vehicle's current speed and estimated path curvature. Extracted from offline_tuner/simulation into a shared module to avoid duplication.

**Functions:**

`curvature_estimate(state) → float`  
Estimates instantaneous path curvature κ = |r / vx| from yaw rate and speed. Safe minimum vx of 0.5 m/s prevents division by near-zero.

`adaptive_R_rate(kappa, R_rate_base) → R`  
Scales R_rate[0,0] (steering jerk cost) by `max(0.35, 1/(1 + 3κ))`.  
At κ=0 (straight): scale=1.0 (no change). At κ=0.2 (R=5 m): scale≈0.63. Floors at 0.35 so jerk cost never vanishes entirely.

`adaptive_R_scaling(vx, R_base) → R_scaled`  
Scales R[0,0] (steering cost) by `1 + (1.5·vx)/(6+vx)` (Hill function; asymptotes to 2.5× at high speed). Scales R[1,1] (acceleration cost) by `1 + 0.05·vx` (linear, gentler).

**Dependencies:** `numpy`.

---

### `optimiser.py`
**Purpose:** Parameterised CVXPY QP formulation solved at each tick by injecting updated parameter values, avoiding the costly CVXPY graph rebuild that would occur if the problem were reconstructed every step.

**Function:** `solve_mpc(x0, Ad, Bd, N, Q, R, u_min, u_max, ...) → u_opt or None`

**Returns:** `u_opt (2,)` = `[δ_cmd, a_cmd]` for the first horizon step, or `None` on solver failure (caller should hold the previous command).

**Solver chain:** OSQP primary → Clarabel fallback. `OPTIMAL_INACCURATE` is accepted with a warning (solution still used at 20 Hz). Any other non-optimal status triggers Clarabel; if that also fails, `None` is returned.

**QP formulation:**
```
min  Σ ||sqrtQ ⊙ x[:,i]||²  +  Σ ||sqrtR ⊙ u[:,i]||²
     + Σ ||sqrtR_rate ⊙ Δu[:,i]||²  +  W_slack * ||slack||²

s.t. x[:,0] = x0
     x[:,k+1] = Ad @ x[:,k] + Bd @ u[:,k]
     u_min ≤ u[:,k] ≤ u_max
     -3.5 - slack ≤ x[0,k] ≤ 3.5 + slack    (soft lane corridor)
```

**Key parameters:**
- `N` — horizon length (default 25 steps = 1.25 s)
- `eps_abs/rel` — OSQP tolerances (`1e-5` live, `1e-4` in offline tuner for speed)
- `warm_start` — reuse previous solution (True in live sim; False on first tuner step per rollout)
- `W_slack = 10000` — large enough to effectively enforce the lane constraint except during recovery

**Dependencies:** `cvxpy`, `numpy`.

---

### `vehicle_physics.py`
**Purpose:** 24-state high-fidelity nonlinear plant. The truth model that the MPC never sees directly — it only receives tracking errors derived from the plant's global position each tick.

**Function:** `step_nonlinear_plant(state, u_cmd, dt, params, road_mu=1.0, tv_gain=0.0) → state`

**Sub-steps:** 4 per control tick (h=0.0125 s) for numerical stability through stiff suspension dynamics.

**State vector (24 elements):**
```
[0-2]   X, Y, ψ          global pose
[3-5]   vx, vy, r        body-frame velocities + yaw rate
[6-7]   δ_act, a_act     actuator lag states
[8-9]   ω_RL, ω_RR       rear wheel spin (rad/s)
[10-13] z_FL..z_RR       suspension deflection from equilibrium (m)
[14-17] dz_FL..dz_RR     suspension velocity (m/s)
[18-21] Fy_FL..Fy_RR     relaxed lateral tyre forces (N)
[22-23] ω_FL, ω_FR       front wheel spin (rad/s)
```

**Physics features:**
- Full Pacejka MF94 lateral and longitudinal tyre model (B, C, D, E, Sv, Sh, camber)
- Per-wheel spring/damper/ARB suspension with dynamic Fz from load transfer
- Split front/rear aerodynamic downforce with pitch sensitivity under braking
- Friction ellipse coupling: Fy scales down when Fx consumes friction budget
- Tyre relaxation length (first-order lag on lateral force; τ≈35 ms at 10 m/s)
- Kinematic camber gain: suspension compression → negative camber → more grip
- Optional torque vectoring (`tv_gain`, default 0 = disabled)
- Road surface µ scaling (`road_mu`, default 1.0 = dry tarmac)
- Semi-implicit Euler integration for suspension (unconditionally stable)

**`VehicleParams` key values:**
```
m=255 kg, lf=0.85 m, lr=0.70 m, L=1.55 m, Iz=110 kg·m²
Cf=25000 N/rad, Cr=20000 N/rad  (linear; used by bicycle_model only)
tau_delta=0.08 s, tau_a=0.02 s
mu=1.9 (Pacejka peak, racing slick)
max_steer=35°, max_accel=12 m/s², max_brake=-9.0 m/s²
k_susp_f=25000 N/m, k_susp_r=30000 N/m
```

**Helper functions:**
- `init_plant_state(X0, Y0, psi0, vx0) → state` — initialise at true static+aero equilibrium with free-rolling wheel speeds (no initial transient)
- `plant_to_tracking_error(state, ref_x, ref_y, ref_psi)` — convert plant state to Frenet-frame tracking errors

**Dependencies:** `numpy`.

---

### `offline_tuner.py`
**Purpose:** Headless CMA-ES weight optimiser. Runs many closed-loop rollouts across synthetic paths and initial conditions to minimise a composite performance score, then prints the best-found `Q_diag`, `R_diag`, `R_rate_diag` ready to paste into `simulation.py`.

**Also exports** (imported by other modules):
- `SYNTHETIC_PATHS`, `PATH_NAMES` — pre-built path library (used by simulation.py)
- `SCORE_WEIGHTS`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY` — shared scoring constants (used by performance_stats.py)
- `curvature_estimate`, `adaptive_R_rate`, `adaptive_R_scaling` — legacy re-exports of model_utils functions (kept for backward compatibility; canonical source is model_utils.py)

**Search space:** 9 multiplicative scale factors (5 Q, 2 R, 2 R_rate), each bounded [0.1, 10.0] relative to the template diagonals.

**Optimiser:** `cma.fmin_lq_surr2` — BIPOP restart strategy with a local quadratic surrogate model. Surrogate reduces actual rollout count by ~3-10×. Parallel evaluation across all `cpu_count - 1` workers via `multiprocessing.Pool`.

**Objective:** `0.7 × weighted_mean + 0.3 × worst_case` across all tasks in `VALIDATION_SUITE × INITIAL_CONDITIONS`. The worst-case term prevents catastrophic failure on any single path type.

**Dependencies:** `vehicle_physics`, `bicycle_model`, `optimiser`, `speed_profile`, `sim_track`, `model_utils`, `cma`, `scipy`.

---

### `speed_profile.py`
**Purpose:** Computes a physically achievable per-point target speed profile along any path using curvature limits, forward acceleration, and backward braking passes. Replaces the previous fixed `v_ref=7.0 m/s` constant.

**Functions:**
- `compute_path_curvature(path_X, path_Y) → kappa[]` — finite-difference curvature κ = (x'y'' − y'x'') / (x'²+y'²)^1.5
- `compute_speed_profile(...) → v_profile[]` — per-point look-ahead: samples upcoming curvature in a window via the 3-point cross-product method and derives a safe speed from the friction circle limit
- `smooth_profile(v_profile, window=9) → v_profile[]` — moving-average smoothing

**Key parameters:**
```
v_max=20.0 m/s       absolute top speed cap
mu=0.6               planning friction (~37% of peak 1.6; gives MPC margin)
a_accel_max=4.0 m/s² matches MPC actuator bound
a_brake_max=-5.0 m/s²
v_min=1.5 m/s        floor speed
safety=1.0           corner speed multiplier (reduce to 0.85–0.95 on tight paths)
scan_end=14.0 m      short-path v_max scaling reference
```

**Dependencies:** `numpy`.

---

### `sim_track.py`
**Purpose:** Simulator equivalents of `perception_node.py` and `planner_node.py`. Shares all real planning code; only the ROS2 message transport layer is replaced.

**`place_cones(path_X, path_Y) → (blue, yellow)`**  
Evenly spaces cones along both boundaries at `CONE_SPACING=3.0 m`, offset `TRACK_HALF_WIDTH=1.75 m` from centreline (3.5 m total, FSG spec). Uses path tangent to compute left/right normals for each side.

**`SimPerception(blue_all, yellow_all)`**
- `visible_cones(car_x, car_y, car_yaw) → (blue_vis, yellow_vis)` — filters static map to car's forward FOV: `MIN_AHEAD=0.5 m`, `LOOK_AHEAD=25 m`, `LOOK_WIDE=10 m`. Directly mirrors `perception_node._publish_visible_cones()`.

**`SimPlanner(v_max, v_min, lookahead_dist)`**
- `update(blue_obs, yellow_obs, car_pos, car_yaw)` — ingest new cones into ConeMap, rebuild centreline + speed profile each step
- `reset()` — clear accumulated cone map and centreline

**`calculate_dynamic_max_steps(path_X, path_Y, dt, fallback_speed, buffer) → int`**  
Computes `ceil((arc_length / fallback_speed) × buffer / dt)` to size the step budget to path length rather than using a fixed 400-step cap.

**Dependencies:** `cone_map`, `boundary`, `path_utils`, `speed_profile`, `numpy`.

---

### `performance_stats.py`
**Purpose:** Scores a completed `sim_history` dict using the identical cost formula as `offline_tuner.run_headless_rollout`, so live simulator scores are directly comparable to offline tuning scores. Called by the "Show Metrics" button in simulation.py.

One approximation: `yaw_rms` and `max_yaw_rate` are derived from `diff(e_psi)/dt` rather than the plant's direct yaw rate state (which is not stored in the history dict). All other terms are exact replicas of the offline tuner's accumulation.

**Function:** `report_performance_metrics(history, log_fn=print) → metrics dict`

**Returns dict keys:** `composite_score`, `lateral_rmse_m`, `heading_rmse_deg`, `speed_rmse_mps`, `yaw_rms_radps`, `control_smooth_rms`, `steering_rms_deg`, `accel_rms_mps2`, `jerk_rms`, `max_steering_deg`, `steering_sat_ratio`, `steering_reversals`, `peak_lateral_error_m`, `completion_pct`, `failed`, `n_steps`.

**Dependencies:** `offline_tuner` (SCORE_WEIGHTS + bonus constants), `numpy`.

---

## Simulator Deep-Dive

### Startup and Path Input

The simulator opens a matplotlib figure. The user either draws a path by clicking and dragging on the map axes, or clicks **Load Test Path** to cycle through the 10 synthetic paths defined in `offline_tuner.build_synthetic_paths()`.

**Drawn paths:** On mouse release, raw points are deduplicated (min 0.5 m gap) and fitted with a clamped `CubicSpline` (endpoint derivatives pinned to chord direction to prevent overshoot). The spline is resampled at 600+ points. Heading `path_Psi` is derived from `arctan2(dy/dt, dx/dt)` of the spline derivative — not from finite differences of the raw points, which would be noisy. `speed_profile.compute_speed_profile()` + `smooth_profile()` runs immediately to build `path_v_profile`.

**Synthetic paths:** Pre-computed at import time by `build_synthetic_paths()`. Each path stores `(path_X, path_Y, path_Psi, path_v, blue_all, yellow_all)`. Geometry is FS-spec: straights 5–10 m, corner radii 5–12 m.

After path creation, `sim_track.place_cones()` populates the static cone arrays that `SimPerception` filters each simulation step.

### Simulation Loop (`simulate_closed_loop`)

Runs for up to `max_steps` steps (dynamically computed from path length) at `dt=0.05 s` (20 Hz). Each step:

**1. Record state** — append `X, Y, psi, v` to history before any computation.

**2. Perception update** — Controlled by the `USE_PLANNER` boolean passed to `simulate_closed_loop()` (and `run_headless_rollout()` in the offline tuner). When `True`: `SimPerception.visible_cones()` filters the static cone map; `SimPlanner.update()` accumulates cones, runs `build_path_walls()`, and recomputes the speed profile.

**3. Reference extraction** — the car is projected onto the active centreline (planner's or reference path's). The nearest waypoint segment gives the reference heading `rpsi`. Lateral error `e_y` is the signed perpendicular distance (positive = left of path).
**4. Error state assembly** — the 8-element MPC state vector:
```
x = [e_y, ė_y, e_ψ, ψ̇, e_v, 0, δ_act, a_act]
```
`e_y_dot` = `vx*sin(e_psi) + vy*cos(e_psi)` (lateral velocity projected onto path normal). `e_v` = `vx - v_target` (speed error relative to planner's desired speed).

**5. Early-exit checks:**
- `|e_y| > 3.50 m` → off-track, `failed = True`
- `consecutive_solver_failures ≥ 5` → `failed = True`
- `idx ≥ len(path) - 2` OR `dist_to_end ≤ 3.0 m` → `reached_end = True`

**6. MPC solve** — `get_8state_discrete_model(vx, dt)` linearises the bicycle model at the current speed. Adaptive gains are applied before the QP:
- `adaptive_R_scaling(vx, R)` — Hill-function steer cost increase with speed (~2.5× at high speed)
- `adaptive_R_rate(kappa, R_rate)` — reduces steering jerk penalty in tight corners (floor at 35%)

`optimiser.solve_mpc()` runs OSQP, falls back to Clarabel, returns `u*=[δ, a]` or `None`. On `None`, previous command is held and failure counter incremented.

**7. Horizon prediction** — linear model rollout of N=25 steps starting from current error state, projected back to global XY coordinates for visualisation only.

**8. Plant advance** — `step_nonlinear_plant(plant_state, u_opt, dt, vehicle_params)` advances the 24-state truth model through 4 Euler sub-steps. The Pacejka tyre forces, suspension dynamics, aero downforce, and tyre relaxation lags are all invisible to the MPC — the next step's tracking error closes the loop.

### Completion and History

`history["reached_end"] = True` if the car reaches within 2 path indices of the end or within 3 m of the final point. `completion_frac = 1.0` if reached, else `n_steps / max_steps`. On failure, `history["failed"] = True` with a `fail_reason` string.

### Scrub Viewer

After simulation, the time-scrub slider replays history frame by frame. The trail, MPC horizon prediction, car triangle marker, and telemetry panel all update together. The telemetry panel shows speed, target speed, position, heading, tracking errors, and control commands at the selected frame.

### Show Metrics

Calls `performance_stats.report_performance_metrics(sim_history)`, which prints a full breakdown to the console and returns a metrics dict. The composite score uses the same `SCORE_WEIGHTS` vector as the offline tuner, making the two directly comparable. The plot title updates with a summary.

---

## Offline Tuner Deep-Dive

### Purpose

Automatically finds `Q`, `R`, `R_rate` weight matrices that minimise a composite performance score across multiple synthetic paths and initial conditions, without running the GUI simulator.

### CMA-ES Strategy

Uses `cma.fmin_lq_surr2`: BIPOP (bi-population) restart strategy combined with a local quadratic surrogate model. The surrogate predicts which candidates are worth truly evaluating, reducing the number of actual rollouts by ~3-10×.

**BIPOP:** Interleaves large restarts (population doubles each time — broad exploration) with small restarts (reduced population — local refinement near the current best). This escapes local minima while exploiting promising regions.

**Parameter space:** 9 multipliers (5 Q, 2 R, 2 R_rate), each bounded `[0.1, 10.0]` relative to the template diagonals. This allows ±1 decade of adjustment in any weight.

**Initial point:** `x0 = sqrt(lower × upper) = 1.0` per parameter (geometric / log-scale midpoint of bounds).

**`sigma0=0.5`, `CMA_stds = 0.23 × log(upper/lower)`** — 23% of the log-space parameter range as the initial per-dimension 1-sigma radius. `log(10/0.1) ≈ 4.6`, so `CMA_stds ≈ 1.06` per dimension.

**`max_restarts=7`, `max_evals=2500`** — total budget including all restarts. Each BIPOP restart uses `incpopsize=2` (large restarts double the population from the previous large restart's size).

### Objective Function

`parallel_evaluate_candidate(vec)` distributes `len(EVAL_TASKS)` rollouts across `cpu_count - 1` worker processes via `pool.map()`. Results are aggregated as:
```
score = 0.7 × weighted_mean + 0.3 × worst_case
```
The 30% worst-case term prevents the optimiser from finding weights that work well on average but catastrophically fail on one path type.

### Headless Rollout (`run_headless_rollout`)

Functionally mirrors `simulate_closed_loop()` but without GUI, matplotlib, or full history storage. Uses looser OSQP tolerances (`eps=1e-4`, `max_iter=5000`) for ~2× speed over the live simulator's `1e-5`. A model cache keyed by `round(vx, 1)` avoids rebuilding ZOH matrices every step.

**DNF conditions (tighter than live simulator):**
- `|e_y| >= 3.50 m`
- `consecutive_fails ≥ 5`
- Less than `3 m` progress in any rolling 60-step (3 s) window (rolling stall detection)

**DNF penalty:** `DNF_PENALTY = 3.0` added for any non-completion; `DNF_OFFTRACK_PENALTY = 3.0` added additionally if the vehicle left the track boundary. Both are flat constants defined in `offline_tuner.py`.

### Composite Score (`SCORE_WEIGHTS`)

```
Index  Metric                Notes
  0    rmse                  primary: combined 1.2 * e_y² + 0.4*e_psi² RMSE
  1    yaw_rms               stability
  2    smooth_rms (Δu)       control smoothness
  3    steer_rms             steering effort magnitude
  4    accel_rms             acceleration effort magnitude
  5    max_steering          peak steering command
  6    steering_sat_ratio    fraction of steps near saturation
  7    jerk_rms (Δ²u)        control jerk
  8    max_yaw_rate          cornering aggressiveness
  9    steering_reversals    sign-change hunting count
 10    peak_lateral_error    worst single-step deviation
 11    speed_rmse            difference between current and planner speed
```

Bonuses (subtracted from score — reward for completing quickly):
- `COMPLETION_BONUS_WEIGHT` × `completion_frac`
- `TIME_BONUS_WEIGHT` × `time_bonus`

Lower composite score is better. A good finishing run typically scores in the range `[-0.5, -0.3]`.

### Inaccuracy Penalty

If OSQP returned `OPTIMAL_INACCURATE` on any steps, the final score is scaled by `1 + min(5, count) × 0.1` (up to 50% penalty). Uses `sign(score) × abs(score) × factor` to preserve the sign of already-negative (good) scores.

### Post-Optimisation Selection

After all evaluations, both `xbest` (lowest score observed across all true evaluations) and `xfavorite` (the distribution mean — more robust to noise) are freshly evaluated serially. Whichever scores lower is selected and printed as copy-paste arrays. Results are appended to `tuning history.txt` with timestamp, duration, and git commit hash.

---

## Running the Simulator

### Requirements

```bash
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

1. Click and drag on the map to draw a reference path (at least 6 points).
2. The path is automatically splined, heading computed, and speed profile generated.
3. Optionally adjust **Initial Lat Error** (±4 m) and **Initial Yaw Error** (±30°) sliders to start the car off-centre.

### Loading a synthetic path

Click **Load Test Path** to cycle through the 10 built-in FS-spec paths:  
`PATH_SUDDEN_TURN`, `PATH_S_BEND`, `PATH_SPIRAL`, `PATH_MICRO_SLALOM`,  
`PATH_OFFSET_CHICANE`, `PATH_ACCELERATION`, `PATH_HAIRPIN`, `PATH_CHICANE`,  
`PATH_FS_CORNER`, `PATH_MIXED`.
Each click advances to the next path. Camera auto-frames to the path with a 15 m margin.

### Running a simulation

Click **Start Sim**. The loop runs synchronously (no live animation during solve). On completion the plot title turns green and the time-scrub slider appears. Drag it to replay history frame by frame; the telemetry panel updates at each frame.

### Viewing metrics

Click **Show Metrics** after a simulation. A full performance breakdown is printed to the console. The composite score, lateral RMSE, heading RMSE, and completion percentage appear in the plot title.

### Resetting

Click **Reset Environment** to clear everything and draw or load a new path.

---

## Running the Offline Tuner

### Launch

```bash
cd /path/to/project
python offline_tuner.py
```

Uses all available CPU cores minus one. A typical run takes 20–60 minutes depending on core count and `MAX_EVALS`.

### Key constants to adjust

In `offline_tuner.py`:

```python
MAX_EVALS     = 1000   # Total true rollout budget (surrogate reduces actual count ~3-10×)
sigma0        = 0.65    # CMA-ES initial step size
max_restarts  = 7      # BIPOP restart budget
VALIDATION_SUITE = [   # Paths used for scoring — add/remove to change coverage
    "PATH_MICRO_SLALOM",
    "PATH_SUDDEN_TURN",
    "PATH_SKIDPAD",
    "PATH_S_BEND",
    "PATH_CHICANE",
]
```

### Reading the output

Each generation prints:
```
[lq-CMA-ES] gen    5 | true_evals    90 | gen_best 0.2341 | overall_best 0.1892 | sigma 6.123e-01
```

On completion:
```
Replace your simulation.py weights with:
Q_diag      = [9.35, 22.2, 18.9, 49.8, 10.8, 0.0, 0.0, 0.0]
R_diag      = [49.3, 45.4]
R_rate_diag = [50.0, 49.6]
```

### Applying optimised weights

Copy the three arrays into:
- `simulation.py` — `Q_diag`, `R_diag`, `R_rate_diag` assignments near the top
- `control_utils.py` — same three arrays inside `MPCController.__init__`

Both files must stay in sync; the tuner was designed against the same plant and horizon used by both.

---

## Tuning Guide

### Weight matrices

`Q` penalises tracking error states. `R` penalises control effort. `R_rate` penalises control rate-of-change (smoothness / jerk).

State order for `Q_diag`: `[e_y, ė_y, e_ψ, ψ̇, e_v, e_a, δ_act, a_act]`

- `Q[0]` (`e_y`) is the primary lateral tracking term. Raising it tightens path following but increases steering effort and saturation risk.
- `Q[2]` (`e_ψ`) penalises heading error. High values force rapid heading correction but can cause oscillation at low speed where the kinematic model dominates.
- `Q[4]` (`e_v`) controls speed tracking aggressiveness. Setting it to 0 disables speed tracking entirely.
- `Q[5..7]` are zero — actuator states are not penalised directly.
- `R[0]` (steering effort) and `R_rate[0]` (steering jerk) are the primary trade-off knobs. Increasing either reduces overshoot but slows response.
- `R_rate[1]` (acceleration jerk) controls longitudinal smoothness. High values prevent jerky throttle/brake transitions.

### Adaptive scaling (model_utils.py)

Two functions modify `R` and `R_rate` at runtime every step:

**`adaptive_R_scaling(vx, R)`** — multiplies `R[0,0]` by `1 + (1.5·vx)/(6+vx)`. At 6 m/s: ×1.75. At 15 m/s: ×2.33. Prevents the MPC from commanding destabilising large steering angles at speed where the linear model is least accurate.

**`adaptive_R_rate(kappa, R_rate)`** — multiplies `R_rate[0,0]` by `max(0.35, 1/(1+3κ))`. In tight corners (high κ) the jerk penalty drops to 35% of base, letting the controller steer aggressively enough to track the corner.

To disable either, return the input unchanged (or set `tv_gain=0`, which it already is).

### Horizon length `N`

Currently `N=25` (1.25 s at 20 Hz). At `v_max=20 m/s` the car travels 25 m per horizon — well-matched to the 25 m perception window. Increasing N improves look-ahead on fast straights but increases QP solve time roughly quadratically.

### Speed profile parameters

The most impactful `speed_profile.compute_speed_profile()` parameters:

- `mu=0.6` — planning friction. Reduce to slow corners further; increase to allow faster cornering speeds.
- `safety=1.0` — multiplier on curvature-derived corner speed. Values 0.85–0.95 compensate for spline smoothing underestimating true curvature at tight corners.
- `v_max=20.0` — absolute top speed cap. The MPC actuator bound (±5 m/s²) limits acceleration from standstill independently.

Tightening the tuner's limit produces more conservative weights. Loosening it allows the tuner to accept borderline trajectories and may produce more aggressive weights.

### Synthetic path diversity

`VALIDATION_SUITE` in `offline_tuner.py` lists which paths score each candidate. Adding a path to both `build_synthetic_paths()` and `VALIDATION_SUITE` forces the optimiser to generalise to it. Removing paths from the suite speeds up each evaluation at the cost of reduced generalisation. Currently PATH_MIXED and PATH_HAIRPIN are commented out of the suite to reduce evaluation time; re-enable them if the tuned weights struggle on compound corners.

---

## Tuning History

Results are automatically logged to `tuning_history.txt` at the end of each tuner run. Each entry records the timestamp, weight diagonals, run duration, offline composite score, and git commit hash.

---

## ROS 2 Integration (fsds)

To use the controller in the fsds simulator, first obtain the fsae_planning repo. Paste the contents of `control_node.py` and `control_utils.py` into the matching files in the `track_utils` package of fsae_planning.

**Topic map for the control node:**
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

**Launching nodes with fsds on Windows:**  
Run the fsds simulator on Windows, bridge via WSL. Clone the repo skipping large LFS files if needed:
```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

Configure the WSL IP in the ROS2 bridge launch file:
```bash
# Get WSL network interface IP:
ip route | grep default | awk '{print $3}'

# In fsds_ros2_bridge.launch.py, set the default_value to your IP:
launch.actions.DeclareLaunchArgument(
    'host',
    default_value='xxx.xx.xxx.x',
    description='IP address of the Windows host running the simulator'
),
```

Use the provided launch script:
```bash
cd /home/Formula-Student-Driverless-Simulator/ros2/
chmod +x launch_all.sh
./launch_all.sh
```

**Building the ROS2 nodes:**
```bash
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

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | ≥1.24 | All numerical computation |
| `scipy` | ≥1.10 | ZOH discretisation (`expm`), spline fitting (`CubicSpline`) |
| `matplotlib` | ≥3.7 | Simulator GUI |
| `cvxpy` | ≥1.4 | MPC QP formulation |
| `osqp` | ≥0.6 | Primary QP solver (via CVXPY) |
| `clarabel` | ≥0.6 | Fallback QP solver (via CVXPY) |
| `cma` | ≥3.3 | CMA-ES optimiser (`fmin_lq_surr2`, BIPOP+surrogate) |
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `Track`, `ControlCommand`, `GoSignal` message types |
| `nav_msgs` | ROS 2 | `Odometry`, `Path` |
| `geometry_msgs` | ROS 2 | `PoseStamped`, `PointStamped` |

## Developer Guide

If you are extending the simulator or tuning the vehicle, follow these guidelines to maintain the integrity of the MPC-plant architecture.

### Modifying Vehicle Parameters
The single source of truth for all vehicle physics is the `VehicleParams` class in `vehicle_physics.py`. 
- **Tyre Data:** We use a Pacejka MF94 model. If you are importing new tyre data (e.g., from TTC), update the `B`, `C`, `D`, and `E` coefficients. Note that the linear bicycle model (`bicycle_model.py`) relies on `Cf` and `Cr`. If you alter the Pacejka peak coefficients (`D`) or stiffness (`B`), you **must** recalculate and update `Cf` and `Cr` to match the new initial slope, or the MPC's internal predictions will heavily diverge from the plant.
- **Actuator Limits:** Changing `max_steer` or `max_accel` automatically propagates to the MPC's hard QP constraints in `optimiser.py`.
- **Tuning knobs:** There are some constants defined at the top of the `VehicleParams` which proportional scale relevant vehicle parameters for ease of tuning. These being `GRIP_SCALE`, `INERTIA_SCALE`, and `COASTING_SCALE`.

### Adding a New Synthetic Path
To test against a new track geometry, you need to append it to the library in `offline_tuner.py`:
1. Navigate to the `build_synthetic_paths()` function.
2. Define your segments. Use `_make_arc(cx, cy, radius, start_deg, end_deg, n)` for constant-radius corners and `np.linspace()` for straights.
3. Concatenate your arrays and pass them through `_resample_path(wx, wy)`.
4. Add the resulting tuple to the `paths` dictionary.
5. *(Optional)* Add the dictionary key to `VALIDATION_SUITE` if you want the CMA-ES tuner to optimise against it.

### Debugging Solver Failures
If the live simulator throws `consecutive_solver_failures` or the GUI terminal flags `OPTIMAL_INACCURATE` frequently, check the following:
* **Weight Scaling:** OSQP is sensitive to poorly conditioned matrices. If any element in `Q`, `R`, or `R_rate` exceeds `1e4` or drops below `1e-4`, the solver may fail to converge. Check the outputs of `adaptive_R_scaling` in `model_utils.py` to ensure high speeds aren't blowing up the steering costs.
* **Kinematic vs. Dynamic Gap:** If the car consistently fails at tight hairpins, the `speed_profile.py` might be commanding speeds that require lateral forces exceeding the Pacejka friction circle. Lower the `mu` planning parameter in `compute_speed_profile()` to force the MPC to approach corners more conservatively.