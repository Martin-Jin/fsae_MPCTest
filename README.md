# FSAE MPC Path Tracking Simulator

A high-fidelity 2D closed-loop simulator and offline weight tuner for a Formula
Student autonomous vehicle. The system pairs a nonlinear 24-state vehicle plant
with a linear time-varying Model Predictive Controller (MPC), and provides
CMA-ES-based automated weight optimisation so the controller's cost weights
don't have to be hand-tuned by trial and error.

This repository also includes a ROS 2 control node (`control_node.py` /
`control_utils.py`) that runs the same MPC live inside the
[FSDS](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)
simulator, by replacing the corresponding controller node file in the
[fsae_planning](https://github.com/UOA-FSAE/fsae_planning) repo. Weights tuned
offline in this project transfer directly to that live controller.

The 2D simulator can optionally simulate the full perception + planning
pipeline (`USE_PLANNER` in `settings.py`) by placing cones along a path
(`sim_track.place_cones()`) and reconstructing a centreline from them using
the shared planning code in the `planning/` folder (taken from the
`fsae_planning` repo). When `USE_PLANNER` is off, the simulator instead tracks
the true reference path directly — faster, and useful for isolating driving
behaviour from planner behaviour.

fsds simulator repo: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator (current implementation uses commit 59f03fa, and the V2.20 release)
fsae planning repo: https://github.com/UOA-FSAE/fsae_planning (current implementation uses commit 28dcd4d)

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Running the Simulator](#running-the-simulator)
3. [Running the Offline Tuner](#running-the-offline-tuner)
4. [Configuring the Project (`settings.py`)](#configuring-the-project-settingspy)
5. [Configuring the Vehicle (`vehicle_physics.py`)](#configuring-the-vehicle-vehicle_physicspy)
6. [How the MPC Works](#how-the-mpc-works)
7. [How the Offline Tuner Works](#how-the-offline-tuner-works)
8. [Module Reference](#module-reference)
9. [ROS 2 Integration (fsds)](#ros-2-integration-fsds)
10. [Manual Drive Mode](#manual-drive-mode)
11. [Dependencies](#dependencies)
12. [Developer Guide](#developer-guide)

---

## Architecture Overview

### Full System Flow

This is the closed loop the simulator runs at 20 Hz. It's the same loop
`offline_tuner.py` runs headless (no plotting) thousands of times during
tuning, and the same loop `control_node.py` runs live against the real/FSDS
vehicle. All three share one implementation (`rollout_core.run_core_rollout()`
for the first two; `control_utils.MPCController` for the live node, kept in
numeric parity with `rollout_core`).

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
  history dict → scrub viewer + performance_stats (Show Metrics / Benchmark All Paths)
```

### Controller / Plant Architecture

```
                    ┌──────────────────────────────────────────┐
                    │   rollout_core.run_core_rollout()         │
                    │   (single shared rollout loop — see below)│
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
are thin wrappers around `rollout_core.run_core_rollout()` — the single
implementation of the tracking-error computation, progress tracking, MPC solve,
delay queue, termination checks, and metric accumulation. `simulation.py` calls
it with `want_history=True` to get a full step-by-step history dict for the GUI;
`offline_tuner.py` calls it with `want_history=False` for a fast, scoring-only
path. This guarantees a path run in the live simulator and the same path
benchmarked offline produce (near-)identical composite scores.

### ROS 2 vs Simulator Mapping

How each component maps to its ROS 2 equivalent in the `fsae_planning` package:

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

The `stanley_control.py` / `stanely_control_utils.py` files under
`fsds simulator/stanley controller/` are the **previous** controller
implementation (a Stanley path-tracking controller), kept only as a reference
for how a ROS 2 control node in this project is structured. The active
controller is the MPC in `control_node.py` / `control_utils.py`.

---

## Running the Simulator

The simulator (`simulation.py`) is an interactive matplotlib GUI for drawing
or loading a path, running one closed-loop MPC rollout against the nonlinear
vehicle plant, and reviewing the result frame by frame.

### 1. Install dependencies

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
```

### 2. Launch

```bash
cd /path/to/project
python simulation.py
```

### 3. Get a path onto the map

Either:

- **Draw one** — click and drag on the map (at least 6 points). On release the
  path is automatically splined, headings computed, and a speed profile
  generated.
- **Load a synthetic one** — click **Load Test Path** to cycle through the
  10 built-in FS-spec paths (`PATH_SUDDEN_TURN`, `PATH_S_BEND`, `PATH_SPIRAL`,
  `PATH_MICRO_SLALOM`, `PATH_OFFSET_CHICANE`, `PATH_ACCELERATION`,
  `PATH_HAIRPIN`, `PATH_CHICANE`, `PATH_FS_CORNER`, `PATH_MIXED`). Each click
  advances to the next path; the camera auto-frames around it with a 15 m
  margin.

### 4. (Optional) set initial conditions

Once a path exists, two sliders appear:

- **Initial Lat Error** (±4 m) — starts the car offset sideways from the path.
- **Initial Yaw Error** (±30°) — starts the car pointing the wrong way.

Useful for stress-testing recovery behaviour rather than always starting
perfectly on-line.

### 5. Run it

Click **Start Sim**. The rollout runs synchronously (no live animation while
it solves — this can take a few seconds for a long path). When it finishes,
the title turns green and a **Time** scrub slider appears below the map.

### 6. Review the run

Drag the **Time** slider to replay the run frame by frame. The trail, the
cyan MPC horizon prediction, the car marker, and the telemetry panel (speed,
position, heading, tracking errors, steering/accel commands) all update
together.

### 7. Score it

Click **Show Metrics** to print a full 12-metric breakdown to the console
(see [Composite Score](#the-composite-score) below) and show a one-line
summary in the plot title. Click **Benchmark All Paths** to run every
synthetic path 3× each with the currently loaded weights and print a
per-path score table — useful for checking a weight set generalises rather
than only working on the path you happened to test.

### 8. Reset

Click **Reset Environment** to clear everything and start over.

---

## Running the Offline Tuner

The offline tuner (`offline_tuner.py`) automatically searches for `Q`, `R`,
`R_rate` cost weights that minimise the [composite score](#the-composite-score)
across a library of synthetic corner shapes, using CMA-ES (see
[How the Offline Tuner Works](#how-the-offline-tuner-works) for the algorithm
itself). It has no GUI — it's a long-running batch job you leave to finish.

### 1. Install dependencies

Same as the simulator (see above) — `offline_tuner.py` uses the same
`cvxpy`/`osqp`/`clarabel`/`cma` stack, plus Python's built-in
`multiprocessing` to spread rollouts across CPU cores.

### 2. Check `settings.py` first

Before running, confirm:

- `VALIDATION_SUITE` lists the corner shapes you want the tuner to optimise
  for (see [Configuring the Project](#configuring-the-project-settingspy)).
- `MAX_EVALS` is set to a budget you're happy to wait for (a good run is
  20 minutes to a few hours depending on core count and `MAX_EVALS`).
- `USE_PLANNER` reflects whether you want the tuner testing the full
  perception/planning pipeline (`True`, default) or driving on the perfect
  reference line (`False`, faster).

### 3. Launch

```bash
cd /path/to/project
python offline_tuner.py
```

This uses all available CPU cores minus one (one is left free for the OS).
Progress prints once per CMA-ES generation:

```
[lq-CMA-ES] gen    5 | true_evals    90 | gen_best 0.2341 | overall_best 0.1892 | sigma 6.123e-01
```

`gen_best` is this generation's best score; `overall_best` is the best score
seen so far across the whole run; `sigma` is CMA-ES's current search-radius
(shrinks as it converges). Lower scores are better throughout.

You can safely stop early with **Ctrl+C** — the tuner finishes its current
generation, then reports the best weights found so far rather than exiting
uncleanly.

### 4. Read the result

On completion (or early stop), the tuner prints the best weight arrays found:

```
Replace your simulation.py weights with:
Q_diag      = [9.35, 22.2, 18.9, 49.8, 10.8, 0.0, 0.0, 0.0]
R_diag      = [49.3, 45.4]
R_rate_diag = [50.0, 49.6]
```

It also prints a list of "improvement milestones" — the point in the search
(by true-evaluation count) at which each meaningfully better score was found,
so you can see how much of the run's time was actually productive.

### 5. Apply the weights

Copy the three arrays into **both**:

- `settings.py` — `Q_diag`, `R_diag`, `R_rate_diag` (used by `simulation.py`
  and, from there, everything that imports them)
- `control_utils.py` — the same three arrays hardcoded inside
  `MPCController.__init__` (used by the live ROS 2 controller)

Both must stay in sync manually — the tuner was designed against the same
plant and horizon used by both, but there is currently no single shared
import between them (`control_utils.py` is a standalone file so the live
ROS 2 node has no simulator dependencies).

### 6. Log the result

Every run appends its result to `tuning_history.txt` automatically
(timestamp, weight diagonals, duration, tuner score, git commit hash). Go
back and manually fill in the `Overall score` field once you've actually
tested the weights in FSDS or on the real car — the offline tuner score
alone doesn't perfectly predict real-world performance, so this file is
where the two get reconciled over time. See existing entries in
`tuning_history.txt` for the expected format.

### Key constants to adjust

All of these live in `settings.py`, not `offline_tuner.py` — see the next
section for what each one does and how much to change it by:

```python
MAX_EVALS         # Total true rollout budget (surrogate reduces actual count ~3-10x)
VALIDATION_SUITE  # Which synthetic corner shapes the tuner scores against
```

`sigma0` (CMA-ES's initial search radius) and `max_restarts` (BIPOP restart
budget) are algorithm-internal tuning knobs rather than project settings —
they're set near the bottom of `offline_tuner.py`'s `__main__` block if you
need to adjust them; see [How the Offline Tuner Works](#how-the-offline-tuner-works)
for what they control.

---
## Configuring the Project (`settings.py`)

`settings.py` is the single place to change tuning knobs, cost weights, and
DNF/validation configuration shared by `simulation.py`, `offline_tuner.py`,
`scoring.py`, `rollout_core.py`, and `performance_stats.py`. It has no
vehicle physics in it — that lives in `vehicle_physics.py` (see next
section). Every setting has a detailed, plain-language explanation directly
above it in the file itself, including what it does, why you'd change it,
and roughly how much to change it by — this section is a quick-reference
summary; **read the comments in `settings.py` before changing anything.**

### General system configuration

| Setting | What it controls |
|---|---|
| `N_HORIZON` | How many 0.05 s steps ahead the MPC plans each solve (25 = 1.25 s look-ahead). Must match `N_horizon` in `simulation.py` and `N` in `control_utils.py`. |
| `USE_PLANNER` | Whether the tuner drives using the full simulated perception/planning pipeline (`True`) or the perfect reference path (`False`). |
| `DELAY_STEPS` | Simulated lag (in 0.05 s steps) between a command being decided and applied — for testing robustness to real actuator/network delay. |
| `MAX_FAILS` | Consecutive MPC solver failures before a rollout is abandoned as a DNF. |
| `OFFTRACK_LIMIT` | Lateral error (m) beyond which the car is considered off-track. Derived from `TRACK_HALF_WIDTH` in `sim_track.py` — change that instead if you want to adjust it. |
| `DT` | Control/simulation timestep (s), 0.05 = 20 Hz. Must match the real controller's timer rate. |

### Cost weights

`Q_diag`, `R_diag`, `R_rate_diag` — the MPC's tracking/effort/smoothness cost
weights. These are the direct output of the most recent `offline_tuner.py`
run (see [Running the Offline Tuner](#running-the-offline-tuner)) and are not
meant to be hand-edited entry-by-entry. See
[How the MPC Works](#how-the-mpc-works) for exactly what each entry means.

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

### Scoring weights

`SCORE_WEIGHTS` — the 12-entry array defining what "good driving" means:
how much each of the 12 measured aspects of a rollout (tracking error,
smoothness, steering effort, saturation, jerk, etc.) contributes to the
final composite score the tuner minimises. Must sum to 1.0 (enforced by an
assertion). See [The Composite Score](#the-composite-score) for exactly what
each of the 12 metrics measures and how they combine.

`VALIDATION_SUITE` — which of the synthetic corner-shape paths (defined in
`offline_tuner.build_synthetic_paths()`) the tuner actually evaluates
candidates against. Commented-out paths are available but excluded by
default to keep each tuning run faster.

### Bonus weights

`COMPLETION_BONUS_WEIGHT` / `TIME_BONUS_WEIGHT` — score reductions
(rewards) for finishing the track at all, and for finishing it quickly.

---

## Configuring the Vehicle (`vehicle_physics.py`)

The single source of truth for all vehicle physics — mass, geometry, tyre
grip, suspension, aerodynamics, actuator limits — is the `VehicleParams`
class in `vehicle_physics.py`. This is what the nonlinear 24-state plant
(the "truth" simulation) uses, and several of these values (`Cf`, `Cr`,
`tau_delta`, `tau_a`, `lf`, `lr`, `m`, `Iz`) also feed directly into the
MPC's own internal linear model in `bicycle_model.py` — see
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
COASTING_SCALE = 3.0   # Scales drag + rolling resistance together (< 1.0 = rolls further, > 1.0 = stops faster)
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
> `bicycle_model.py`) to match the new Pacejka curve's initial slope near
> zero slip angle: `C_eff ≈ mu * Fz_nominal * B * C * D`. If `Cf`/`Cr` don't
> match the new Pacejka peak (`D`) and stiffness (`B`), the MPC's internal
> prediction model will diverge from the plant it's actually controlling,
> degrading tracking performance in ways that are hard to diagnose from the
> symptoms alone.

### Actuator limits

`max_steer`, `max_accel`, `max_accel_brake` — changing these automatically
propagates to the MPC's hard QP constraints in `optimiser.py` and
`control_utils.py` (both read `VehicleParams` directly), so the controller
will never be asked to command something the (simulated) vehicle physically
can't do.

---
## How the MPC Works

This section explains the controller in full: the state vector, where every
entry of every matrix comes from, the cost function, the solver, and the two
runtime adaptive features layered on top. The implementation is split across
three files that must be kept in numeric agreement — `bicycle_model.py`
(the prediction model), `optimiser.py` (the QP formulation, used by the
simulator/tuner), and `control_utils.py` (a self-contained duplicate of both,
used by the live ROS 2 node so it has no simulator dependencies).

### What "MPC" means here

At every control tick (20 Hz), the controller:

1. Measures the current tracking error (`x0`).
2. Predicts, using a simplified **linear** model, what the tracking error
   would do over the next `N_HORIZON` steps (1.25 s) for every possible
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
`tau_delta`, `tau_a` in `vehicle_physics.py`). Tracking the *actual*
(lagged) actuator state, not just the commanded value, lets the model
correctly predict how the car will really move over the horizon.

### The 2-input control vector

```
u = [delta_cmd, a_cmd]
```

`delta_cmd` (rad) and `a_cmd` (m/s²) are the raw commands sent to the
actuator lag filters — not the actual steering angle / acceleration
themselves (those are states 6 and 7 above, which lag behind `u`).

### Building the prediction model (`bicycle_model.py`)

The MPC's internal model of "how does the error state change if I apply
this control input" is a linearised **bicycle model** — the vehicle is
approximated as having one front wheel and one rear wheel on the
centreline, rather than four separate corners. Two different linearisations
are blended together based on speed, because a single linear model can't
represent vehicle behaviour accurately across the full speed range.

#### 1. Kinematic model (used below ~1 m/s)

At very low speed, tyres haven't built up meaningful lateral (cornering)
force yet, so the car turns purely by geometry — like pushing a shopping
trolley. The relevant continuous-time equations are:

```
ė_y   = v_x · e_psi
ė_psi = v_x · delta_act / L        (L = wheelbase = lf + lr)
```

In code (`A_kin`, an 8×8 matrix — entry `[row, col]` means "this much of
`col`'s value feeds into `row`'s rate of change"):

```python
A_kin[0, 2] = v_x_safe          # ė_y = v_x * e_psi
A_kin[2, 6] = v_x_safe / L      # ė_psi = v_x/L * delta_act  (Ackermann geometry)
```

#### 2. Dynamic model (used above ~2.5 m/s)

At higher speed, tyre lateral force (cornering stiffness × slip angle)
dominates over pure geometry. This is the standard linearised bicycle model
derived from Newton/Euler planar rigid-body equations under the small
slip-angle approximation:

```
ë_y   = -(2Cf+2Cr)/(m·vx) · ė_y  +  (2Cf+2Cr)/m · e_psi
        + (-2Cf·lf+2Cr·lr)/(m·vx) · e_psi_dot  +  (2Cf)/m · delta_act

ë_psi = (-2Cf·lf+2Cr·lr)/(Iz·vx) · ė_y  +  (2Cf·lf-2Cr·lr)/Iz · e_psi
        - (2Cf·lf²+2Cr·lr²)/(Iz·vx) · e_psi_dot  +  (2Cf·lf)/Iz · delta_act
```

Where `Cf`/`Cr` are the front/rear linear cornering stiffnesses (N/rad,
from `VehicleParams`), `lf`/`lr` are the CoM-to-axle distances, `m` is mass,
and `Iz` is yaw inertia. The `1/vx` terms exist because at higher speed, the
same lateral velocity produces a *smaller* slip angle (the tyre has less
time to "notice" the sideways motion relative to how far it's rolled
forward), so cornering force builds more slowly per unit of lateral drift.

In code (`A_dyn`):

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

#### 3. Shared rows (identical in both models)

Regardless of which regime is active, the speed-error and actuator-lag rows
are the same:

```python
A_kin[4, 5] = A_dyn[4, 5] = 1.0             # ė_v = e_a
A_kin[5, 7] = A_dyn[5, 7] = 1.0             # ė_a = a_act (structural; e_a itself is unused)
A_kin[6, 6] = A_dyn[6, 6] = -1.0 / tau_delta  # dδ_act/dt = -δ_act/tau_delta (decays toward 0 with no input)
A_kin[7, 7] = A_dyn[7, 7] = -1.0 / tau_a      # da_act/dt = -a_act/tau_a
```

...and the input matrix `B` (how commands `u` drive the actuator lag states)
is identical for both models:

```python
B[6, 0] = 1.0 / tau_delta   # delta_cmd drives the steering lag integrator
B[7, 1] = 1.0 / tau_a       # a_cmd drives the acceleration lag integrator
```

This is the first-order lag ODE `dδ_act/dt = (delta_cmd − δ_act)/tau_delta`
split across the two matrices: the `-δ_act/tau_delta` self-decay term lives
in `A[6,6]`, and the `+delta_cmd/tau_delta` drive term lives in `B[6,0]`.

#### 4. Blending kinematic and dynamic models

```python
alpha = clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
A_c   = (1.0 - alpha) * A_kin + alpha * A_dyn
```

`alpha` ramps linearly from 0 to 1 as speed goes from 1 m/s to 2.5 m/s: pure
kinematic model below 1 m/s, pure dynamic model above 2.5 m/s, and a smooth
linear blend of the two matrices in between. This avoids a discontinuous
jump in the model's predicted behaviour as the car accelerates away from a
standstill — which would otherwise show up as a sudden jerk in commanded
steering right around walking pace.

`B` is identical for both models, so it's used unchanged regardless of `alpha`.

#### 5. Discretisation: Zero-Order Hold (ZOH)

The equations above are continuous-time (`ẋ = A_c·x + B_c·u`), but the MPC
needs a **discrete** one-step prediction `x[k+1] = Ad·x[k] + Bd·u[k]` at
`dt = 0.05 s` to build its QP. ZOH is the exact discretisation for a linear
system driven by a piecewise-constant input — which is precisely what MPC
does, since it holds each `u` constant for one 0.05 s tick. This is more
accurate than simple Euler discretisation, which introduces `O(dt²)` error
per step.

Both `Ad` and `Bd` are computed simultaneously via one matrix exponential on
an augmented matrix, avoiding an explicit (and potentially ill-conditioned)
inversion of `A_c`:

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

#### Note on OSQP sparsity

All matrices start as `1e-12` (not exact `0.0`) rather than `np.zeros(...)`.
OSQP analyses which matrix entries are nonzero on its *first* solve and
caches that pattern for speed. If a later solve produces an entry that
rounds exactly to zero where it was previously nonzero (which can happen as
`vx` changes and terms like `1/vx` shrink), OSQP's cached factorisation
becomes invalid and it throws a reallocation error. Filling every entry
with a tiny nonzero epsilon keeps the sparsity pattern — the *set* of
nonzero locations — identical at every speed, so OSQP never needs to
re-analyse it mid-run.

### The cost function and QP (`optimiser.py`)

Each solve minimises, over the predicted `N`-step horizon:

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
particular quantity relative to the others (see
[Tuning Guide](#tuning-guide) below and the comments in `settings.py` for
what each entry means practically). They're expressed and injected as
square roots (`sqrtQ`, `sqrtR`, `sqrtR_rate`) so the cost can be written with
`cp.sum_squares`, which CVXPY maps efficiently onto OSQP's internal
quadratic-cost matrix — this is a numerical-stability/implementation choice,
not a change in what's being penalised (`‖√w·x‖² = w·x²`).

**Why states 5-7 (`e_a`, `delta_act`, `a_act`) are never tuned:** only the
first 5 diagonal entries of `Q` (`e_y` through `e_v`) and both entries of
`R`/`R_rate` are exposed to the offline tuner (`TUNABLE_Q_IDX = [0,1,2,3,4]`
in `offline_tuner.py`). `Q[5,5]` (`e_a`) stays at 0 because that state is a
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

**Primary: OSQP.** Exploits the QP's sparsity, supports warm-starting
(reusing the previous tick's solution as the starting guess — since
consecutive MPC solves differ by only one step in a receding horizon, this
converges in ~50-200 iterations instead of 500-2000 cold), and typically
solves in 1-5 ms at `N=25`.

**Fallback: Clarabel.** A slower but more numerically robust interior-point
solver, used only if OSQP returns a non-optimal status (infeasible,
unbounded, numerical trouble, or hit its iteration cap).

**If both fail**, the simulator/tuner returns `None` and the caller holds
the previous command; the live `control_utils.MPCController` instead
returns a full-brake command (`[u_prev[0], -a_max_brake]`) — braking is the
safer default for a real vehicle than continuing to coast on a stale plan.

**`OPTIMAL_INACCURATE`** (OSQP converged, but not to full tolerance) is
still accepted and used — refusing it and holding the previous command
would generally be worse than using a slightly-under-converged-but-still-
reasonable solution at 20 Hz. The offline tuner counts these occurrences and
applies a scoring penalty (see [The Composite Score](#the-composite-score))
so weight sets that cause frequent `OPTIMAL_INACCURATE` are still
discouraged, without discarding the run outright.

### Adaptive gain scheduling (`model_utils.py`)

The tuned `Q`, `R`, `R_rate` weights are optimised as if for a single
"average" operating point. Two functions rescale `R` and `R_rate` *every
tick* to compensate for known, predictable ways the required control
authority changes with speed and curvature — without needing a separate
tuned weight set for every regime.

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

**`adaptive_R_rate(kappa, R_rate)`** — softens the steering *jerk* penalty
in tight corners:

```
scale = max(0.35, 1 / (1 + 3·κ))       # → 1.0 at κ=0 (straight), → 0.35 floor at high κ
```

`κ` (curvature) is estimated causally from the plant's own current yaw rate
and speed (`curvature_estimate()`: `κ = |yaw_rate| / vx`) — it reflects the
curvature the car is *currently experiencing*, not a look-ahead of the path
geometry. In a straight, the full smoothness penalty applies (discourage
unnecessary steering jitter). In a tight corner, the penalty is floored at
35% of base rather than removed entirely — enough softening to let the
controller make the fast steering changes a tight corner demands, without
ever allowing the rate cost to vanish completely (which would permit
arbitrarily rapid, oscillatory steering).

Both functions return a **copy** of the base matrix — the tuned weights in
`settings.py` are never mutated, only scaled per-tick on top of.

### Where this is duplicated, and why

`control_utils.py`'s `MPCController` re-implements `_discrete_model`
(mirrors `bicycle_model.py`), `_adaptive_R_scaling`/`_adaptive_R_rate`
(mirrors `model_utils.py`), and `_build_qp` (mirrors `optimiser.py`'s
`init_parameterized_mpc`, including the same `±3.5 m` soft boundary,
`W_slack=10000`, and step-0/subsequent rate-cost split) as self-contained
local copies, rather than importing the shared modules. This is deliberate:
`control_utils.py` runs inside a ROS 2 node on the real/FSDS vehicle and
must have zero simulator dependencies. **Any change to the cost/constraint
structure in one location must be mirrored in the other**, or weights tuned
by `offline_tuner.py` will not transfer faithfully to the live controller.
`control_utils.py` additionally enforces a hard per-step slew-rate limit
(`self.du_max`) on top of the soft `R_rate` cost — a hardware-safety measure
not present in the simulator's QP, since the simulator's nonlinear plant
doesn't model an actuator that could be damaged by too-fast commands the way
real hardware could.

---
## How the Offline Tuner Works

`offline_tuner.py` searches for `Q`, `R`, `R_rate` cost weights automatically
rather than requiring hand-tuning, by running many closed-loop rollouts and
minimising a single scalar score. This section covers the search algorithm;
see [The Composite Score](#the-composite-score) for exactly what's being
minimised.

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
be biased toward the larger bound).

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
objective = 0.7 · weighted_mean(scores) + 0.3 · max(scores)
```

The 30% worst-case term exists specifically so CMA-ES can't find a weight
set that scores well *on average* by driving one corner shape perfectly and
another one badly — every task in the suite has to be reasonably good, not
just the average.

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

### The 12 metrics

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
| 9 | `steering_reversals` | Count of times the steering sign flips (beyond a 0.02 rad noise threshold) — penalises "hunting"/indecisive steering. |
| 10 | `peak_lateral_error` | The single worst `|e_y|` reached at any point — a safety-margin measure independent of the average. |
| 11 | `speed_rmse` | RMS of `v_actual - v_target` — how well the car tracks the planner's requested speed. |

### Combining into one score

```python
score = SCORE_WEIGHTS @ metrics                                # weighted sum of the 12 metrics
score -= COMPLETION_BONUS_WEIGHT * progress + TIME_BONUS_WEIGHT * time_bonus
if dnf:       score += DNF_PENALTY
if offtrack:  score += DNF_OFFTRACK_PENALTY
if inaccurate_count > 0:
    factor = min(5, inaccurate_count) * 0.1                     # capped at 50%
    score += abs(score) * factor
```

**Lower is always better.** A good finishing run typically scores in
`[-0.5, -0.3]` — negative because the completion/time bonuses usually
outweigh the (small, well-tuned) metric costs. `SCORE_WEIGHTS` is defined
once in `settings.py` and must sum to exactly `1.0` (enforced by an
assertion) so the relative weighting between metrics stays interpretable —
see [Configuring the Project](#configuring-the-project-settingspy) for
guidance on adjusting individual weights.

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
| `simulation.py` | Interactive matplotlib GUI — draw/load a path, run one closed-loop rollout, scrub through history, view metrics. Thin wrapper around `rollout_core.run_core_rollout(want_history=True)`. |
| `rollout_core.py` | The single shared closed-loop rollout loop used by both `simulation.py` and `offline_tuner.py`. Not GUI-safe to import from `simulation.py`'s multiprocessing workers, so it's split out into its own dependency-light module. |
| `scoring.py` | The single implementation of the 12-metric accumulation and composite score. See [The Composite Score](#the-composite-score). |
| `bicycle_model.py` | Builds the MPC's linear 8-state prediction model. See [How the MPC Works](#how-the-mpc-works). |
| `model_utils.py` | Runtime curvature/speed-based rescaling of `R`/`R_rate`. See [Adaptive gain scheduling](#adaptive-gain-scheduling-model_utilspy). |
| `optimiser.py` | The parameterised CVXPY/OSQP QP formulation and solve. See [The cost function and QP](#the-cost-function-and-qp-optimiserpy). |
| `vehicle_physics.py` | The 24-state nonlinear "truth" plant (Pacejka tyres, suspension, aero) that the MPC never observes directly — only through tracking error. See [Configuring the Vehicle](#configuring-the-vehicle-vehicle_physicspy). |
| `offline_tuner.py` | Headless CMA-ES weight search. See [How the Offline Tuner Works](#how-the-offline-tuner-works). Also exports the synthetic path library (`SYNTHETIC_PATHS`, `PATH_NAMES`) and the speed-keyed model cache (`get_cached_model`) used by both the tuner and the simulator. |
| `speed_profile.py` | Curvature-based per-point target speed (`compute_speed_profile`), with a moving-average smoothing pass (`smooth_profile`). Uses the friction-circle approximation `v = sqrt(a_lat_max / κ)` over a forward look-ahead window. |
| `sim_track.py` | Simulator-side mirrors of the real perception/planner nodes: `place_cones()` (static track layout), `SimPerception` (FOV filter), `SimPlanner` (cone accumulation → centreline + speed profile). |
| `performance_stats.py` | Scores a completed simulator run for the **Show Metrics** button by replaying its stored history through the exact same `scoring.RolloutMetrics` accumulator the tuner uses. Also exposes `benchmark_weights()` for **Benchmark All Paths**. |
| `manual_drive.py` | Standalone WASD/mouse drive mode against the 24-state nonlinear plant — no MPC, no scoring, purely open-loop human control for building intuition or sanity-checking a track. See [Manual Drive Mode](#manual-drive-mode). |
| `settings.py` | All project-level tuning/scoring/DNF configuration. See [Configuring the Project](#configuring-the-project-settingspy). |
| `control_node.py` / `control_utils.py` | The live ROS 2 MPC controller for FSDS. See [ROS 2 Integration](#ros-2-integration-fsds). |

---

## ROS 2 Integration (fsds)

To run the controller against the FSDS simulator, first obtain the
`fsae_planning` repo, then paste the contents of `control_node.py` and
`control_utils.py` into the matching files in its `track_utils` package.

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

**Control loop phases** (see `control_node.py::_control_loop`):

1. **Hold at start line** — full brake until the `/fsds/signal/go` signal is
   received.
2. **Stale-path emergency brake** — full brake, and `MPCController.reset()`,
   if no fresh path has arrived within `TARGET_TIMEOUT` (0.5 s) or the path
   has fewer than 2 points. The reset discards the QP's warm start and
   actuator-lag memory so the controller doesn't resume from stale state
   once the path returns.
3. **Normal MPC solve** — `MPCController.compute()`.
4. **Cone-proximity brake override** — hard-overrides throttle/brake (not
   steering) if a fused cone is inside a dynamic corridor directly ahead.
   After `CONE_RESET_THRESHOLD` (0.3 s) of continuous braking the controller
   is reset once (edge-triggered, re-armed once the brake clears).
5. **Telemetry logging** (optional, `LOG_DIR`) — logs the *final*,
   post-override command, so the CSV reflects what was actually sent to the
   vehicle.
6. **Publish.**

**Launching nodes with FSDS on Windows, bridged via WSL:**

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git --recurse-submodules
```

Configure the WSL IP in the ROS 2 bridge launch file:

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

**Building the ROS 2 nodes:**

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

## Manual Drive Mode

`manual_drive.py` is a small standalone app for driving the nonlinear plant
directly — useful for building intuition for the vehicle's handling limits,
eyeballing track/cone geometry, and generating a human reference trace to
compare against MPC runs on the same path. It shares the same 24-state
nonlinear plant and synthetic path library as the simulator, but is entirely
open-loop: no tracking error is computed, no MPC solve happens, and nothing
is scored.

**Run it:**

```bash
python manual_drive.py
```

**Controls:** `W`/`S` throttle/brake, `A`/`D` steer left/right, `SPACE` full
brake (overrides throttle). Inputs are rate-limited toward the key-held
target so taps feel analog rather than an on/off step.

**Workflow:** **Load Test Path** to cycle through the synthetic path
library and place cones → **Start Driving** to spawn the plant at the
path's start pose → drive → **Reset** to stop and clear the trail.

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | ≥1.24 | All numerical computation |
| `scipy` | ≥1.10 | ZOH discretisation (`expm`), spline fitting (`CubicSpline`) |
| `matplotlib` | ≥3.7 | Simulator/manual-drive GUI |
| `cvxpy` | ≥1.4 | MPC QP formulation |
| `osqp` | ≥0.6 | Primary QP solver (via CVXPY) |
| `clarabel` | ≥0.6 | Fallback QP solver (via CVXPY) |
| `cma` | ≥3.3 | CMA-ES optimiser (`fmin_lq_surr2`, BIPOP+surrogate) |
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `Track`, `ControlCommand`, `GoSignal` message types |
| `nav_msgs` | ROS 2 | `Odometry`, `Path` |
| `geometry_msgs` | ROS 2 | `PoseStamped`, `PointStamped` |

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
```

---

## Developer Guide

If you're extending the simulator or tuning the vehicle, follow these
guidelines to keep the MPC/plant architecture consistent.

### Modifying vehicle parameters

See [Configuring the Vehicle](#configuring-the-vehicle-vehicle_physicspy)
above. The short version: `VehicleParams` in `vehicle_physics.py` is the
single source of truth; if you import new Pacejka tyre data, you must also
recompute `Cf`/`Cr` to match its initial slope, or the MPC's internal model
will silently diverge from the plant it's controlling.

### Adding a new synthetic path

1. In `offline_tuner.py`, open `build_synthetic_paths()`.
2. Define your segments — `_make_arc(cx, cy, radius, start_deg, end_deg, n)`
   for constant-radius corners, `np.linspace()` for straights.
3. Concatenate the segment arrays and pass them through `_resample_path(wx, wy)`.
4. Add the resulting tuple to the `paths` dictionary under a new key.
5. *(Optional)* Add that key to `VALIDATION_SUITE` in `settings.py` if you
   want the tuner to optimise against it — see
   [Configuring the Project](#configuring-the-project-settingspy).

### Debugging solver failures

If the live simulator reports `consecutive_solver_failures` or the console
frequently shows `OPTIMAL_INACCURATE`:

- **Weight scaling** — OSQP is sensitive to poorly-conditioned matrices. If
  any entry of `Q`, `R`, or `R_rate` exceeds `1e4` or drops below `1e-4`,
  convergence can suffer. Check `adaptive_R_scaling()`'s output at your
  test speed isn't blowing up the steering cost unexpectedly.
- **Kinematic vs. dynamic gap** — if the car consistently fails at tight
  hairpins, `speed_profile.py` may be commanding a speed that demands more
  lateral force than the Pacejka friction circle can supply at that
  curvature. Lower `mu` in `compute_speed_profile()` to force more
  conservative corner-entry speeds.
- **Model-plant mismatch at extremes** — remember the MPC's internal model
  is linear and only blends kinematic/dynamic behaviour between 1-2.5 m/s;
  well outside that (very low speed under load, or very high lateral
  acceleration near the tyre limit) is where the biggest prediction error
  will show up, and where `adaptive_R_scaling`/`adaptive_R_rate` matter most.