# FSAE MPC Path Tracking Simulator

A high-fidelity 2D closed-loop simulator and offline weight tuner for a Formula
Student autonomous vehicle. The system pairs a nonlinear 24-state vehicle plant
with a linear time-varying Model Predictive Controller (MPC), and provides
CMA-ES-based automated weight optimisation so the controller's cost weights
don't have to be hand-tuned by trial and error.

This repository also includes a ROS 2 control node (`mpc_controller_standalone.py`
/ `mpc_core.py`, staged under `fsds_simulator/` — see below) that runs the
same MPC live inside the
[FSDS](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)
simulator, by pasting it into the matching file in the
[fsae_planning](https://github.com/UOA-FSAE/fsae_planning) repo (see
[docs/planning_control_sync.md](docs/planning_control_sync.md) for the exact
file mapping — `mpc_controller_standalone.py` is a distinct node from
upstream's default `mpc_controller.py`). Weights tuned offline in this
project transfer directly to that live controller, because both preserve the
MPC's own throttle/brake output rather than routing speed through
`fsds_bridge.py`'s separate P-loop.

`fsds_simulator/` is a staging area, not a live module of this repo: its
subfolders mirror `fsae_planning`'s own ROS 2 package hierarchy exactly (e.g.
`fsds_simulator/control/fsae_control/fsae_control/mpc_core.py`), so a file
can be copied straight across to the matching path with no manual re-pathing.
Nothing under `fsds_simulator/` is imported by the simulator or tuner —
those live under `planning/`, `sim/`, `model/`, `controller/` instead.

The 2D simulator can optionally simulate the full perception + planning
pipeline (`USE_PLANNER` in `settings.py`) by placing cones along a path
(`sim_track.place_cones()`) and reconstructing a centreline from them using
the shared planning code in the `planning/` folder (taken from the
`fsae_planning` repo). When `USE_PLANNER` is off, the simulator instead tracks
the true reference path directly — faster, and useful for isolating driving
behaviour from planner behaviour. **Load Recorded Track** in `gui/simulation.py`
loads a real cone map recorded from a live FSDS lap (via `fsae_planning`'s
`cone_recorder` node) instead of a synthetic one — see
[Recording a track from FSDS](docs/developer_guide.md#recording-a-track-from-fsds).

fsds simulator repo: https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator (current implementation uses commit 59f03fa, and the V2.20 release)
fsae planning repo: https://github.com/UOA-FSAE/fsae_planning (current implementation uses commit 28dcd4d)

---

## Quick Start

### 1. Install dependencies

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
pip install optuna  # optional: only needed for USE_OPTUNA_PRESEARCH in settings.py
```

### 2. Launch the simulator

```bash
cd /path/to/project
python -m gui.simulation
```

Click **Load Test Path** to cycle through the built-in synthetic paths (or
**Load Recorded Track** to load a real cone map recorded from FSDS — see
below), then **Start Sim** to run a closed-loop MPC rollout. See
[docs/developer_guide.md](docs/developer_guide.md#running-the-simulator) for
the full walkthrough (drawing a path, initial-condition sliders, scoring a
run), and [docs/developer_guide.md](docs/developer_guide.md#running-the-offline-tuner)
for how to run the CMA-ES weight tuner instead.

---

## Documentation

| Doc | Covers |
|---|---|
| [docs/architecture.md](docs/architecture.md) | How the system works: the closed-loop architecture, `settings.py`/`model/vehicle_physics.py` configuration reference, the full MPC formulation (state vector, cost function, solver), how the offline CMA-ES tuner works, the composite scoring function, and a per-file module reference. |
| [docs/developer_guide.md](docs/developer_guide.md) | How to use and extend the project: running the simulator and offline tuner, FSDS/ROS 2 integration (including Windows/WSL/Docker setup from scratch), manual drive mode, dependencies, adding a new synthetic path, and debugging solver failures. |
| [docs/vehicle_physics_guide.md](docs/vehicle_physics_guide.md) | Plain-English walkthrough of the 24-state nonlinear plant in `model/vehicle_physics.py` (tyres, suspension, weight transfer, aero) for readers who don't already know vehicle dynamics. |
| [docs/planning_control_sync.md](docs/planning_control_sync.md) | Reference for re-syncing `planning/` and `fsds_simulator/` against a newer `fsae_planning` upstream clone: file mapping, deliberate non-mirrors, numeric-parity constants, and the resync procedure. |

---

## Key modules

| File | Purpose |
|---|---|
| `gui/simulation.py` | Interactive matplotlib GUI — draw/load a path, run one closed-loop rollout, scrub through history, view metrics. Also renders the live planner centreline (magenta) alongside the true target path when `USE_PLANNER=True`. |
| `sim/track_io.py` | Loads a recorded cone map (JSON, from `fsae_planning`'s `cone_recorder` node) into the same path/cones tuple shape as a synthetic path, for **Load Recorded Track**. |
| `tuner/offline_tuner.py` | Headless CMA-ES weight search across a library of synthetic corner shapes. |
| `sim/rollout_core.py` | The single shared closed-loop rollout loop used by both `gui/simulation.py` and `tuner/offline_tuner.py`. |
| `model/vehicle_physics.py` | `VehicleParams` — the single source of truth for vehicle physics (mass, geometry, tyres, suspension, aero, actuator limits). |
| `model/bicycle_model.py` / `controller/optimiser.py` / `controller/model_utils.py` | The MPC's linear prediction model, QP formulation/solve, and adaptive gain scheduling. |
| `settings.py` | All project-level tuning/scoring/DNF configuration. |
| `gui/manual_drive.py` | Standalone keyboard-driven test mode against the nonlinear plant. |
| `mpc_controller_standalone.py` / `mpc_core.py` / `control_utils.py` (staged under `fsds_simulator/control/fsae_control/fsae_control/`) | The live ROS 2 MPC controller for FSDS. |

See [docs/architecture.md#module-reference](docs/architecture.md#module-reference)
for the complete per-file index.

---

## Dependencies

Core stack: `numpy`, `scipy`, `matplotlib`, `cvxpy` (with `osqp` and
`clarabel` solvers), and `cma`. `optuna` is optional, only needed for the
offline tuner's TPE pre-search (`USE_OPTUNA_PRESEARCH` in `settings.py`).
ROS 2 nodes additionally need `rclpy`,
`fs_msgs`, `nav_msgs`, and `geometry_msgs`. See
[docs/developer_guide.md#dependencies](docs/developer_guide.md#dependencies)
for the full version/purpose table and FSDS/ROS 2 setup instructions.
