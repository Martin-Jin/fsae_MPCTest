# FSAE MPC Path Tracking Simulator

A high-fidelity 2D closed-loop simulator and offline weight tuner for a Formula
Student autonomous vehicle. The system pairs a nonlinear 24-state vehicle plant
with a Model Predictive Controller, and provides CMA-ES-based automated weight
optimisation so the controller's cost weights don't have to be hand-tuned by
trial and error.

There are **two interchangeable MPC implementations**, selected by a single
flag (`use_nmpc` in `settings.py` / the live ROS 2 node's launch args):

- **LTV-QP** (default) — `mpc_core.MPCController`, a linear time-varying MPC
  solved as one convex QP per tick.
- **NMPC** — `nmpc_core.NMPCController`, a Frenet-frame nonlinear MPC that
  tracks arc length as a state and looks up path curvature directly, closing
  a structural blind spot the LTV-QP's linear prediction has no term for.

See [docs/architecture.md](docs/architecture.md)'s "Second controller:
nonlinear MPC" section for the full comparison and why the NMPC exists.

This repository also includes a ROS 2 control node (`mpc_controller_standalone.py`
/ `mpc_core.py`, staged under `fsds_simulator/` — see below) that runs the
same MPC live inside the
[FSDS](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator)
simulator, by pasting it into the matching file in the
[fsae_planning](https://github.com/UOA-FSAE/fsae_planning) repo (see
[`docs/reference/`](`docs/reference/`) for the exact
file mapping — `mpc_controller_standalone.py` is a distinct node from
upstream's default `mpc_controller.py`). Weights tuned offline in this
project transfer directly to that live controller, because both preserve the
MPC's own throttle/brake output rather than routing speed through
`fsds_bridge.py`'s separate P-loop.

`fsds_simulator/` is a staging area, not a live module of this repo: it
mirrors `fsae_planning`'s entire ROS 2 workspace — every package
(`fsae_interfaces`, `fsae_bringup`, `fsae_sim_perception`, `fsae_planning`,
`fsae_control`), including build scaffolding, not just the MPC-relevant
files — at the exact same relative paths, so someone with only this repo and
FSDS can build and run the full stack (Stanley, `mpc`, or `mpc_standalone`)
with no separate `fsae_planning` checkout. See
[fsds_simulator/README.md](fsds_simulator/README.md) for build/run steps.
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
`cone_recorder` node) instead of a synthetic one. Recorded tracks, and their
exported speed/raceline CSVs, live one-per-directory under `tracks/<name>/`
— physically inside the separate `fsae_planning` repo (`tracks/__init__.py`
here just points at it), so FSDS + `fsae_planning` alone can drive any
already-recorded track with no `fsae_MPCTest` checkout. See
[Recording, exporting and driving a track](docs/developer_guide.md#recording-exporting-and-driving-a-track)
for the full record → export → drive workflow and how to switch which track
the live car uses.

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
| [docs/architecture.md](docs/architecture.md) | How the system works: the closed-loop architecture, `settings.py`/`model/vehicle_physics.py` configuration reference, the full MPC and NMPC formulations (state vector, cost function, solver), how the offline CMA-ES tuner works, the composite scoring function, and a per-file module reference. |
| [docs/tuning.md](docs/tuning.md) | Practical tuning reference: what each `Q`/`R`/`R_rate` weight and adaptive-gain constant actually does, the corner-factor scheduler's fields, and how to change/validate a weight set — the "which knob, and why" companion to architecture.md's "how it works." |
| [docs/developer_guide.md](docs/developer_guide.md) | How to use and extend the project: running the simulator and offline tuner, FSDS/ROS 2 integration (including Windows/WSL/Docker setup from scratch), manual drive mode, dependencies, adding a new synthetic path, recording/exporting a track, and debugging solver failures. |
| [docs/reference/reference_path_and_speed.md](docs/reference/reference_path_and_speed.md) | Where the car drives and how fast: the three-pass speed profile, the raceline/centreline exporters, which exported file supplies speed vs geometry, and how to switch either. |
| [docs/vehicle_physics_guide.md](docs/vehicle_physics_guide.md) | Plain-English walkthrough of the 24-state nonlinear plant in `model/vehicle_physics.py` (tyres, suspension, weight transfer, aero) for readers who don't already know vehicle dynamics. |
| [`docs/reference/`](`docs/reference/`) | Reference for re-syncing `planning/` and `fsds_simulator/` against a newer `fsae_planning` upstream clone: file mapping, deliberate non-mirrors, numeric-parity constants, and the resync procedure. |
| [docs/junior_project_mpc_docs.md](docs/junior_project_mpc_docs.md) | Standalone high-level wiki page introducing MPC/NMPC/tuning to a newcomer with no prior control-theory background — self-contained by design, so it overlaps other docs rather than just linking to them. |
| [docs/fsae_planning_pending_pr.md](docs/fsae_planning_pending_pr.md) | Running summary of uncommitted changes in the live `fsae_planning` checkout, for PR purposes — mirrors that repo's own `CHANGES.md`. |
| [docs/logs/](docs/logs/) | Append-only research/investigation logs (sim-to-real gap, late-turn/lookahead history, NMPC introduction) — historical record, not current-state reference; see each file's own header for scope. |
| [fsds_simulator/README.md](fsds_simulator/README.md) | How to build and run the full ROS 2 stack (Stanley / MPC / MPC-standalone) from `fsds_simulator/` alone, plus FSDS — no separate `fsae_planning` checkout needed. |

---

## Key modules

| File | Purpose |
|---|---|
| `gui/simulation.py` | Interactive matplotlib GUI — draw/load a path, run one closed-loop rollout, scrub through history, view metrics. Also renders the live planner centreline (magenta) alongside the true target path when `USE_PLANNER=True`. |
| `sim/track_io.py` | Loads a recorded cone map (JSON, from `fsae_planning`'s `cone_recorder` node) into the same path/cones tuple shape as a synthetic path, for **Load Recorded Track**. |
| `tuner/offline_tuner.py` | Headless CMA-ES weight search across a library of synthetic corner shapes. |
| `tuner/tools/plot_playback.py` | Time-scrubbing map/telemetry viewer for one or more exported control-telemetry CSVs — signals + slider on the left (one line per signal per run, with a per-run checkbox when comparing multiple), full trajectory and live planner-path overlay top right, zoomed current-section view (with live e_y/e_psi) bottom right. Each run gets its own colour, consistent across all panels. Run with no args to auto-load and overlay every CSV dropped into `fsds_simulator/recorded_runs/` (`--latest-only` for just the newest). See [docs/developer_guide.md#plotting-and-scrubbing-exported-csv-telemetry](docs/developer_guide.md#plotting-and-scrubbing-exported-csv-telemetry). |
| `sim/rollout_core.py` | The single shared closed-loop rollout loop used by both `gui/simulation.py` and `tuner/offline_tuner.py`. |
| `model/vehicle_physics.py` | `VehicleParams` — the single source of truth for vehicle physics (mass, geometry, tyres, suspension, aero, actuator limits). |
| `model/bicycle_model.py` / `controller/optimiser.py` / `controller/model_utils.py` | The MPC's linear prediction model, QP formulation/solve, and adaptive gain scheduling. |
| `settings.py` | All project-level tuning/scoring/DNF configuration. |
| `gui/manual_drive.py` | Standalone keyboard-driven test mode against the nonlinear plant. |
| `mpc_controller_standalone.py` / `mpc_core.py` / `control_utils.py` (staged under `fsds_simulator/control/fsae_control/fsae_control/`) | The live ROS 2 MPC controller for FSDS. |
| `fsds_simulator/` | Full staging mirror of the live ROS 2 workspace (all packages, not just control) — see [fsds_simulator/README.md](fsds_simulator/README.md). |

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
