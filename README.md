# fsae_MPCTest: offline MPC tuning and the FSDS staging mirror

This repo is the offline half of a Formula Student Driverless autonomy project. It tunes and tests the MPC path-tracking controllers (LTV-QP, called LMPC here, and NMPC) without FSDS running, and it stages the ROS 2 code that runs the same controllers inside [FSDS](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator), the AirSim and Unreal simulator.

- **Offline simulator**: a nonlinear 24-state vehicle plant driven by the controller, headless or through a 2D matplotlib GUI. It is a detailed approximation. It is not validated against FSDS or the real car.
- **Automatic tuner**: CMA-ES search over the cost weights, so they are not hand-tuned by trial and error.
- **Staging mirror**: `fsds_simulator/` holds a snapshot of the live `ros2/src/fsae_planning` workspace. It is where changes to that workspace are stored.

The project has two separate simulators that are easy to conflate: FSDS and this repo's offline simulator. Neither is a validated stand-in for the car. Do not trust an offline score on its own. See [glossary.md](docs/reference/glossary.md) and [simulator_fidelity.md](docs/reference/simulator_fidelity.md).

Start with the [documentation index](docs/README.md). Per-module references: [offline simulator](docs/modules/offline_sim.md) and [FSDS ROS 2 package](docs/modules/fsds_ros2.md).

## Repo map

| Path | Contents |
|---|---|
| `settings/` | All offline tuning, scoring and DNF constants, split into `general`, `noise`, `planner`, `lmpc`, `nmpc`, `solver` and `scoring`. `__init__.py` re-exports every name. Consumers use `import settings; settings.X` |
| `model/` | `bicycle_model.py` (the controller's linear prediction model) and `model/vehicle_physics/` (the 24-state plant: `state`, `params`, `tyres`, `plant_step`, `tracking`) |
| `controller/` | `lmpc/` (QP build and solve), `nmpc/` (Frenet-frame nonlinear MPC: layout, reference, dynamics, outputs, weight schedule, QP model, SQP step, solver) and `model_utils.py` (adaptive gains) |
| `sim/` | Closed-loop rollout in `sim/rollout/` (`core`, `reference`, `speed_target`, `delay`, `tick_solve`), plus `perception.py`, `planner.py`, `speed_profile.py`, `scoring.py`, `sensor_noise.py`, `track_io.py` |
| `planning/` | Shared planning code taken from `fsae_planning`: `boundary`, `cone_map`, `cone_sorting`, `path_utils`, `geometry` |
| `tracks/` | Python helpers for track paths (`TRACKS_DIR`, `newest_track()`, `DEFAULT_MAP`). Track data is not here. It lives in `ros2/src/fsae_planning/tracks/` |
| `tuner/` | `offline_tuner.py` (CMA-ES), `tuner/validation/` (the three checks below), `tuner/investigations/` (one-off diagnostic scripts), `tuner/tools/` (exporters, `plot_playback.py`, `sync_mpc_params.py`, `doc_lint.py`) |
| `gui/` | `gui/launcher/` (tabbed control app), `simulation.py` (2D simulator), `manual_drive.py` |
| `settings_profiles/` | Saved settings sets (JSON) loaded by the launcher's Profiles tab |
| `angles.py` | `wrap_angle` |
| `fsds_simulator/` | Staging mirror of the live ROS 2 workspace. See [its README](fsds_simulator/README.md) |
| `docs/` | Guides, controller docs, reference, FSDS docs and frozen logs. See the [index](docs/README.md) |

The repo expects to sit at `<FSDS repo root>/fsae_MPCTest/`, beside the outer repo's `ros2/` folder. The launcher and the track helpers find `ros2/launch_all.sh`, the live parameter files and the track data by that relative location.

## Quickstart

Install the dependencies:

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
pip install optuna   # optional, only for USE_OPTUNA_PRESEARCH
```

Run everything from the repo root.

### Launcher (main entry point)

```bash
python -m gui.launcher
```

A tabbed Tk app: launch the live simulation (including a record-a-new-track mode), debug a recorded log, run the offline simulator, edit `settings/` constants and manage profiles. See [debugging_tools.md](docs/guides/debugging_tools.md).

### 2D simulator

```bash
python -m gui.simulation
```

Load a test path or a recorded track, then start a closed-loop rollout. See [offline_guide.md](docs/guides/offline_guide.md).

### Validation checks

There is no test suite. These three scripts are the correctness bar for a change. Each is headless.

| Command | Run after changing | What it does |
|---|---|---|
| `python -m tuner.validation.recorded_map_rollout` | MPC weights, the plant model, scoring | Replays the recorded map and prints the metrics the sim-to-real comparison tables are quoted in (steering saturation, heading error, lateral acceleration) |
| `python -m tuner.validation.nmpc_offline_check` | `controller/nmpc/` | Solver self-consistency checks for the NMPC path |
| `python -m tuner.validation.plant_openloop_validation` | `model/vehicle_physics/` | Replays the two open-loop FSDS system-ID experiments through the plant and reports the residuals |

Passing them does not validate a change on the car. Validate in a full FSDS session or on the car. See [simulator_fidelity.md](docs/reference/simulator_fidelity.md).

### Offline tuner

```bash
python -m tuner.offline_tuner
```

Stop early with Ctrl+C and it reports the best weights so far. Results append to `docs/logs/tuning_history.txt`. See [tuning.md](docs/guides/tuning.md).

## Live and offline stay identical by hand

The MPC and planning stack exists twice: in `ros2/src/fsae_planning/` (live) and in this repo (offline). This repo cannot import the live code, so numeric parity is kept by hand. A change to weights, the plant model, delay handling, tracking-error maths or scoring must land on both sides. See [offline_live_parity.md](docs/reference/offline_live_parity.md).

## Documentation

- [docs/README.md](docs/README.md): index of every doc.
- [docs/modules/offline_sim.md](docs/modules/offline_sim.md) and [docs/modules/fsds_ros2.md](docs/modules/fsds_ros2.md): what each module does.
- [docs/guides/getting_started.md](docs/guides/getting_started.md): newcomer introduction to MPC, NMPC and tuning.
- [docs/fsds/integration_guide.md](docs/fsds/integration_guide.md): running against FSDS, recording and driving a track.
- [docs/logs/README.md](docs/logs/README.md): frozen investigation logs.

Upstream versions this work targets: FSDS release V2.2.0 (the Windows `.exe` used by `ros2/launch_all.sh` is `fsds-v2.2.0-windows`). Commit pins are not recorded here. Read `git log -1` in each repo instead.
