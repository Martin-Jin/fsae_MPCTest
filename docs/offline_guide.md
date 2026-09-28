# Offline Guide: 2D GUI, Tuner, Manual Drive

**This doc covers the offline (`fsae_MPCTest`) side only**: the 2D matplotlib GUI, the CMA-ES auto-tuner, manual drive mode, and offline dependencies/extension points. None of this runs against FSDS or the real car. For FSDS/live setup and integration, see [docs/fsds/fsds_integration_guide.md](fsds/fsds_integration_guide.md). For what "offline," "FSDS," and "2D GUI" mean and how they relate, see [docs/reference/simulator_glossary.md](reference/simulator_glossary.md).

For the deep technical explanation of *why* the system is built this way, see [Architecture](architecture.md). For diagnostic/debugging tools, see [debugging_tools.md](debugging_tools.md).

## Table of Contents

1. [Running the 2D GUI](#running-the-2d-gui)
2. [Running the Offline Tuner](#running-the-offline-tuner)
   - [The tuner/ layout at a glance](#the-tuner-layout-at-a-glance)
   - [Plotting and scrubbing exported CSV telemetry](#plotting-and-scrubbing-exported-csv-telemetry)
3. [Manual Drive Mode](#manual-drive-mode)
4. [Dependencies](#dependencies)
5. [Extending and Debugging](#extending-and-debugging)
   - [Modifying vehicle parameters](#modifying-vehicle-parameters)
   - [Adding a new synthetic path](#adding-a-new-synthetic-path)
   - [Working with the NMPC (`USE_NMPC`)](#working-with-the-nmpc-use_nmpc)

---

## Running the 2D GUI

The 2D GUI (`gui/simulation.py`) is an interactive matplotlib tool for drawing or loading a path, running one closed-loop MPC rollout against the nonlinear vehicle plant, and reviewing the result frame by frame. **Its own dynamics do not match FSDS or the real car**; it's for visualization and manual prototyping, not a validated prediction of live behaviour (see [simulator_glossary.md](reference/simulator_glossary.md)).

### 1. Install dependencies

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
```

### 2. Launch

```bash
cd /path/to/project
python -m gui.simulation
```

Or launch it (plus the live sim, log playback, and settings.py editing) from one place: `python -m gui.launcher` — see [debugging_tools.md](debugging_tools.md#centralized-launcher-guilauncherpy).

### 3. Get a path onto the map

Either:

- **Draw one**, click and drag on the map (at least 6 points). On release the path is automatically splined, headings computed, and a speed profile generated.
- **Load a synthetic one**, click **Load Test Path** to cycle through the 10 built-in FS-spec paths (`PATH_SUDDEN_TURN`, `PATH_S_BEND`, `PATH_SPIRAL`, `PATH_MICRO_SLALOM`, `PATH_OFFSET_CHICANE`, `PATH_ACCELERATION`, `PATH_HAIRPIN`, `PATH_CHICANE`, `PATH_FS_CORNER`, `PATH_MIXED`). Each click advances to the next path; the camera auto-frames around it with a 15 m margin.
- **Load a recorded track**, click **Load Recorded Track** to cycle (newest-first) through `tracks/*/cone_map.json` (and, for captures predating that layout, `fsds_simulator/cone_maps/*.json`), the cone maps written by `fsae_planning`'s `cone_recorder` ROS 2 node after a live FSDS lap (see [Recording, exporting and driving a track](fsds/fsds_integration_guide.md#recording-exporting-and-driving-a-track), an FSDS/live workflow with one offline step in the middle). Unlike the synthetic paths, the blue/yellow cones rendered are the *actual recorded cones*, not `place_cones()` output, a real perception recording, resimulated exactly as `SimPerception`/`SimPlanner` would drive it live (see [Simulated Perception and Planning](architecture.md#simulated-perception-and-planning-use_planner) for how those two work). The centreline drawn on load is only a reconstruction for the oracle-mode reference path and initial camera framing (see `sim/track_io.py`). With `USE_PLANNER = True`, the actual driving line during the rollout instead comes from `SimPlanner` rebuilding it cone-by-cone, exactly as for a synthetic path, but `USE_PLANNER = False` is the default, so by default the rollout tracks this reconstructed oracle path/speed profile directly, matching the live ROS side's `path_map_path` mode.

### 4. (Optional) set initial conditions

Once a path exists, two sliders appear:

- **Initial Lat Error** (±4 m), starts the car offset sideways from the path.
- **Initial Yaw Error** (±30°), starts the car pointing the wrong way.

Useful for stress-testing recovery behaviour rather than always starting perfectly on-line.

### 5. Run it

Click **Start Sim**. The rollout runs synchronously (no live animation while it solves, this can take a few seconds for a long path). When it finishes, the title turns green and a **Time** scrub slider appears below the map.

### 6. Review the run

Drag the **Time** slider to replay the run frame by frame. The trail, the cyan MPC horizon prediction, the car marker, and the telemetry panel (speed, position, heading, tracking errors, steering/accel commands) all update together.

### 7. Score it

Click **Show Metrics** to print a full 13-metric breakdown to the console (see [Composite Score](architecture.md#the-composite-score) below) and show a one-line summary in the plot title. Click **Benchmark All Paths** to run every synthetic path 3× each with the currently loaded weights and print a per-path score table, useful for checking a weight set generalises rather than only working on whichever single path was tested.

### 8. Reset

Click **Reset Environment** to clear everything and start over.

---

## Running the Offline Tuner

The offline tuner (`tuner/offline_tuner.py`) automatically searches for `Q`, `R`, `R_rate` cost weights that minimise the [composite score](architecture.md#the-composite-score) across a library of synthetic corner shapes, using CMA-ES (see [How the Offline Tuner Works](architecture.md#how-the-offline-tuner-works) for the algorithm itself). It has no GUI. It's a long-running batch job left to finish on its own.

**A tuned result is a starting point, not a validated one.** The tuner scores candidates against the headless rollout (`sim/rollout_core.py` driving `model/vehicle_physics.py`), which is rough validation only: it checks the control math behaves sensibly and gets weights into the right ballpark, it is not matched against FSDS or the real car and carries no measured accuracy figure. A weight set found here still needs validation against FSDS and, ultimately, the real car (see [simulator_glossary.md](reference/simulator_glossary.md) and [simulator_fidelity.md](reference/simulator_fidelity.md)) before it's trusted.

### 1. Install dependencies

Same as the 2D GUI (see above). `tuner/offline_tuner.py` uses the same `cvxpy`/`osqp`/`clarabel`/`cma` stack, plus Python's built-in `multiprocessing` to spread rollouts across CPU cores.

### 2. Check `settings.py` first

Before running, confirm:

- `VALIDATION_SUITE` lists the corner shapes the tuner should optimise for (see [Configuring the Project](architecture.md#configuring-the-project-settingspy)).
- `MAX_EVALS` is set to an acceptable budget to wait for (a good run is 20 minutes to a few hours depending on core count and `MAX_EVALS`).
- `USE_PLANNER` reflects whether the tuner should test the full perception/planning pipeline (`True`, see [Simulated Perception and Planning](architecture.md#simulated-perception-and-planning-use_planner)) or drive on the perfect reference line (`False`, the default, also faster).
- The `Q_diag`/`R_diag`/`R_rate_diag` cost weights and `SCORE_WEIGHTS`/ `METRIC_SCALES` the tuner optimises against, see [tuning.md](tuning.md) for what each one does and how to tune it.
- `USE_OPTUNA_PRESEARCH` (default `True`), set `False` to skip the short Optuna TPE search that runs before CMA-ES starts and seeds its starting point, falling back instead to the fixed geometric midpoint (see [Optional Optuna TPE pre-search](architecture.md#optional-optuna-tpe-pre-search)). Requires `optuna` to be installed (see [Dependencies](#dependencies)).

### 3. Launch

```bash
cd /path/to/project
python -m tuner.offline_tuner
```

This uses all available CPU cores minus one (one is left free for the OS). If `USE_OPTUNA_PRESEARCH` is enabled, the Optuna TPE pre-pass runs first and prints one line per trial, then a short summary before CMA-ES begins:

```
[Optuna TPE] trial   12/ 375 | score 0.3120 | best 0.1896
[Offline Tuner] Optuna pre-pass done in 8.42 min | trials run: 375/375 | best score: 0.1896
  x0 (Optuna-seeded): [2.451, 0.873, 4.201, 1.05, 0.612, 3.31, 0.774, 2.9, 5.14]
```

CMA-ES then starts from that seeded point instead of the fixed midpoint. Progress prints once per CMA-ES generation:

```
[lq-CMA-ES] gen    5 | true_evals    90 | gen_best 0.2341 | overall_best 0.1892 | sigma 6.123e-01
```

`gen_best` is this generation's best score; `overall_best` is the best score seen so far across the whole run; `sigma` is CMA-ES's current search-radius (shrinks as it converges). Lower scores are better throughout.

It is safe to stop early with **Ctrl+C**. The tuner finishes its current generation, then reports the best weights found so far rather than exiting uncleanly.

### 4. Read the result

On completion (or early stop), the tuner prints the best weight arrays found:

```
Replace your gui/simulation.py weights with:
Q_diag      = [9.35, 22.2, 18.9, 49.8, 10.8, 0.0, 0.0, 0.0]
R_diag      = [49.3, 45.4]
R_rate_diag = [50.0, 49.6]
```

It also prints a list of "improvement milestones", the point in the search (by true-evaluation count) at which each meaningfully better score was found, showing how much of the run's time was productive.

### 5. Apply the weights

Copy the values into **both**:

- `settings.py`: `Q_diag`, `R_diag`, `R_rate_diag` (used by `gui/simulation.py` and, from there, everything that imports them)
- `mpc_params.py` (`ros2/src/fsae_planning/control/fsae_control/fsae_control/`, staged under `fsds_simulator/`), the matching individual fields on the `MPCParams` dataclass (`q_e_y`, `q_e_yd`, `q_e_psi`, `q_r`, `q_e_v`, `r_delta`, `r_a_accel`/`r_a_brake`, `r_rate_delta`, `r_rate_a`). `mpc_core.py` builds its own `Q_diag`/`R_diag`/`R_rate_diag` from `self.params.*` at `MPCController.__init__` time, with no hardcoded weights, so `mpc_params.py` is the file to edit, not `mpc_core.py` itself.

Both must stay in sync manually. The tuner runs against the same plant and horizon used by both, but there is no single shared import between them (the live ROS 2 node has no simulator dependencies). See [`docs/reference/`](reference/)'s "MPC weight/gain parity" table for the full field-by-field mapping.

### 6. Log the result

Every run appends its result to `tuning_history.txt` automatically (timestamp, weight diagonals, duration, tuner score, git commit hash). Go back and manually fill in the `Overall score` field once you've tested the weights in FSDS or on the real car. The offline tuner score alone doesn't perfectly predict real-world performance, so this file is where the two get reconciled over time. See existing entries in `tuning_history.txt` for the expected format.

### Key constants to adjust

All of these live in `settings.py`, not `tuner/offline_tuner.py`, see the next section for what each one does and how much to change it by:

```python
MAX_EVALS         # Total true rollout budget (surrogate reduces actual count ~3-10x)
VALIDATION_SUITE  # Which synthetic corner shapes the tuner scores against
```

`sigma0` (CMA-ES's initial search radius) and `max_restarts` (BIPOP restart budget) are algorithm-internal tuning knobs rather than project settings. They're set near the bottom of `tuner/offline_tuner.py`'s `__main__` block if you need to adjust them; see [How the Offline Tuner Works](architecture.md#how-the-offline-tuner-works) for what they control.

### The tuner/ layout at a glance

`tuner/` has grown past the offline weight search it started as, it now holds the CMA-ES tuner, its benchmark/scoring companion, shared CSV-parsing helpers, reusable standalone tools, and a library of one-off/reusable sim-to-real investigation scripts. Three tiers:

**`tuner/` root, core infra, imported by the other two tiers:**

| File | Purpose |
|---|---|
| `offline_tuner.py` | CMA-ES weight search, see [Running the Offline Tuner](#running-the-offline-tuner). |
| `performance_stats.py` | Scoring/benchmarking a fixed weight set across `VALIDATION_SUITE`. |
| `csv_log.py` | Shared CSV parsing helpers (comment-header stripping, malformed-row filtering, column loading) used by every script below that reads a telemetry CSV. |
| `recorded_map_rollout.py` | Headless rollout baseline against the default recorded map (`comp_test_map_3`), the shared "run the sim against this map" entry point `tuner/checks/` scripts build on. |

**`tuner/tools/`, reusable standalone tools:**

| File | Purpose |
|---|---|
| `plot_playback.py` | Time-scrubbing map/telemetry viewer, see [debugging_tools.md](debugging_tools.md#telemetry-playback-tunertoolsplot_playbackpy). |
| `export_speed_profile.py` | Exports a recorded cone map's oracle path + speed profile to CSV, see [Export the speed profile and raceline](fsds/fsds_integration_guide.md#2-export-the-speed-profile-and-raceline-offline-fsae_mpctest) in the FSDS integration guide. |
| `raceline_optimizer.py` | Minimum-time racing line optimiser, same CSV output, see the same section above. `--mode centerline` exports the centreline instead. |
| `doc_lint.py` | Flags docs that break this project's writing conventions, see [debugging_tools.md](debugging_tools.md#doc-conventions-tunertoolsdoc_lintpy). |

**`tuner/checks/`, one-off and reusable diagnostic scripts from sim-to-real debugging**, plus a few similar scripts at `tuner/` root (`steering_chatter_check.py`, `reference_heading_geometry_check.py`, `reference_excess_mechanism_check.py`, `nmpc_offline_check.py`, `recorded_map_rollout.py`). See [debugging_tools.md](debugging_tools.md#which-tool-for-which-question) for what question each answers and how to run it, and [docs/logs/sim_to_real_investigation.md](logs/sim_to_real_investigation.md) for the investigation narrative behind them.

### Plotting and scrubbing exported CSV telemetry

`tuner/tools/plot_playback.py` turns one or more of the control CSVs from a live/FSDS run (see [CSV telemetry logging](fsds/fsds_integration_guide.md#csv-telemetry-logging) in the FSDS integration guide) into an interactive, time-scrubbing map/telemetry viewer. See [debugging_tools.md](debugging_tools.md#telemetry-playback-tunertoolsplot_playbackpy) for the full usage, flags, and auto-search-folder behaviour.

---

## Manual Drive Mode

`gui/manual_drive.py` is a small standalone app for driving the nonlinear plant directly, useful for building intuition for the vehicle's handling limits, eyeballing track/cone geometry, and generating a human reference trace to compare against MPC runs on the same path. It shares the same 24-state nonlinear plant and synthetic path library as the 2D GUI, but is entirely open-loop: no tracking error is computed, no MPC solve happens, and nothing is scored.

**Run it:**

```bash
python -m gui.manual_drive
```

**Controls:** `W`/`S` throttle/brake, `A`/`D` steer left/right, `SPACE` full brake (overrides throttle). Inputs are rate-limited toward the key-held target so taps feel analog rather than an on/off step.

**Workflow:** **Load Test Path** to cycle through the synthetic path library and place cones → **Start Driving** to spawn the plant at the path's start pose → drive → **Reset** to stop and clear the trail.

---

## Dependencies

| Package | Version | Purpose |
|---|---|---|
| `numpy` | ≥1.24 | All numerical computation |
| `scipy` | ≥1.10 | ZOH discretisation (`expm`), spline fitting (`CubicSpline`) |
| `matplotlib` | ≥3.7 | 2D GUI / manual-drive GUI |
| `cvxpy` | ≥1.4 | MPC QP formulation |
| `osqp` | ≥0.6 | Primary QP solver (via CVXPY) |
| `clarabel` | ≥0.6 | Fallback QP solver (via CVXPY) |
| `cma` | ≥3.3 | CMA-ES optimiser (`fmin_lq_surr2`, BIPOP+surrogate) |
| `optuna` | ≥4.0 | Optional TPE pre-search that seeds CMA-ES's starting point (`tuner/offline_tuner.py`, only needed if `USE_OPTUNA_PRESEARCH = True` in `settings.py`) |
| `rclpy` | ROS 2 Humble+ | ROS 2 nodes only |
| `fs_msgs` | FSDS | `ControlCommand`, `GoSignal` message types |
| `fsae_interfaces` | `fsae_planning` | `ConeDetection` (cone-proximity brake input) |
| `nav_msgs` | ROS 2 | `Odometry` |
| `geometry_msgs` | ROS 2 | `Pose`, `PoseArray` |

```bash
pip install numpy scipy matplotlib cvxpy cma
pip install cvxpy[osqp] cvxpy[clarabel]
pip install optuna  # optional: only needed for USE_OPTUNA_PRESEARCH in settings.py
```

---

## Extending and Debugging

These guidelines keep the MPC/plant architecture consistent when extending the offline simulator or tuning the vehicle.

### Modifying vehicle parameters

See [Configuring the Vehicle](architecture.md#configuring-the-vehicle-modelvehicle_physicspy) above. The short version: `VehicleParams` in `model/vehicle_physics.py` is the single source of truth; importing new Pacejka tyre data also requires recomputing `Cf`/`Cr` to match its initial slope, or the MPC's internal model will silently diverge from the plant it's controlling.

### Adding a new synthetic path

1. In `tuner/offline_tuner.py`, open `build_synthetic_paths()`.
2. Define your segments, `_make_arc(cx, cy, radius, start_deg, end_deg, n)` for constant-radius corners, `np.linspace()` for straights.
3. Concatenate the segment arrays and pass them through `_resample_path(wx, wy)`.
4. Add the resulting tuple to the `paths` dictionary under a new key.
5. *(Optional)* Add that key to `VALIDATION_SUITE` in `settings.py` if you want the tuner to optimise against it, see [Configuring the Project](architecture.md#configuring-the-project-settingspy).

### Working with the NMPC (`USE_NMPC`)

To try the nonlinear controller during development, flip `settings.USE_NMPC = True` and re-run any tuner/rollout script, `run_core_rollout()` takes `use_nmpc` explicitly, so nothing else needs to change.

Before trusting a result, run `python -m tuner.nmpc_offline_check`: it re-verifies model parity, Jacobians, SQP convergence, and a closed-loop LTV-QP-vs-NMPC A/B on every call, so a broken change fails loudly instead of silently degrading a tuning run.

If the LTV-QP's solver fails (`consecutive_solver_failures`, `OPTIMAL_INACCURATE`) or the NMPC's SQP misbehaves (non-improving steps, oscillation), see [debugging_tools.md](debugging_tools.md#debugging-solver-failures) for the checklist, plus, for the NMPC specifically, `nmpc_solve_budget_ms`/ `nmpc_sqp_iters` too tight for the horizon, or a weight override (`NMPC_Q_E_Y` etc. in `settings.py`, `-1` inherits from the base weight) pushing the cost badly out of scale. See `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section for the model and weight-mapping details, and `tuning.md` §4.5d for the tuning surface.
