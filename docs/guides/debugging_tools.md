# Debugging Tools

Catalogue of the diagnostic tools in this repo and the outer `ros2/` scripts: which question each answers and how to run it. For why a tool was built and what it found, see [docs/logs/](../logs/README.md). This guide covers current usage only.

Run every `python -m` command from `fsae_MPCTest/` so `tuner`, `gui` and `settings` resolve as packages.

## Which tool answers which question

| Tool | Question it answers | Run with |
|---|---|---|
| `gui/launcher/` | What is the fastest way to launch the sim, debug a log, run the offline sim or retune a weight without editing a script? | `python -m gui.launcher` |
| `live_viz` | Which tracking error or control-law term is driving steering and throttle right now? | started by `ros2/launch_all.sh`, or `ros2 run fsae_control live_viz` |
| `tuner/validation/recorded_map_rollout.py` | Does a plant or weight change still reproduce the closed-loop baseline on the recorded map? | `python -m tuner.validation.recorded_map_rollout` |
| `tuner/validation/plant_openloop_validation.py` | Does the offline plant reproduce FSDS's measured open-loop behaviour? | `python -m tuner.validation.plant_openloop_validation [--ab] [--robustness]` |
| `tuner/validation/nmpc_offline_check.py` | Is the offline NMPC still internally consistent? | `python -m tuner.validation.nmpc_offline_check` |
| `test/nmpc_offline_check.py` in `fsae_control` | The same for the live NMPC port. | `python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py` |
| `tuner/investigations/live_vs_sim_diagnostics.py` | Where does a live run diverge from an offline run in speed-tracking error and saturation-episode structure? | `python -m tuner.investigations.live_vs_sim_diagnostics` |
| `tuner/investigations/analyze_adaptive_log.py` | Which adaptive feature fired in each corner of a live log, and was the corner achievable at that speed? | `python -m tuner.investigations.analyze_adaptive_log <control_csv>` |
| `tuner/investigations/steering_response.py` | What understeer coefficient and full-lock deficit does a live control CSV imply? | `python -m tuner.investigations.steering_response <control_csv> [...]` |
| `tuner/investigations/steering_sysid_analysis.py` | Which mechanism explains FSDS's steering-to-yaw response across speed (sustained cornering)? | `python -m tuner.investigations.steering_sysid_analysis <log.csv>` |
| `tuner/investigations/steering_step_analysis.py` | Is the yaw cap a hard limit, scaled authority or active damping (step-input transient)? | `python -m tuner.investigations.steering_step_analysis <log.csv>` |
| `tuner/investigations/brake_sysid_analysis.py` and `ros2/run_brake_sysid.sh` | What deceleration does FSDS deliver for a given brake command and speed? | `cd ros2 && ./run_brake_sysid.sh` |
| `tuner/tools/plot_playback.py` | What did this run, or these runs compared, do signal by signal and on the map? | `python -m tuner.tools.plot_playback [csv ...]` |
| `tuner/investigations/steering_chatter_check.py` | Does a weight or setting change make tick-to-tick steering chatter better or worse? | `python -m tuner.investigations.steering_chatter_check [--controller nmpc\|ltv] [--set NAME=VALUE ...]` |
| `tuner/investigations/reference_heading_geometry_check.py` | Is a reference-heading swing caused by the online planner's rebuild, or by the track geometry itself? | `python -m tuner.investigations.reference_heading_geometry_check` |
| `tuner/investigations/reference_excess_mechanism_check.py` | Are fast-reference-heading ticks explained by the boundary planner's seed-midpoint anchor jump? | `python -m tuner.investigations.reference_excess_mechanism_check` |
| `tuner/investigations/ref_heading_limiter_ab.py` and `ref_heading_limiter_suite_check.py` | Does the reference-heading rate limit help or hurt, on one map or across the validation suite? | `python -m tuner.investigations.ref_heading_limiter_ab` and `python -m tuner.investigations.ref_heading_limiter_suite_check` |
| `ros2/clock_drift_check.py` | Is FSDS's simulation clock falling behind wall time, separate from message-delivery timing? | started by `ros2/launch_all.sh`, or `python3 clock_drift_check.py <output_csv_path>` |
| `ros2 topic hz` capture block in `ros2/launch_all.sh` | Are odom, `/clock`, the SLAM pose and the IMU arriving at their expected rates? | started by `ros2/launch_all.sh`, logs to `fsae_logs/topic_hz_diagnostics/` |
| `tuner/tools/sync_mpc_params.py` | After a param change is live-tested, are `fsae_autonomous` and the `fsds_simulator` mirror still on the old values? | `python -m tuner.tools.sync_mpc_params [--apply]`, or the Settings tab's "Overwrite All Params..." button |
| `tuner/tools/doc_lint.py` | Does a doc break the project's writing conventions, links or module-reference coverage? | `python -m tuner.tools.doc_lint [--max N] [--strict] [paths ...]` |

The CMA-ES weight search (`tuner/offline_tuner.py`) and the `sim/` modules are simulation infrastructure, not diagnostics. See [offline_guide.md](offline_guide.md). Track export tools (`tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py`) are covered in [integration_guide.md](../fsds/integration_guide.md).

**Two scripts share the name `nmpc_offline_check.py`.** They check different implementations and different sets of properties, and are read side by side, not interchanged.

| Script | Checks |
|---|---|
| `tuner/validation/nmpc_offline_check.py` (offline plant, `controller/nmpc/`) | four: scalar against vectorised model parity, SQP cost convergence, turn-in from an on-line start, closed-loop LTV-QP against NMPC on the recorded map |
| `test/nmpc_offline_check.py` (live `fsae_control.nmpc`, mirrored under `fsds_simulator/control/fsae_control/test/`) | six: model parity, Jacobians (forward against central differences), SQP convergence, turn-in, wrong-direction transient, closed loop |

The live closed-loop check needs a sibling `fsae_MPCTest` checkout and is skipped without one.

Investigation scripts named in [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md) that no longer exist: `gap_attribution_ledger`, `blend_reset_diagnostics`, `reference_heading_vs_rebuild`, `combined_factors_sweep`, `plot_control_log`. They were one-off scripts for concluded questions. `plot_playback.py` replaced `plot_control_log`.

## The launcher: `gui/launcher/`

A Tk desktop app that wraps the tools on this page. It holds no simulation, plotting or tuning logic. Each button rewrites a config file in place, with the same effect as a hand edit, and then shells out to an existing tool.

```bash
cd fsae_MPCTest && python -m gui.launcher
```

**Required layout.** `fsae_MPCTest/` sits directly inside the outer FSDS repo root, next to `ros2/`. `gui/launcher/paths.py` resolves every path from its own location (`fsae_MPCTest` and its parent), so a checkout anywhere else cannot find `ros2/launch_all.sh`, the tracks or `mpc_params.py`.

**Package layout.** `theme.py`, `paths.py`, `file_edit.py`, `process_utils.py`, `app.py` and one module per tab under `gui/launcher/tabs/`: `launch.py`, `log_debug.py`, `offline_sim.py`, `settings.py`, `profiles.py`.

| Tab | What it does |
|---|---|
| Launch Sim | Rewrites `ros2/launch_all.sh` variables from a form (preview and confirm first), then runs it. Stop sends Ctrl+C to the process group. Also holds Export & Save Track and Run Brake Sysid. |
| Debug a Log | Lists `*_control_*.csv` files in `<FSDS root>/fsae_logs/` and `fsds_simulator/recorded_runs/` (including one level of subfolder). Debug Selected opens them in `plot_playback`. Debug Latest runs it with no arguments. |
| Run Offline Sim | Starts `gui/simulation.py` (`python -m gui.simulation`), the 2D matplotlib tool. Its dynamics do not match FSDS, so cross-check anything that matters against `tuner.validation.recorded_map_rollout` or an FSDS session. |
| Settings | Edits commonly retuned `settings/` constants and syncs them to the live and mirror files. |
| Profiles | Named snapshots of the Settings and Launch tab values. |

### Files each tab writes

| File | Written by | What changes |
|---|---|---|
| `ros2/launch_all.sh` | Launch button | `TRACK`, `CONTROLLER`, `USE_NMPC`, `STANDALONE_OUTPUT`, `USE_PRECOMPUTED_SPEED`, `USE_PRECOMPUTED_PATH`, `V_MAX`, `V_MIN`, and (NMPC only) the commented-out `NMPC_PROGRESS_ENABLED` line. It does not set `NMPC_SLACK_LINEAR_WEIGHT`. Brake Sysid flips `RUN_BRAKE_SYSID` on for the launch and back to false. |
| `settings/*.py` | Settings Save, Profiles Load | `Q_diag`, `R_diag`, `R_rate_diag`, `R_A_ACCEL`, `R_A_BRAKE`, `SPEED_TARGET_DEFICIT_MAX`, the NMPC rate-shaping scalars, every `NMPC_*` weight override, the NMPC progress-term numbers and the feature flags. Each name is located by searching the submodules. |
| Live `mpc_params.py` and `nmpc_params.py` | Settings Save, Profiles Load | the matching field for each setting that has a live counterpart |
| Live `fsae_params.yaml` | Settings Save, Profiles Load | the same fields. The YAML overrides the dataclass default at ROS parameter declaration, so a save that skipped it would leave the car on the old value with no visible sign. |
| `fsds_simulator/.../mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml` | Settings Save, Profiles Load | the same fields again, keeping the mirror identical to live |
| `settings_profiles/<name>.json` | Profiles Save and Delete | a snapshot, tracked in git |
| `ros2/src/fsae_planning/tracks/<name>/` and `fsds_simulator/tracks/<name>/` | Export & Save Track | `speed_profile.csv`, `raceline.csv`, `centerline.csv` for a new track, then a copy into the mirror |
| `fsae_logs/*.csv` to `fsds_simulator/recorded_runs/<Controller>/` | Stop (moves, does not create) | only after the prompt is accepted |

The first write of a session to `launch_all.sh` or any `settings/*.py` file saves a `.bak` copy beside it. Recovery is `mv launch_all.sh.bak launch_all.sh`, independent of git.

### Launch tab: recording a new track

Checking Record new track and typing a name applies the recommended recording setup: `CONTROLLER=stanley`, `USE_PRECOMPUTED_SPEED=false`, `USE_PRECOMPUTED_PATH=false`. The previous values are restored when the box is unchecked. `launch_all.sh` writes `cone_map.json` for whatever `TRACK` is set to, so recording mode makes sure it lands in a new folder from a live drive.

After Stop, Export & Save Track runs `tuner.tools.export_speed_profile <name>`, then `tuner.tools.raceline_optimizer <name>` and `tuner.tools.raceline_optimizer <name> --mode centerline`, then copies the folder into the mirror. Re-exporting an existing name asks first. Track workflow: [integration_guide.md](../fsds/integration_guide.md).

Stop also offers to move the run's control and path CSVs from `fsae_logs/` into `fsds_simulator/recorded_runs/<Controller>/`. It only offers logs written after the current launch. The node closes its CSV asynchronously from its own signal handler, so the launcher polls for up to 5 seconds. An optional label is spliced between the tag and `_control_` or `_path_`.

### Settings tab

Fields are grouped the way `MPCParams` field metadata classifies them:

- Both controllers: `delay_compensation_enabled`, with no offline constant because the offline rollout always has it on.
- LTV-QP only: adaptive Q scaling, steer-rate anti-hunt, reference-heading rate limit, reversal penalty.
- NMPC only: steer-rate anti-hunt, reversal penalty, rate-cost stage ramp, three-zone schedule, corner rate-blend.

Every `NMPC_*` weight override has an override checkbox: checked uses the number, unchecked writes the `-1.0` inherit sentinel. Field descriptions and units come from the `mpc_params.py` metadata, so the text follows the field.

Settings with no live counterpart stay offline-only: `R_diag[1]` (nominal only, superseded by `R_A_ACCEL` and `R_A_BRAKE`) and the last three `Q_diag` entries (always 0.0). If the live files are missing at the expected path, Save writes `settings/` alone and warns.

Precedence hazard: a launched node reads a `launch_all.sh` argument before `fsae_params.yaml` and the dataclass. The Settings tab never writes `launch_all.sh`. A field that can be set by a launch argument therefore belongs on the Launch tab only, and the NMPC progress flag is deliberately absent here for that reason. A Settings save of a field that an uncommented `launch_all.sh` line overrides has no effect on the next launch.

Not exposed: solver internals such as `NMPC_HORIZON` and `NMPC_SQP_ITERS`, the corner-mode Q/R split and most still-default flags. They stay manual edits.

### Overwrite All Params

Save pushes what the tab's widgets hold outward. Overwrite All Params runs `tuner.tools.sync_mpc_params` as a subprocess and pushes the live checkout's on-disk files into `fsae_autonomous` and the mirror. It ignores the tab's widgets and any unsaved edits, so save and push live first. Two steps: a dry run (already in sync stops with an info dialog), then a confirmation naming the destinations that differ. Declining writes nothing.

### Profiles tab

A profile is one JSON file in `settings_profiles/` holding every Settings-tab field plus the Launch tab's track, controller, precomputed speed and path, `V_MAX`, `V_MIN` and progress choice.

- Save Current As Profile reads the widgets, so unsaved edits are captured. A same-name profile asks before overwrite.
- Load Selected pushes values into the widgets and then runs the Settings tab's own save routine, so all files are rewritten as if each field were retyped. The Launch-tab part only updates that tab's widgets and does not write `launch_all.sh` until the next Launch click. A name no field table recognises is skipped silently.
- Delete Selected removes the file. Recovery is git history only.

Loading a profile writes files but does not restart a running sim.

## Pushing live-tested params: `tuner/tools/sync_mpc_params.py`

**Question.** A param was retuned and validated in `fsae_planning`. Is `fsae_autonomous`, or the `fsds_simulator` mirror, still on the old value?

```bash
python -m tuner.tools.sync_mpc_params            # dry run, prints a diff per file per destination
python -m tuner.tools.sync_mpc_params --apply    # overwrite
```

- **Scope.** Exactly `mpc_params.py`, `nmpc_params.py` and `fsae_params.yaml`. Code changes to `mpc_controller.py`, the `lmpc/` and `nmpc/` packages or `live_viz/` need the ordinary manual propagation.
- **Direction.** One way, from `ros2/src/fsae_planning/` to `fsae_autonomous` and `fsae_MPCTest/fsds_simulator/`. Destination values are never read as a source.
- **Why the YAML is in scope.** It overrides the dataclass default, so syncing only the `.py` files would leave the destination on an old number.
- **Safety.** Dry run by default. Each destination file gets a one-time `.bak` before an overwrite. Only the local working tree of `fsae_autonomous` is written. Commit and push there stay separate human steps.
- **`fsae_autonomous` location.** The script tries `fsae_autonomous` and `ros2_autonomous/src/fsae_autonomous/` under the FSDS root (`_AUTONOMOUS_CANDIDATES`) and warns and skips that destination if neither has a `.git`. If the checkout moves again, add the new path there.

## Live debug window: `live_viz`

Sim-only visualiser: car, cone map, reference path, driven trail, NMPC predicted horizon and a weighted-error breakdown, redrawn from ROS 2 topics on a timer. The code is the `live_viz/` package under `fsae_control` (`panels.py`, `node.py`, `app.py`). It is started by `ros2/launch_all.sh` or the launcher, never by `fsae_autonomous`.

```bash
ros2 run fsae_control live_viz
```

**It follows the running controller.** MPC and Stanley share a node name and, in `cmd_vel` mode, an output topic, so the active controller is identified by which debug topic last published: `/fsae/control/debug_weights` (MPC) or `/fsae/control/debug_stanley` (Stanley). The window title, stats text and debug figure switch accordingly.

- **MPC active.** Grouped step-0 panels (tracking, effort, rate) plus a horizon-summed panel, all against `total_cost`.
- **Stanley active.** Two panels: heading against lateral error (`e_y`, `e_psi`, share of their sum), and the three additive law terms `heading_error + atan2(k_cte e, v + k_soft) - k_d yaw_rate` (see `StanleyController` in `control_utils.py`), each as a share of the sum of absolute values.

**Per-lap score and horizon accuracy.** A panel at bottom left of the map lists each completed lap: number, time, composite score, horizon accuracy. It is fed by `/fsae/control/lap_summary` (JSON from `ControlLogger.finish_lap()`) and reads "none completed (needs precomputed path)" until a lap finishes. A live-planner run has no `LapProgressTracker` and never publishes a lap.

Horizon accuracy is how well the controller's 1-second prediction matched the driven path (100 percent is exact). It is mean predicted-against-actual position error over the horizon divided by the horizon's arc length, computed by `HorizonAccuracyTracker` in `telemetry/horizon_tracker.py`. It is independent of the composite score and NMPC-only, because the LTV-QP exposes no Cartesian prediction. LTV-QP rows show "horizon n/a".

**Display design.** Each of these fixes a specific defect:

- Both debug figures are built once and shown or hidden as a whole. `Figure.set_visible()` does not touch the Tk window, so the inactive figure's window is also `withdraw()`n and `deiconify()`ed, only on an actual controller switch so a moved window is not fought back into place.
- Bar panels keep a fixed row order for the window's lifetime (`DEBUG_BAR_GROUPS`, `DEBUG_HORIZON_TERMS`, `STANLEY_ERROR_TERMS`, `STANLEY_LAW_TERMS`). A term with no data this tick shows as an empty grey row, and the horizon panel is not re-sorted per frame. Otherwise rows shifted or swapped while values moved smoothly.
- The debug figures use `layout='constrained'` with explicit `hspace` and `wspace` (0.6 and 0.35). A one-time `tight_layout()` cannot re-reserve margin after a resize, which clipped y-axis labels.

## Steering system-ID harness

FSDS's lateral-acceleration ceiling was found by commanding fixed steering angles at fixed speeds on an empty map and recording the achieved yaw rate. Repeat that methodology whenever a plant-versus-car discrepancy is suspected, because a closed-loop lap log cannot separate a plant defect from a controller or reference one.

**Current state.** The ROS 2 nodes that drove the sweep and step tests no longer exist, and the shell scripts that called them are deleted. Repeating the measurement needs new nodes, see [offline_live_parity.md](../reference/offline_live_parity.md#steering-system-id-harness).

What survives:

- The analysers `tuner/investigations/steering_sysid_analysis.py` and `steering_step_analysis.py` read a log and name the mechanism. The sweep analyser fits candidate models to achieved yaw rate and reports the margin to the runner-up, and refuses a verdict when too few windows contain real motion. The step analyser separates a hard yaw-rate clip, speed-scaled authority and active damping from overshoot, plateau and rise shape.
- `tuner/validation/plant_openloop_validation.py` replays previously recorded sweeps and steps through the offline plant. Run it after any change to the vehicle model.
- The brake harness is intact. `brake_sysid` is a registered entry point, and `ros2/run_brake_sysid.sh` runs the sweep and analyses the log. Flags: `--no-sim` (FSDS already running), `--quick` (6 points instead of 20), and `-p name:=value` passes through to the node. It bypasses the controller and does not steer or avoid cones, so run it in open space. The launcher's Run Brake Sysid button starts it through `RUN_BRAKE_SYSID`. Logs land in `fsae_logs/brake_sysid_<epoch>.csv`.
- All three harness scripts are diagnostic-only and separate from `launch_all.sh`. Any other publisher on `/fsds/control_command` interleaves with the sweep and corrupts the measurement.

The analysis step at the end of each `run_*_sysid.sh` script still calls the old module paths (`tuner.checks...`, `tuner.steering_*_analysis`). Run the analyser by hand with its `tuner.investigations` path.

## Telemetry playback: `tuner/tools/plot_playback.py`

Turns one or more control-telemetry CSVs (written as described in [integration_guide.md](../fsds/integration_guide.md)) into an interactive figure showing both what a signal did over the run and where the car was at a chosen moment.

- **Left.** Scored signals on a shared time axis, one line per signal per run, with a cursor at "now". Default signals: `e_y`, `e_psi_deg`, `corner_frac`, `steer_deg`, `v`.
- **Top right.** Each run's driven trajectory plus the planner's latest path snapshot at "now", with a triangle marking car position and heading.
- **Bottom right.** The same scene zoomed to the car's current section of track, with `e_y` and `e_psi` in the title.

A slider scrubs one shared "now" through all runs. Each run has its own colour, used for its lines, trajectory, path overlay and marker. With more than one run, a Show/hide checkbox per run and a Zoom focus radio (which run the zoomed view follows, first-loaded by default) appear at bottom left.

```bash
python -m tuner.tools.plot_playback                       # auto-load, see below
python -m tuner.tools.plot_playback --latest-only         # only the single newest run
python -m tuner.tools.plot_playback --all                 # every run in every subfolder
python -m tuner.tools.plot_playback run.csv               # one explicit run
python -m tuner.tools.plot_playback a.csv b.csv           # overlay two runs
python -m tuner.tools.plot_playback run.csv --signals e_y,yaw_rate,solve_ms
python -m tuner.tools.plot_playback --recorded-runs <dir> # search another folder
```

- A signal missing from a log (for example `solve_ms` on a Stanley run) is skipped for that run with a warning, so logs with different columns still overlay on what they share.
- The sibling `<tag>_path_<stamp>.csv` (same folder and stamp) is loaded automatically and drives the path overlay. Copy both files together, or the map and zoom views fall back to the driven trajectory alone. The slider shows the latest snapshot at or before the selected time, with no interpolation.
- Runs may differ in sampling and length. The slider spans the longest run, and the signal plots clip to the shortest run's end so the comparable part is not dwarfed.
- Each completed lap gets a dashed marker on every signal plot (`L<n> <score>/<horizon%>`) and a running table in the map panel, from the `lap_score` and `lap_pred_acc_pct` columns. An older log falls back to the header's whole-run `composite_score`. Horizon accuracy is NMPC-only and shows "n/a" otherwise. `pred_acc_pct` is also selectable with `--signals` and is appended to the default set when any loaded log has the column.
- On Windows PowerShell, put the command on one line, use `$HOME\...` in place of `~`, and note that a bad path fails with a raw `FileNotFoundError` from `csv_log.py`.

### Auto-search: `fsds_simulator/recorded_runs/`

With no CSV argument the tool searches `fsds_simulator/recorded_runs/` recursively for `*_control_*.csv`, including per-controller subfolders (`LMPC`, `NMPC`, `Stanley`).

- Runs are ordered by the timestamp in the filename, not file mtime. `_stamp()` decodes both the `%Y%m%d-%H%M%S` form and the older epoch-seconds form so mixed folders sort as one set.
- By default it loads the newest run from each subfolder. Runs left flat in `recorded_runs/` count as one folder.
- A run's label is its folder name (`LMPC`, `NMPC`, `Stanley`), because the filename tag is ambiguous (LMPC and NMPC both use `mpc_standalone`). Flat runs use the filename tag. Duplicates under `--all` get ` #2`, ` #3`.
- A descriptive segment between tag and stamp (for example `mpc_standalone_postjitterfix_best_control_<stamp>.csv`) does not affect discovery or sibling pairing.
- CSVs under `recorded_runs/` are tracked in git so reference runs travel with the repo.
- A live run writes to `log_dir`. `ros2/launch_all.sh` sets it to `<FSDS root>/fsae_logs`. `ControlLogger`'s own fallback when no `log_dir` is given is `~/fsae_logs`. Copy or move the pair into the right subfolder afterward, or accept the launcher's Stop prompt:

```bash
cp fsae_logs/mpc_standalone_control_<stamp>.csv fsae_logs/mpc_standalone_path_<stamp>.csv fsds_simulator/recorded_runs/LMPC/
```

**Curated drop zone: the `graph` folder in `recorded_runs`.** If it holds any `*_control_*.csv` (the folder is not scanned for subfolders), auto-load uses only those files. Copy runs in to control exactly what a bare `python -m tuner.tools.plot_playback` shows, without moving them from their controller folder. Every run in it loads, unlike the newest-per-folder default, and labels use the filename tag. `--latest-only` narrows it to its newest run. `--recorded-runs <dir>` bypasses it. The folder ships with a `.gitkeep`, and may hold comparison runs in a working checkout.

## Live pose and RPC diagnostics

Temporary instrumentation for the open [periodic car-position teleport bug](../logs/periodic_pose_teleport_investigation.md). `ros2/launch_all.sh` starts it right after the bridge comes up (search for `TEMPORARY (2026-08-19)`). The block, its `cleanup()` teardown and `ros2/clock_drift_check.py` should be removed once the root cause is found.

| Capture | What it checks |
|---|---|
| `ros2 topic hz -w 5 /fsds/testing_only/odom` | bridge's raw 250 Hz odom arrival rate |
| `ros2 topic hz -w 5 /fsae/slam/car_position` | `sim_perception.py`'s 20 Hz relay arrival rate |
| `ros2 topic hz -w 5 /clock` | sim clock arrival rate |
| `ros2 topic hz -w 5 /fsds/imu` | IMU arrival rate |
| `ros2/clock_drift_check.py` | whether the `/clock` value falls behind wall time |

Logs go to `fsae_logs/topic_hz_diagnostics/`.

`ros2/clock_drift_check.py` subscribes to `/clock` and logs `(wall_time, sim_time)` per message. `ros2 topic hz` only shows how often a message arrives. This shows whether the clock's value lags, which tests whether FSDS itself runs slower than real time as opposed to a delivery problem. `d(sim_time)/d(wall_time)` near 1.0 means the simulation keeps up. A sustained value below 1.0 means FSDS is the bottleneck.

```bash
# started by ros2/launch_all.sh; standalone, after sourcing the workspace:
python3 clock_drift_check.py <output_csv_path>
```

Findings so far and what remains open: [periodic_pose_teleport_investigation.md](../logs/periodic_pose_teleport_investigation.md).

## Doc conventions: `tuner/tools/doc_lint.py`

Checks docs (every README and `docs/**/*.md`, with `docs/logs/` exempt apart from its README) for long unstructured prose, references to AI-assistant instruction files, transcript voice, broken relative links and heading anchors, backticked paths and `python -m` targets that do not exist, style (em dashes, extra H1s, intensifier words) and missing module-reference entries in `docs/modules/`. It also flags `from settings.<sub> import ...` in code, because consumers must use `import settings; settings.X` for runtime overrides to reach them.

```bash
python -m tuner.tools.doc_lint                          # report only
python -m tuner.tools.doc_lint --strict                 # exit 1 if anything is flagged
python -m tuner.tools.doc_lint --max 10                 # stricter paragraph ceiling (default 12)
python -m tuner.tools.doc_lint docs/guides/tuning.md    # specific files
```

## Debugging solver failures

Offline, `settings.MAX_FAILS` (5) consecutive solver failures ends a rollout as a DNF. Live, the LTV-QP prints `[MPC] Warning: OSQP OPTIMAL_INACCURATE` when it accepts an inaccurate solution, and the NMPC reports a per-tick `nmpc_status` (`solved`, `budget`, `rejected` or `warm-start-only`).

- **Weight scaling.** OSQP is sensitive to poorly conditioned matrices. Entries of `Q`, `R` or `R_rate` far above 1e4 or below 1e-4 can hurt convergence. Check the output of `adaptive_R_scaling` in `controller/model_utils.py` at the test speed is not inflating the steering cost. The 1e4 and 1e-4 thresholds are a rule of thumb, not verified against this solver.
- **Speed profile too aggressive.** If the car fails at tight hairpins, the profile may demand more lateral force than the plant supplies. The corner-speed knob is `CURVATURE_SPEED_A_LAT_MAX` in `sim/speed_profile.py`. The `mu` argument of `compute_speed_profile()` is kept for signature compatibility and has no effect.
- **Model-plant mismatch at extremes.** The LTV-QP's linear model blends kinematic and dynamic behaviour between 1 and 2.5 m/s. The largest prediction errors appear well outside that band, at very low speed under load or at high lateral acceleration near the tyre limit.

For NMPC SQP problems (non-improving steps, oscillation), the usual causes are the same, plus `nmpc_solve_budget_ms` or `nmpc_sqp_iters` too tight for the horizon, or a weight override (`NMPC_Q_E_Y` and others, `-1` inherits) that pushes the cost out of scale. See [control_mechanisms.md](../reference/control_mechanisms.md) for the model and weight mapping and [tuning.md](tuning.md) for the NMPC tuning surface. A low-speed stall with a zero output is the RK4 instability described in the structural-settings table there.
