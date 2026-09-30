# Offline simulator module reference

File-by-file reference for `fsae_MPCTest` excluding the `fsds_simulator/` mirror (see [fsds_ros2.md](fsds_ros2.md) for that).
Each entry says what the file does, what a developer typically changes in it, what not to do, and the key names.
Conceptual background: [architecture](../reference/architecture.md) and [offline guide](../guides/offline_guide.md).

## Where to change what

- Retune LTV-QP weights or adaptive shaping: `settings/lmpc.py`, `controller/model_utils.py` (shape only), live `mpc_params.py`, then run `python -m tuner.validation.recorded_map_rollout`
- Retune NMPC weights, zones or flags: `settings/nmpc.py`, live `nmpc_params.py`, `fsae_params.yaml`, then run `python -m tuner.validation.nmpc_offline_check` and `python -m tuner.validation.recorded_map_rollout`
- Change NMPC solver internals (SQP, trust region, QP rows): `controller/nmpc/qp_model.py`, `controller/nmpc/sqp_step.py`, `controller/nmpc/solver.py` plus the live twins, then run `python -m tuner.validation.nmpc_offline_check`
- Change NMPC prediction model or output rows: `controller/nmpc/dynamics.py`, `controller/nmpc/outputs.py`, `controller/nmpc/layout.py` plus live twins, then run `python -m tuner.validation.nmpc_offline_check`
- Change the NMPC reference or curvature handling: `controller/nmpc/reference.py`, `settings/nmpc.py`, then run `python -m tuner.validation.recorded_map_rollout`
- Change LTV-QP structure or solve: `controller/lmpc/build.py`, `controller/lmpc/solve.py`, live `lmpc/controller.py`, then run `python -m tuner.validation.recorded_map_rollout`
- Change horizon, delay or pose-hold model: `settings/general.py`, live `MPCController` horizon and delay params, then run `python -m tuner.validation.recorded_map_rollout`
- Change scoring weights or thresholds: `settings/scoring.py`, `sim/scoring.py`, live `telemetry/scoring.py` inlined constants, then run `python -m tuner.validation.recorded_map_rollout`
- Refit the plant or alat ceiling: `model/vehicle_physics/params.py`, `model/vehicle_physics/plant_step.py`, `settings/nmpc.py` (`ALAT_CEILING_*`), then run `python -m tuner.validation.plant_openloop_validation` and `python -m tuner.validation.recorded_map_rollout`
- Change the LTV prediction model: `model/bicycle_model.py`, live `lmpc/controller.py`, then run `python -m tuner.validation.recorded_map_rollout`
- Add a plant state or change tracking error definition: `model/vehicle_physics/state.py`, `model/vehicle_physics/tracking.py`, then run `python -m tuner.validation.plant_openloop_validation`
- Change the planner: `planning/boundary.py`, `planning/path_utils.py`, `planning/cone_map.py`, `planning/cone_sorting.py`, `planning/geometry.py`, `settings/planner.py`, live planning twins, then run `python -m tuner.validation.recorded_map_rollout` with `USE_PLANNER=True`
- Add or switch a recorded track: `tracks/__init__.py` helpers, exporters in `tuner/tools/`, `ros2/launch_all.sh` `TRACK=`, then run `python -m tuner.validation.recorded_map_rollout`
- Add a settings constant: submodule in `settings/`, re-export in `settings/__init__.py`, access as `settings.X`, then run `python -m tuner.validation.recorded_map_rollout`
- Change tuner budget or DNF penalties: `settings/solver.py`, then run `tuner/offline_tuner.py` with `FAST_TEST_MODE` as a smoke test
- retune LTV-QP weights (Q, R, R_rate): the `settings/` lmpc submodule and the live `mpc_params.py`, then run `python -m tuner.validation.recorded_map_rollout`
- change NMPC solver or cost code: `controller/nmpc/` files and the matching live `nmpc/` files, then run `python -m tuner.validation.nmpc_offline_check` and `python -m tuner.validation.recorded_map_rollout`
- add or change an NMPC setting passed to the controller: `sim/rollout/tick_solve.py` (`build_nmpc`), the `settings/` nmpc submodule and `nmpc_params.py`, then run `python -m tuner.validation.nmpc_offline_check`
- change the speed-target limiters (rise rate, gate rate, deficit max): `sim/rollout/speed_target.py`, the `settings/` submodule holding `SPEED_TARGET_DEFICIT_MAX` and the live node constants, then run `python -m tuner.validation.recorded_map_rollout`
- change delay compensation or jitter: `sim/rollout/delay.py` and the live delay code, then run `python -m tuner.validation.recorded_map_rollout --planner`
- change the plant or the lateral-acceleration ceiling: `model/vehicle_physics/`, then run `python -m tuner.validation.plant_openloop_validation` and `python -m tuner.validation.recorded_map_rollout`
- change scoring metrics or weights: `sim/scoring.py`, the `settings/` scoring submodule, then re-copy to the live `telemetry/scoring.py`, then run `python -m tuner.validation.recorded_map_rollout`
- change sensor noise or pose-hold modelling: `sim/sensor_noise.py` and the `settings/` noise submodule, then run `python -m tuner.validation.recorded_map_rollout --planner`
- change the speed-profile heuristic: `sim/speed_profile.py` and the live `control_utils.py`, then run `python -m tuner.tools.export_speed_profile` and `python -m tuner.validation.recorded_map_rollout`
- import a newly recorded track: `sim/track_io.py` (only if reconstruction fails), then run `python -m tuner.tools.export_speed_profile <track>` and `python -m tuner.tools.raceline_optimizer <track>`
- run automatic weight search: `tuner/offline_tuner.py` bounds and `TUNABLE_NMPC`, then run `python -m tuner.offline_tuner` and re-check the result with `python -m tuner.validation.recorded_map_rollout`
- push tuned live params to the mirror and production tree: `tuner/tools/sync_mpc_params.py`, then run `python -m tuner.tools.sync_mpc_params` (dry run) and `--apply`
- diagnose a live steering-saturation gap: `tuner/investigations/live_vs_sim_diagnostics.py`, then run `python -m tuner.investigations.live_vs_sim_diagnostics --no-sim` for live logs only
- test a reference-heading limiter idea: fix `tuner/investigations/ref_heading_limiter_ab.py` to patch `settings` first, then run `python -m tuner.investigations.ref_heading_limiter_ab`
- replay a live log: `tuner/tools/plot_playback.py`, then run `python -m tuner.tools.plot_playback <control_csv>`
- Add a Settings-tab field: `gui/launcher/tabs/settings.py` (one of the field tables), matching field in the live `mpc_params.py` or `nmpc_params.py` and `settings/`, then save in the GUI and `git diff` the live tree, the mirror and both `fsae_params.yaml`.
- Add a Launch-tab option backed by a `launch_all.sh` variable: `gui/launcher/tabs/launch.py` (`_pending_values`, `refresh_from_disk`, `apply_profile_values`), then Launch once and diff `ros2/launch_all.sh`.
- Change what a saved profile captures: `capture_profile_values` and `apply_profile_values` in the Settings or Launch tab file, then save and load a profile and inspect the JSON in `settings_profiles/`.
- Fix or extend regex file editing: `gui/launcher/file_edit.py`, then rewrite a copy of the target file and diff it.
- Point the GUI at a moved checkout or new file: `gui/launcher/paths.py`, then print `_repo_paths()` and open every tab.
- Add a launcher tab: new file in `gui/launcher/tabs/`, register in `gui/launcher/app.py`, update the docstring in `gui/launcher/__init__.py`, then run pyflakes on `gui/launcher` (a dev tool, not a repo dependency) and click through.
- Add a telemetry CSV column: `ADAPTIVE_COLUMNS` in the live `telemetry/columns.py` and its `fsds_simulator/` mirror (append only), then a short run and check the header, and that `tuner.tools.plot_playback` still loads it.
- Add a track file type or change track lookup: `tracks/__init__.py` plus the exporter in `tuner/tools/`, then `python -c "import tracks; print(tracks.list_tracks())"`.
- Change the matplotlib simulator behaviour: `gui/simulation.py` (UI) or `sim/rollout/core.py` (loop), then `python -m gui.simulation` and cross-check with `python -m tuner.validation.recorded_map_rollout`.
- Change tuner history output: `log_results_to_history` in `tuner/offline_tuner.py`, then a short tuning run and read the appended entry.

## settings/

Every offline tuning constant, split by topic. `settings/__init__.py` re-exports every name so `settings.X` works from anywhere. Consumers must read constants as `settings.X` (never `from settings.lmpc import X`), because runtime overrides (`setattr(settings, name, value)`, used by `tuner/investigations/steering_chatter_check.py --set`) rebind the name in the package namespace only. Edit a constant in the submodule that holds it (find with grep). Each MPC weight or flag has a hand-kept live twin in `mpc_params.py`/`nmpc_params.py`.

### `settings/__init__.py`
Does: re-exports all settings constants at package scope and applies the `FAST_TEST_MODE` overrides after every submodule is imported.
Change it to:
- add a re-export line when a new constant is added to a submodule (forgetting it makes `settings.NEW` raise AttributeError); run `python -m tuner.validation.recorded_map_rollout`
- change what `FAST_TEST_MODE` shortens (for example a smaller `MAX_EVALS`); run one tuner smoke run
Don't:
- import a constant from a submodule in consumer code, because the override mechanism only rebinds the package-level name and a submodule import holds a stale copy
- move the `FAST_TEST_MODE` block into one submodule, because it overrides names from general, solver and scoring
Key API: every re-exported constant name (146 as counted by the restructure brief), `FAST_TEST_MODE` block

### `settings/general.py`
Does: horizon length, planner and speed-profile switches, delay and jitter model, DNF thresholds, reference-heading limiter, pose-hold model.
Change it to:
- change `N_HORIZON` (must equal the live LMPC horizon, see the note in `lmpc/controller.py`); run `python -m tuner.validation.recorded_map_rollout`
- change `DELAY_STEPS` or `POSE_HOLD_PROB` to match a newly measured car latency; run `python -m tuner.validation.recorded_map_rollout`
- toggle `USE_PRECOMPUTED_SPEED_PROFILE` or `USE_PLANNER`; run the rollout and compare with the baseline table
Don't:
- change `N_HORIZON` on one side only, because tuned weights then stop transferring to the car
- treat the `N_HORIZON` comment block as current: it says 25 steps and 1.25 s but the value is 35, and it names `simulation.py`/`control_utils.py` as the sites to keep equal
- rely on `OFFTRACK_LIMIT` being independent of the sim, because it is computed from `sim.perception.TRACK_HALF_WIDTH` at import (this file imports `sim.perception`)
Key API: `N_HORIZON`, `DELAY_STEPS`, `DELAY_JITTER_STEPS`, `OFFTRACK_LIMIT`, `POSE_HOLD_*`, `USE_PLANNER`, `USE_PRECOMPUTED_SPEED_PROFILE`, `REF_HEADING_RATE_LIMIT_ENABLED`, `DT`

### `settings/noise.py`
Does: SLAM pose noise (jitter, drift) and cone-detection noise, all off by default.
Change it to:
- enable `SLAM_NOISE_ENABLED` to test weight robustness to localisation error; run `python -m tuner.validation.recorded_map_rollout`
- change a seed to get a different noise draw; rerun the same command
Don't:
- enable noise and compare the score with the noise-free baseline table, because the table assumes the defaults (FSDS supplies ground-truth pose)
Key API: `SLAM_NOISE_ENABLED`, `SLAM_POS_JITTER_STD`, `SLAM_DRIFT_TAU`, `CONE_NOISE_ENABLED`, `CONE_POS_JITTER_STD`

### `settings/planner.py`
Does: the four `SimPlanner` tunables (smoothing, crop radius, plan horizon, path blend).
Change it to:
- retune `PLANNER_SMOOTH_PER_PT` after changing the live planner; run `python -m tuner.validation.recorded_map_rollout` with `USE_PLANNER=True`
Don't:
- let these drift from the live `centerline_planner` block in `common/fsae_bringup/config/fsae_params.yaml` (`smooth`, `look_radius`, `plan_horizon`, `path_blend`), because `planning/boundary.py` and `planning/path_utils.py` defaults differ from the tuned live values and the sim would silently use the wrong ones
Key API: `PLANNER_SMOOTH_PER_PT`, `PLANNER_LOOK_RADIUS`, `PLANNER_PLAN_HORIZON`, `PLANNER_PATH_BLEND`

### `settings/lmpc.py`
Does: LTV-QP cost weights (`Q_diag`, `R_diag`, `R_rate_diag`), corner-factor scheduler, low-speed boost, accel/brake weights and the flags gating adaptive shaping.
Change it to:
- retune corner Q weights (`Q_EY_CORNER`, `Q_EPSI_CORNER`); edit `MPCParams` in `mpc_params.py` in step, then run `python -m tuner.validation.recorded_map_rollout`
- change `SPEED_TARGET_DEFICIT_MAX`; same parity edit, same command
Don't:
- edit only this file, because every field has a live `MPCParams` twin that is kept equal by hand (`python -m tuner.tools.sync_mpc_params` only syncs live to autonomous, never these constants)
- assume the weights are unit-normalised, because they multiply raw-unit error terms (metres squared, radians squared)
Key API: `Q_diag`, `R_diag`, `R_rate_diag`, `R_A_ACCEL`, `R_A_BRAKE`, `CORNER_FACTOR_K`, `ADAPTIVE_Q_SCALING_ENABLED`, `SPEED_TARGET_DEFICIT_MAX`

### `settings/nmpc.py`
Does: NMPC weights (with `-1.0` meaning inherit the LMPC value), solver internals, alat-ceiling constants and every NMPC-only feature flag.
Change it to:
- retune `NMPC_RJERK_DELTA` or the `NMPC_RRATE_ZONE_*` endpoints; mirror in `nmpc_params.py`, run `python -m tuner.validation.nmpc_offline_check` then `python -m tuner.validation.recorded_map_rollout` with `USE_NMPC=True`
- change `NMPC_SQP_ITERS` or the RK/Jacobian substep counts; run `python -m tuner.validation.nmpc_offline_check`
- refit the three `ALAT_CEILING_*` constants; must equal `VehicleParams.alat_ceiling*` and the live `NMPCParams`; run `python -m tuner.validation.plant_openloop_validation`
Don't:
- lower `NMPC_RK_SUBSTEPS`/`NMPC_JAC_SUBSTEPS` below 4 at low speed without the speed gates, because the low-speed lateral dynamics are RK4-unstable and the solver freezes at zero output
- change solver internals without the `nmpc_offline_check` run, because state-anchoring and Hessian sparsity errors pass the rollout score
Key API: `USE_NMPC`, `NMPC_Q_*`, `NMPC_R_*`, `NMPC_HORIZON`, `NMPC_SQP_ITERS`, `NMPC_PROGRESS_ENABLED`, `NMPC_LATENCY_COMPENSATION_*`, `ALAT_CEILING_*`

### `settings/scoring.py`
Does: composite-score scaling, weights, validation suite, and the constrained scoring constants (floor, threshold, objective, quality weights).
Change it to:
- change a `SCORE_WEIGHTS` entry to shift what the tuner optimises; run `python -m tuner.validation.recorded_map_rollout` and re-read the score table
- change `COMPLETION_THRESHOLD` or `CONSTRAINT_FLOOR`; same command
Don't:
- change these without updating the inlined copy of the constants in the live `telemetry/scoring.py` (byte-identical to `sim/scoring.py` apart from inlined constants), because a live score must stay comparable to an offline one
- compare a score across a `METRIC_SCALES` change, because every metric is rescaled
Key API: `METRIC_SCALES`, `SCORE_WEIGHTS`, `VALIDATION_SUITE`, `CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT`, `TAIL_QUANTILE`

### `settings/solver.py`
Does: rollout OSQP tolerances, DNF penalties, tuning-engine budget (`MAX_EVALS`, Optuna pre-pass) and `FAST_TEST_MODE`.
Change it to:
- raise `MAX_EVALS` for a longer tuning run; run the tuner (`tuner/offline_tuner.py`) once to check runtime
- flip `FAST_TEST_MODE` to smoke-test tuner code changes; do not commit it as True
Don't:
- tighten `ROLLOUT_EPS`/`ROLLOUT_MAX_ITER` in one place only, because `gui/simulation.py` and the tuner pass these so live and offline solves stay comparable
- use `settings._stop_requested` as the stop flag, because `tuner/offline_tuner.py` keeps its own module-level `_stop_requested` and the settings copy appears unused
Key API: `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`, `ROLLOUT_EPS`, `ROLLOUT_MAX_ITER`, `MAX_EVALS`, `PATH_N_POINTS`, `USE_OPTUNA_PRESEARCH`, `FAST_TEST_MODE`

## controller/

The two controllers the closed loop can solve each tick: `lmpc/` (linear time-varying QP via CVXPY) and `nmpc/` (Frenet-frame nonlinear MPC via Gauss-Newton SQP and OSQP), plus `model_utils.py` (adaptive-gain helpers). `sim/rollout/core.py` calls one of them per tick, chosen by `settings.USE_NMPC`. `controller/__init__.py` is empty apart from a docstring. `controller/` never imports from `sim/`. Both packages are hand-kept numerically identical to the live `lmpc/` and `nmpc/` packages under `fsds_simulator/control/fsae_control/fsae_control/` (and `ros2/src/fsae_planning/...`).

### `controller/__init__.py`
Does: package marker.
Change it to:
- nothing normally
Don't:
- add imports here, because it would make `controller` import order matter for `sim/`
Key API: none

### `controller/lmpc/__init__.py`
Does: re-exports `init_parameterized_mpc` and `solve_mpc`.
Change it to:
- re-export a new public solver function
Don't:
- (nothing specific beyond the parity and validation notes above)
Key API: `init_parameterized_mpc`, `solve_mpc`

### `controller/lmpc/build.py`
Does: builds the parameterised CVXPY problem once (cost, dynamics, input bounds, hard slew limit, soft corridor).
Change it to:
- add a cost term or constraint row; mirror in live `lmpc/controller.py` (live merges build and solve into `MPCController`), then run `python -m tuner.validation.recorded_map_rollout`
- change `du_max` handling; live `du_max` must be equal
Don't:
- change a constraint on one side only, because the step-0 slew constraint exists specifically for parity with live and offline weights would stop transferring
- rebuild the problem per tick, because the compiled-parameter design is what keeps the solve near 1 to 5 ms at 20 Hz
Key API: `init_parameterized_mpc`

### `controller/lmpc/solve.py`
Does: per-tick `solve_mpc` (inject parameters, OSQP with Clarabel fallback) and the module-level `_mpc_cache`.
Change it to:
- add a solve parameter (for example a new weight split like `r_a_accel`/`r_a_brake`); thread it from `sim/rollout/core.py` and mirror live `lmpc/controller.py`; run the rollout
- change solver fallback behaviour; run the rollout and check the OPTIMAL_INACCURATE count
Don't:
- rely on `R[1,1]` for accel effort, because `a_cmd` cost comes from `r_a_accel`/`r_a_brake` and only `R[0,0]` is read
- pass `warm_start=True` on the first step of a rollout, because it inherits state from the previous rollout via the module cache
Key API: `solve_mpc`, `_mpc_cache`

### `controller/model_utils.py`
Does: current-state gain-shaping helpers (speed-scaled R, corner factor, low-speed boost, anti-hunt, reversal penalty, Q softening) for the LTV-QP and NMPC.
Change it to:
- retune the corner-factor curve or a boost shape; mirror live `lmpc/adaptive_gains.py`, then run `python -m tuner.validation.recorded_map_rollout`
- add a new adaptive helper; call it from `sim/rollout/tick_solve.py` and the live `MPCController.compute`
Don't:
- edit either copy alone, because live `lmpc/adaptive_gains.py` (`_corner_factor`, `_blend`, `_low_speed_corner_boost`) is its numeric twin
- put a speed-dependent scale on accel effort, because braking authority would then fall as the car slows into a corner
- reintroduce the removed lookahead scaling family, because reweighting today's cost from a forward scan did not change what the horizon predicts
Key API: `adaptive_R_scaling`, `adaptive_Q_scaling`, `steer_rate_anti_hunt`, `reversal_penalty_boost`, `_corner_factor`, `_blend`, `_low_speed_corner_boost`, `curvature_estimate`

### `controller/nmpc/__init__.py`
Does: re-exports the NMPC public and internal names so `controller.nmpc.X` matches the old single-module names, and holds the module map.
Change it to:
- re-export a new helper used by `tuner/validation/nmpc_offline_check.py`
Don't:
- rename a file without renaming its live counterpart, because the split is 1:1 with live `nmpc/` (layout, reference, dynamics, outputs, weight_schedule, qp_model, sqp_step, solver)
Key API: `NMPCController`, `PathReference`, `_step`, `_step_scalar`, `_outputs`

### `controller/nmpc/layout.py`
Does: state, input and output index constants (`IDX_*`, `NX`, `NU`, `NH_*`), finite-difference step sizes, `_DENOM_FLOOR`.
Change it to:
- change a finite-difference step; run `python -m tuner.validation.nmpc_offline_check`
- add an output row; update `outputs.py`, `qp_model.py` and the live `nmpc/layout.py`
Don't:
- change indices or `NH_*` on one side only, because the solver, Jacobians and weights all index by these names
Key API: `IDX_S`, `IDX_EY`, `NX`, `NU`, `NH_TRACKING`, `NH_PROGRESS`, `NH_FRICTION`, `_wrap`

### `controller/nmpc/reference.py`
Does: `PathReference`, the arc-length parameterisation with spline-derived curvature and reference heading.
Change it to:
- retune spline vs legacy reference (`NMPC_SPLINE_REFERENCE_ENABLED`) or smoothing; mirror live `nmpc/reference.py`, run `python -m tuner.validation.recorded_map_rollout` with `USE_NMPC=True`
Don't:
- use the raw per-waypoint tangent as reference heading, because it steps by ds/R, reads as tracking error and drives a steering limit cycle
- remove the legacy path when the spline flag is on, because the flag-off branch is still selectable
- point this at a planner change without re-reading the centreline-curvature-spike defect notes, because the smoothing here is a workaround layer
Key API: `PathReference`

### `controller/nmpc/dynamics.py`
Does: NMPC prediction model: `_Plant` constants read from `VehicleParams`, tyre forces, continuous dynamics `_f` and RK4 steppers.
Change it to:
- change tyre-force fade or the alat-ceiling law in the model; mirror live `nmpc/dynamics.py`; run `python -m tuner.validation.nmpc_offline_check`, then the rollout
Don't:
- let `_f_scalar`/`_step_scalar` and `_f`/`_step` diverge, because the rollout uses the scalar pair and the Jacobians use the vector pair; `nmpc_offline_check` asserts they match
- copy vehicle constants here, because the offline side reads them from `VehicleParams` (the live side has its own copy in `MPCParams`)
Key API: `_Plant`, `_tyre_forces`, `_f`, `_f_scalar`, `_step`, `_step_scalar`

### `controller/nmpc/outputs.py`
Does: `_outputs`, the residual rows of the least-squares cost (tracking errors, speed error or speed-cap hinge plus progress reward, optional friction-circle forces).
Change it to:
- change the progress or speed-cap row; mirror live `nmpc/outputs.py`; run `python -m tuner.validation.nmpc_offline_check`
Don't:
- weight the friction-circle rows, because they exist only to build constraint rows
- switch between the two row-4 layouts mid-run, because `NH` is fixed per controller instance
Key API: `_outputs`

### `controller/nmpc/weight_schedule.py`
Does: corner-zone scaling and near-stage ramp of the steering-rate weight.
Change it to:
- retune zone endpoints or ramp shape; edit `settings/nmpc.py` and live `nmpc_params.py` too; run the rollout with `USE_NMPC=True` and check the `m_Rrate_zone` column
Don't:
- rely on the zone schedule with the LTV-QP default `k=8`, because a track whose tightest corner is near 0.2 1/m never saturates the curve and the schedule degrades to a mild global boost (NMPC uses `NMPC_CORNER_FACTOR_K`)
Key API: `_rrate_zone_scale`, `_rrate_stage_ramp`

### `controller/nmpc/qp_model.py`
Does: `_QPModelMixin`: builds the fixed-sparsity OSQP problem, horizon rollout, Jacobians, stage-0 rate weight and the cost.
Change it to:
- change constraint rows or cost assembly; mirror live `nmpc/qp_model.py`; run `python -m tuner.validation.nmpc_offline_check`
Don't:
- change sparsity patterns casually, because the OSQP structure is built once and a changed pattern needs a full check of solver consistency
- replace finite-difference Jacobians without re-verifying `_step_scalar == _step`
Key API: `_QPModelMixin`, `_csc_pattern`, `_build_qp`, `_rollout`, `_jacobians`, `_output_jacobians`, `_cost`

### `controller/nmpc/sqp_step.py`
Does: `_SQPStepMixin`: one Gauss-Newton SQP step (`_solve_step`) and `_project_feasible`.
Change it to:
- change trust region, backtracking or feasibility projection; mirror live `nmpc/sqp_step.py`; run `python -m tuner.validation.nmpc_offline_check` (monotonic convergence check)
Don't:
- drop the feasibility projection, because `dU=0` must stay feasible for the SQP loop to be safe
Key API: `_SQPStepMixin`, `_solve_step`, `_project_feasible`

### `controller/nmpc/solver.py`
Does: `NMPCController`: construction, `reset`, `path_reference`, `compute_step` (SQP loop, weight scheduling, latency compensation, telemetry dict).
Change it to:
- add a feature flag or a per-tick weight rule; add the constant to `settings/nmpc.py`, `nmpc_params.py`, `fsae_params.yaml`, live `nmpc/solver.py` and thread it from `sim/rollout/core.py`; run `python -m tuner.validation.nmpc_offline_check` and the rollout with `USE_NMPC=True`
Don't:
- construct it inside the step loop, because the warm start persists across ticks
- compose the corner blend and anti-hunt scalings, because they are alternatives (composition silently discarded one in an earlier bug)
Key API: `NMPCController`, `compute_step`, `path_reference`, `reset`

## model/

The plant and the controller's own prediction model. `model/vehicle_physics/` is the 25-state nonlinear plant that stands in for FSDS ("truth"). `bicycle_model.py` is the linearised 8-state model the LTV-QP predicts with. The gap between them is deliberate. `model/__init__.py` is a docstring-only marker.

### `model/__init__.py`
Does: package marker.
Change it to:
- nothing normally
Don't:
- add imports, because `bicycle_model.py` imports `model.vehicle_physics` and cycles are easy
Key API: none

### `model/bicycle_model.py`
Does: builds the 8-state discrete linear model (kinematic to dynamic blend, ZOH discretisation via `expm`) used by the LTV-QP each tick.
Change it to:
- change the kinematic/dynamic blend band (1 to 2.5 m/s) or state layout; live `lmpc/controller.py` holds the same model, edit both; run `python -m tuner.validation.recorded_map_rollout`
Don't:
- use it as ground truth, because it omits the reference path rotation and predicts zero turn-in before tracking error exists (the gap `controller/nmpc/` closes)
- change `VehicleParams` geometry without checking this file, because it reads the same `lf`/`lr`/`Iz`/tyre values
Key API: `get_8state_discrete_model`

### `model/vehicle_physics/__init__.py`
Does: re-exports the plant's public names (indices, `VehicleParams`, `init_plant_state`, `step_nonlinear_plant`, tyre functions, tracking helpers) so `import model.vehicle_physics as vp` keeps working.
Change it to:
- add an export when a new public helper is added to a submodule
Don't:
- treat its docstring as current for the state size: it says 24 states, `state.py` defines `N_STATES = 25`
Key API: `N_STATES`, `VehicleParams`, `step_nonlinear_plant`, `init_plant_state`, `plant_to_tracking_error`

### `model/vehicle_physics/state.py`
Does: named state indices (`IDX_*`, `N_STATES = 25`) and `init_plant_state`.
Change it to:
- add a state; update `plant_step.py`, `tracking.py` and every consumer of the vector; run `python -m tuner.validation.plant_openloop_validation`
Don't:
- reorder indices 0 to 7, because they line up with the MPC's 8-state vector so readers need no remapping
Key API: `IDX_X` ... `IDX_ALAT_LIM`, `N_STATES`, `init_plant_state`

### `model/vehicle_physics/params.py`
Does: `VehicleParams`, the single source of vehicle mass, geometry, tyre, suspension, aero, actuator and alat-ceiling values.
Change it to:
- refit `alat_ceiling_gain` or `alat_ceiling_tau`; run `python -m tuner.validation.plant_openloop_validation`, then `python -m tuner.validation.recorded_map_rollout`
- change `lf`/`lr`/`Iz`/`max_steer`; match live `lmpc/constants.py` and `MPCParams`
Don't:
- imitate the FSDS lateral-acceleration ceiling with tyre parameters (`mu`, cornering stiffness), because that was tried and fails full-lock and closed-loop checks; tune `alat_ceiling*`
- keep `alat_ceiling*` unequal to `ALAT_CEILING_*` in `settings/nmpc.py`, because the NMPC model and plant must share the law
- edit this without an expensive-tier review, because a wrong physical mechanism can pass offline validation
Key API: `VehicleParams`, `alat_ceiling_at`, `static_fz_per_corner`

### `model/vehicle_physics/tyres.py`
Does: stateless Pacejka MF94 lateral and longitudinal force functions.
Change it to:
- adjust curve coefficients passed by `plant_step.py`; run `python -m tuner.validation.plant_openloop_validation`
Don't:
- add state, because callers assume pure functions evaluated once per wheel per sub-step
Key API: `pacejka_lateral_mf94`, `pacejka_longitudinal_mf94`, `_mf94`

### `model/vehicle_physics/plant_step.py`
Does: `step_nonlinear_plant`, one control step (4 sub-steps) of suspension, tyre relaxation, drivetrain, aero and the alat-ceiling restoring moment.
Change it to:
- change the ceiling law or a force model; run `python -m tuner.validation.plant_openloop_validation` then `python -m tuner.validation.recorded_map_rollout`
Don't:
- change sub-step count casually, because tyre relaxation and suspension are stiff and the plant goes unstable
- model the ceiling as a clip, because the accepted form is a leaky-integral restoring yaw moment with lag
Key API: `step_nonlinear_plant`

### `model/vehicle_physics/tracking.py`
Does: bounded nearest-reference search, interpolated reference point and `plant_to_tracking_error` (e_y, e_psi, ...).
Change it to:
- change the search window or error definition; run `python -m tuner.validation.recorded_map_rollout`
Don't:
- change sign conventions here without checking the live `_error_state` in `lmpc/controller.py`, because both must define e_y and e_psi identically
Key API: `find_closest_reference_bounded`, `get_interpolated_ref_point`, `plant_to_tracking_error`

## planning/

Offline copy of the cone-to-centreline planner used by `sim/planner.py` and `sim/track_io.py`. Live counterparts are `boundary.py`, `cone_map.py`, `cone_sorting.py`, `path_utils.py` in `fsds_simulator/planning/fsae_planning/fsae_planning/` (and `ros2/src/fsae_planning/planning/fsae_planning/fsae_planning/`). The offline package imports as `planning.*`, the live one as `fsae_planning.*`, and the segment helpers are in `planning/geometry.py` offline but inlined in the live `boundary.py`. Files differ from the mirror (41 and 58 changed lines in boundary and path_utils, cone_map has extra comment lines, cone_sorting is identical), so a planner change needs a manual diff. Read the centreline-curvature-spike section of `docs/reference/reference_path_and_speed.md` before changing any of it.

### `planning/boundary.py`
Does: cone-wall mesh centreline planner (`build_path_walls`).
Change it to:
- change wall/midpoint distances or `_WALL_MAX_TURN_COS`; mirror the live `boundary.py`; run `python -m tuner.validation.recorded_map_rollout` with `USE_PLANNER=True`
Don't:
- change it without re-measuring the controller workarounds (curvature smoothing, tracking-error speed gate, speed-target rise limiter), because they are defence in depth for this planner's spikes
- rely on the defaults of the smoothing arguments, because `sim/planner.py` must pass the four tunables from `settings/planner.py` explicitly
Key API: `build_path_walls`, `build_wall_segments`

### `planning/cone_map.py`
Does: `ConeMap`, the persistent cone accumulator that merges detections within `MERGE_DIST`.
Change it to:
- change `MERGE_DIST`; mirror the live `cone_map.py`; run the rollout with `USE_PLANNER=True`
Don't:
- reset it mid-run, because the map grows monotonically and historical walls are preserved
Key API: `ConeMap`, `MERGE_DIST`

### `planning/cone_sorting.py`
Does: cone ordering, pairing and window/forward filtering helpers.
Change it to:
- change `filter_cones_window` radius defaults; mirror the live file (currently identical); run the rollout with `USE_PLANNER=True`
Don't:
- change it on one side only, because it is byte-identical to the live copy today
Key API: `sort_cones_nn`, `pair_cones_nn`, `filter_cones_forward`, `filter_cones_window`

### `planning/geometry.py`
Does: segment-intersection helpers shared by `boundary.py` and `path_utils.py`.
Change it to:
- fix an intersection edge case; the live `boundary.py` holds inline copies (`segment_crosses_walls`, `_seg_intersect`), edit both
Don't:
- import `boundary` from here, because the file exists to break the boundary/path_utils import cycle
Key API: `segment_crosses_walls`, `_seg_intersect`

### `planning/path_utils.py`
Does: centreline smoothing, path building, lookahead, resampling and `blend_paths`.
Change it to:
- retune `DEFAULT_SMOOTH_PER_PT` or the blend; mirror the live `path_utils.py`; run the rollout with `USE_PLANNER=True`
Don't:
- change smoothing defaults expecting the sim to follow, because live values come from `fsae_params.yaml` and offline values from `settings/planner.py`
- lift `_SMOOTH_N_CAP`, because the near-field shape would then change as the far horizon grows
Key API: `smooth_centreline`, `build_local_path`, `blend_paths`, `compute_centreline`, `DEFAULT_SMOOTH_PER_PT`

## angles.py

Top-level shared helper, kept outside `controller/` and `sim/` so neither dependency direction is violated.

### `angles.py`
Does: `wrap_angle`, wraps an angle to (-pi, pi] via atan2.
Change it to:
- nothing normally
Don't:
- reintroduce local copies, because it replaced two identical copies; `controller/nmpc/layout.py::_wrap` and `sim/rollout/reference.py::_normalize_angle` are now aliases of it
Key API: `wrap_angle`

## tracks/

Python package that locates recorded-track data (cone map, speed profile, raceline, centreline). Data files are not in this repo: `TRACKS_DIR` resolves to `ros2/src/fsae_planning/tracks/<name>/` and holds `cone_map.json`, `speed_profile.csv`, `raceline.csv`, `centerline.csv`. Filesystem-only, no numpy or scipy.

### `tracks/__init__.py`
Does: track path helpers, newest-track resolution, dated names, `resolve_map_arg` and the `DEFAULT_MAP` constant.
Change it to:
- add an exported file kind; update the exporters in `tuner/tools/`, then run `python -m tuner.validation.recorded_map_rollout`
- change default resolution; check `ros2/launch_all.sh` `TRACK=` and the launch-file defaults
Don't:
- expect `DEFAULT_MAP` to follow a newly recorded track in a running process, because it is computed once at import from `newest_track()`
- write a new track only into this repo, because a `fsae_planning`-only checkout must drive; files under `TRACKS_DIR` are local edits to the fsae_planning checkout and are never committed by an agent
- make `resolve_map_arg` fall back silently on an unknown name, because exporting the wrong track is costly to notice
- rename `DEFAULT_TRACK` semantics, because `None` means resolve the newest track at call time
Key API: `TRACKS_DIR`, `newest_track`, `cone_map_path`, `speed_profile_path`, `raceline_path`, `centerline_path`, `geometry_path`, `list_tracks`, `resolve_map_arg`, `default_out_for`, `dated_track_name`, `DEFAULT_MAP`

## `sim/`

The closed-loop rollout and its supporting simulator-side code. `sim/rollout/core.py` owns the tick loop, the phase files beside it hold stateless per-tick steps, and the remaining files supply perception, planning, noise, scoring, speed profiles and track loading. `sim/__init__.py` and `sim/rollout/__init__.py` are empty package markers (docstring only, or nothing).

Runtime overrides work because the phase files read `settings.X` at call time. Default argument values (`run_core_rollout`'s `use_planner`, `use_nmpc`, `n_horizon`, `eps`, `max_iter`, and `run_headless_rollout`'s `use_planner`) are bound once at import, so an override of those must happen before the first import of `sim.rollout.core`.

### `sim/rollout/core.py`
Does: runs one closed-loop rollout (nonlinear plant plus LTV-QP or NMPC) and returns score, metrics and optional history.
Change it to:
- add a new termination rule or a new history series, for example a new DNF trigger next to the stall check. Run `python -m tuner.validation.recorded_map_rollout` after.
- change the stall or launch constants (`STALL_CHECK_INTERVAL`, `STALL_MIN_DISTANCE`, `LAUNCH_SPEED_MPS`). `LAUNCH_SPEED_MPS` must equal the live lap tracker's launch speed, or offline and live lap times stop being comparable.
Don't:
- add a second rollout loop in a script or the GUI, because this is the only loop and the GUI, tuner and validation scripts all call it (a second copy drifts and scores stop matching).
- import GUI code here, because `tuner/offline_tuner.py` runs rollouts in multiprocessing workers and a GUI import would open a window in each.
- pass a misspelt key in `nmpc_overrides`, because keys are not validated and a typo is ignored without warning (check a swept field moves before trusting a null result).
Key API: `run_core_rollout`, `compute_step_budget`, `STALL_CHECK_INTERVAL`, `STALL_MIN_DISTANCE`

### `sim/rollout/reference.py`
Does: computes the controller's tracking error (e_y, e_psi) and reference heading, from the planner centreline when ready, else the oracle path.
Change it to:
- change the reference-heading rate limit (`_rate_limit_ref_psi`, gated by `settings.REF_HEADING_RATE_LIMIT_ENABLED`, default False, rate `settings.REF_HEADING_RISE_RATE`, default 90 deg/s). Run `python -m tuner.validation.recorded_map_rollout --planner`.
- change how a held pose (pose_age_ticks > 0) skips planning. This models the live pose-feed stall, so keep it consistent with `sim/sensor_noise.py`.
Don't:
- re-plan while the pose is held, because the live planner is triggered by pose, so a frozen pose means no new centreline and no new error (an earlier version re-planned and the controller was never blind).
- apply the limiter to the oracle branches, because the fixed path does not carry the excess the limiter targets.
Key API: `compute_reference`, `_rate_limit_ref_psi`

### `sim/rollout/speed_target.py`
Does: builds the speed target (oracle profile, optional live curvature cap), then applies the tracking-error gate, rise-rate limit and deficit clamp; also the time bonus.
Change it to:
- change `SPEED_TARGET_RISE_RATE`, `GATE_RATE_LIMIT` or `V_CURV_FALL_RATE`. Each mirrors a constant in the live controller node, so change both sides. Run `python -m tuner.validation.recorded_map_rollout`.
- change the deficit clamp value. That value is `settings.SPEED_TARGET_DEFICIT_MAX`, not a constant in this file, so edit it in the `settings/` submodule that holds it (grep for it) and mirror `MPCParams.speed_target_deficit_max`.
Don't:
- gate the target on measured speed near zero, because no target means no speed error, so no throttle and the car never moves (the clamp holds the target at `v_actual + DEFICIT_MAX` instead).
- compute time bonus as `1 - sim/optimal`, because sim time is never below the optimum so it would clip to 0 on every run.
Key API: `compute_speed_target`, `gate_and_rate_limit_speed_target`, `compute_time_bonus`, `PLANNER_V_MAX`, `PLANNER_V_MIN`

### `sim/rollout/delay.py`
Does: delay compensation (roll the linear error state through pending commands), believed pending-command count with jitter, and ground-truth tracking error for scoring.
Change it to:
- change the small-angle clip in `predict_ahead` (`_PREDICT_EPSI_CLIP`, 0.5 rad). Run `python -m tuner.validation.recorded_map_rollout --planner`, the sharp-corner case is where it matters.
- change jitter handling in `believed_pending_cmds` (`settings.DELAY_JITTER_STEPS`). Keep the over-estimate cap consistent with the live `MAX_DELAY_COMPENSATION_STEPS`.
Don't:
- draw more or fewer than one `delay_rng` sample per tick in `believed_pending_cmds`, because the rollout is seeded and reproducible per score and the draw count fixes the stream.
- score on the controller's e_y, because with SLAM noise or a planner reference it is a belief, not ground truth (`true_tracking_error` must be given `diverged=True` in those cases).
Key API: `predict_ahead`, `believed_pending_cmds`, `true_tracking_error`

### `sim/rollout/tick_solve.py`
Does: builds the NMPC controller once per rollout and runs the per-tick NMPC or LTV-QP solve, plus history recording.
Change it to:
- pass a new NMPC setting into `NMPCController`, for example a new `settings.NMPC_*` constant. Add it in `build_nmpc`, and mirror the live param. Run `python -m tuner.validation.nmpc_offline_check` then `python -m tuner.validation.recorded_map_rollout`.
- change the LTV-QP gain schedule in `solve_ltv_tick` (corner blend of Q, R, R_rate). Mirror the live LMPC adaptive-gain code.
Don't:
- drop a multiplier when setting `R_rate_scaled[0, 0]` from the corner blend, because that line overwrites the base value and every earlier multiplier (`m_rrate_antihunt`, `m_rrate_reversal`) must be reapplied (this bug class has recurred).
- expect `nmpc_overrides` to reach every field, because only these keys are read: `slack_linear_weight`, `corner_factor_k`, `rrate_stage_ramp_enabled`, `rrate_stage_near`, `rrate_zone_enabled`, `rrate_zone_boost_straight`, `rrate_zone_ease_approach`, `rrate_zone_floor_corner`, `rjerk_delta`, `rjerk_a`, `progress_enabled`, `q_progress`, `progress_reach`, `progress_v_min`.
- apply the LTV gain schedule to the NMPC path, because the nonlinear model anticipates bends structurally and the schedule would double-count.
Key API: `build_nmpc`, `solve_nmpc_tick`, `solve_ltv_tick`, `record_solve_history`, `record_horizon_prediction`

### `sim/perception.py`
Does: places boundary cones along a path (`place_cones`) and filters the cones the car can see (`SimPerception`), mirroring the live perception node.
Change it to:
- change the field of view (`LOOK_AHEAD`, `LOOK_WIDE`, `MIN_AHEAD`) or cone spacing. Mirror the live sim perception node, then run `python -m tuner.validation.recorded_map_rollout --planner`.
Don't:
- change `TRACK_HALF_WIDTH` casually, because a `settings` submodule imports it from here (it is 1.75 m, the FS spec half width) and it sets where synthetic cones sit.
- import `settings.PLANNER_*` at module top here, because `settings` imports this module (circular import).
Key API: `place_cones`, `SimPerception`, `TRACK_HALF_WIDTH`, `LOOK_AHEAD`

### `sim/planner.py`
Does: accumulates visible cones, builds a centreline with the shared planning code, and blends it over time (`SimPlanner`); sizes the step budget (`calculate_dynamic_max_steps`).
Change it to:
- change how the offline planner is fed, for example a new argument to `build_path_walls`. The live node passes the same values, so mirror `centerline_planner.py` and the `PLANNER_*` settings, which must equal the values in `fsae_params.yaml`. Read the known centreline-curvature-spike defect notes before editing planner behaviour.
Don't:
- omit the `PLANNER_*` keyword arguments, because the planning functions then fall back to their own hardcoded defaults instead of the live-tuned values.
Key API: `SimPlanner`, `calculate_dynamic_max_steps`

### `sim/sensor_noise.py`
Does: models imperfect localisation and perception for the rollout: `SlamNoise` (pose jitter plus drift), `ConeNoise` (per-cone jitter), `PoseFeedHold` (pose feed repeating its last sample).
Change it to:
- change the hold model (`settings.POSE_HOLD_*`) or noise magnitudes. `PoseFeedHold` is the measured dominant sim-to-real gap, so re-check with `python -m tuner.validation.recorded_map_rollout --planner`.
Don't:
- apply noise to the plant state or the score, because the car is judged on where it went.
- move `PoseFeedHold` before `SlamNoise`, because a held tick must repeat the corrupted pose the controller saw, not a freshly corrupted one.
Key API: `SlamNoise`, `ConeNoise`, `PoseFeedHold`

### `sim/scoring.py`
Does: the single definition of the 13 metrics (`RolloutMetrics`) and the composite score (`compute_composite_score`); lower is better.
Change it to:
- change a metric's accumulation or the tier logic. Then re-copy the change to the live `telemetry/scoring.py`, which is a byte-identical copy apart from inlined constants. Run `python -m tuner.validation.recorded_map_rollout`.
- change weights or scales. Those are in the `settings/` scoring submodule (`SCORE_WEIGHTS`, `METRIC_SCALES`), and the live copy inlines them, so edit both.
Don't:
- edit the live scoring copy first, because this file is the source of truth.
- reorder the `IDX_*` constants or the arguments of `compute_composite_score` without reordering `SCORE_WEIGHTS`, because the metrics array is built positionally.
- threshold on `progress` to decide "finished" when `reached_end` is available, because progress stops near 0.90 on a completed run.
Key API: `compute_composite_score`, `RolloutMetrics`, `IDX_RMSE`

### `sim/speed_profile.py`
Does: curvature-based speed profiling: `curvature_speed` (live-parity heuristic), `compute_speed_profile` (three-pass oracle profile), `tracking_error_speed_gate`, `optimal_lap_time`, corner-slowdown and smoothing helpers.
Change it to:
- change a `CURVATURE_SPEED_*` default. These must equal the defaults of the live `control_utils.curvature_speed`, or the oracle profile drifts faster than the car can go. Run `python -m tuner.validation.recorded_map_rollout`, then re-export tracks with `python -m tuner.tools.export_speed_profile`.
- change `OPTIMAL_LAP_*` limits (these set the time-bonus reference).
Don't:
- shorten `CURVATURE_SPEED_SCAN_END` (24 m) or raise `CURVATURE_SPEED_SCAN_START` above 0, because a short scan sees a hairpin too late and a nonzero start can skip a short corner entirely.
- add a second curvature heuristic, because `compute_speed_profile` calls `curvature_speed` so the oracle and live targets cannot diverge.
Key API: `curvature_speed`, `compute_speed_profile`, `compute_corner_slowdown_profile`, `tracking_error_speed_gate`, `optimal_lap_time`, `smooth_profile`, `compute_path_curvature`

### `sim/track_io.py`
Does: turns a recorded cone-map JSON into the `(path_X, path_Y, path_Psi, path_v, blue, yellow)` tuple by marching a virtual car around the lap with the live boundary planner.
Change it to:
- change the reconstruction (the `_MARCH_*` constants) when a new recording produces a broken centreline. Test on the affected track with `python -m tuner.validation.recorded_map_rollout --map <track>`, then re-export CSVs with `python -m tuner.tools.export_speed_profile <track>`.
Don't:
- replace the march with one global nearest-neighbour sort of all cones, because it pairs cones from different legs wherever the lap crosses near itself.
- change `PATH_N_POINTS` without checking `tuner/offline_tuner.py`, which keeps a matching default.
Key API: `load_recorded_track`, `load_cone_map`, `PATH_N_POINTS`

## `tuner/validation/`

The three commands that count as a correctness bar for offline changes. None is a test suite: each prints numbers or PASS/FAIL lines to read. Run from `fsae_MPCTest/`.

### `tuner/validation/recorded_map_rollout.py`
Does: runs one headless closed-loop rollout on the recorded map and prints steering saturation, heading error and a_lat against the live-car reference values.
Change it to:
- add a reported statistic in `summarise`, or a CLI knob in `main` (the `--mode/--gain/--tau/--ceiling/--no-ceiling` flags override `VehicleParams` plant fields). Run `python -m tuner.validation.recorded_map_rollout`.
Don't:
- compare the default oracle-path run with the `LIVE` table, because the live values were measured planner-in-loop, so use `--planner` for a like-for-like comparison.
- treat the `LIVE` numbers as refreshed, because they are hardcoded (21.1 %, 15.9 / 42.0 deg, 12.34, 9.8 %, 1.62 per s).
Key API: `run`, `summarise`, `LIVE`

### `tuner/validation/nmpc_offline_check.py`
Does: NMPC self-consistency checks: scalar vs vectorised step parity, monotone SQP cost, turn-in versus the LTV-QP, and a closed-loop LTV-vs-NMPC run on the recorded map.
Change it to:
- add a check as a `test_*` function that calls `check(name, ok, detail)` and is listed in `main`. Run `python -m tuner.validation.nmpc_offline_check` after any edit to `controller/nmpc/`.
Don't:
- skip it after touching the NMPC solver files, because the `_step_scalar == _step` parity and the monotone-cost check are the only guards on Jacobian and state-anchoring mistakes.
- read a `[SKIP]` on check 4 as a pass, because it means the recorded track could not be loaded.
Key API: `main`, `check`, `test_model_parity`, `test_convergence`, `test_turn_in`, `test_closed_loop`

### `tuner/validation/plant_openloop_validation.py`
Does: replays the measured open-loop steering step and sweep experiments through the offline plant and prints residuals (positive means the sim corners harder than FSDS).
Change it to:
- add a report or a plant parameter override. Run `python -m tuner.validation.plant_openloop_validation` (also `--ab`, `--robustness`) after any edit to `model/vehicle_physics/`.
Don't:
- run it without measurement logs, because it reads the newest `steering_step_*.csv` and `steering_sysid_*.csv` from `~/fsae_logs`; presence of those logs on this machine is not verified.
- read the low-speed rows as findings, because they are a known rig confound (the ceiling never engages below about 6 to 7 m/s).
Key API: `run_plant`, `report_step`, `report_sweep`, `report_ab`, `report_robustness`

## `tuner/investigations/`

One-off diagnostic scripts, kept because a null result nobody can find gets re-tested. Each answers one question; run each with `python -m` and its dotted module path under `tuner.investigations`. They are not validation gates. Several read live logs or need an FSDS measurement run first.

### `tuner/investigations/analyze_adaptive_log.py`
Does: answers "which adaptive multiplier caused a wide corner" from a live control CSV, per corner, splitting corners by whether `corner_demand` exceeds 1.
Change it to:
- update the `GROUPS` column names if the live adaptive columns change. Run it on a fresh live CSV.
Don't:
- rely on it for current logs without checking columns first, because it needs `corner_demand`, `m_Q_ey_*`, `uturn_severity` and similar columns from the removed lookahead gain-scheduling family, and the current live `ADAPTIVE_COLUMNS` may no longer carry all of them (not verified per column). It exits with a message when `corner_demand` is absent.
Key API: `load`, `corners`, `main`

### `tuner/investigations/brake_sysid_analysis.py`
Does: answers "what deceleration does FSDS deliver for a given brake command and speed" from a `brake_sysid` node log, and fits flat versus flat-plus-slope laws.
Change it to:
- change the fit or binning in `summarise_points`. Run it on a new brake sweep log.
Don't:
- feed it a normal driving log, because it needs `phase == record` windows from the open-loop brake node.
Key API: `load`, `summarise_points`, `main`

### `tuner/investigations/live_vs_sim_diagnostics.py`
Does: answers "what does live saturation look like, and does the offline sim match it like for like" (saturation episode structure, speed tracking, reference quality).
Change it to:
- add a compared quantity in `describe_*`/`conditional` and call it from both `live_report` and `sim_report`.
Don't:
- expect live rows without logs, because `live_report` globs `mpc_standalone_control_*.csv` in `<repo>/fsae_logs` (note: `plant_openloop_validation` reads `~/fsae_logs`, a different folder). Use `--no-sim` for live only.
Key API: `live_report`, `sim_report`, `episodes`, `reference_quality`

### `tuner/investigations/ref_heading_limiter_ab.py`
Does: intended to answer "does the reference-heading rate limit close the steering-saturation gap" by sweeping `REF_HEADING_RISE_RATE` on the recorded map.
Change it to:
- set the values through `settings` (`settings.REF_HEADING_RATE_LIMIT_ENABLED`, `settings.REF_HEADING_RISE_RATE`) instead of on the `sim.rollout.core` module. `compute_reference` reads them from `settings`.
Don't:
- trust any row of its table, because it assigns `rc.REF_HEADING_RATE_LIMIT_ENABLED` and `rc.REF_HEADING_RISE_RATE` on a module that never bound those names, so every row silently uses the `settings` default (limiter off), not the rate the row claims.
- read `rollout.get("score")` as a score, because the result key is `composite_score` (the value is not printed).
Key API: `run_once`, `main`

### `tuner/investigations/ref_heading_limiter_suite_check.py`
Does: intended to answer "does the reference-heading limiter's gain hold across `settings.VALIDATION_SUITE` or is it a one-map artefact".
Change it to:
- patch `settings` rather than `sim.rollout.core`, as for `ref_heading_limiter_ab.py`.
Don't:
- trust its OFF, 70 and 65 deg/s rows, because it monkeypatches `rc.REF_HEADING_*` names never bound where patched, so all rows silently use the `settings` default.
Key API: `run_one`, `main`

### `tuner/investigations/reference_excess_mechanism_check.py`
Does: answers "are ticks where the planner reference heading swings much faster than the fixed reference explained by the seed-midpoint anchor jump".
Change it to:
- wrap a different planner function in the same non-invasive way (replace `boundary.build_path_walls` and `sim_planner.build_path_walls`, restore in `finally`).
Don't:
- leave the wrapper installed, because it replaces module attributes and only the `finally` block restores them.
Key API: `main`, `_wrapped`, `_seed_midpoint`

### `tuner/investigations/reference_heading_geometry_check.py`
Does: answers "is the reference-heading swing caused by the online planner rebuild, or by the track geometry itself" by comparing `psi - e_psi` with `psi - e_psi_true`.
Change it to:
- run on another track by changing the map passed to `load_recorded_track`.
Don't:
- read its ratio as a planner defect verdict on its own, because it measures one recorded map with one weight set.
Key API: `main`

### `tuner/investigations/steering_chatter_check.py`
Does: answers "how much tick-to-tick steering chatter does a controller and settings combination produce" (delta std, mean abs delta, sign-flip rate) on a static path.
Change it to:
- add a metric to the printout. Reproduce a sweep with `--set NAME=VALUE`, for example `python -m tuner.investigations.steering_chatter_check --set NMPC_SQP_ITERS=2`.
Don't:
- run it inside a long-lived process, because overrides are applied to `settings` before `sim.rollout.core` is imported (default-argument constants bind at import), so a fresh process per configuration is required.
- trust a low-chatter result that also warns about non-solved NMPC ticks, because solver failures make the metrics not comparable.
Key API: `main`, `_parse_args`

### `tuner/investigations/steering_response.py`
Does: answers "how weak is the car's yaw response to steering" from a live control CSV by inverting the kinematic bicycle (`delta = atan(L r / v)`) and fitting understeer `K_us`.
Change it to:
- change the quasi-steady gate constants (`MAX_CMD_RATE_RAD_S`, `MAX_YAW_ACCEL`, `MIN_SPEED`, `MIN_STEER_RAD`). Run it on a new live log.
Don't:
- tighten the gate to a multi-tick hold, because on a hunting car that returns zero rows.
- change `WHEELBASE` without checking `model/vehicle_physics/params.py`, since it must equal `lf + lr` (0.70 + 0.85 = 1.55 m today).
Key API: `report`, `fit_understeer`, `quasi_steady_mask`, `load_control_log`

### `tuner/investigations/steering_step_analysis.py`
Does: answers "which mechanism caps FSDS yaw rate" (hard clip, speed-scaled authority, or active damping) from `steering_step` node transients.
Change it to:
- change discriminators in `analyse_trial` (overshoot, peak versus ceiling, rise shape, settling).
Don't:
- drop the median filter (`MEDFILT`), because single-sample yaw spikes in FSDS telemetry make the overshoot statistic measure the glitch.
Key API: `analyse_trial`, `load`, `main`

### `tuner/investigations/steering_sysid_analysis.py`
Does: answers "what steering ratio does FSDS achieve per (speed, steering) point, and which mechanism does the pattern imply" from a `steering_sysid` node log; also fits the implied wheelbase.
Change it to:
- change the pattern tests in `main` or the fit in `fit_wheelbase`. Then run `python -m tuner.validation.plant_openloop_validation` if the plant ceiling is being refitted from the result.
Don't:
- use it to fit tyre parameters to imitate the cap, because the cap is modelled by `alat_ceiling*` and tyre scaling was already tried and failed.
Key API: `summarise_points`, `fit_wheelbase`, `main`

## `tuner/tools/`

Utilities that produce or check artefacts: exported track CSVs, log playback, parameter sync and the doc linter. Run each with `python -m` and its dotted module path under `tuner.tools`.

### `tuner/tools/export_speed_profile.py`
Does: exports a recorded map's oracle path and speed profile to `speed_profile.csv` (x, y, psi, v_target) for the live node.
Change it to:
- change the export format or the corner-test mode (`--corner-slowdown`). Re-run after any change to `sim/speed_profile.py` or a new map, then `python -m tuner.validation.recorded_map_rollout`.
Don't:
- change the four CSV columns without changing the live loader, because it is a small CSV reader with no scipy.
- overwrite `speed_profile.csv` with a corner-test profile, because the corner mode writes a separate corner-test CSV next to it on purpose.
Key API: `export`, `main`, `CORNER_TEST_NAME`

### `tuner/tools/raceline_optimizer.py`
Does: writes a minimum-lap-time racing line (`raceline.csv`) or a centreline with optimised speed (`--mode centerline`, `centerline.csv`) using an iterative curvature-reduction method.
Change it to:
- change margins or iteration count (`--iters`, `--margin`, `ALAT_MARGIN`, `BRAKE_MARGIN`). Re-export, then run `python -m tuner.validation.recorded_map_rollout`.
Don't:
- profile against the physical grip limit, because the car is held to the FSDS lateral-acceleration ceiling and the module uses `alat_ceiling_at(v)` with a margin (planning at 100 percent leaves no budget for corrections).
- pair a raceline path with a centreline speed profile, because speed must describe the line being driven.
Key API: `optimize_raceline`, `export`, `check_slip`, `build_shaped_heading_profile`, `max_yaw_rate`

### `tuner/tools/plot_playback.py`
Does: interactive time-scrubbing playback of one or more control logs, with signal panels, trajectory and a zoomed car view.
Change it to:
- add a default signal (`DEFAULT_SIGNALS`) or a log locator. Test by opening a log from `fsds_simulator/recorded_runs/`.
Don't:
- expect it to find live logs by itself, because auto-load reads `fsds_simulator/recorded_runs/` (one subdirectory per controller: LMPC, NMPC, Stanley, plus an optional graph directory), and runs must be copied there from the log directory.
- put a bare `kappa` in `--signals`, because there is no such column (`corner_frac` stands in).
Key API: `Playback`, `load_log`, `find_latest_log`, `find_latest_per_folder`, `find_all_logs`, `main`

### `tuner/tools/sync_mpc_params.py`
Does: copies the three live param files (`mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`) from the live tree to the `fsds_simulator/` mirror and the `fsae_autonomous` working tree.
Change it to:
- add a destination in `_AUTONOMOUS_CANDIDATES` when the checkout moves. Dry run first: `python -m tuner.tools.sync_mpc_params`, then `--apply`.
Don't:
- use it for non-param source files (`controller.py`, `solver.py` and so on), because those need manual mirroring.
- treat it as syncing `settings/`, because the offline constants are a separate parity boundary edited by hand.
- commit or push `fsae_autonomous` from it, because it only writes the local working tree.
Key API: `main`, `_destinations`, `_find_autonomous_root`

### `tuner/tools/doc_lint.py`
Does: lints docs and code conventions: links and anchors, backticked paths, `python -m` targets, style, voice, module-reference coverage, and `from settings.<sub> import` misuse.
Change it to:
- add a rule as a new `check_*` function called from `main`. Run `python -m tuner.tools.doc_lint --strict`.
Don't:
- lint `docs/logs/` contents, because they are frozen (only their README is checked).
Key API: `main`, `check_links`, `check_paths`, `check_style`, `module_ref_coverage`, `check_code_imports`

## `tuner/` (top level)

The CMA-ES weight tuner, the metric reporter for the GUI, and the shared CSV helpers. `tuner/__init__.py` is a package marker.

### `tuner/offline_tuner.py`
Does: searches Q, R and R_rate multipliers (plus optional NMPC rate-shaping values) with Optuna pre-search and BIPOP lq-CMA-ES over synthetic paths, and appends results to `docs/logs/tuning_history.txt`.
Change it to:
- change search bounds (`Q_BOUNDS`, `R_BOUNDS`, `R_RATE_BOUNDS`) or the NMPC search list (`TUNABLE_NMPC`). After editing `TUNABLE_NMPC`, move each field alone to a far bound and confirm the score changes, then run `python -m tuner.validation.recorded_map_rollout` on the result.
- add a synthetic path in `build_synthetic_paths`, and list it in `settings.VALIDATION_SUITE` if it should be scored.
Don't:
- lower the `Q_BOUNDS[0]` floor below 1.0, because CMA-ES then found weight sets that collapsed the e_y cost and let heading error grow before turn-in (the scoring gap remains open).
- tune NMPC fields with `settings.USE_NMPC` false, because the LTV-QP ignores them and those dimensions waste the population; set it before this module is imported.
- take a tuned set to the car on the offline score alone, because offline scores do not yet predict the car (see the simulator fidelity reference).
Key API: `run_headless_rollout`, `evaluate_all_paths`, `get_cached_model`, `SYNTHETIC_PATHS`, `PATH_NAMES`, `INITIAL_CONDITIONS`, `vector_to_weights`, `vector_to_nmpc_overrides`, `run_optuna_presearch`, `log_results_to_history`

### `tuner/performance_stats.py`
Does: scores a stored rollout history dict through `RolloutMetrics` and prints a breakdown (`report_performance_metrics`), and benchmarks weights over every synthetic path (`benchmark_weights`).
Change it to:
- add a printed line to the report. Keep scoring itself in `sim/scoring.py`. Run `python -m tuner.validation.recorded_map_rollout` if metric definitions were touched upstream.
Don't:
- re-derive metric formulas here, because it replays history through the same accumulator on purpose.
- import it in a worker process expecting a light import, because it imports `tuner.offline_tuner` (and so `cma`).
Key API: `report_performance_metrics`, `benchmark_weights`

### `tuner/csv_log.py`
Does: shared CSV parsing for telemetry logs that start with a `#` metadata block: `read_data_lines`, `parse_rows`, `load_columns`, `medfilt`.
Change it to:
- handle a new log quirk in `parse_rows`. Run any investigation script on a real log to confirm it still loads.
Don't:
- parse logs separately in new scripts, because this module already drops the comment block and malformed trailing rows.
Key API: `load_columns`, `read_data_lines`, `parse_rows`, `medfilt`

## GUI: the launcher package and the two matplotlib tools

Two different GUIs live under `gui/`. They share nothing except the folder.

- **Launcher** (`python -m gui.launcher`, run from the repo root): a Tk window with five tabs. It contains no simulation, plotting or tuning logic. Every button shells out to an existing tool or rewrites one line of a text file with a regex.
- **`gui/simulation.py`** (`python -m gui.simulation`): a 2D matplotlib closed-loop tester. Rough signal only, its dynamics do not match FSDS.
- **`gui/manual_drive.py`** (`python -m gui.manual_drive`): a keyboard-driven variant of the same plant.

Launch as a module from the repo root. Both matplotlib scripts import top-level packages (`model`, `sim`, `tuner`, `settings`) and run their whole UI at import time (there is no `main()` guard), so importing them opens a window. Their own docstrings say `python gui/simulation.py`, which does not put the repo root on `sys.path`. The launcher's "Run Offline Sim" button uses `-m gui.simulation`.

There is no automated GUI test and Tk cannot be instantiated headless. Manual check after any GUI edit: run `python -m gui.launcher`, click through every tab, and confirm the edited button does what it says (a `NameError` from a missing import only shows when the button is clicked). A static check that catches missing imports without a display: pyflakes on `gui/launcher` (a dev tool, not a repo dependency).

### Which file is edited by which tab

This table is the precedence map. A launched node reads a param from a `launch_all.sh` CLI arg first, `fsae_params.yaml` second and the dataclass default last. A tab that writes only a lower layer has no effect when a higher layer holds the value.

| Tab | Writes | Layer it controls |
|---|---|---|
| Launch Sim | `ros2/launch_all.sh` (plain vars and the commented-out `NMPC_PROGRESS_ENABLED` shortlist line) | top layer (CLI args) |
| Settings | `settings/*.py`, live `mpc_params.py` and `nmpc_params.py`, live `fsae_params.yaml`, and the same three files under `fsds_simulator/` | offline constants, YAML layer, dataclass layer |
| Profiles | nothing directly, it calls Settings' and Launch's own capture and apply methods | both |
| Debug a Log | nothing | none |
| Run Offline Sim | nothing | none |

A field that can be set both by a `launch_all.sh` CLI arg and by a dataclass or YAML default belongs in the Launch tab only. A Settings-tab copy silently does nothing while the `launch_all.sh` line is honoured. `NMPC_PROGRESS_ENABLED` was duplicated this way and the Settings copy was removed.

### `gui/__init__.py`

Does: package marker with a one-line docstring.
Change it to:
- Nothing. Add a re-export only if another module needs one, then check with `python -c "import gui"`.
Don't:
- Import `gui.simulation` or `gui.manual_drive` here, because both open a matplotlib window at import time.
Key API: none.

### `gui/launcher/__init__.py`

Does: package docstring (the best overview of the launcher) and re-export of `LauncherApp` and `main`.
Change it to:
- Update the docstring tab list when a tab is added, and keep the `LAYOUT` list in step with the real modules.
Don't:
- Put logic here, because `python -m gui.launcher` enters through `__main__.py` and the docstring is the only documentation the package has.
Key API: `LauncherApp`, `main`.

### `gui/launcher/__main__.py`

Does: lets `python -m gui.launcher` run now that the launcher is a package.
Change it to:
- Nothing. It calls `gui.launcher.app.main()`.
Don't:
- Add argument parsing, because the launcher takes no arguments and resolves every path from its own location.
Key API: none (calls `main`).

### `gui/launcher/app.py`

Does: `LauncherApp`, the five-tab `tk.Tk` window, plus `main()`.
Change it to:
- Add a tab: build it in `LauncherApp.__init__` and call `notebook.add(...)`. If it needs Settings or Launch state, pass those tab objects in the way `ProfilesTab` receives them. Check by launching and switching to the new tab.
- Change what happens on tab switch: edit `_on_tab_changed`.
Don't:
- Remove the `refresh_from_disk()` call on arrival at the Launch tab, because the tab seeds its widgets from `launch_all.sh` once and the next Launch click writes those widgets back, silently reverting any newer on-disk edit.
- Replace the `after_idle` reset of `_reentering_tab_guard` with a try-finally block, because Tk delivers the re-select event after `select()` returns, so a `finally` clears the guard too early.
- Drop the 200 ms `_pump_for_signals` timer, because Tk's `mainloop()` blocks in C and never runs the Python SIGINT handler without it. Ctrl+C would then kill the GUI and strand the `launch_all.sh` process group.
Key API: `LauncherApp`, `main`, `_on_close` (stops a running sim, warns about unsaved Settings edits), `_on_tab_changed`.

### `gui/launcher/paths.py`

Does: `RepoPaths`, a frozen dataclass of every file and directory the GUI touches, resolved once from the file's own location.
Change it to:
- Add a path: add a field and fill it in `_repo_paths()`. Check by printing `_repo_paths()` from a Python shell.
- Handle a moved checkout: the layout is assumed (`fsae_MPCTest/` inside the outer FSDS repo, with `ros2/` beside it). Edit `_repo_paths()` only.
Don't:
- Point `fsae_logs_dir` at the home directory, because it must equal `launch_all.sh`'s `log_dir` (outer repo root plus `fsae_logs`). The Stop button's save-this-run prompt only finds a CSV there.
- Forget the mirror fields, because a Settings save writes the live files and their `fsds_simulator/` copies together.
Key API: `RepoPaths`, `_repo_paths`. Fields: `fsae_mpctest`, `fsds_root`, `launch_all_sh`, `tracks_dir`, `recorded_runs_dir`, `fsae_logs_dir`, `settings_dir`, `mpc_params_py`, `nmpc_params_py`, `fsae_params_yaml`, `mirror_*` (three), `profiles_dir`.

### `gui/launcher/file_edit.py`

Does: regex read and write helpers for `NAME = value` lines in `launch_all.sh` and `settings/*.py`, commented shortlist lines, YAML `controller:` fields, and dataclass `field(default=...)` values.
Change it to:
- Support a new file shape: add a `_xxx_pattern` plus a `_read_xxx` and `_rewrite_xxx` pair. Check by rewriting a copy of the file and diffing.
Don't:
- Loosen the `\b` after the name in the patterns, because `NMPC_Q_E_Y` would then match the `NMPC_Q_E_YD` line.
- Change the dataclass pattern to capture the whole line, because `metadata` dicts span several lines and must stay untouched. It captures only the default's value up to the next comma.
- Change the fixed 4-space YAML indent to `\s*`, because it keeps the pattern from matching a same-named key under a different node's `ros__parameters`.
- Assume a settings field lives in a known submodule, because `_settings_file_for` searches every `settings/*.py` fresh on each call.
Key API: `_read_var`, `_rewrite_var` (file or `settings/` directory), `_read_shortlist_var`, `_rewrite_shortlist_var`, `_read_yaml_field`, `_rewrite_yaml_field`, `_read_dataclass_field`, `_rewrite_dataclass_field`, `_read_dataclass_field_desc` (tooltip text taken from the dataclass metadata so it cannot drift), `_backup_once` (writes `<file>.bak` once per session, per submodule for `settings/`).

### `gui/launcher/process_utils.py`

Does: detached subprocess start and stop, sibling path-CSV lookup, and run-label insertion for saved runs.
Change it to:
- Change the saved-run naming: edit `_insert_run_label`. Check that `tuner.tools.plot_playback` still discovers the renamed file (it needs `_control_` or `_path_` plus the stamp).
Don't:
- Start children without `start_new_session=True`, because `_stop_process` signals the whole process group with SIGINT (the same as Ctrl+C) and relies on `launch_all.sh`'s own `trap cleanup`.
- Import from `tuner`, because the GUI deliberately re-implements the `_control_` to `_path_` name swap to stay independent of tuner internals.
Key API: `_run_detached`, `_stop_process`, `_sibling_path_csv`, `_insert_run_label`.

### `gui/launcher/theme.py`

Does: the dark "clam"-based ttk palette and shared widget helpers.
Change it to:
- Add a style: add it in `apply_theme`. Runtime colour changes on ttk widgets need a named style, so add one (as `Success.TLabel` does) instead of calling `.config(foreground=...)`.
Don't:
- Build a tab body without `_make_scrollable`, because the fixed 780x680 window clips overflow with no way to reach it.
Key API: `Palette`, `apply_theme`, `_field_label` (label plus a muted description row below it), `_make_scrollable`.

### `gui/launcher/tabs/__init__.py`

Does: empty package marker.
Change it to:
- Nothing.
Don't:
- Put shared helpers here, because shared code lives one level up (`theme.py`, `file_edit.py`, `process_utils.py`).
Key API: none.

### `gui/launcher/tabs/launch.py`

Does: Tab 1, Launch Sim. Rewrites the commonly changed `launch_all.sh` variables, then runs `bash launch_all.sh` detached from `ros2/`. Also Stop, save-run-to-`recorded_runs/`, new-track export and a temporary brake sysid button.
Change it to:
- Add a `launch_all.sh` variable: add its widget, then add it to `_pending_values` (plain `NAME=value` lines only), `refresh_from_disk` and `apply_profile_values`. Check by clicking Launch, confirming, and diffing `ros2/launch_all.sh` (a `.bak` is made on the first write of the session).
- Add a commented-out shortlist var: use `_read_shortlist_var` and `_rewrite_shortlist_var`, like `NMPC_PROGRESS_ENABLED`. The plain helpers only match uncommented lines.
Don't:
- Add a checkbox here for a field the Settings tab also edits (or the reverse), because the CLI arg written here overrides YAML, which overrides the dataclass default. The Settings copy would have no effect.
- Skip `refresh_from_disk` on new fields, because the tab otherwise reverts newer on-disk edits on the next Launch click.
- Do the export work on the Tk thread or call `after()` from the worker, because `after()` from a worker thread hung in practice. The worker only pushes onto a queue that `_poll_export_queue` drains on the main thread.
- Rely on the record-new-track toggle to persist, because it saves the prior controller and precomputed toggles, forces Stanley with both precomputed toggles off, and restores them on uncheck. Record mode is not captured in profiles.
Key API: `LaunchTab`, `_pending_values`, `refresh_from_disk`, `capture_profile_values`, `apply_profile_values`, `stop_running_sim`, `_on_export` (runs `tuner.tools.export_speed_profile`, then `tuner.tools.raceline_optimizer` twice, second with `--mode centerline`).

Behaviour to know:

- **Controller radio**: Stanley writes `CONTROLLER=stanley`. LTV writes `CONTROLLER=mpc`, `USE_NMPC=false`. NMPC writes `CONTROLLER=mpc`, `USE_NMPC=true`.
- **Progress term row**: shown only for NMPC, experimental, off by default. The confirm dialog notes that `NMPC_SLACK_LINEAR_WEIGHT` is not set by this checkbox.
- **Stop**: sends SIGINT to the process group, then polls `fsae_logs/` for up to 5 s for a new `*_control_*.csv` (mtime within 1 s of launch start) and offers to move it and its path CSV into `fsds_simulator/recorded_runs/<Stanley|LMPC|NMPC>/`, with an optional label.
- **Export & Save Track**: enabled only after a launch in record mode. After the three exporters succeed it copies the track directory into `fsds_simulator/tracks/`. That copy adds a directory to the mirror that may not have existed, which conflicts with the rule against adding files to `fsds_simulator/`.
- **Run Brake Sysid**: temporary. Sets `RUN_BRAKE_SYSID=true`, starts the script, then immediately writes `false` back.

### `gui/launcher/tabs/log_debug.py`

Does: Tab 2, Debug a Log. Lists `*_control_*.csv` under `fsae_logs/` and `fsds_simulator/recorded_runs/` (root and one sub-folder level) and opens `tuner.tools.plot_playback` on the selection.
Change it to:
- List another log location: extend `_find_control_csvs`. Check with Refresh.
Don't:
- Parse CSVs here, because playback is a separate process and this tab only passes file paths.
Key API: `LogDebugTab`, `_find_control_csvs`. "Debug Latest (auto)" runs `plot_playback` with no arguments.

### `gui/launcher/tabs/offline_sim.py`

Does: Tab 3, Run Offline Sim. One button that runs `python -m gui.simulation` from the repo root.
Change it to:
- Change the warning text. Keep the statement that results are a rough signal and must be cross-checked with `python -m tuner.validation.recorded_map_rollout` or a real FSDS session.
Don't:
- Add tuning controls here, because Q/R weights come from `settings/` (Settings tab) and track selection lives inside the tool.
Key API: `OfflineSimTab`.

### `gui/launcher/tabs/settings.py`

Does: Tab 4, Settings. Edits the commonly retuned constants and syncs them to every place a launched node could read them.
Change it to:
- Add a scalar field: add a row to `_SCALAR_FIELDS` (label, `settings/` name, dataclass field name or `None`, kind). Add an NMPC override to `_NMPC_OVERRIDE_FIELDS`, a flag to `_FEATURE_GROUPS`, a progress-term numeric to `_NMPC_PROGRESS_FIELDS`. `_profile_field_names()` is derived from these tables, so profiles pick the field up automatically. Check by saving, then running `git diff` in `fsae_MPCTest/` and the live tree.
- Add a live-only flag: use `None` for the `settings/` name (as `delay_compensation_enabled` does). Such a flag is not captured by profiles.
Don't:
- Add a field that a `launch_all.sh` CLI arg can also set, because the CLI arg overrides what this tab writes. Put it in the Launch tab.
- Write only the dataclass default. `_sync_live_field` writes the dataclass in `mpc_params.py` or `nmpc_params.py`, both `fsds_simulator/` copies, and both `fsae_params.yaml` copies. The YAML overrides the dataclass at ROS parameter declaration, and a stale YAML kept `r_a_accel` and `nmpc_track_halfwidth` at old values after a GUI save.
- Treat the `None` entries in `_LIST_FIELDS` as unused states. They mean no live dataclass field to sync to. `R_diag[1]` (a_cmd) is a real, nominal-only weight. Display names come from `_LIST_FIELD_INDEX_NAMES`.
- Expect a save to change a running sim. Restart the sim.
Key API: `SettingsTab`, `_on_save`, `_sync_live_field`, `capture_profile_values`, `apply_profile_values` (fills widgets then calls `_on_save`, so profile loading writes every file exactly as a manual save does), `has_unsaved_changes`, `_profile_field_names`, `_on_overwrite_all_params` (dry run of `tuner.tools.sync_mpc_params`, a confirm dialog naming the destinations that differ, then `--apply`).

Behaviour to know:

- **NMPC override convention**: an unchecked override writes `-1.0`, which means inherit the base weight. A checked override writes the entered number.
- **Value round-trip**: floats are written with `repr`, booleans as `True` or `False` (YAML gets lowercase).
- **Dirty tracking**: a write-trace on every widget variable. The app warns on switching away from Settings or closing with unsaved edits. A failed save keeps the dirty flag.
- **Overwrite All Params**: independent of the tab's widgets and does not touch `settings/`. It copies the live tree's three param files into `fsae_autonomous` and the mirror. It never commits.

### `gui/launcher/tabs/profiles.py`

Does: Tab 5, Profiles. Save, load and delete named JSON snapshots in `settings_profiles/`.
Change it to:
- Change what a profile holds: change `capture_profile_values` and `apply_profile_values` on the Settings or Launch tab, not this file. The two key sets are disjoint (`settings/` names such as `Q_diag`, and `launch_all.sh` names such as `TRACK`), so they merge into one flat dict.
Don't:
- Write files from this tab on load, because loading goes through `SettingsTab.apply_profile_values` (which saves immediately) and `LaunchTab.apply_profile_values` (widgets only, `launch_all.sh` changes at the next Launch click).
Key API: `ProfilesTab`, `_save_current`, `_load_selected`, `_delete_selected`.

### `gui/manual_drive.py`

Does: drive the 24-state nonlinear plant by hand (W throttle, S brake, A and D steer, Space full brake) over a synthetic path with placed cones. No MPC, no scoring, no tracking error.
Change it to:
- Change the feel: `STEER_RATE` (3.0 rad/s) and `ACCEL_RATE` (20.0 m/s^3) ramp the command toward the key target each tick. Check by running `python -m gui.manual_drive`, loading a path and driving.
Don't:
- Treat a manual run as a scored or MPC-comparable result, because nothing here computes errors.
Key API (module level, no `main`): `load_test_path`, `start_driving`, `reset_drive`, `update_frame` (`FuncAnimation` at `settings.DT`), `get_car_triangle`. Imports `step_nonlinear_plant`, `init_plant_state`, `VehicleParams`, `SYNTHETIC_PATHS`, `PATH_NAMES`, `place_cones`.

### `gui/simulation.py`

Does: interactive closed-loop tester. Load a synthetic path or a recorded track, set initial lateral and yaw error, run one rollout through `sim.rollout.core.run_core_rollout`, scrub the history, show metrics, benchmark all synthetic paths.
Change it to:
- Change what a run uses: weights come from `settings.Q_diag`, `R_diag`, `R_rate_diag` at import. `use_planner` defaults to `settings.USE_PLANNER` (`False` in `settings/general.py`, so the default is the oracle reference path). Check by running `python -m gui.simulation` and Start Sim.
- Change track sources: `RECORDED_TRACK_DIR` is `tracks.TRACKS_DIR` (the `fsae_planning` sibling), `LEGACY_RECORDED_TRACK_DIR` is `fsds_simulator/cone_maps/`. "Load Recorded Track" cycles newest first over both.
Don't:
- Trust its score against FSDS. Cross-check with `python -m tuner.validation.recorded_map_rollout`.
- Add rollout logic here, because the loop lives in `sim/rollout/core.py`. Only the initial-condition jitter (`rng_seed`) is GUI-only.
- Change how `optimal_time` is computed without matching `tuner/offline_tuner.py`, because `speed_profile.optimal_lap_time` anchors the score's time term and the GUI runs arbitrary paths.
Key API (module level): `simulate_closed_loop`, `run_simulation`, `run_optimize` (Show Metrics), `run_benchmark` (3 repeats per synthetic path, blocks the GUI), `load_test_path`, `load_recorded_track_cb`, `update_scrub_frame`.

## Top-level tracked files

### `angles.py`

Does: `wrap_angle`, the one angle-wrap helper shared by `controller/` and `sim/`.
Change it to:
- Nothing normally. Check any change against every caller with `grep -rn wrap_angle`.
Don't:
- Move it under `controller/` or `sim/`, because `sim/` imports `controller/` and never the reverse, so a shared helper needs a home that neither direction rules out.
Key API: `wrap_angle(angle)` returns the angle in (-pi, pi] via `arctan2(sin, cos)`.

### `tracks/__init__.py`

Does: filesystem-only path helpers for track directories. `TRACKS_DIR` points across the repo boundary into `ros2/src/fsae_planning/tracks/`. No track data lives in this repo.
Change it to:
- Add a per-track file: add a `*_NAME` constant and a `*_path()` helper. Check with `python -c "import tracks; print(tracks.TRACKS_DIR, tracks.list_tracks())"`.
Don't:
- Import scipy, numpy or ROS here, because the module is kept dependency-free so the live package could adopt it.
- Make `resolve_map_arg` fall back to the default track on an unknown name, because exporting the wrong track shows up only as odd driving on the car later.
- Move `DEFAULT_MAP` above `list_tracks` and `newest_track`, because `cone_map_path()` calls them at import time.
- Read `DEFAULT_TRACK` as a name. It is `None`, meaning the newest track by `cone_map.json` mtime, not by directory name.
Key API: `TRACKS_DIR`, `list_tracks`, `newest_track`, `dated_track_name`, `track_dir`, `cone_map_path`, `speed_profile_path`, `raceline_path`, `centerline_path`, `geometry_path` (centreline if present, else raceline, else `None`), `resolve_map_arg`, `default_out_for`, `DEFAULT_MAP`.

## Data format: settings profiles

Files: `settings_profiles/no_progress_tuned.json` and `settings_profiles/progress_tuning.json`. Written by the Profiles tab, one file per profile, read only by the same tab. No other tool reads them.

Change it to:
- Hand-editing is safe. Loading skips unknown keys, so removed fields do not break an old profile.
Don't:
- Expect a profile to reproduce a run unless it has the Launch keys. `progress_tuning.json` was saved before Launch state was captured and holds only Settings keys.
Key API: `ProfilesTab._save_current`, `ProfilesTab._load_selected`.

**Schema**

```json
{"name": "<text typed in the dialog>", "values": {"<NAME>": "<raw text>", "...": "..."}}
```

- The file name is the typed name with `\ / : * ? " < > |` replaced by `_`. Saving over an existing name asks first.
- Every value is a string holding the literal that would sit right of the `=` in the target file. Lists look like `"[6.4, 0.0, 1.65, 1.0, 1.5, 0.0, 0.0, 0.0]"`, booleans `"True"` or `"False"` (Settings) and `"true"` or `"false"` (Launch), floats `"-1.0"`.
- Keys are sorted on write (`sort_keys=True`, `indent=2`).

**What each tab contributes**

| Source | Keys | Notes |
|---|---|---|
| Settings tab (`SettingsTab.capture_profile_values`) | every `settings/` name the tab manages: `Q_diag`, `R_diag`, `R_rate_diag`, scalar fields (`R_A_ACCEL`, `R_A_BRAKE`, `SPEED_TARGET_DEFICIT_MAX`, `NMPC_RJERK_*`, `NMPC_RRATE_ZONE_*`), the twelve `NMPC_*` overrides (`-1.0` means inherit), the five `NMPC_*` progress numerics, and the feature flags that have a `settings/` name | read from widgets, so unsaved edits are captured. `delay_compensation_enabled` has no `settings/` name and is not captured |
| Launch tab (`LaunchTab.capture_profile_values`) | `TRACK`, `CONTROLLER`, `USE_NMPC`, `STANDALONE_OUTPUT`, `USE_PRECOMPUTED_SPEED`, `USE_PRECOMPUTED_PATH`, `V_MAX`, `V_MIN`, `NMPC_PROGRESS_ENABLED` | record-new-track mode is not captured. `NMPC_PROGRESS_ENABLED` is `"true"` only when the controller is NMPC and the box is ticked |

The current profile files hold 47 keys (`no_progress_tuned.json`, Settings and Launch) and 33 keys (`progress_tuning.json`, Settings only, and it still lists `NMPC_PROGRESS_ENABLED` as a Settings key from before that flag moved to the Launch tab).

**How loading applies**

1. A confirm dialog states that every Settings and Launch value will be overwritten.
2. `SettingsTab.apply_profile_values` fills its widgets, then calls `_on_save`. That writes `settings/*.py`, the live `mpc_params.py` and `nmpc_params.py`, both `fsae_params.yaml` copies and both `fsds_simulator/` mirrors, with `.bak` backups on first touch.
3. `LaunchTab.apply_profile_values` fills the Launch widgets only. `launch_all.sh` changes at the next Launch click.
4. The running sim is not affected. Restart it.

`CONTROLLER` and `USE_NMPC` are applied together only when both are present. `TRACK` is skipped while record mode is on.

## Data format: track directories

Location: `ros2/src/fsae_planning/tracks/<name>/`, resolved by `tracks.TRACKS_DIR`. The data ships with the `fsae_planning` checkout, not this repo, so a `fsae_planning`-only checkout can drive. Edits there are local edits to that checkout and are reported, never committed by an agent. A new track name gets a date suffix (`<name>_<YYYYmmdd>`, `tracks.dated_track_name`). Re-recording an existing name keeps its directory. `tracks.list_tracks` only lists directories that contain `cone_map.json`.

Present now: `acceleration_20260916` (cone map, speed profile, centreline), `comp_test_map_2_20260916` (all four), `comp_test_map_3` (all four plus an extra corner-test speed profile CSV written by `export_speed_profile` in corner-slowdown mode).

| File | Writer | Readers |
|---|---|---|
| `cone_map.json` | `cone_recorder` node in `fsae_planning` (`perception/fsae_sim_perception/.../cone_recorder.py`), live during a lap | `sim/track_io.py` (`load_cone_map`, `load_recorded_track`), the exporters, `gui/simulation.py`, offline validation scripts |
| `speed_profile.csv` | `tuner.tools.export_speed_profile` | live launch arg `map_path`, `tracks.speed_profile_path` |
| `raceline.csv` | `tuner.tools.raceline_optimizer <name>` | live launch arg `path_map_path` (via `launch_all.sh`), `tracks.raceline_path` |
| `centerline.csv` | `tuner.tools.raceline_optimizer <name> --mode centerline` | live launch arg `path_map_path` (preferred over the raceline by `tracks.geometry_path` and by `launch_all.sh`'s `PATH_CSV` default) |

Exporters write next to their source map (`tracks.default_out_for`). `--no-overwrite` refuses to replace an existing file.

**`cone_map.json`**

```json
{"blue": [[x, y], ...], "yellow": [[x, y], ...], "source": "fsae_sim_perception.cone_recorder", "lap_closed": true}
```

- `blue` is the left boundary and `yellow` the right, in the global frame the car pose used during recording (origin is where the car started), metres.
- `lap_closed` is `false` when the recorder stopped on its time limit before the car returned to its start pose.
- `load_cone_map` reads only `blue` and `yellow` and reshapes each to (N, 2). `source` and `lap_closed` are informational.

**`speed_profile.csv`** (4 columns)

- Header comments: two `#` lines (description, `# source_map=<abs path>`), plus corner-slowdown parameters in the corner-test variant.
- Columns: `x,y,psi,v_target`. Metres, radians (geometric path tangent), metres per second. Formats `%.4f`, `%.4f`, `%.5f`, `%.4f`.
- The path is the reconstructed oracle centreline with the physics-based speed profile.

**`raceline.csv` and `centerline.csv`** (5 columns)

- Columns: `x,y,psi,psi_target,v_target`. `psi_target` is the shaped heading-lead reference (not the geometric tangent). It is used only when `use_precomputed_heading_profile` is on.
- Header comments include `# source_map=`, `# lateral_offsets=True|False`, and lap times (`centreline_lap_time_s`, plus `raceline_lap_time_s` and `improvement_pct` for a raceline).
- `centerline.csv` stays on the track middle. Its lateral error reads directly as distance from the middle, which a cornering-optimised raceline's does not.

**Loader tolerance.** The live `_load_profile_csv` in `control_utils.py` skips `#` lines and any `x,y` header, then detects 4 versus 5 columns per row. For a 4-column file `psi_target` equals `psi`. A file needs at least two valid rows.

## Data format: recorded-run CSVs

Written by `ControlLogger` in `telemetry/control_logger.py` (live tree, mirrored into `fsds_simulator/`), one pair per run, for Stanley, LTV and NMPC alike. Controllers without a feature leave its cells empty so the column set never changes.

Change it to:
- Add a control column: append to `ADAPTIVE_COLUMNS` in `telemetry/columns.py` (at the end, so name-based readers and older logs stay valid), on the live side and the `fsds_simulator/` mirror. Offline scripts parse by header name through `tuner/csv_log.py`. Check with a short run and `head` on the CSV.
Don't:
- Insert columns mid-row or rename one, because name-based analysis scripts and old logs break.
- Assume an empty cell is zero. An empty cell means the feature was off or not reported, distinct from a reported `1.0`.
Key API: `tuner.csv_log.load_columns`, `tuner.csv_log.read_data_lines`, `tuner.tools.plot_playback.load_log`.

**Naming and location**

- Files: `<tag>_control_<stamp>.csv` and `<tag>_path_<stamp>.csv`, stamp `YYYYmmdd-HHMMSS` in local time. Older runs use epoch seconds, and `plot_playback._stamp` orders both.
- Tags: `stanley`, `mpc`, `mpc_standalone` (chosen by `standalone_output`). The LTV and NMPC controllers share a tag, so playback labels a run by its `recorded_runs/<folder>` name when it has one.
- Directory: `log_dir` parameter, default `~/fsae_logs`. `launch_all.sh` sets it to the outer repo's `fsae_logs/`. The Launch tab's Stop offers to move a run to `fsds_simulator/recorded_runs/<Stanley|LMPC|NMPC>/`. `recorded_runs/graph/` is a curated drop zone.
- Retention: none, `fsae_logs/` grows without bound.

**Control CSV columns, in order**

| Group | Columns |
|---|---|
| Time and pose | `t` (s since first sample, both files share it), `car_x`, `car_y`, `car_yaw`, `v_actual`, `v_desired` |
| Tracking | `steer_deg`, `e_y`, `e_psi_deg`, `yaw_rate` (front-axle Frenet errors vs the path, positive left or CCW) |
| Commands | `delta_cmd` (rad), `a_cmd` (m/s^2), `solver_failed`, `inaccurate` (0 or 1) |
| Latency | `pose_age_s`, `path_age_s`, `n_delay`, `solve_ms`, `cmd_latency_ms` |
| Lap | `lap_idx`, `pred_err_m`, `pred_acc_pct`, `lap_score`, `lap_pred_acc_pct` (the last two are filled only on the row that completes a lap) |
| `ADAPTIVE_COLUMNS` | see below |

`ADAPTIVE_COLUMNS` (from `telemetry/columns.py`, live side; 37 entries at the time of writing, count from the file):

| Group | Columns |
|---|---|
| Corner-factor scheduler | `corner_factor`, `low_speed_corner_boost`, `corner_frac` |
| Base weights after the straight/corner blend | `Q_ey_base`, `Q_epsi_base`, `Q_r_base`, `R_steer_corner_blend`, `Rrate_steer_corner_blend` |
| Per-feature multipliers (1.0 means the feature did nothing) | `m_Q_ey_soften`, `m_R_speed`, `m_Rrate_antihunt`, `m_Rrate_zone`, `m_Rrate_reversal` |
| Final weights handed to the QP | `Q_ey_eff`, `Q_epsi_eff`, `Q_r_eff`, `R_steer_eff`, `Rrate_steer_eff`, `R_a_accel_eff`, `R_a_brake_eff` |
| NMPC solver state (empty for LTV and Stanley) | `nmpc_iters`, `nmpc_status`, `nmpc_cost`, `nmpc_s0`, `nmpc_kappa_horizon_end`, `nmpc_pred_ey_end`, `nmpc_pred_epsi_end`, `nmpc_pred_ey_max_abs`, `n_latency` |
| Friction-circle diagnostics (only when that feature is on) | `nmpc_fyf_max_abs`, `nmpc_fyr_max_abs` |
| Progress-term diagnostics (only when that feature is on) | `nmpc_v_cap`, `nmpc_speed_cap_over`, `nmpc_s_target_gap_end` |

The product of a weight's `m_*` columns and its base value gives its `*_eff` column. That decomposition shows which feature moved a weight. A run where `nmpc_status` sits at 0, or where `nmpc_pred_ey_end` disagrees with the `e_y` reached about a second later, points to a model or solver problem, not a weighting one.

**Path CSV**

- Columns: `t,idx,x,y`. One row per point of one planner path snapshot. Rows that share `t` form one polyline.
- Snapshots are written at most every `path_period` seconds (default 1.0).
- `plot_playback._load_path_snapshots` groups rows by `t` and picks the latest snapshot at or before the playback time.

**`#` header (control CSV only)**

The body is written live. At `close()` the file is rewritten with a commented block on top, via a `.tmp` file and `os.replace`. A run killed before `close()` has no header and is still readable.

- Identity: a first line of `# fsae control log` plus `tag=<tag>`, `# t0_epoch_s=` (epoch of `t=0`), a frame line (global ENU, yaw 0 along +x), a score-source line.
- `# score_is_partial=0|1`. `1` means `time_bonus` or off-track was unavailable live, for example a run on the live planner topic with no precomputed path. The weighted-metric part is still comparable with an offline score.
- Configuration block (`# ── run configuration ...`), from `telemetry/config_lines.py`: `# controller=`, `# use_nmpc=`, `# launch.<key>=<repr>`, `# mpc_params.<field>=<repr>` (every `MPCParams` field via `dataclasses.asdict`), `# nmpc_params.<field>=`, and the resolved NMPC weights. This records the numbers only, not the adaptive-gain scheme, which is code.
- Timing: `# lap_time_s=`, `# optimal_time_s=` (integral of distance step over target speed over the precomputed profile, scaled by progress).
- Per-lap block: `# lap_<N>_score=`, `# lap_<N>_time_s=`, `# lap_<N>_pred_acc_pct=` (the text n/a when nothing matured).
- Whole-run metrics as `# key=value`: `composite_score`, `n_steps`, `rmse`, `peak_lateral_error_m`, `speed_rmse_mps`, `yaw_rms_radps`, `max_yaw_rate_radps`, `control_smooth_rms`, `jerk_rms`, `steer_rms`, `accel_rms_mps2`, `max_steering_rad`, `max_accel_mps2`, `steering_sat_ratio`, `steering_reversal_rms`, `steering_reversal_rate`, `steering_reversals`, `inaccurate_count`. Then `# pred_acc_pct=` (NMPC only, mean of per-lap means).
- Score formula: `telemetry/scoring.py`, a copy of `sim/scoring.py` with inlined constants. A meaningful `composite_score` needs real progress and completion data from `LapProgressTracker`. Without it the score pins at the DNF floor.

**Readers**

- `tuner/csv_log.py`: `read_data_lines` drops every `#` line, `parse_rows` keeps only rows whose field count matches the header (dropping a truncated last row), `load_columns` returns a dict of float arrays with empty cells as `nan` (named string columns stay strings).
- `tuner/tools/plot_playback.py`: `load_log` returns `(columns, metadata)`. `_read_header_metadata` reads each leading `# key=value` line into a dict, keeping only the first whitespace-separated token of the value, so callers use `.get()` with a default. `_path_csv_for` derives the sibling path CSV by swapping `_control_` for `_path_`.

## Data format: tuning history

File: `docs/logs/tuning_history.txt`. Appended to by `log_results_to_history` in `tuner/offline_tuner.py` at the end of a tuning run (also when interrupted). Path constant: `TUNING_HISTORY_PATH`. The doc logs folder is otherwise frozen, this file is the one exception because the tuner writes it.

Change it to:
- Add a field to each entry: edit `log_results_to_history` and keep the existing line prefixes, because entries are read by eye and by the follow-up manual edit of the placeholder lines.
Don't:
- Compare "Tuner score" across entries above the `COMPARABLE HISTORY RESUMES HERE` marker (about line 346), because the planner, `SCORE_WEIGHTS` and the tuner changed under them. The file header explains this.
- Delete the placeholder lines. The description and the overall score are filled in by hand after the weights are tested in FSDS.
Key API: `log_results_to_history(Q, R, R_rate, duration, score, optuna_info=None)`.

**Entry layout** (one blank line, then):

```
# dd/mm/yy HH:MM - [Pending Description: yet to be tested]
Q_diag      = [8 floats]
R_diag      = [2 floats]
R_rate_diag = [2 floats]
Score weights = rmse=..., yaw_rms=..., smooth_rms=..., steer_rms=..., accel_rms=..., max_steering=..., steering_sat_ratio=..., jerk_rms=..., max_yaw_rate=..., steering_reversal_rms=..., peak_lateral_error=..., speed_rmse=...
Bonus/penalty weights = completion_bonus=..., time_bonus=..., dnf_penalty=...
Duration    = <minutes> minutes
Overall score (avged from all testing scenarios)  = Haven't been tested.
Tuner score (tuning scenarios / validation suite) = <score>
Optuna TPE pre-pass = Yes (<n> trials, best <score>) (long dash) CMA-ES x0 seeded from best trial      (or "No (fixed geometric-midpoint x0)")
  Optuna Q_diag / R_diag / R_rate_diag = ...      (only when the pre-pass ran)
Commit hash = <git hash>
```

The score weights are logged by name so an entry stays interpretable after `settings/scoring.py` changes. The metric-name list `_SCORE_METRIC_NAMES` must match the index order in `sim/scoring.py`.
