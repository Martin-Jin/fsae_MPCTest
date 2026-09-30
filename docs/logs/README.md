# Investigation Logs: Frozen History, Not Current Reference

These logs are a historical record of past investigations. They were written before the code and docs restructure, they are frozen, and they are not maintained. Do not edit them.

- **They may cite dead paths.** Code paths, module names, function locations and doc names inside the logs use the old layout. Some cited docs were deleted.
- **Their conclusions may be superseded.** For current behaviour, read the docs listed in the [docs index](../README.md) and the code itself.
- **Use them for reasoning and evidence.** Each log records what was measured, what was tried, and what was falsified, with numbers. That is the reason to keep them.
- **Cross-references between logs still resolve.** Section numbers such as `sim_to_real_investigation.md` section 57 are unchanged.

## Old doc names to new

| Old doc | Now |
|---|---|
| junior_project_mpc_docs.md | [guides/getting_started.md](../guides/getting_started.md) |
| offline_guide.md | [guides/offline_guide.md](../guides/offline_guide.md) |
| tuning.md | [guides/tuning.md](../guides/tuning.md) |
| debugging_tools.md | [guides/debugging_tools.md](../guides/debugging_tools.md) |
| lmpc.md, nmpc.md, stanley.md | [controllers/lmpc.md](../controllers/lmpc.md), [controllers/nmpc.md](../controllers/nmpc.md), [controllers/stanley.md](../controllers/stanley.md) |
| architecture.md | [reference/architecture.md](../reference/architecture.md) |
| vehicle_physics_guide.md | [reference/vehicle_physics.md](../reference/vehicle_physics.md) |
| error_state_reference.md | [reference/error_states.md](../reference/error_states.md) |
| reference/simulator_glossary.md | [reference/glossary.md](../reference/glossary.md) |
| removed_mechanisms.md, reference/superseded_mechanisms.md | [reference/retired_mechanisms.md](../reference/retired_mechanisms.md) |
| fsds/fsds_integration_guide.md | [fsds/integration_guide.md](../fsds/integration_guide.md) |
| fsds/fsds_ros_integration.md | [fsds/ros_integration.md](../fsds/ros_integration.md) |
| fsds/fsds_settings.md | [fsds/settings.md](../fsds/settings.md) |
| tuning history.txt (repo root) | [tuning_history.txt](tuning_history.txt) in this directory |

Deleted with no single replacement:

- planning_control_sync.md: its content was distributed across the reference docs, mainly [offline_live_parity.md](../reference/offline_live_parity.md), [simulator_fidelity.md](../reference/simulator_fidelity.md) and [reference_path_and_speed.md](../reference/reference_path_and_speed.md). Search there for a topic.
- reference/README.md: an index. Use [reference/offline_live_parity.md](../reference/offline_live_parity.md) for the file mapping and score parity, and the [docs index](../README.md).
- fsae_planning_pending_pr.md and steering_turn_in_upgrade_options.md: deleted, no replacement.

## Old code paths to new

Offline side (this repo):

| Old | Now |
|---|---|
| settings.py | `settings/` package (`general`, `noise`, `planner`, `lmpc`, `nmpc`, `solver`, `scoring`). Consumers still use `import settings; settings.X` |
| gui/launcher.py | `gui/launcher/` package. Run `python -m gui.launcher` |
| sim/rollout_core.py, tuner/rollout_core.py | `sim/rollout/core.py` |
| sim/rollout_phases.py | `sim/rollout/` (`reference`, `speed_target`, `delay`, `tick_solve`) |
| sim/sim_track.py | `sim/perception.py` (`place_cones`, `SimPerception`) and `sim/planner.py` (`SimPlanner`) |
| model/vehicle_physics.py | `model/vehicle_physics/` package (`state`, `params`, `tyres`, `plant_step`, `tracking`) |
| controller/optimiser.py | `controller/lmpc/` (`build`, `solve`) |
| controller/nmpc_optimiser.py, controller/nmpc/solver.py | `controller/nmpc/` (`qp_model`, `sqp_step`, `solver`) |
| tuner/nmpc_offline_check.py, tuner/recorded_map_rollout.py, tuner/checks/plant_openloop_validation.py | `tuner/validation/`, run as `python -m tuner.validation.recorded_map_rollout` and likewise for the other two |
| tuner/checks/*, stray tuner/*_check.py scripts | `tuner/investigations/` |
| tuner/export_speed_profile.py, tuner/raceline_optimizer.py | `tuner/tools/` |
| recorded_map_rollout.DEFAULT_MAP | `tracks/__init__.py` (`DEFAULT_MAP`) |
| deleted/ | removed |

Live side (`ros2/src/fsae_planning`, mirrored in `fsds_simulator/`), under `control/fsae_control/fsae_control/`:

| Old | Now |
|---|---|
| mpc/mpc_core.py, mpc_core.py | `lmpc/` (`constants`, `predict`, `adaptive_gains`, `controller`) |
| mpc/nmpc_core.py, nmpc_core.py | `nmpc/` (`layout`, `reference`, `dynamics`, `outputs`, `weight_schedule`, `qp_model`, `sqp_step`, `solver`), one file per offline `controller/nmpc/` file |
| telemetry_logger.py | `telemetry/` (`columns`, `config_lines`, `horizon_tracker`, `lap_progress`, `control_logger`) |
| scoring.py | `telemetry/scoring.py` |
| live_viz.py | `live_viz/` (`panels`, `node`, `app`) |
| mpc_controller_standalone.py | merged into `mpc/mpc_controller.py`, selected by its `standalone_output` parameter |

`mpc/mpc_controller.py` stays the node. `mpc/` also holds `control_step.py`, `debug_publish.py`, `node_constants.py`, `mpc_params.py` and `nmpc_params.py`. Entry points are unchanged: `controller`, `mpc_controller`, `fsds_bridge`, `live_viz`, `brake_sysid`.

## The logs

| File | What it records |
|---|---|
| [sim_to_real_investigation.md](sim_to_real_investigation.md) | Chronological account (from 2026-08-06) of tracking down the gap between the offline simulator and the car, including every wrong hypothesis and why it looked right |
| [fsae_planning_sim_to_real_investigation_pt1.md](fsae_planning_sim_to_real_investigation_pt1.md) | Commit-by-commit history, 2026-08-04 to 2026-08-07, of making the offline simulator trustworthy. Ends on the unresolved real-versus-offline driving gap |
| [fsae_planning_sim_to_real_investigation_pt2.md](fsae_planning_sim_to_real_investigation_pt2.md) | Continuation, 2026-08-07 to 2026-08-09, chasing the remaining gap after a lateral-acceleration ceiling was found and modelled |
| [late_turn_in_investigation.md](late_turn_in_investigation.md) | Working notes from 2026-08-12 on late turn-in and lookahead: derivations, live test data and rejected ideas |
| [nmpc_introduction.md](nmpc_introduction.md) | Introduction of the nonlinear MPC on 2026-08-13, staged, covering only the NMPC work |
| [steering_chatter_investigation.md](steering_chatter_investigation.md) | NMPC steering twitching and reluctance to turn in, two causes found and fixed (steering-rate cost about 18 times too low, and a raceline that demanded more grip than the plant provides) |
| [nmpc_low_speed_accel_stall_investigation.md](nmpc_low_speed_accel_stall_investigation.md) | NMPC commanding near-zero acceleration at 2.5 to 6 m/s, traced to two RK4 substep counts that were too low, both fixed |
| [nmpc_progress_term_investigation.md](nmpc_progress_term_investigation.md) | Progress-term NMPC does not reliably finish a lap. Also finds that `SPEED_TARGET_DEFICIT_MAX` at 2.5 was the binding limit on acceleration for 36.8% of a lap |
| [nmpc_speed_limit_investigation.md](nmpc_speed_limit_investigation.md) | `nmpc_speed_limit_enabled` rejected twice (2026-08-19 and 2026-09-15): good offline, off-track live, and its constraint never engages |
| [nmpc_planner_only_corner_failure.md](nmpc_planner_only_corner_failure.md) | Planner-only NMPC loses heading control at one corner because the live planner path truncated into a frozen tail |
| [planner_only_speed_target_oscillation.md](planner_only_speed_target_oscillation.md) | Planner-only NMPC oscillates and stalls from an unrated collapse of the curvature speed target, not a path defect |
| [planner_only_lap2_corner_spinout.md](planner_only_lap2_corner_spinout.md) | Planner-only NMPC survives lap 1 but spins out at the same corner on lap 2 |
| [periodic_pose_teleport_investigation.md](periodic_pose_teleport_investigation.md) | Car pose goes stale on a fixed period of about 31.7 to 34.1 s, controller-agnostic, root cause narrowed to a shared transport stall and not found |
| [tuning_history.txt](tuning_history.txt) | Not a log to read through. `tuner/offline_tuner.py` appends each run's best weights and metadata here. Entries before 2026-08-06 are not comparable to later ones |
