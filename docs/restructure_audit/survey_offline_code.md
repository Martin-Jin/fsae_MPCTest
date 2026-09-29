# Survey: fsae_MPCTest (excluding fsds_simulator/, .git, .claude, __pycache__)

Read-only survey for restructure planning. Scope: `fsae_MPCTest/` proper.

## 1. File inventory

Legend: L=lines. "CLI"=has `if __name__=="__main__"`/argparse. "Invoked by string"=referenced as a path/module string outside its own file (docs, launcher.py subprocess calls, shell scripts, CLAUDE.md).

### Top level

| Path | L | Purpose | Main exports | Imported by | CLI | Invoked by string |
|---|---|---|---|---|---|---|
| `settings.py` | 1625 | Single source of truth for ALL tuning constants (LMPC/NMPC weights, noise, scoring, solver). Imports `sim.sim_track.TRACK_HALF_WIDTH`. | ~150 module-level constants | 16 files (see §6) | no | `settings.py`, `settings.py.bak` named throughout docs/CLAUDE.md |
| `settings.py.bak` | - | Stray backup (launcher's `_backup_once`/manual). Not code. | - | - | - | - |
| `tracks/__init__.py` | 265 | Path resolver for `ros2/src/fsae_planning/tracks/` (cross-repo). `TRACKS_DIR`, `newest_track()`, `cone_map_path()`, `resolve_map_arg()`, `dated_track_name()`. | yes | `tuner/recorded_map_rollout.py`, `tuner/steering_chatter_check.py`, `tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py`, `gui/simulation.py` | no | `tracks/__init__.py TRACKS_DIR` (CLAUDE.md) |
| `tuning history.txt` | - | Free-text tuning log (space in filename). | - | - | - | - |
| `README.md` | - | Top-level docs, entry points. | - | - | - | - |

### `controller/`

| Path | L | Purpose | Main exports | Imported by | CLI |
|---|---|---|---|---|---|
| `controller/__init__.py` | 1 | empty pkg marker | - | - | no |
| `controller/model_utils.py` | 464 | Shared adaptive-gain helpers (corner factor, anti-hunt, rrate zone) used by BOTH LMPC and NMPC paths. | `_corner_factor`, adaptive gain fns | `sim/rollout_phases.py`, `controller/nmpc/solver.py`, `controller/nmpc/weight_schedule.py` | no |
| `controller/lmpc/__init__.py` | 14 | re-exports `init_parameterized_mpc`, `solve_mpc` | - | `sim/rollout_phases.py`, `tuner/nmpc_offline_check.py` | no |
| `controller/lmpc/build.py` | 267 | cvxpy problem construction (LTV-QP) | `init_parameterized_mpc` | `controller/lmpc/solve.py` | no |
| `controller/lmpc/solve.py` | 242 | solves the built QP each tick | `solve_mpc` | `controller/lmpc/__init__.py` | no |
| `controller/nmpc/__init__.py` | 87 | re-exports whole NMPC package surface | `NMPCController`, `PathReference`, layout/dynamics/outputs symbols | `sim/rollout_phases.py`, `tuner/nmpc_offline_check.py` | no |
| `controller/nmpc/layout.py` | 47 | state/input vector index constants, `_wrap` | index consts | dynamics.py, outputs.py, solver.py, reference.py (indirect) | no |
| `controller/nmpc/dynamics.py` | 281 | `_Plant`, `_step`, `_step_scalar`, `_tyre_forces` (RK4 prediction model, must match `model/vehicle_physics.py`) | yes | outputs.py, solver.py | no |
| `controller/nmpc/reference.py` | 193 | `PathReference` (spline reference-path lookup) | yes | solver.py | no |
| `controller/nmpc/outputs.py` | 109 | `_outputs` (post-solve diagnostics extraction) | yes | solver.py | no |
| `controller/nmpc/weight_schedule.py` | 93 | `_rrate_zone_scale`, `_rrate_stage_ramp` (corner-aware weight scheduling) | yes | solver.py | no |
| `controller/nmpc/solver.py` | 1042 | `NMPCController` (the actual SQP/OSQP solve loop) | yes | `sim/rollout_phases.py` (via `controller.nmpc`) | no |

### `model/`

| Path | L | Purpose | Main exports | Imported by | CLI |
|---|---|---|---|---|---|
| `model/__init__.py` | 1 | empty | - | - | no |
| `model/vehicle_physics.py` | 1290 | THE plant model: `VehicleParams`, `step_nonlinear_plant`, Pacejka tyre model, `alat_ceiling*`, tracking-error/reference-point helpers. Danger-zone per CLAUDE.md. | `VehicleParams`, `step_nonlinear_plant`, `init_plant_state`, `plant_to_tracking_error`, `find_closest_reference_bounded`, `get_interpolated_ref_point`, `pacejka_*` | almost everything: `model/bicycle_model.py`, `sim/rollout_core.py`, `sim/rollout_phases.py`(indirect), `tuner/offline_tuner.py`, `tuner/nmpc_offline_check.py`, `tuner/recorded_map_rollout.py`, `tuner/performance_stats.py`, `tuner/checks/*`, `gui/manual_drive.py`, `gui/simulation.py` | no |
| `model/bicycle_model.py` | 204 | `get_8state_discrete_model` (linearized A/B for LTV-QP), imports `model.vehicle_physics as vp` | yes | `sim/rollout_phases.py`(indirect via LMPC build), `tuner/offline_tuner.py`, `tuner/nmpc_offline_check.py`, `tuner/steering_chatter_check.py` | no |

### `planning/`

| Path | L | Purpose | Main exports | Imported by | CLI |
|---|---|---|---|---|---|
| `planning/cone_map.py` | 91 | `ConeMap` (accumulating cone memory/merge logic) | yes | `sim/sim_track.py` | no |
| `planning/cone_sorting.py` | 102 | `filter_cones_window` and boundary-ordering helpers | yes | `planning/boundary.py`, `planning/path_utils.py` | no |
| `planning/boundary.py` | 405 | `build_path_walls`, wall/segment geometry, centreline-quality workaround territory (CLAUDE.md flags this) | yes | `sim/sim_track.py`, `sim/track_io.py`, `tuner/reference_excess_mechanism_check.py` | no |
| `planning/path_utils.py` | 446 | `blend_paths`, `build_local_path`, `smooth_centreline`, spline utilities | yes | `sim/sim_track.py`, `sim/track_io.py`, `planning/boundary.py` (one function, lazy-imported) | no |

Note: `planning/path_utils.py:404` does a deferred `from planning.boundary import segment_crosses_walls` **inside a function**, while `boundary.py:15` imports `path_utils` at module top level — this is a near-circular import currently avoided only because one side is lazy. Flag for the split.

### `sim/`

| Path | L | Purpose | Main exports | Imported by | CLI |
|---|---|---|---|---|---|
| `sim/__init__.py` | 1 | empty | - | - | no |
| `sim/rollout_core.py` | 594 | `run_core_rollout`, `compute_step_budget` — the main headless closed-loop driver, imports settings, model, sim_track, scoring, rollout_phases, sensor_noise | yes | `gui/simulation.py`, `tuner/offline_tuner.py`, `tuner/recorded_map_rollout.py`, `tuner/steering_chatter_check.py`, `tuner/reference_excess_mechanism_check.py`, `tuner/reference_heading_geometry_check.py`, `tuner/checks/ref_heading_limiter_*.py`, `tuner/nmpc_offline_check.py` | no |
| `sim/rollout_phases.py` | 829 | Per-tick phase functions factored out of rollout_core: reference/speed-target computation, NMPC/LTV tick solve, tracking-error, delay modeling | yes (14 functions) | `sim/rollout_core.py`, `sim/sensor_noise.py` (one fn) | no |
| `sim/scoring.py` | 372 | `RolloutMetrics`, `compute_composite_score` — SOURCE OF TRUTH for scoring, mirrored verbatim (inlined) into live `fsae_planning/.../scoring.py` | yes | `sim/rollout_core.py`, `tuner/performance_stats.py` | no |
| `sim/sensor_noise.py` | 225 | `SlamNoise`, `ConeNoise`, `PoseFeedHold` | yes | `sim/rollout_core.py` | no |
| `sim/sim_track.py` | 353 | `SimPerception`, `SimPlanner`, `calculate_dynamic_max_steps`, `place_cones` | yes | `sim/rollout_core.py`, `gui/manual_drive.py`, `gui/simulation.py`, `tuner/reference_excess_mechanism_check.py` (as `sim_track`) | no |
| `sim/speed_profile.py` | 892 | curvature/speed-profile computation, oracle profile, dynamic cap | yes | `gui/simulation.py`, `sim/track_io.py`, `tuner/offline_tuner.py`, `tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py` | no |
| `sim/track_io.py` | 369 | `load_recorded_track`, `load_cone_map`, `_reconstruct_centreline`, `_resample_dense`, `PATH_N_POINTS` | yes | `sim/rollout_core.py`(indirect), `gui/simulation.py`, `tuner/recorded_map_rollout.py`, `tuner/steering_chatter_check.py`, `tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py`, `tuner/checks/ref_heading_limiter_ab.py`, `tuner/reference_heading_geometry_check.py`, `tuner/reference_excess_mechanism_check.py` | no |

### `gui/`

| Path | L | Purpose | Main exports | Imported by | CLI |
|---|---|---|---|---|---|
| `gui/__init__.py` | 1 | empty | - | - | no |
| `gui/launcher.py` | 2436 | Tk desktop app: Launch/Debug/OfflineSim/Settings/Profiles tabs. Reads/writes `launch_all.sh`, `settings.py`, `mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml` via regex (see §3). | `LauncherApp`, `RepoPaths`, tab classes | none in-repo (top-level entry) | **yes**, `python -m gui.launcher` | README.md, docs/debugging_tools.md |
| `gui/simulation.py` | 954 | 2D matplotlib GUI sim + auto-tuning ("second, simpler simulator" per CLAUDE.md); imports settings, model, tuner.offline_tuner, tuner.performance_stats, sim.* | interactive callbacks | launched by `gui/launcher.py` subprocess (`-m gui.simulation`) | **yes** | README.md, docs, launcher.py:1299 |
| `gui/manual_drive.py` | 407 | matplotlib manual-drive tool, imports model/tuner/sim | interactive callbacks | none | **yes** (matplotlib animation loop) | README.md (implied) |

### `tuner/` (top level, not `checks/` or `tools/`)

| Path | L | Purpose | CLI | Invoked by string |
|---|---|---|---|---|
| `tuner/__init__.py` | 1 | empty | no | - |
| `tuner/csv_log.py` | 58 | `load_columns`, `medfilt`, `read_data_lines` — shared CSV parsing helper | no | imported by 5 `tuner/checks/*` files + `tuner/tools/plot_playback.py` |
| `tuner/nmpc_offline_check.py` | 257 | solver self-consistency checks (`_step_scalar==_step`, SQP convergence, turn-in sign) | **yes** | CLAUDE.md, docs, "Testing" section |
| `tuner/offline_tuner.py` | 1568 | CMA-ES/Optuna weight-tuning driver: synthetic paths, parallel rollout eval, git-hash logging | **yes** (multiprocessing, signal handling) | `docs/junior_project_mpc_docs.md`, `gui/simulation.py` imports `get_cached_model`/`SYNTHETIC_PATHS`/`PATH_NAMES` from it |
| `tuner/performance_stats.py` | 415 | `benchmark_weights`, `report_performance_metrics` — human-readable score/metric reporting | no (imported only) | imported by `gui/simulation.py` |
| `tuner/recorded_map_rollout.py` | 192 | THE headless correctness-bar script (CLAUDE.md's `tuner.recorded_map_rollout`) | **yes** | CLAUDE.md ("Testing"), many docs, `tuner/checks/live_vs_sim_diagnostics.py` (imports `DEFAULT_MAP`, `run`), `tuner/reference_*` checks (import `DEFAULT_MAP`) |
| `tuner/reference_excess_mechanism_check.py` | 169 | investigation script (boundary planner seed-jump) | **yes** | docs/logs |
| `tuner/reference_heading_geometry_check.py` | 127 | investigation script (ref-heading swing vs track geometry) | **yes** | docs/logs |
| `tuner/steering_chatter_check.py` | 112 | reproduces NMPC steering chatter with `--set NAME=VALUE` settings overrides via `setattr(settings, ...)` | **yes** | docs/debugging_tools.md |

### `tuner/checks/`

| Path | L | Purpose | CLI |
|---|---|---|---|
| `tuner/checks/__init__.py` | 1 | empty | no |
| `tuner/checks/analyze_adaptive_log.py` | 145 | analyzes an adaptive-gain CSV log | **yes** |
| `tuner/checks/brake_sysid_analysis.py` | 184 | brake system-ID analysis, uses `tuner.csv_log` | **yes** |
| `tuner/checks/live_vs_sim_diagnostics.py` | 275 | live-vs-offline divergence report; lazy-imports `tuner.tools.plot_playback._stamp`, `model.vehicle_physics.VehicleParams`, `tuner.recorded_map_rollout.{DEFAULT_MAP,run}` | **yes** |
| `tuner/checks/plant_openloop_validation.py` | 359 | replays two open-loop system-ID experiments through the plant | **yes** | CLAUDE.md ("Testing"), docs |
| `tuner/checks/ref_heading_limiter_ab.py` | 91 | A/B test of `REF_HEADING_RATE_LIMIT`, lazy-imports `sim.rollout_core`, `model.vehicle_physics`, `tuner.recorded_map_rollout.DEFAULT_MAP`, `sim.track_io`, `settings`, `tuner.offline_tuner.get_cached_model` | **yes** |
| `tuner/checks/ref_heading_limiter_suite_check.py` | 89 | same, across the whole validation suite | **yes** |
| `tuner/checks/steering_response.py` | 144 | understeer-coefficient/full-lock-deficit analysis from a live CSV | **yes** |
| `tuner/checks/steering_step_analysis.py` | 246 | step-input transient analysis | **yes** |
| `tuner/checks/steering_sysid_analysis.py` | 297 | steady-state sweep analysis | **yes** |

### `tuner/tools/`

| Path | L | Purpose | CLI |
|---|---|---|---|
| `tuner/tools/__init__.py` | 1 | empty | no |
| `tuner/tools/doc_lint.py` | 114 | lints `docs/*.md` for prose-block length, banned refs, transcript voice | **yes** |
| `tuner/tools/export_speed_profile.py` | 211 | writes `speed_profile.csv` into `ros2/src/fsae_planning/tracks/<name>/` via `tracks.TRACKS_DIR` | **yes** | docs |
| `tuner/tools/plot_playback.py` | 874 | interactive matplotlib playback/comparison of run CSVs | **yes** | CLAUDE.md, docs, `launcher.py:1258/1262` subprocess |
| `tuner/tools/raceline_optimizer.py` | 1016 | minimum-time raceline / centerline optimizer, writes into `tracks/<name>/` | **yes** | docs, `launcher.py:1130/1131` subprocess |
| `tuner/tools/sync_mpc_params.py` | 210 | one-way param sync `fsae_planning` → `fsae_autonomous` + `fsds_simulator` mirror | **yes** | CLAUDE.md, docs, `launcher.py:1967` subprocess (GUI "Overwrite All Params") |

### `settings_profiles/` and other assets

| Path | Purpose |
|---|---|
| `settings_profiles/no_progress_tuned.json` | named GUI Settings-tab snapshot |
| `settings_profiles/progress_tuning.json` | named GUI Settings-tab snapshot |

### `deleted/` (see §8)

| Path | L | Status |
|---|---|---|
| `deleted/controller/optimiser.py` | 484 | orphaned old LTV-QP implementation, superseded by `controller/lmpc/` |
| `deleted/controller/nmpc_optimiser.py` | 1759 | orphaned old NMPC implementation, superseded by `controller/nmpc/` package |

Nothing in the live tree imports either file (confirmed by grep for `deleted.controller`/`from deleted`/`import deleted`: zero hits). `settings.py` still has a **stale comment** (lines 594-600) referring to `controller/nmpc_optimiser.py` as if it's the live NMPC module; it isn't anymore (superseded by `controller/nmpc/solver.py`'s `NMPCController`). Doc drift, not a functional coupling.

---

## 2. Dependency graph summary (package-level edges)

```
settings.py        -> sim.sim_track (TRACK_HALF_WIDTH)      [only reverse: sim depends on settings elsewhere]
model               (no internal deps except vehicle_physics <- bicycle_model)
planning            -> planning (internal only: boundary <-> path_utils near-circular, cone_sorting standalone)
controller.model_utils     (standalone, no controller.* deps)
controller.lmpc     -> (nothing outside controller.lmpc)
controller.nmpc     -> controller.model_utils
sim.sim_track       -> planning.cone_map, planning.boundary, planning.path_utils
sim.track_io        -> planning.boundary, planning.path_utils, sim.speed_profile
sim.rollout_phases  -> model.vehicle_physics, controller.lmpc, controller.nmpc, controller.model_utils, sim.speed_profile, settings
sim.rollout_core    -> model.vehicle_physics, sim.sim_track, sim.scoring, settings, sim.rollout_phases, sim.sensor_noise
sim.sensor_noise    -> sim.rollout_phases (_normalize_angle)   ** reverse edge: sim.sensor_noise depends on the file that depends on it via rollout_core **
sim.scoring         -> settings
tuner.offline_tuner -> sim.rollout_core, model.vehicle_physics, model.bicycle_model, sim.speed_profile, sim.sim_track, settings
tuner.performance_stats -> model.vehicle_physics, sim.scoring, tuner.offline_tuner, settings
tuner.recorded_map_rollout -> model.vehicle_physics, settings, sim.rollout_core, sim.track_io, tuner.offline_tuner, tracks
tuner.checks.*      -> tuner.recorded_map_rollout (DEFAULT_MAP/run), tuner.offline_tuner (get_cached_model), sim.*, model.*, settings, tuner.csv_log, tuner.tools.plot_playback (_stamp, one lazy import)
tuner.tools.*       -> sim.track_io, sim.speed_profile, tracks, tuner.csv_log
gui.launcher        -> (subprocess only, no direct Python import of sim/tuner/model)
gui.simulation      -> model.vehicle_physics, tuner.performance_stats, tuner.offline_tuner, sim.speed_profile, sim.sim_track, sim.rollout_core, sim.track_io, tracks, settings
gui.manual_drive    -> model.vehicle_physics, tuner.offline_tuner, sim.sim_track, settings
```

**Circular/near-circular:**
- `planning.boundary` (top-level `from planning.path_utils import ...`) vs `planning.path_utils` (function-local `from planning.boundary import segment_crosses_walls` at line 404). Works today only because the second import is deferred to call time. A straightforward package split must keep this deferral or break the cycle by moving `segment_crosses_walls` out.
- `sim.sensor_noise` imports `_normalize_angle` from `sim.rollout_phases`, while `sim.rollout_core` (which owns the "top" of the sim package) imports `sim.rollout_phases` AND `sim.sensor_noise` side by side. Not circular in the strict sense (sensor_noise doesn't import rollout_core), but it means `rollout_phases` is a dependency of `sensor_noise`, an unusual direction for a "phases" file to be a leaf dependency of a "noise" file — a `_normalize_angle` general utility living in the "phases" file is a misplacement (see §7).

**Odd cross-package reaches:**
- None of `tuner/` imports `gui/`. `gui/simulation.py` and `gui/manual_drive.py` import `tuner.offline_tuner` and `tuner.performance_stats` (gui → tuner is the only cross-package reach in that direction, expected since gui is UI-on-top-of-tuner).
- `gui/launcher.py` imports nothing from `sim`/`model`/`controller`/`tuner` directly — it only shells out via `subprocess` with `-m module.path` strings, so a module rename silently breaks it only at runtime (no static import to catch it).
- `tuner.checks.*` and `tuner/reference_*` files reach into `tuner.recorded_map_rollout` for `DEFAULT_MAP`/`run`, i.e. "checks" depend on a "top-level tuner script," an inversion if `checks/` is meant to be a leaf-level diagnostics folder.

---

## 3. Places that WRITE or PARSE source files by text/regex

| Writer | Edits | Target-locate mechanism | Break risk from settings.py -> package split / file moves |
|---|---|---|---|
| `gui/launcher.py::_rewrite_var` / `_read_var` (lines 318-345) | **`settings.py`** (also `launch_all.sh`) | Regex `^(\s*NAME\b\s*=\s*)([^#\n]*?)(\s*(?:#.*)?)$` anchored on flat `NAME = value` at any indent, single-file, whole-text search-and-replace. Used at ~15 call sites in `SettingsTab` (search `paths.settings_py`, lines 1558-1857) for every NAME the Settings tab knows about (LMPC weights, NMPC overrides, noise flags, etc.) | **HIGH.** If `settings.py` becomes `settings/__init__.py` + submodules, this regex will only find `NAME = value` lines that are still physically in whichever single file `paths.settings_py` points at. Any constant moved to a different submodule vanishes from the GUI's read/write (silently returns `None`/no-op — `_rewrite_var` returns `False` and the caller only records it as an error string, doesn't crash) unless `RepoPaths.settings_py` is changed to a dict of {name: file} or the write logic is taught to search across submodules. |
| `gui/launcher.py::_rewrite_shortlist_var` / `_read_shortlist_var` (348-383) | `launch_all.sh` | Same style regex, plus a leading `# ?` group to detect/toggle a commented-out shortlist line. | Not affected by settings.py split (targets a shell script). |
| `gui/launcher.py::_rewrite_yaml_field` / `_read_yaml_field` (386-418) | `fsae_params.yaml` (live repo, both original and mirror) | Regex anchored on exactly 4-space indent under `controller:` block: `^(    NAME\b:\s*)([^#\n]*?)(\s*(?:#.*)?)$` | Not affected by settings.py split; affected only if YAML indent convention changes. |
| `gui/launcher.py::_rewrite_dataclass_field` / `_read_dataclass_field` (421-452) | `mpc_params.py` / `nmpc_params.py` (live `fsae_planning` repo, out of this survey's scope but same file) | Regex anchored on `name\s*:\s*\w+\s*=\s*field(\s*default\s*=\s*)`, captures only up to next top-level comma (handles multi-line metadata dicts) | Not affected by settings.py split; would break if `MPCParams`/`NMPCParams` dataclass field declaration style changed away from `field(default=...)`. |
| `gui/launcher.py::_read_dataclass_field_desc` (460-500) | reads (never writes) `mpc_params.py`/`nmpc_params.py` metadata `desc`/`unit` for GUI tooltips | Bracket-depth scan from `field(` to matching `)`, then nested regex for `"desc"`/`"unit"` string literals (handles adjacent-string-literal concatenation) | Same as above, unaffected by settings.py split. |
| `tuner/tools/sync_mpc_params.py` | `mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml` in `fsae_autonomous` + `fsds_simulator` mirror | **Whole-file** `read_text()`/`write_text()`, no regex — copies the live `fsae_planning` file verbatim. Out of scope for `fsae_MPCTest`'s own internal restructure (targets `fsae_planning`'s change-boundary files, not `settings.py`). | Zero interaction with a `settings.py` split; it never touches `fsae_MPCTest/settings.py` at all (explicitly documented in its own docstring). |
| `tuner/tools/doc_lint.py` | none (read-only lint of `docs/*.md`) | glob `docs/**/*.md` | No interaction with settings.py. |
| `tuner/steering_chatter_check.py::main` | RUNTIME monkeypatch, not a file write: `setattr(settings, name, ast.literal_eval(value))` before importing `sim.rollout_core` | Direct attribute set on the imported `settings` module object | **MEDIUM.** If constants move to submodules re-exported into a top-level `settings/__init__.py` namespace (`from .weights import Q_diag`), `setattr(settings, 'Q_diag', ...)` still works (it rebinds the name in `settings.__init__`'s namespace) but any *other* code that does `from settings.weights import Q_diag` directly (bypassing the re-export) will NOT see the override, since it holds its own reference to the original submodule's binding. This is the single biggest hazard of a `settings.py` → `settings/` package split: overrides only propagate correctly if every consumer imports through the package's top-level namespace, never through a submodule import path. |

**No other text/regex source-file writers found** in scope (`fsae_MPCTest` excluding `fsds_simulator/`).

---

## 4. Hard-coded filesystem/module path strings

| String | Where | Purpose |
|---|---|---|
| `"ros2/src/fsae_planning/tracks"` (built via `Path` joins) | `tracks/__init__.py` (`TRACKS_DIR`), `gui/launcher.py::_repo_paths` (`tracks_dir`), `tuner/tools/sync_mpc_params.py` (indirectly, via `_LIVE_ROOT`) | cross-repo track-data location |
| `"ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc"` | `gui/launcher.py::_repo_paths` (`fsae_planning_mpc`), `tuner/tools/sync_mpc_params.py` (`_RELATIVE_PATHS`) | live mpc_params.py/nmpc_params.py location |
| `"common/fsae_bringup/config/fsae_params.yaml"` | same two files | live YAML params location |
| `"fsds_simulator"` (mirror root) | `gui/launcher.py::_repo_paths`, `tuner/tools/sync_mpc_params.py` (`_MIRROR_ROOT`) | PR-staging mirror root |
| `"fsae_autonomous"` and `"ros2_autonomous/src/fsae_autonomous"` | `tuner/tools/sync_mpc_params.py::_AUTONOMOUS_CANDIDATES` | production repo, path known to have moved once already |
| `"$HOST_REPO_ROOT/fsae_logs"` equivalent (`fsds_root / "fsae_logs"`) | `gui/launcher.py::_repo_paths` (`fsae_logs_dir`) | must match `launch_all.sh`'s own variable exactly |
| `"settings_profiles"` | `gui/launcher.py::_repo_paths` (`profiles_dir`) | GUI profile JSONs |
| `"tuner.recorded_map_rollout"`, `"tuner.nmpc_offline_check"`, `"tuner.tools.export_speed_profile"`, `"tuner.tools.raceline_optimizer"`, `"tuner.tools.plot_playback"`, `"tuner.tools.sync_mpc_params"`, `"gui.simulation"`, `"gui.launcher"` | module-path strings, used both as `python -m X` in docs/CLAUDE.md and as literal `[sys.executable, "-m", "X", ...]` argv lists in `gui/launcher.py` (lines 1129-1131, 1258-1299, 1967) | **These break silently on any module rename/move** — no static import, only resolved at subprocess-spawn time. Any restructure that moves `tuner/tools/*.py` or `gui/simulation.py` must update every one of these string literals AND every doc reference. |
| Dead/stale module-path strings found in `docs/logs/sim_to_real_investigation.md`: `tuner.live_vs_sim_diagnostics`, `tuner.combined_factors_sweep`, `tuner.gap_attribution_ledger`, `tuner.blend_reset_diagnostics`, `tuner.ref_heading_limiter_ab`/`ref_heading_limiter_suite_check` (missing the now-real `tuner.checks.` prefix) | docs only | Pre-existing doc drift, unrelated to this restructure, but the restructure should not compound it — fix or flag when touching that doc. |

---

## 5. Files over ~400 lines: proposed split

| File | Lines | Sections (line ranges) | Suggested split | Coupling hazards |
|---|---|---|---|---|
| `gui/launcher.py` | 2436 | Theming/Palette (65-245); `RepoPaths`+file-rewrite helpers (246-521); `LaunchTab` (597-1188); `LogDebugTab` (1189-1269); `OfflineSimTab` (1270-1497); `SettingsTab` (1521-2131); `ProfilesTab` (2132-2284); misc parsing helpers + `LauncherApp`/`main` (2285-2436) | `gui/launcher/paths.py` (RepoPaths+_repo_paths), `gui/launcher/file_edit.py` (all `_rewrite_*`/`_read_*` regex helpers), `gui/launcher/theme.py` (Palette/apply_theme), `gui/launcher/tabs/{launch,log_debug,offline_sim,settings,profiles}.py`, `gui/launcher/app.py` (LauncherApp/main) | The five tab classes all close over the same `RepoPaths` instance and some share widget-parsing helpers (`_to_float`, `_parse_float_list` at 2285-2314) — must land in a shared module both tabs import, not duplicated. `ProfilesTab` reaches into both `LaunchTab` and `SettingsTab`'s `capture_profile_values`/`apply_profile_values` methods — a split into separate tab modules needs an explicit shared protocol/interface, not implicit duck-typing across files that used to be co-located. |
| `settings.py` | 1625 | General/system config (19-189); SLAM/cone noise (190-345); planner tunables + DT/limits (346-502); LMPC cost weights (503-593); NMPC block: flags, weights, solver internals, alat ceiling, progress term (594-1046); DNF penalty config (1047-1072); solver settings for headless rollouts + Optuna presearch (1073-1180); scoring weights (`METRIC_SCALES`/`SCORE_WEIGHTS`/`VALIDATION_SUITE`) (1181-1424); performance bonus weights (1425-1625) | `settings/general.py` (system config, noise, planner, DT), `settings/lmpc_weights.py`, `settings/nmpc_weights.py` (the largest single block, ~450 lines), `settings/scoring.py` (DNF + METRIC_SCALES + SCORE_WEIGHTS + bonuses), `settings/solver.py` (Optuna/rollout iteration limits); `settings/__init__.py` re-exports every public name so `import settings; settings.Q_diag` and `from settings import Q_diag` both keep working for every existing caller. | **This is the highest-risk split in the whole survey.** (1) `settings.py` imports `sim.sim_track.TRACK_HALF_WIDTH` at module scope (line 16) to compute `OFFTRACK_LIMIT` — whichever submodule holds `OFFTRACK_LIMIT` inherits this import, and `sim.sim_track` itself does NOT import settings at its own top level (grep confirms a lazy `from settings import (...)` at `sim/sim_track.py:253`, inside a function) so there's no true cycle today, but a careless split could introduce one if a different submodule tries to import `sim.sim_track` where `sim.sim_track` also wants a settings constant at import time. (2) `tuner/steering_chatter_check.py`'s `setattr(settings, NAME, value)` (see §3 above) only works correctly post-split if every read of that constant elsewhere goes through the top-level `settings` namespace, never a submodule import — this must be an explicit rule applied to the whole codebase, not just launcher.py. (3) `gui/launcher.py`'s regex file-editors need to be taught which submodule file owns which NAME (currently a single `paths.settings_py` constant) — either a `{name: submodule_path}` lookup table generated from the split, or a search-across-all-submodules fallback. |
| `deleted/controller/nmpc_optimiser.py` | 1759 | N/A — orphaned, no live imports. Not a split candidate; a deletion/archival candidate instead (flagged separately, see §8). | — | — |
| `tuner/offline_tuner.py` | 1568 | synthetic path/track construction (262-666); multiprocessing worker + headless rollout eval (667-960); candidate scoring/CMA-ES glue (961-1130); git-hash + result logging (1131-1568, plus large docstring/config block 1-261) | `tuner/offline_tuner/synthetic_paths.py`, `tuner/offline_tuner/rollout_eval.py` (worker/parallel eval), `tuner/offline_tuner/cma_driver.py` (main CMA-ES loop, `run_optuna_presearch`), `tuner/offline_tuner/history_log.py` (`get_git_revision_hash`, `log_results_to_history`) | `gui/simulation.py` and `gui/manual_drive.py` import `SYNTHETIC_PATHS`, `PATH_NAMES`, `get_cached_model` directly from `tuner.offline_tuner` — these three names must stay resolvable from a re-export at `tuner/offline_tuner/__init__.py` (or wherever the package root ends up) for those two GUI files to keep working without their own edits (though the task says all callers get updated, so this is a "must update these 2 import lines" note, not a blocker). Multiprocessing (`init_worker`, module-level globals set at worker-process start for `Q_init`/`R_init`/`R_rate_init`) is genuine shared mutable state across a process pool — splitting the file must keep the worker-init globals and the function that reads them in the same module (or pass them explicit, not via module attribute) since forked/spawned worker processes each get their own fresh copy of whatever module state exists at fork time. |
| `gui/simulation.py` | 954 | mostly one flat script: event-callback functions bound to matplotlib widgets, no classes. `simulate_closed_loop` (602-738) is the core rollout wrapper; the rest are UI callbacks (`load_test_path`, `run_simulation`, `run_optimize`, `run_benchmark`, mouse handlers). | Lower priority to split (script-style GUI code, not library code with many importers) — if split, separate "closed-loop driver" (`simulate_closed_loop`, `run_benchmark`, `run_optimize`) from "widget callbacks/plot setup". | `USE_PLANNER` is read as a **default argument value** at function-definition time (`def simulate_closed_loop(..., use_planner=USE_PLANNER)`, line 602) — this bakes in whatever `settings.USE_PLANNER` was at import time; a runtime `setattr(settings, 'USE_PLANNER', ...)` (as `steering_chatter_check.py` does for other constants) would NOT affect an already-defined default argument. Existing hazard, unrelated to any split, but relevant to "import-time side effects" generally. |
| `tuner/tools/raceline_optimizer.py` | 1016 | large single-purpose CLI script (raceline optimization + centerline export mode) | Lower priority; could split geometry/optimization core from CLI arg-parsing + file I/O, but only one caller path (its own `__main__`) so less urgent than settings.py/launcher.py. | none notable beyond ordinary function decomposition |
| `controller/nmpc/solver.py` | 1042 | `NMPCController` class: init, per-tick SQP loop, trust-region/backtracking, OSQP call, diagnostics | Already reasonably factored out of the once-monolithic `deleted/controller/nmpc_optimiser.py` into the `controller/nmpc/` package (layout/dynamics/reference/outputs/weight_schedule already separated) — `solver.py` itself is the remaining "orchestrator", further splitting it risks the exact SQP/Hessian danger zone CLAUDE.md flags as expensive-tier; **not recommended to split further without a high-effort/expensive-tier pass**, per CLAUDE.md's own model-usage guidance for NMPC solver internals. |
| `model/vehicle_physics.py` | 1290 | `VehicleParams` dataclass (124-446); Pacejka tyre model fns (446-568); `step_nonlinear_plant` (568-1088, the single largest function in the repo, ~520 lines); init/reference helpers (1088-1290) | Splitting is explicitly the "danger zone" CLAUDE.md calls out for expensive-tier review only (`alat_ceiling*`, tyre-parameter dead ends). **Do not split this file as part of a routine restructure pass** — flag it as an explicit, separately-scoped, high-effort task if it's split at all, and re-run `tuner.plant_openloop_validation` bit-for-bit before/after even a pure reorganization (moving code across files can change floating-point evaluation order in numpy in rare cases; the existing verification command is exactly the tool to catch that). |

Files close to but under 400 (`tuner/checks/plant_openloop_validation.py` 359, `sim/track_io.py` 369, `sim/sim_track.py` 353) are not flagged for splitting; they read as single-responsibility already.

---

## 6. How `settings.py` is consumed

- **`import settings` (module handle)**: `tuner/offline_tuner.py` (comment: "for the NMPC tail's shipped x0 seed"), `tuner/steering_chatter_check.py` (for `setattr` overrides), `tuner/nmpc_offline_check.py` (`import settings as S`, comment notes Q_diag/R_diag/etc. still read from here).
- **`from settings import NAME, NAME2, ...` (bare-name import)**: the overwhelming majority — `sim/rollout_core.py`, `sim/rollout_phases.py`, `sim/scoring.py`, `sim/sim_track.py` (function-local, line 253), `tuner/offline_tuner.py`, `tuner/recorded_map_rollout.py`, `tuner/performance_stats.py`, `tuner/checks/ref_heading_limiter_ab.py`, `tuner/checks/ref_heading_limiter_suite_check.py` (plus a nested function-local `from settings import VALIDATION_SUITE`), `tuner/reference_excess_mechanism_check.py`, `tuner/reference_heading_geometry_check.py`, `gui/simulation.py`, `gui/manual_drive.py` (`DT` only). **This is the exact hazard called out in the task's premise**: bare-name imports bind the *value at import time*; any `settings.py`→`settings/` split that changes WHICH submodule a name's canonical binding lives in is transparent to this style of import (still `from settings import Q_diag` works via the package's re-export), but a runtime override via `setattr(settings, 'Q_diag', ...)` performed AFTER these modules already did `from settings import Q_diag` never reaches them (this is a pre-existing hazard of Python's import semantics, not introduced by a split — the codebase already documents this in `steering_chatter_check.py`'s own docstring: "Run this script directly ... or later overrides in the same process will not take effect on already-imported names").
- **`settings.X = ...` runtime mutation**: **none found via direct attribute-assignment grep** in `fsae_MPCTest` proper (excluding `fsds_simulator/`). The only runtime mutation is `steering_chatter_check.py`'s `setattr(settings, name, ast.literal_eval(value))`, functionally identical to `settings.name = value` but done generically by name from a CLI arg.
- **getattr/setattr by name**: only `tuner/steering_chatter_check.py` (setattr, see above). No getattr-by-name pattern found.
- **GUI Settings tab**: does not `import settings` as a Python module at all for its read/write path — it treats `settings.py` purely as **text**, via `_read_var`/`_rewrite_var` regex (see §3). This means the GUI's view of "current value" can be out of sync with whatever a differently-running Python process has (e.g., if a tuner process had already monkeypatched its own in-memory copy), though that's an existing, not newly-introduced, characteristic.

---

## 7. Current folder-organisation problems

- **`controller/model_utils.py` sits at the `controller/` package root**, not inside `controller/lmpc/` or `controller/nmpc/`, despite being consumed by both `sim/rollout_phases.py` (LMPC path) and `controller/nmpc/weight_schedule.py`/`controller/nmpc/solver.py` (NMPC path) — this is actually the CORRECT location for a genuinely shared helper (not a misplacement), but its name doesn't signal "shared by both controllers" versus e.g. "lmpc-only" the way `controller/nmpc/*` files signal NMPC-only; a rename to something like `controller/shared_gain_utils.py` would make the sharing explicit. Low priority.
- **`tuner/` mixes three tiers with no folder boundary between two of them**: top-level `tuner/*.py` contains both a primary correctness-bar script (`recorded_map_rollout.py`), the tuning driver (`offline_tuner.py`), AND several one-off investigation scripts (`reference_excess_mechanism_check.py`, `reference_heading_geometry_check.py`, `steering_chatter_check.py`) that are peers of what's already under `tuner/checks/`. There's no principled reason `reference_heading_geometry_check.py` lives at `tuner/` while `ref_heading_limiter_ab.py` (same investigation family, cross-referenced in the same docs section) lives at `tuner/checks/`. Recommend consolidating all "one-shot investigation/reproduction scripts" into `tuner/checks/`, keeping only `offline_tuner.py`, `recorded_map_rollout.py`, `nmpc_offline_check.py`, `csv_log.py` at `tuner/` top level as the "primary driver + shared utility" tier.
- **`sim/sensor_noise.py` imports `_normalize_angle` from `sim/rollout_phases.py`** — a general angle-wrapping utility living inside a file named for "rollout phases" rather than in a shared low-level utils module; this is the naming inconsistency underlying the near-circular-shaped edge noted in §2. Recommend a `sim/geometry_utils.py` (or reuse `controller/nmpc/layout.py`'s existing `_wrap`, which is the same operation under a different name in a different package — a duplicate-helper case, see below).
- **Duplicate angle-wrap helpers**: `controller/nmpc/layout.py::_wrap` and `sim/rollout_phases.py::_normalize_angle` both wrap an angle into a canonical range; not verified byte-identical but almost certainly the same operation implemented twice across packages. Worth consolidating into one shared utility during the restructure (per CLAUDE.md's own code-review checklist: "No duplicated logic where an existing utility already does it").
- **Dead code**: `deleted/controller/optimiser.py` and `deleted/controller/nmpc_optimiser.py` — confirmed via grep, nothing in the live tree imports either (see §8). No other orphaned modules found; every other `.py` file in the inventory has at least one importer or is a standalone CLI script with real doc/CLAUDE.md references.
- **`settings.py.bak`**: a stray backup file at the repo root, presumably from `gui/launcher.py`'s or a manual `_backup_once`-style safety copy. Not referenced by any code; candidate for cleanup (or explicit "leave it, it's a safety net" decision) when `settings.py` is restructured, since a stale `.bak` sitting next to a freshly-split `settings/` package could be mistaken for the source of truth later.

---

## 8. `deleted/` folder and `.claude/worktrees`

- **`deleted/controller/optimiser.py`** (484 lines) and **`deleted/controller/nmpc_optimiser.py`** (1759 lines): confirmed zero references anywhere in `fsae_MPCTest` (excluding `fsds_simulator/`) via `grep -rn "deleted.controller\|from deleted\|import deleted"` — no hits. `settings.py`'s prose comments (lines 594-600) still narrate `controller/nmpc_optimiser.py` as if it's live, which is stale but harmless (a comment, not a binding). These two files appear to be the pre-refactor monoliths that `controller/lmpc/` and `controller/nmpc/` (the current packages) superseded. Given the task's premise ("`deleted/` will be removed"), removing both is safe from a live-import standpoint; the only loss would be historical reference value (git history already preserves that if ever needed, so the folder itself is redundant with version control).
- **`.claude/worktrees`**: exists as a directory but empty (a `find -maxdepth 4` on it returned only the directory itself and its immediate parent, no files/subdirs listed) — no content to account for in the restructure.

---

## 9. Verification entry points

| Command | Invoke from | Args | Runtime | Determinism | Files written |
|---|---|---|---|---|---|
| `python -m tuner.recorded_map_rollout` | `fsae_MPCTest/` | none required; optional flags exist per its own `--help` (not enumerated here, see file) | ~2 min (per CLAUDE.md) | Deterministic given fixed `settings.py`/plant/map — no RNG without a fixed seed (SLAM/cone noise seeds are fixed constants in settings.py when enabled, off by default). Suitable for bit-identical before/after comparison **as long as `settings.py`'s values are unchanged** — a restructure that only reorganizes code (same values) should reproduce identical console output/metrics. | Writes rollout output to stdout (a comparison table); does not appear to write log files itself based on its import list (no `csv_log`/file-write imports) — confirm by reading the file directly if a written-file inventory becomes load-bearing. |
| `python -m tuner.nmpc_offline_check` | `fsae_MPCTest/` | none | fast (solver self-consistency, no full rollout sweep implied by size) | Deterministic (self-consistency assertions, not a scored rollout) | none apparent (pure assertions/printed results) |
| `python -m tuner.checks.plant_openloop_validation [--ab] [--robustness]` | `fsae_MPCTest/` | `--ab`, `--robustness` optional flags per docs/debugging_tools.md | moderate | Deterministic given fixed plant params; replays two fixed open-loop experiments | reads existing CSV logs (via `tuner.csv_log`), does not appear to write new ones from its own import list |
| `python -m tuner.tools.sync_mpc_params [--apply]` | `fsae_MPCTest/` | `--apply` (default is dry-run/diff-only) | fast | N/A (not a scoring check) — dry run is side-effect-free by design | with `--apply`: overwrites live-boundary files in `fsae_autonomous`/`fsds_simulator` mirror plus one-time `.bak` backups; **out of scope for this internal restructure** since it never touches anything under survey here except by reading `fsae_MPCTest`'s own path constants |
| `python -m tuner.tools.doc_lint [--max N] [--strict]` | repo root (docstring says "from the repo root", i.e. `fsae_MPCTest/`) | `--max`, `--strict`, optional positional paths | fast | Deterministic (pure text scan) | none, read-only |

**Not yet confirmed by direct execution** (read-only survey, no runs performed per task constraints): exact `--help` flag lists for `recorded_map_rollout.py`/`nmpc_offline_check.py`, and whether `recorded_map_rollout.py` writes any file under `fsae_logs/` — these would need a live run (out of scope here) or a closer read of the specific function bodies handling output if that detail becomes load-bearing for the restructure plan.
