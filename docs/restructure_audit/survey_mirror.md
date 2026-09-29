# Mirror-as-source-of-truth restructuring survey

MIRROR = `fsae_MPCTest/fsds_simulator/`
LIVE = `ros2/src/fsae_planning/` (git repo, branch `feature/nmpc-and-controller-improvements`, uncommitted changes + 3 `.bak` files + untracked `brake_sysid.py`)

## 1. File-level diff inventory (implementation files, exclusions applied)

Compared 67 common relative paths (excludes tracks/, recorded_runs/, __pycache__, build/install/log, .bak/.orig, cone_maps/).

| Class | Count | Files |
|---|---|---|
| SAME (byte-identical) | 65 | everything else, incl. `mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`, `mpc_core.py`, `nmpc_core.py`, `live_viz.py`, `control_utils.py`, `telemetry_logger.py`, `setup.py` (all packages), all launch files, all planning/perception files |
| LIVE-NEWER | 1 | `control/fsae_control/fsae_control/mpc/mpc_controller.py` |
| MIRROR-only doc (not a sync target) | 1 | `README.md` — each side has its own purpose-built README (live: project README; mirror: staging-mirror README describing the mirror itself). Not a parity conflict, do not overwrite mirror's with live's. |
| ONLY IN LIVE | 3 | `.gitignore`, `CHANGES.md` (24KB historical changelog, live-only, not part of mirror's tracked scope), `launch_terminals.sh` |
| ONLY IN MIRROR | 2 | `fsds_ros2_custom.Dockerfile`, `fsds_simulator/launch_all.sh` (mirror's adapted-path copy; the outer-repo original is `ros2/launch_all.sh`, a different location, not inside `fsae_planning`) |
| BOTH-CHANGED (conflict) | 0 | none found |
| TRIVIAL | 0 | none found |

**Untracked-in-live files**: `brake_sysid.py` — already present, byte-identical, in mirror (`control/fsae_control/fsae_control/brake_sysid.py`, also has a `.pyc` in mirror's `__pycache__`, ignore). Its `setup.py` entry_point (`brake_sysid = fsae_control.brake_sysid:main`) is already registered in the mirror's `control/fsae_control/setup.py`. Live simply hasn't `git add`ed it yet (read-only repo for agents, not agent's job to fix).

**`.bak` files** (`mpc_params.py.bak`, `nmpc_params.py.bak`, `fsae_params.yaml.bak`): pre-edit snapshots of the *tracked* files' previous values (e.g. `speed_target_deficit_max` 5.0→2.55, `nmpc_q_progress` 4.25→1.0, `nmpc_slack_linear_weight` 500→1000, `nmpc_progress_reach` 3.0→1.25). These are older than the current tracked files, not a separate divergence from the mirror — current tracked `mpc_params.py`/`nmpc_params.py`/`fsae_params.yaml` are already SAME as mirror (see table above). No action needed; excluded from overwrite per task scope anyway (`.bak`).

### Named files spot-check (CLAUDE.md's explicit list)
`mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`, `mpc_core.py`, `nmpc_core.py`, `live_viz.py`, `control_utils.py`, `telemetry_logger.py`, `setup.py` (all 4 packages) — **all byte-identical**, mirror already current. `brake_sysid.py` untracked in live but present and identical in mirror.

### `mpc_controller.py` diff (LIVE-NEWER, 15 lines)
Live has an extra block absent from mirror:
```python
# TEMPORARY: disables curvature_speed() in the live (no precomputed-profile)
# branch below, falling back to a flat v_max instead...
DISABLE_LIVE_CURVATURE_SPEED = False
```
plus the corresponding `elif DISABLE_LIVE_CURVATURE_SPEED:` branch (~8 lines) in the speed-target logic. mtime: live 2026-09-28 11:33, mirror 2026-09-28 11:43 (mirror's is *newer by mtime* despite lacking this block — mtime reflects last mirror sync commit, not feature recency). Live's own git log for this file shows no commit yet containing this exact flag (top 5 commits: `ba0e580`, `e339901`, `b244b2e`, `79ff129`, `68055d3`), consistent with it being part of live's current *uncommitted* working-tree changes. **Classification: LIVE-NEWER, must pull into mirror before restructuring.**

### `README.md` — not a parity file
Live's version: project README for `fsae_planning` (quick start against real FSDS, repo purpose). Mirror's version: explains the mirror's own role, build instructions to reconstruct a standalone workspace, points to `docs/reference/` for the file-mapping table. 485-line diff is expected divergence, not drift; keep mirror's own README as-is (task's plan of "overwrite fsae_planning's implementation files" should explicitly exclude README.md, same as tracks/, recorded_runs/, etc.).

### `launch_all.sh`: outer (`ros2/launch_all.sh`) vs mirror (`fsds_simulator/launch_all.sh`)
18-line diff, all config/toggle differences (not code-structure differences):
- `TRACK=comp_test_map_3` (outer, active) vs `# TRACK=...` (mirror, commented)
- `USE_PRECOMPUTED_SPEED`/`USE_PRECOMPUTED_PATH`: `true`/`true` (outer) vs `false`/`false` (mirror)
- `NMPC_Q_E_Y=7.5` commented in outer vs active in mirror
- `NMPC_SLACK_LINEAR_WEIGHT` commented value 1000.0 (outer) vs 0.0 (mirror)

These read as live session-local experimentation state (which track/mode is currently being run), not a structural code change. Flag as **BOTH-CHANGED / needs a human** to decide which toggle-state is "correct" to carry forward — do not silently pick one.

## 2. Mirror structure inventory

| File | Lines | Package | Purpose | Main classes/functions | Entry point? |
|---|---|---|---|---|---|
| `control/fsae_control/fsae_control/mpc/nmpc_core.py` | 2364 | fsae_control | NMPC solver core: Frenet nonlinear MPC plant model, dynamics, SQP step, rrate-zone shaping | `PathReference`, `_Plant`, `_tyre_forces`, `_f`/`_f_scalar`, `_step`/`_step_scalar`, `_outputs`, `_rrate_zone_scale`, `_rrate_stage_ramp`, `_csc_pattern`, `NMPCController` | No (imported by `mpc_controller.py`) |
| `control/fsae_control/fsae_control/mpc/mpc_core.py` | 1499 | fsae_control | LTV-QP MPC core: adaptive gain scaling, anti-hunt, corner boosts, `MPCController` | `predict_ahead`, `_adaptive_R_scaling`, `_steer_rate_anti_hunt`, `_reversal_penalty_boost`, `_adaptive_Q_scaling`, `_curvature`, `_corner_factor`, `_blend`, `_low_speed_corner_boost`, `MPCController` | No |
| `control/fsae_control/fsae_control/mpc/mpc_controller.py` | 1220 | fsae_control | ROS2 node wrapping both MPC/NMPC controllers, param declaration, main control loop | `MPCControllerNode(Node)`, `main()` | **Yes**: `mpc_controller = fsae_control.mpc.mpc_controller:main` |
| `control/fsae_control/fsae_control/telemetry_logger.py` | 1004 | fsae_control | CSV logging, lap-progress tracking, horizon-accuracy tracking, score header emission | `build_config_lines`, `HorizonAccuracyTracker`, `LapProgressTracker`, `ControlLogger` | No (imported) |
| `control/fsae_control/fsae_control/live_viz.py` | 811 | fsae_control | Live matplotlib debug visualisation node (cost breakdown, horizon prediction) | `get_car_triangle`, `LiveVizNode(Node)`, `main()` | **Yes**: `live_viz = fsae_control.live_viz:main` |
| `control/fsae_control/fsae_control/mpc/nmpc_params.py` | 695 | fsae_control | `NMPCParams` dataclass (subset of parity-tracked fields) | dataclass only | No |
| `control/fsae_control/fsae_control/control_utils.py` | 582 | fsae_control | Stanley controller, speed-gate/curvature-speed/dynamic-speed-cap helpers, CSV profile loaders | `_heading_error`, `compute_steering`, `StanleyController`, `tracking_error_speed_gate`, `curvature_speed`, `dynamic_speed_cap`, `_load_profile_csv`, `load_speed_profile_csv`, `load_path_profile_csv`, `load_path_heading_profile_csv`, `precomputed_speed_at` | No |
| `planning/fsae_planning/fsae_planning/path_utils.py` | 476 | fsae_planning | Path/geometry utilities for planner | (not enumerated in this pass) | No |
| `control/fsae_control/test/nmpc_offline_check.py` | 443 | fsae_control | pytest-style solver self-consistency checks (mirrors `tuner.nmpc_offline_check`) | test functions | No (test) |
| `common/fsae_bringup/launch/control.launch.py` | 422 | fsae_bringup | Launch composition, generates MPC/NMPC launch args from `MPCParams`/`NMPCParams` field metadata | launch-arg generation functions | N/A (launch file) |
| `planning/fsae_planning/fsae_planning/boundary.py` | 414 | fsae_planning | Track-boundary computation for planner | (not enumerated) | No |
| `control/fsae_control/fsae_control/stanley_controller.py` | 386 | fsae_control | Standalone Stanley controller ROS2 node | node class + `main` | **Yes**: `controller = fsae_control.stanley_controller:main` |
| `control/fsae_control/fsae_control/scoring.py` | 374 | fsae_control | Live copy of `sim/scoring.py` (parity, see §7) | `compute_composite_score`, `RolloutMetrics` | No (imported by telemetry_logger) |
| `planning/fsae_planning/fsae_planning/special_utils/skidpad_planner.py` | 342 | fsae_planning | Skidpad-event planner | node class | **Yes**: `skidpad_planner = fsae_planning.special_utils.skidpad_planner:main` |
| `perception/fsae_sim_perception/fsae_sim_perception/sim_perception.py` | 337 | fsae_sim_perception | FSDS oracle/odom → `/fsae/*` bridge | node class | **Yes**: `sim_perception = fsae_sim_perception.sim_perception:main` |
| `control/fsae_control/fsae_control/mpc/mpc_params.py` | 327 | fsae_control | `MPCParams` dataclass, 106 fields total combined w/ NMPCParams | dataclass, `declare_mpc_params`, `mpc_params_from_node` (per CLAUDE.md) | No |
| `planning/fsae_planning/fsae_planning/centerline_planner.py` | 274 | fsae_planning | Centerline planning node | node class | **Yes**: `centerline_planner = fsae_planning.centerline_planner:main` |
| `perception/fsae_sim_perception/fsae_sim_perception/cone_recorder.py` | 240 | fsae_sim_perception | Cone-map recording utility | node class | **Yes**: `cone_recorder = fsae_sim_perception.cone_recorder:main` |
| `common/fsae_bringup/launch/sim.launch.py` | 234 | fsae_bringup | Top-level sim launch composition | launch functions | N/A (launch file) |
| `control/fsae_control/fsae_control/brake_sysid.py` | 233 | fsae_control | Open-loop braking system-ID node | node class | **Yes**: `brake_sysid = fsae_control.brake_sysid:main` |
| `planning/fsae_planning/fsae_planning/special_utils/skidpad.py` | 208 | fsae_planning | Skidpad geometry helper | functions | No |
| `control/fsae_control/fsae_control/fsds_bridge.py` | 174 | fsae_control | cmd_vel → FSDS bridge node | node class | **Yes**: `fsds_bridge = fsae_control.fsds_bridge:main` |
| `planning/fsae_planning/fsae_planning/special_utils/speed_input.py` | 124 | fsae_planning | Manual speed-input helper | functions | No |
| `planning/fsae_planning/fsae_planning/cone_sorting.py` | 102 | fsae_planning | Cone-sorting logic (boundary/left-right assignment) | functions | No |
| `planning/fsae_planning/fsae_planning/cone_map.py` | 93 | fsae_planning | Cone-map data structure | class | No |
| remaining launch/setup/test files | ≤32 each | various | trivial (planning.launch.py, perception.launch.py, cone_recorder.launch.py, 4×setup.py, 3 flake8/pep257/copyright test stubs) | — | launch files: N/A; setup.py: N/A |

## 3. Proposed splits for files over ~400 lines

**`nmpc_core.py` (2364 lines) — highest priority split.** Internal sections already delimited by comment banners and clear class boundaries:
- Lines 1–160: module docstring, imports, state/input/output layout constants → keep as `nmpc_core/__init__.py` or `nmpc_types.py`
- 161–425: `_wrap`, `PathReference` → `nmpc_core/path_reference.py`
- 425–503: `_Plant`, `_tyre_forces` → `nmpc_core/plant.py`
- 503–763: `_f`, `_f_scalar`, `_step_scalar`, `_step`, `_outputs` (dynamics/integration) → `nmpc_core/dynamics.py`
- 862–961: `_rrate_zone_scale`, `_rrate_stage_ramp`, `_csc_pattern` (rate-limiting/sparsity shaping) → `nmpc_core/shaping.py`
- 962–2364 (bulk of the file, ~1400 lines): `NMPCController` itself → likely needs its own internal breakdown (not enumerated at method level in this pass; recommend a follow-up `grep -n "    def " nmpc_core.py` before committing to sub-files, since a single 1400-line class is the real split target, not just the module-level helpers already broken out above).

**`mpc_core.py` (1499 lines).** Comment-banner sections map cleanly:
- 173–333: `predict_ahead`, `_adaptive_R_scaling`, `_steer_rate_anti_hunt`, `_reversal_penalty_boost` → `mpc_core/adaptive_gains.py`
- 334–432: `_adaptive_Q_scaling`, `_curvature`, `_corner_factor`, `_blend`, `_low_speed_corner_boost` → `mpc_core/corner_shaping.py`
- 433–1499: `MPCController` class → main `mpc_core.py` or its own sub-package, same caveat as NMPC above (needs method-level breakdown before finalizing).

**`mpc_controller.py` (1220 lines).** Single `MPCControllerNode` class (226–1206) plus `main()`. A ROS2 node's callback-heavy structure resists clean splitting without becoming several files that all need the same `Node` instance; lowest-priority split candidate of the three big files.

**`telemetry_logger.py` (1004 lines).** Already three distinct classes with a clear boundary (163/238/376/600 line breaks): `build_config_lines` + `HorizonAccuracyTracker` → `telemetry/horizon_tracker.py`; `LapProgressTracker` → `telemetry/lap_progress.py`; `ControlLogger` → `telemetry/control_logger.py`. Clean, low-risk split.

**`live_viz.py` (811 lines).** Mostly one `LiveVizNode` class (142–389); moderate split candidate (debug-panel rendering vs. node/subscription logic), lower priority than nmpc_core/mpc_core.

**`nmpc_params.py` (695 lines)** and **`control_utils.py` (582 lines)**: under the ~400-line stated threshold's near-miss zone but flagged since named in the task. `nmpc_params.py` is a single dataclass (any split risks breaking the "one dataclass, one file" simplicity CLAUDE.md's parity section relies on — recommend NOT splitting this one, it's the single source of truth and splitting it fights the stated design goal). `control_utils.py` mixes Stanley-controller code with speed-gate/profile-loading helpers; a `control_utils/stanley.py` + `control_utils/speed_profiles.py` split is reasonable but not urgent (still under threshold).

### ROS2 packaging constraints for any split (applies to all of the above)
- **`setup.py` `packages=find_packages(exclude=['test'])`**: auto-discovers sub-packages via `__init__.py`, so turning e.g. `mpc/nmpc_core.py` into a `mpc/nmpc_core/` sub-package with its own `__init__.py` is discovered automatically — no `setup.py` edit needed **provided** the public re-export surface (`NMPCController`, module-level constants other files import) is preserved via `nmpc_core/__init__.py`.
- **`entry_points` console_scripts** reference dotted paths (e.g. `fsae_control.mpc.mpc_controller:main`) — splitting `mpc_controller.py` itself (not just files it imports) requires updating this string in `control/fsae_control/setup.py`.
- **Launch files** (`control.launch.py` at line ~281 generates launch args "from `MPCParams`' own field metadata, not hand-written" per CLAUDE.md) import `mpc_params`/`nmpc_params` directly by module path; splitting those files, or moving them out of `fsae_control.mpc`, breaks this import unless the split preserves the exact `fsae_control.mpc.mpc_params`/`fsae_control.mpc.nmpc_params` import path.
- **`data_files`**: only launch/config/package.xml globs are declared per-package; a new sub-package directory does not need a new `data_files` entry (Python files aren't installed via `data_files`, only via `packages`), but a new resource dir (rare) would.
- **`package.xml`**: no code-path dependency, but should be checked for `<export><build_type>` / dependency declarations if a split introduces a new external import.
- **symlink-install** (`colcon build --symlink-install` in `launch_all.sh`): symlinks the whole package dir into the install space, so a split into a sub-package works transparently as long as `find_packages()` still discovers it — no incremental-rebuild hazard beyond the normal "restart node after a source change" one already noted in `launch_all.sh`'s rebuild step.
- **External consumers reaching in by exact file path** (§5 below) must be updated for every renamed/moved file — this is the biggest real hazard, bigger than the ROS2 packaging mechanics themselves.

## 4. Folder organisation problems in the mirror

- **`nmpc_core.py` and `mpc_core.py` both live directly under `mpc/`** with no further grouping despite being 2364 and 1499 lines respectively — the single biggest structural problem, not a naming issue.
- **No duplicated helpers found** between `control_utils.py` and the MPC modules (checked every top-level function name in `control_utils.py` against `mpc_core.py`/`mpc_controller.py`/`nmpc_core.py`; zero collisions) — this specific concern from the task is not present.
- **`control_utils.py` name is generic** for what it contains (Stanley controller + speed-profile/curvature-speed helpers) — could be split/renamed per §3, but isn't misleading enough to call a defect on its own.
- **`special_utils/` naming** (`skidpad.py`, `skidpad_planner.py`, `speed_input.py`) is vague; all three are skidpad-event/manual-input specific, not general utilities — `special_utils/` reads as a catch-all rather than a description of contents.
- **No dead code identified** in this pass (would need a deeper import-graph check than this survey's scope to confirm confidently; nothing obviously unreferenced turned up while reading top-level defs).

## 5. Live-side / cross-repo consumers that reach into these paths by exact string (would break on a move)

**`fsae_MPCTest/gui/launcher.py`** (38 references total), key exact path constructions:
```python
fsae_planning_mpc = (fsds_root / "ros2" / "src" / "fsae_planning" / "control" ...)  # -> mpc_params.py location
mirror_root = fsae_mpctest / "fsds_simulator"
tracks_dir = fsds_root / "ros2" / "src" / "fsae_planning" / "tracks"
recorded_runs_dir = fsae_mpctest / "fsds_simulator" / "recorded_runs"
mpc_params_py = fsae_planning_mpc / "mpc_params.py"
nmpc_params_py = fsae_planning_mpc / "nmpc_params.py"
fsae_params_yaml = (fsds_root / "ros2" / "src" / "fsae_planning" / "common" ...)
```
Any move of `mpc_params.py`/`nmpc_params.py`/`fsae_params.yaml` out of their current relative paths breaks this tool's Settings-tab sync and "Overwrite All Params" feature silently (no import-time error, since these are plain `Path` string joins, not Python imports).

**`fsae_MPCTest/tuner/tools/sync_mpc_params.py`**, exact relative-path map (`_SOME_FILES` equivalent):
```python
_FILE_MAP = {
    "mpc_params.py": "control/fsae_control/fsae_control/mpc/mpc_params.py",
    "nmpc_params.py": "control/fsae_control/fsae_control/mpc/nmpc_params.py",
    "fsae_params.yaml": "common/fsae_bringup/config/fsae_params.yaml",
}
_LIVE_ROOT = _FSDS_ROOT / "ros2" / "src" / "fsae_planning"
_MIRROR_ROOT = _FSAE_MPCTEST / "fsds_simulator"
_AUTONOMOUS_CANDIDATES = [
    _FSDS_ROOT / "fsae_autonomous",
    _FSDS_ROOT / "ros2_autonomous" / "src" / "fsae_autonomous",
]
```
Same three files, same hazard: this script's whole purpose (one-way param sync live→mirror/autonomous) breaks silently if any of the three moves.

**Live-side launch/build machinery**: `control.launch.py`/`sim.launch.py` import `fsae_control.mpc.mpc_params`/`fsae_control.mpc.nmpc_params` as Python modules (not path strings), so these specifically need the *importable dotted path* preserved, not just the file's disk location — a bigger constraint than the GUI/sync-script's plain path joins. `ros2/launch_all.sh` invokes `ros2 run fsae_control mpc_controller` / `ros2 run fsae_control controller` / `ros2 run fsae_bringup ...` etc. by **entry_point name**, not file path, so those are only broken by an `entry_points` string edit, not a file move per se (see §3).

## 6. Non-implementation content in `fsae_planning` vs. mirror coverage

| Item | In `fsae_planning` | In mirror | Notes |
|---|---|---|---|
| `tracks/` (cone_map.json, speed_profile.csv, raceline.csv) | Yes, canonical | Excluded from mirror by design | Per CLAUDE.md "Track data lives in fsae_planning" — correctly NOT mirrored; `fsae_MPCTest/tracks/__init__.py`'s `TRACKS_DIR` points across the boundary instead of duplicating data. |
| `README.md` | Yes (project README) | Yes (mirror-specific README) | Intentionally different documents, see §1. |
| `CHANGES.md` (24KB) | Yes | No | Live-only historical changelog; not part of the ROS2 workspace the mirror stages, reasonably excluded. |
| `.gitignore` | Yes (`__pycache__/`, `.venv/`) | N/A (mirror is inside `fsae_MPCTest`'s own `.gitignore`, see below) | Not an implementation file, correctly not mirrored as its own artifact. |
| `launch_terminals.sh` | Yes | No | A convenience script, not referenced by `launch_all.sh` or any setup.py; low-risk omission but flag as MIRROR-MISSING if it's meant to be staged too (unclear from this survey alone whether it's intentionally excluded or simply not yet propagated — no commit history suggests either). |
| `fsds_ros2_custom.Dockerfile` | No | Yes | Mirror-only; presumably build tooling for the standalone-workspace story described in mirror's own README, not something live needs. |
| `package.xml` (each of 4 packages) | present (implicit, not separately checked) | present (implicit, in `same.txt`) | Confirmed identical via the byte-for-byte common-file diff above (all `package.xml` paths fell in the 65 SAME files). |

`fsae_MPCTest/.gitignore`'s `fsae_planning` pattern is **already anchored** (`/fsae_planning/`, root-relative), so the historical bug where `git status` hid the mirror's own `fsds_simulator/planning/fsae_planning/` package is fixed; confirmed via `grep -n fsae_planning fsae_MPCTest/.gitignore` showing only the anchored line plus an explanatory comment.

## 7. Scoring parity check

`fsds_simulator/control/fsae_control/fsae_control/scoring.py` (374 lines) vs `fsae_MPCTest/sim/scoring.py` (372 lines): diff is **entirely docstrings/comments and the documented inlined-constants difference** (`SCORE_WEIGHTS`, `METRIC_SCALES`, `COMPLETION_BONUS_WEIGHT`, `TIME_BONUS_WEIGHT`, `DNF_PENALTY`, `DNF_OFFTRACK_PENALTY`, `CONSTRAINT_FLOOR`, `COMPLETION_THRESHOLD`, `TIME_OBJECTIVE_WEIGHT`, `QUALITY_WEIGHT` inlined as module constants in the live copy vs. imported from `settings` in the offline copy). Manually compared every inlined numeric constant against `sim/scoring.py`'s corresponding `settings.py`-sourced values referenced in the diff context — **numerically identical** (0.505/0.09/0.040/... weights, 0.40/0.45/0.30/... scales, CONSTRAINT_FLOOR=10.0, COMPLETION_THRESHOLD=0.98, etc.). No formula divergence in `compute_composite_score`/`RolloutMetrics.add_step`/`finalize` beyond comment wording. **Parity holds**, exactly as documented.

---

## Summary (under 500 words)

**Diff class counts** (67 common implementation files + path-existence check): 65 SAME, 1 LIVE-NEWER, 1 non-parity doc divergence (README, expected), 0 MIRROR-NEWER, 0 BOTH-CHANGED among tracked implementation files, plus one config-only BOTH-CHANGED (`launch_all.sh`, see below). 3 live-only non-code files (`.gitignore`, `CHANGES.md`, `launch_terminals.sh`), 2 mirror-only files (`Dockerfile`, mirror's `launch_all.sh` copy).

**LIVE-NEWER (must pull into mirror before restructuring)**:
- `control/fsae_control/fsae_control/mpc/mpc_controller.py` — live has an extra 15-line `DISABLE_LIVE_CURVATURE_SPEED` temporary flag/branch (part of live's current uncommitted working tree, no matching commit yet) that the mirror lacks.

**BOTH-CHANGED / needs a human decision**:
- `ros2/launch_all.sh` (outer, root) vs mirror's `fsds_simulator/launch_all.sh` — 18-line diff, all live config toggles (`TRACK`, `USE_PRECOMPUTED_SPEED/PATH`, two commented NMPC overrides), not code structure. Someone needs to decide which toggle-state is the intended baseline to carry into the restructured mirror; don't auto-merge.

Everything else named in the task (`mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`, `mpc_core.py`, `nmpc_core.py`, `live_viz.py`, `control_utils.py`, `telemetry_logger.py`, `setup.py` ×4, and untracked `brake_sysid.py`) is **already byte-identical** between live and mirror — no pull needed for these. `.bak` files are older pre-edit snapshots of already-synced files, not a separate divergence, and are excluded from any overwrite anyway.

**Scoring parity confirmed intact**: `fsds_simulator/.../scoring.py` vs `sim/scoring.py` differ only in comments/docstrings and the documented inlined-constants scheme; every numeric constant checked matches.

**Top structural problems**: `nmpc_core.py` (2364 lines) and `mpc_core.py` (1499 lines) are the real split targets, both already have clean internal section boundaries (dynamics/plant/adaptive-gains/shaping helpers separable from each controller's still-large main class, which needs a follow-up method-level breakdown before finalizing a class split). `telemetry_logger.py` (1004 lines) splits cleanly into its 3 existing classes. `nmpc_params.py` (695 lines) should probably NOT be split despite its size, it's a single dataclass acting as the deliberate single source of truth; splitting fights that design. No duplicated helpers found between `control_utils.py` and the MPC modules. `special_utils/` is a vague catch-all name for skidpad+manual-input code specifically.

**ROS2 packaging hazards**: `find_packages()` auto-discovers a new sub-package with no `setup.py` change, provided public names are re-exported via `__init__.py`; entry_points strings only break if `mpc_controller.py`/`stanley_controller.py`/etc. themselves move; `control.launch.py`/`sim.launch.py` import `mpc_params`/`nmpc_params` by dotted module path (bigger constraint than plain path joins). **Biggest real hazard**: `fsae_MPCTest/gui/launcher.py` (38 refs) and `tuner/tools/sync_mpc_params.py` hardcode the exact relative paths to `mpc_params.py`/`nmpc_params.py`/`fsae_params.yaml` and will silently (no import error) stop working if those three files move.
