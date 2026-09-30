# fsae_MPCTest restructure plan

Full restructure of `fsae_MPCTest` (offline sim + `fsds_simulator/` mirror), a doc audit and rewrite, and new module references. This file and `docs/restructure_audit/` are temporary. Both are deleted in the final phase.

**Status:** planned, not started. Each phase stops for user approval before the next begins.

## How to use this plan

- Phases run in order. Each ends at a **gate**: run the listed checks, report, wait for approval.
- Raw audit and survey findings (per-doc issue tables, file inventories, split line ranges) are in `docs/restructure_audit/`. This plan cites them rather than repeating them.
- Every decision below came from the scoping interview. Don't re-litigate them. Anything not decided is under "Open questions".

## Decisions

### Scope

- In scope: everything in `fsae_MPCTest/`, including `fsds_simulator/`.
- `fsae_planning` gets the restructured mirror copied over its implementation files, as a local edit only. **Never commit or push there.**
- `fsae_autonomous` is not edited. `sync_mpc_params.py` only gets its source paths updated. Destination paths inside `fsae_autonomous` stay as they are.
- `CLAUDE.md`: fact fixes only (currently stale items plus items the restructure makes stale), plus one new rule (Phase 8).
- `docs/logs/`: contents untouched. Add `docs/logs/README.md` saying the logs predate the refactor, are unmaintained, and may contain dead paths. Include an old-to-new doc name table.

### Code

- Split big files and regroup packages. No backward-compat shims: every caller, doc, GUI subprocess string and shell script gets updated.
- `settings.py` becomes a `settings/` package. All consumers switch to `import settings` / `settings.X` access, so runtime `setattr` overrides reach everything.
- Danger-zone files (`model/vehicle_physics.py`, `controller/nmpc/solver.py`, the mirror's `nmpc_core.py`) are split with **pure code moves only** on the expensive tier. The gate is byte-identical validation output.
- The mirror's controller layout copies the offline layout 1:1, so a parity check becomes a file-to-file diff.
- Nothing live-side is fixed: package names, entry-point names and param-file paths may all change, and every consumer gets updated.
- `tuner/` becomes `validation/` + `investigations/` + `tools/` + `offline_tuner/`.
- Delete `deleted/` (git history keeps it). Retired files are deleted outright from now on.
- Delete `settings.py.bak`. Move `tuning history.txt` to `docs/logs/tuning_history.txt`. Gitignore `.vscode/`.
- `.claude/worktrees/` is empty and not in `git worktree list`, so remove it.
- **No comment cleanup during the restructure.** It's all in the follow-up pass. Exception: a comment that states a now-wrong path or name gets corrected, because that's a fact error.

### Docs

- Fact-check paths, symbols, commands, defaults (including runtime overrides in `launch_all.sh` and YAML), links, and status claims. Re-run quoted numbers only when a log of the exact configuration exists. Never guess a configuration.
- Keep CLAUDE.md's writing rules: plain English first, then mechanism; no em dashes; no intensifiers; no "you" (except the onboarding guide). Concision comes from cutting redundancy, not detail.
- One H1 per file (the title), `##` for major sections, `###` for subsections.
- `.md` source: one line per paragraph.
- NZ/British spelling in prose (centreline, behaviour, optimise). Code identifiers stay as they are.
- Mechanism docs use a fixed pattern. Empty sections are omitted:
  - **What it does** (plain English)
  - **Why it exists**
  - **Why this design** (and what was rejected)
  - **How it works** (maths, code pointers)
  - **Tuning and pitfalls**
- Missing rationale: search `docs/logs/`, git log and memory first. If still not found, write "Rationale not recorded" and add it to "Open questions".
- Docs are merged, renamed and retired freely. The mapping table is below.

### Verification

- Capture a baseline of every check before any change (Phase 1).
- Required at each code gate:
  - byte-identical output from the validation scripts
  - every entry point imports and runs
  - GUI smoke test
- Live FSDS run comes last, and only after asking the user.

### Git

- Commit the other session's pending `fsae_MPCTest` edits to `main` first.
- Then branch `refactor/restructure`, one commit per phase. Merge to `main` after Phase 9 and user approval.
- Commits use the user as sole author, no `Co-Authored-By`.
- The outer FSDS repo (`CLAUDE.md`, `ros2/launch_all.sh`, `ros2/run_*.sh`) gets local edits only, never committed.

## Phase 0: preflight

1. In each of the four repos, check `git status` and `git worktree list` for work from other sessions. Stop if anything unexpected turns up.
2. Commit `fsae_MPCTest`'s pending edits to `main` as they are:
   - `README.md` and 4 docs
   - `tuner/offline_tuner.py`, `tuner/steering_chatter_check.py`
   - the deleted `best_with_e_v.json` and the new profiles

   Exclude `.vscode/`, `docs/restructure_plan.md` and `docs/restructure_audit/`.
3. Create branch `refactor/restructure`. Commit this plan and `docs/restructure_audit/` as the first commit.
4. Record `git -C ros2/src/fsae_planning status` and `diff --stat` into the baseline folder, so the user can review the later overwrite against it.

## Phase 1: baseline capture (before any code change)

Store everything in `fsae_logs/restructure_baseline/`. It is outside all repos and already gitignored. Delete it after the merge.

| Check | Command (current paths) | Compared how |
|---|---|---|
| Recorded-map rollout | `python -m tuner.recorded_map_rollout` | byte diff; strip wall-clock timing lines first if any |
| NMPC self-consistency | `python -m tuner.nmpc_offline_check` | byte diff |
| Plant open-loop | `python -m tuner.checks.plant_openloop_validation` (plus `--ab`) | byte diff |
| Tuner eval | harness: serial scoring of the default weights over `VALIDATION_SUITE` via the tuner's own eval function (no CMA-ES, no multiprocessing) | byte diff of scores |
| Mirror golden outputs | harness: construct the live `MPCController` and `NMPCController` from default params, feed a fixed synthetic state/path sequence, dump every output | byte diff |
| Mirror NMPC check | `fsds_simulator/control/fsae_control/test/nmpc_offline_check.py` | byte diff |
| GUI Settings read | harness: call the launcher's read helpers for every field it knows, dump name to value | exact match |
| Entry points | `python -m X --help` (or import) for every CLI module; list saved | all still resolve at new paths |
| Launch args | `ros2 launch fsae_bringup sim.launch.py --show-args` and the same for `control.launch.py` | same arg set and defaults |

Harness scripts live in the baseline folder, not in any repo.

## Phase 2: reconcile live into mirror

The mirror is the source of truth, but the live tree has newer local state. From the survey: 65 of 67 files are identical.

- `mpc_controller.py`: the live-only `DISABLE_LIVE_CURVATURE_SPEED` temporary flag is **dropped**. The mirror version stands, and the overwrite removes the flag from live.
- `launch_all.sh`: copy `ros2/launch_all.sh` over `fsds_simulator/launch_all.sh` (it matches `settings_profiles/no_progress_tuned.json`). Then set `NMPC_SLACK_LINEAR_WEIGHT=500.0` uncommented in both copies, to match the profile.
- Re-diff all implementation files right before Phase 4. Live may have moved again, since other sessions work in it.
- Overwrite excludes:
  - `tracks/`, `recorded_runs/`, `cone_maps/`
  - build/install/log, `__pycache__`, `*.bak`
  - each side's own `README.md`
  - live-only `CHANGES.md`, `.gitignore`, `launch_terminals.sh`

**Gate:** diff report showing live and mirror identical apart from the exclusions.

## Phase 3: offline code restructure

Order: move files first (one commit), then split files (one commit per package), then rewire `settings` access. Run the Phase 1 checks after every commit.

### Target layout

```
fsae_MPCTest/
  settings/            __init__.py re-exports; system.py, noise.py, planner.py,
                       lmpc.py, nmpc.py, scoring.py, solver.py
  model/               vehicle_params.py, tyre.py, plant.py, tracking.py,
                       bicycle_model.py          (from vehicle_physics.py)
  controller/
    adaptive_gains.py  (was model_utils.py: shared by LMPC + NMPC)
    lmpc/              build.py, solve.py
    nmpc/              layout, dynamics, reference, outputs, weight_schedule,
                       solver.py (public API), qp_build.py, sqp_step.py
  planning/            boundary, path_utils, cone_sorting, cone_map,
                       geometry.py (segment_crosses_walls, breaks the cycle)
  sim/
    rollout/           core.py, phases split into reference.py,
                       speed_target.py, tick_solve.py, delay.py
    perception.py, planner.py   (from sim_track.py)
    speed_profile/     curvature.py, profile.py, dynamic_cap.py
    sensor_noise.py, scoring.py, track_io.py
    angles.py          (single angle wrap; replaces _wrap + _normalize_angle)
  tracks/              __init__.py path resolver; gains DEFAULT_MAP
  gui/
    launcher/          app.py, theme.py, paths.py, file_edit.py,
                       tabs/{launch,log_debug,offline_sim,settings,profiles}.py
    simulation/        app.py (widgets/callbacks), closed_loop.py
    manual_drive.py
  tuner/
    offline_tuner/     synthetic_paths.py, rollout_eval.py, cma_driver.py,
                       history_log.py, performance_stats.py, __main__.py
    validation/        recorded_map_rollout, nmpc_offline_check,
                       plant_openloop_validation
    investigations/    everything now in tuner/checks/ + the stray
                       top-level *_check.py scripts
    tools/             doc_lint, export_speed_profile, sync_mpc_params,
                       plot_playback/ (split), raceline_optimizer/ (split)
    csv_log.py
```

Exact split line ranges are in `docs/restructure_audit/survey_offline_code.md` §5. Final sub-module names get confirmed at the start of the phase after a method-level read.

### Hazards and how each is handled

- **`settings` split.**
  - Every consumer switches to `settings.X` access. There is no `from settings.<sub> import`, and `doc_lint` enforces this (Phase 7).
  - `settings.py` imports `sim.sim_track.TRACK_HALF_WIDTH` at module scope. Move that constant into `settings/` or `sim/` so no import cycle forms.
  - `gui/simulation.py` bakes `USE_PLANNER` in as a default argument. Change it to read at call time.
- **GUI text-editing of `settings`.** `_read_var`/`_rewrite_var` assume one file. Replace the single `paths.settings_py` with a name-to-submodule lookup built by scanning `settings/*.py`. The GUI Settings-read harness must match the baseline exactly.
- **Subprocess module strings.** `gui/launcher.py` calls modules through `-m` strings (lines ~1129, 1258, 1299, 1967). Grep for every `"-m"` and every `tuner.`/`gui.` string, and update them.
- **Multiprocessing state.** `offline_tuner`'s worker-init globals must stay in the same module as the functions that read them.
- **Danger-zone splits.** For `vehicle_physics.py` and `nmpc/solver.py`, move whole definitions only. `NMPCController` methods move via mixins (`_QPBuildMixin`, `_SQPStepMixin`) so no method body changes. Any byte difference in validation output means revert and investigate. Never "fix" it forward.
- **Import inversion.** Investigations currently import `DEFAULT_MAP` from `recorded_map_rollout`. It moves to `tracks/`.

**Gate:**
- all Phase 1 offline checks byte-identical
- every entry point resolves at its new path
- GUI smoke test: launcher opens, every tab renders, the Settings read dump matches, and a write round-trip on a temp copy works; `gui.simulation` runs one sim
- `nmpc_offline_check` passes

## Phase 4: mirror restructure, then overwrite `fsae_planning`

**Scope narrowed by the user (2026-09-30): only MPC-related files get
refactored/split. Every other file in `fsds_simulator/` (perception,
`fsae_planning`'s `cone_sorting.py`/`boundary.py`/`path_utils.py`/
`centerline_planner.py`, `special_utils/`, `fsae_bringup`'s non-MPC launch
and config plumbing) may be MOVED for sorting if a move is actually needed,
but is never split or internally refactored.** The rest of this section
applies only to the MPC-related files listed below.

### MPC-related files (in scope for split/refactor)

- `control/fsae_control/fsae_control/mpc/{mpc_params,nmpc_params,mpc_core,nmpc_core,mpc_controller}.py`
- `control/fsae_control/fsae_control/{telemetry_logger,scoring}.py` (telemetry/scoring are MPC-run output, in scope)
- `control/fsae_control/fsae_control/live_viz.py` (MPC debug visualisation, in scope)
- `common/fsae_bringup/launch/control.launch.py`/`sim.launch.py` (generate MPC/NMPC launch args from `MPCParams`/`NMPCParams` field metadata)
- `common/fsae_bringup/config/fsae_params.yaml`'s `controller:` block

### Target layout (MPC files only, under `control/fsae_control/fsae_control/`)

```
params/        mpc_params.py, nmpc_params.py, ros_params.py
               (declare_mpc_params, mpc_params_from_node)
lmpc/          adaptive_gains.py, predict.py, controller.py   (from mpc_core.py)
nmpc/          layout, dynamics, reference, outputs, weight_schedule,
               solver.py, qp_build.py, sqp_step.py             (from nmpc_core.py, 1:1 with offline)
telemetry/     config_lines.py, horizon_tracker.py, lap_progress.py,
               control_logger.py, scoring.py
nodes/         mpc_node.py (+ helpers split out of the 1220-line node class),
               live_viz/ (node.py, panels.py)
```

Explicitly NOT split or refactored (moved only if a move is needed for
sorting, e.g. an MPC file relocating out from under a shared directory):
`control_utils.py` (Stanley controller + speed-gate/profile-loader
helpers -- Stanley is not MPC, and the speed-gate/profile helpers are
shared with Stanley, so splitting risks tangling a non-MPC consumer),
`stanley_controller.py`, `fsds_bridge.py`, `brake_sysid.py`,
`planning/fsae_planning/fsae_planning/*` (boundary, cone_sorting,
cone_map, path_utils, centerline_planner, special_utils/),
`perception/fsae_sim_perception/*`.

Other mirror changes (MPC-file paths only):

- Entry-point names and package names for the MPC files above may change. Update every consumer:
  - `setup.py` entry points
  - launch files
  - both `launch_all.sh` copies and `ros2/run_*.sh`
  - `gui/launcher.py` `RepoPaths`
  - `sync_mpc_params.py` source paths (`fsae_autonomous` destination paths unchanged)
  - the mirror README
- `scoring.py` stays a verbatim copy of `sim/scoring.py` with its inlined constants.

### Overwrite procedure

1. Re-run the Phase 2 diff. Stop if live changed since.
2. Copy the mirror over `ros2/src/fsae_planning/`, excluding the Phase 2 list. Delete implementation files in live that no longer exist in the mirror.
3. Clean rebuild: remove `build/ install/ log/` at the FSDS root, then `colcon build --symlink-install`.
4. Leave `fsae_planning` uncommitted. Report its `git status` to the user.

**Gate:**
- mirror golden-output harness and mirror NMPC check byte-identical
- colcon build clean
- `--show-args` arg sets match the baseline (names may be renamed per the mapping; defaults identical)
- every `ros2 run` target resolves
- `sync_mpc_params` dry run reports "no differences"

### Phase 4 outcome (executed 2026-09-30)

Done, all gated. Deviations from the target layout above, and why:

- `mpc_params.py`/`nmpc_params.py` were **not** moved or split (no `params/`, no
  `ros_params.py`): `sync_mpc_params` writes them by relative path into
  `fsae_autonomous`, whose layout is not being changed, and the launch files and
  GUI `RepoPaths` import them by that path. They stay in `fsae_control/mpc/`.
- `mpc_core.py` -> `lmpc/` (constants, predict, adaptive_gains, controller).
- `nmpc_core.py` -> `nmpc/` (1:1 with offline `controller/nmpc/`).
- `telemetry_logger.py` -> `telemetry/` (columns, config_lines, horizon_tracker,
  lap_progress, control_logger); `scoring.py` moved in unchanged.
- `mpc_controller.py` stays the node module (entry point unchanged) with
  `_ControlStepMixin`, `_DebugPublishMixin` and `node_constants.py` beside it in `mpc/`.
- `live_viz.py` -> `live_viz/` (panels, node, app); `main()` itself was not split,
  it is one closure-heavy matplotlib builder and a split would be a logic change.
- Verification: AST-identical entities, pyflakes (no undefined names), mirror MPC and
  NMPC goldens, mirror `nmpc_offline_check`, a telemetry CSV golden, real `colcon
  build` of the mirror in a temporary workspace, `--show-args` and `ros2 pkg
  executables` identical to a build of the pre-split tree, real node startup.
- Overwrite: `ros2/src/fsae_planning` working tree now equals the mirror (backup of
  the previous state in `fsae_logs/restructure_baseline/`), uncommitted. The real
  `ros2/install` was not rebuilt; `launch_all.sh` rebuilds on launch.

## Phase 5: docs restructure and rewrite

### Target tree and mapping

| Old | New | Action |
|---|---|---|
| `README.md` (repo) | `README.md` | Rewrite as map + quickstart: what the repo is, how to run the three validation checks and the GUI, and a link to `docs/README.md` |
| (none) | `docs/README.md` | New index: "to do X, read Y", one line per doc |
| `docs/junior_project_mpc_docs.md` | `docs/guides/getting_started.md` | Rewrite; may address the reader as "you"; relative links, not github URLs |
| `docs/offline_guide.md` | `docs/guides/offline_guide.md` | Rewrite |
| `docs/tuning.md` | `docs/guides/tuning.md` | Rewrite; absorb the outcome of `steering_turn_in_upgrade_options.md` |
| `docs/debugging_tools.md` | `docs/guides/debugging_tools.md` | Rewrite; steering sys-ID harness section defers to parity doc |
| `docs/lmpc.md`, `nmpc.md`, `stanley.md` | `docs/controllers/*.md` | Rewrite |
| `docs/architecture.md` | `docs/reference/architecture.md` | Rewrite; its module table moves to `docs/modules/` |
| `docs/reference/control_mechanisms.md` | same | Rewrite with mechanism pattern |
| `docs/reference/offline_live_parity.md` | same | Rewrite; parity table re-derived from code |
| `docs/reference/reference_path_and_speed.md` | same | Rewrite |
| `docs/reference/simulator_fidelity.md` | same | Rewrite |
| `docs/vehicle_physics_guide.md` | `docs/reference/vehicle_physics.md` | Rewrite |
| `docs/error_state_reference.md` | `docs/reference/error_states.md` | Rewrite |
| `docs/reference/simulator_glossary.md` | `docs/reference/glossary.md` | Rewrite |
| `docs/removed_mechanisms.md` + `docs/reference/superseded_mechanisms.md` | `docs/reference/retired_mechanisms.md` | Merge |
| `docs/reference/README.md` | (folded into `docs/README.md`) | Delete |
| `docs/fsds/fsds_integration_guide.md`, `fsds_ros_integration.md`, `fsds_settings.md` | `docs/fsds/integration_guide.md`, `ros_integration.md`, `settings.md` | Rewrite |
| `fsds_simulator/README.md` | same | Rewrite for new layout |
| `docs/planning_control_sync.md` | (none) | Delete; redirect inbound links |
| `docs/fsae_planning_pending_pr.md` | (none) | Delete; the mirror workflow is described in `docs/modules/fsds_ros2.md` |
| `docs/steering_turn_in_upgrade_options.md` | (none) | Delete after folding into `tuning.md` and `nmpc.md` |
| (none) | `docs/modules/offline_sim.md`, `docs/modules/fsds_ros2.md` | New (Phase 6) |
| (none) | `docs/logs/README.md` | New: pre-refactor notice + this mapping table |

### Per-doc work

1. Apply every STALE or WRONG item from `docs/restructure_audit/`.
2. Re-check every code path and name against the **post-restructure** tree. The audit lists every path each doc mentions.
3. Rewrite for concision and structure under the style decisions above.
4. Add missing "why" and "why this design" from logs and git. If still missing, write "Rationale not recorded".
5. Update inbound links from code comments, other docs, both READMEs and CLAUDE.md.

Highest-impact fixes from the audit:

- Dead file names cited across many docs: `controller/optimiser.py`, `nmpc_optimiser.py`, `mpc_controller_standalone.py`.
- Links into `docs/reference/README.md#…` sections that no longer exist.
- `[text](`path`)` links with backticks inside the URL.
- Wrong defaults:
  - `r_rate_delta` is 100.0, not 52.5
  - `nmpc_rk_substeps`/`nmpc_jac_substeps` are 4/4
  - `nmpc_track_halfwidth` is 3.35
  - `speed_target_deficit_max` is 2.55
  - `MAX_EVALS` is 1500
  - the anti-hunt constants were halved
  - `accel_scale` is fixed at 1.0
- Status claims wrong:
  - rrate zone, LTV anti-hunt and adaptive Q are **on**
  - `nmpc_horizon_speed_profile_enabled` and `curvature_forcing_enabled` don't exist
  - the cone-map dedup fix is already in live
  - `alat_ceiling` is speed-dependent (slope/intercept)
- `getting_started.md` names `fsds_simulator/` as the live tree. Live is `ros2/src/fsae_planning/`.
- `vehicle_physics.md` claims plant states 0-7 equal the MPC states. They don't.
- The FSDS WSL-IP step says to hardcode the IP, but the launch file now reads `FSDS_HOST_IP`.
- Field counts: `MPCParams` 69 + `NMPCParams` 35 = 104. Recount after Phase 4.

**Gate:** extended `doc_lint --strict` passes on every doc outside `docs/logs/` (Phase 7 runs first if needed), and the user reviews a sample of rewritten docs.

## Phase 6: module references

`docs/modules/offline_sim.md` covers `fsae_MPCTest` excluding the mirror. `docs/modules/fsds_ros2.md` covers `fsds_simulator/` (equal to `fsae_planning`). They are separate documents with no shared sections.

- **Coverage:** every `.py` file, launch file, config (`fsae_params.yaml`, `setup.py`, `package.xml`), shell script (`launch_all.sh`, `run_*.sh`, Dockerfile) and data format (`settings_profiles/*.json`, the `tracks/<name>/` files, `recorded_runs` CSV columns).
- **Structure:** one `##` per package, then one entry per file:

  ```
  ### `path/to/file.py`
  Does: one line.
  Change it to:
  - e.g. "retune corner Q weights"
  Don't:
  - <thing>, because <why>
  Key API: fn_a, ClassB
  ```

- Each doc opens with a task-first index, e.g. "change the plant model: `model/plant.py`, then run `validation.plant_openloop_validation`".
- `fsds_ros2.md` also documents the mirror workflow (edit in the mirror, overwrite `fsae_planning`, param sync), which replaces `fsae_planning_pending_pr.md`.

## Phase 7: extend `tuner/tools/doc_lint.py`

Add checks, all on by default and fatal under `--strict`:

- broken links and heading anchors
- backticked paths and `python -m` targets that don't exist
- style: em dashes, more than one H1, intensifiers ("genuinely", "actually", "really", "crucially")
- module-reference coverage: every `.py`, `.sh`, launch and config file has an entry in `docs/modules/*.md`
- a code check: `from settings.<sub> import` anywhere is flagged

`docs/logs/` stays exempt except for its README.

## Phase 8: CLAUDE.md, memory, cleanup

- **CLAUDE.md fact fixes.** All items in `docs/restructure_audit/audit_claude_md.md`, plus every path and command changed by Phases 3-6:
  - validation commands and new module paths
  - `planning_control_sync.md` citations moved to their real targets
  - `nmpc_optimiser.py` and `mpc_controller_standalone.py` references
  - the `mpc/` subpath
  - field count 104 (recount)
  - `fsae_autonomous` now has an NMPC and tests
  - the "Testing" section (test files now exist in `fsae_autonomous` and the mirror)
  - the "prefer `deleted/`" rule becomes "delete; git history keeps it"
  - new doc locations and `docs/modules/`
  - `docs/logs/` pre-refactor status
  - the `fsae_logs/restructure_baseline/` data-retention note is removed once the folder is deleted
- **New CLAUDE.md rule** (user request): after any change, before calling it done, sweep every related code comment, doc and `docs/modules/` entry, and update what the change made stale. This is part of the code-review checklist too.
- **Memory.** Fix or delete stale notes (e.g. "CLAUDE.md ~36 sections stale", dead file and flag names) and update `MEMORY.md`.
- **Cleanup.** Delete `docs/restructure_plan.md`, `docs/restructure_audit/` and `fsae_logs/restructure_baseline/`.

## Phase 9: live FSDS validation (ask the user first)

A full FSDS session using the overwritten `fsae_planning`, profile `no_progress_tuned`. Compare the logged score and metrics to a pre-restructure run of the same profile, if one exists. Mind the known periodic teleport bug: re-run at most three times before concluding anything.

Then merge `refactor/restructure` into `main` in `fsae_MPCTest` after user approval. Push `fsae_MPCTest` only.

## Follow-up plan (after this one): comment pass and diagrams

Planned in detail once the restructure is merged. Scope:

- Covers **every** code file in `fsae_MPCTest` including the mirror, then re-mirrored to `fsae_planning`.
- Add a short 1-2 line comment to each calculation or derivation: what is computed and why it's done this way.
- Trim existing comments. Remove changelog text, dates and score history (move them to `docs/logs/` if worth keeping). Cut long essays down to the non-obvious why.
- Last: link beginner-friendly diagrams (trusted external sources first, generated only as a fallback) into docs, spending little effort on it.

## Open questions and risks

Resolved:

- `anti_hunt_boost_max`: add `settings.ANTI_HUNT_BOOST_MAX` with the current hardcoded value in Phase 3. Behaviour doesn't change, so the byte-identical gate applies.
- `ros2/run_steering_sysid.sh` and `run_steering_step.sh` call nodes that don't exist. Delete both scripts in Phase 4. They're local edits in the outer repo, not committed. Docs say the harness was discarded.
- `fsds_simulator/recorded_runs/graph/` CSVs are stray output. Leave the files. Docs describe the folder as scratch output.

Open:

- Rationale gaps found during Phase 5 get listed here as they turn up.
- **Risk:** other sessions editing live or the mirror mid-restructure. Mitigated by the re-diff before the overwrite and the `git status` check at every gate.
- **Risk:** byte-identical checks can't catch GUI-only or node-only regressions. Covered by the GUI smoke test and the Phase 9 live run.

## Found during execution: pre-existing bug, out of scope to fix here

- `tuner/investigations/ref_heading_limiter_ab.py` and
  `ref_heading_limiter_suite_check.py` monkeypatch
  `rc.REF_HEADING_RATE_LIMIT_ENABLED`/`rc.REF_HEADING_RISE_RATE` on the
  `sim.rollout.core` module object (was `sim.rollout_core`). Neither name
  was ever imported into that module, before or after this restructure;
  the real binding `compute_reference()` reads lives in
  `sim/rollout/reference.py`'s own namespace (was `sim/rollout_phases.py`).
  The monkeypatch has silently done nothing since before this restructure
  started (confirmed against the pre-restructure file), so both scripts'
  "OFF (baseline)" and swept-rate rows have always used whatever
  `settings.py`'s own default is, not the value the row claims. Not fixed
  here: this restructure moves files, it does not fix investigation-script
  logic bugs found along the way. Flagging for a follow-up task.
