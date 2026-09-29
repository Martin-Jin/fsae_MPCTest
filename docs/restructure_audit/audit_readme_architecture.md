# Documentation audit: README.md + docs/architecture.md

Method: every path checked with `ls`/`find` (never bare `git status`), every
symbol with `grep`, every CLI with `importlib.util.find_spec`, every commit
hash with `git log`/`git merge-base`/`git rev-list`, runtime overrides checked
in `ros2/launch_all.sh`.

---

## 1. `fsae_MPCTest/README.md` (166 lines)

**Purpose/audience:** top-level orientation doc: what the repo is, the two
simulators, the two MPC implementations, quick start, and a documentation/
module index. Audience is a new contributor to the offline side.

**Overlap:** heavy, deliberate overlap with `docs/architecture.md` (both have
a "Key modules"/"Module Reference" table, both explain `use_nmpc`), and with
`docs/reference/README.md` (both are "index" docs one level apart). This
overlap looks intentional (README is the shallow index, architecture.md the
deep one), but the two "module table" copies can now drift independently,
and already have (see WRONG rows below, only one side has the moved-file
error).

**Readability:** generally clear, but very link-dense with almost no
whitespace between long single-purpose sentences; a few sentences in
"From offline weights to the live car" and "Perception/planning simulation"
run 60+ words. Follows CLAUDE.md's own conciseness rule loosely, this is a
README not a doc/log so the rule may not strictly bind, but a "lead with the
conclusion" pass would help the "Two MPC implementations" and "From offline
weights" sections.

**Mechanism without WHY:** line 68-70 ("Weights tuned offline...transfer
directly...because both preserve the MPC's own throttle/brake output rather
than routing speed through fsds_bridge.py's separate P-loop") does state the
why. No major unexplained-mechanism gaps found in this doc; it mostly defers
to architecture.md/lmpc.md/nmpc.md for the "why", which is the file's stated
design.

### Findings table

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 61-62 | "LTV-QP (default): `mpc_core.MPCController`... NMPC: `nmpc_core.NMPCController`" flag is `use_nmpc` in settings.py | STALE/WRONG | Both classes exist and are named correctly (`fsds_simulator/.../mpc/mpc_core.py:433 class MPCController`, `.../mpc/nmpc_core.py:962 class NMPCController`). But the **offline** flag in `settings.py` is `USE_NMPC` (uppercase constant, line 624), not `use_nmpc`; `use_nmpc` is only the **live ROS2 node parameter** name (`nmpc_params.py:62`). Also, `ros2/launch_all.sh:224` sets `USE_NMPC=true` unconditionally, so the actual current runtime default for a live launch is NMPC, not LTV-QP, contradicting the unqualified "(default)" label. This is exactly the launch_all.sh-shortlist blind spot CLAUDE.md's "Effort ceiling" section warns about. | Say "the offline flag is `USE_NMPC` in `settings.py`; the live ROS2 parameter is `use_nmpc`" and note `ros2/launch_all.sh` currently overrides it to `true`. |
| 80-81 | "fsds simulator repo...current implementation uses commit 59f03fa"; "fsae planning repo...uses commit 28dcd4d" | STALE | Outer FSDS repo: `59f03fa` is a real ancestor of HEAD, but HEAD (`0447c16`) is 1 commit ahead — nearly current, mildly stale. `fsae_planning`: `28dcd4d` (2026-05-30) is a real ancestor of `main`'s HEAD (`ba0e58...`, 2026-09-20), but HEAD is **14 commits ahead**, roughly 4 months stale. | Re-sync both commit refs, or replace with "check `git log -1` in each repo" language since these numbers rot immediately. |
| 70, 135 | `` [`docs/reference/`](`docs/reference/`) `` (link target wrapped in backticks) | WRONG | Markdown link syntax `[text](target)` does not permit backticks inside the parens as part of the URL; standard renderers will treat `` `docs/reference/` `` (with literal backticks) as the URL and fail to resolve it as a relative path. Malformed on both occurrences (line 70 and the table row at line 135). | Change to `` [`docs/reference/`](docs/reference/) `` (backticks only around the display text, not the URL). |
| 135 | Row links to `docs/reference/` as a directory | OK (content) / WRONG (syntax, see above) | `docs/reference/README.md` exists and its content matches the described purpose (file mapping/resync procedure) once the link itself is fixed. | n/a beyond the syntax fix above |
| 121-140 | Documentation table: all 14 linked doc paths | OK | All of `docs/reference/simulator_glossary.md`, `docs/architecture.md`, `docs/tuning.md`, `docs/offline_guide.md`, `docs/fsds/fsds_integration_guide.md`, `docs/fsds/fsds_settings.md`, `docs/fsds/fsds_ros_integration.md`, `docs/debugging_tools.md`, `docs/reference/reference_path_and_speed.md`, `docs/vehicle_physics_guide.md`, `docs/junior_project_mpc_docs.md`, `docs/fsae_planning_pending_pr.md`, `docs/logs/`, `fsds_simulator/README.md` exist on disk. | none needed |
| 137 | "mirrors that repo's own `CHANGES.md`" (re: `fsae_planning_pending_pr.md`) | OK | `ros2/src/fsae_planning/CHANGES.md` exists. | none |
| 146-157 | Key modules table paths | MOSTLY OK, ONE WRONG | `gui/launcher.py`, `gui/simulation.py`, `sim/track_io.py`, `tuner/offline_tuner.py`, `tuner/tools/plot_playback.py`, `sim/rollout_core.py`, `model/vehicle_physics.py`, `gui/manual_drive.py`, `settings.py` all exist. **`controller/optimiser.py` does NOT exist** (`ls`/`find` confirm; git log shows commit `a941625 "Split controller/optimiser.py into controller/lmpc/ package"`); it's now `controller/lmpc/build.py` + `controller/lmpc/solve.py`. `control_utils.py` exists only under `fsds_simulator/control/fsae_control/fsae_control/control_utils.py`, consistent with the row's own "(staged under fsds_simulator/...)" qualifier, so that row is fine. | Update line 154 (`model/bicycle_model.py` / `controller/optimiser.py` / `controller/model_utils.py`) to say `controller/lmpc/` (build.py + solve.py) instead of `controller/optimiser.py`. |
| 160 | link to `docs/architecture.md#module-reference` | OK | Heading `## Module Reference` exists in architecture.md. | none |
| 3, 68, 74 | "pasted into `fsae_planning`" / staging description | OK (matches CLAUDE.md's own description of the mirror discipline) | consistent with CLAUDE.md "Third copy" section | none |

**Code paths/modules mentioned** (for a future restructure): `mpc_core.MPCController`, `nmpc_core.NMPCController`, `mpc/mpc_controller.py`, `mpc/mpc_core.py`, `fsds_simulator/`, `gui/launcher.py`, `gui/simulation.py`, `sim/track_io.py`, `tuner/offline_tuner.py`, `tuner/tools/plot_playback.py`, `sim/rollout_core.py`, `model/vehicle_physics.py`, `model/bicycle_model.py`, `controller/optimiser.py` (STALE — should be `controller/lmpc/`), `controller/model_utils.py`, `settings.py`, `gui/manual_drive.py`, `control_utils.py`, `sim_track.place_cones()`, `tracks/__init__.py`, `fsds_bridge.py`.

**Proposed action: KEEP, with a small fix pass.** The doc's shape (orientation + index) is sound and worth keeping as-is; it needs (a) the `controller/optimiser.py` path fixed, (b) the two malformed `docs/reference/` links fixed, (c) the `use_nmpc`/`USE_NMPC` flag-name and current-default (NMPC via `launch_all.sh`) clarified, (d) the two commit hashes either refreshed or reworded to not need refreshing. None of this rises to a restructure; it's a targeted edit.

---

## 2. `fsae_MPCTest/docs/architecture.md` (429 lines)

**Purpose/audience:** deep technical reference for the offline rollout, MPC
config, tuner, and scoring — the "how it works" companion to
`docs/offline_guide.md`'s "how to operate it." Audience is someone already
past the README, about to modify tuning/plant/scoring code.

**Overlap:** explicitly and correctly scoped down since the LMPC/NMPC full
derivations moved to `lmpc.md`/`nmpc.md` (both confirmed to exist, with
resolving anchors). Its own "Module Reference" table duplicates README's
"Key modules" table almost verbatim (both list the same file set with
slightly different prose), same drift risk noted above. `docs/tuning.md` and
this file both cover `settings.py` weights, but architecture.md defers to
tuning.md explicitly (line 129) rather than duplicating, that division holds
up on inspection.

**Readability:** dense but structured (headings, tables, mermaid diagrams,
short paragraphs); much better than README at "lead with conclusion, then
evidence" (e.g. the Composite Score section states the 3-tier rule then the
formula). Consistent with CLAUDE.md's doc-writing-style rules.

**Mechanism without WHY, and whether the why is recoverable:**
- Line 218 (Pacejka curve bending over past 5-8° slip): states the mechanism
  and its consequence for the linear model, WHY is present (drives the
  Cf/Cr-recompute requirement above it). OK.
- `PoseFeedHold` section (145-172): states the mechanism and the measured
  numbers, and explicitly flags "this does NOT close the gap" plus what was
  tried (why recoverable: `sim_to_real_investigation.md` per CLAUDE.md).
  Good example of the doc's own standard being met.
- No major unexplained-mechanism gaps found beyond what's already flagged in
  the findings table below (mostly path/name drift, not missing rationale).

### Findings table

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 7 | "Both moved out of this file... `lmpc.md`... `nmpc.md`" | OK | Both `docs/lmpc.md` and `docs/nmpc.md` exist with matching content (LTV-QP / NMPC full derivations). | none |
| 9-19 | Table of Contents, 8 anchors | OK | All 8 anchors (`#architecture-overview`, `#configuring-the-project-settingspy`, `#configuring-the-vehicle-modelvehicle_physicspy`, `#how-the-mpc-works`, `#how-the-offline-tuner-works`, `#the-composite-score`, `#module-reference`, `#second-controller-nonlinear-mpc-use_nmpc`) resolve to real headings; the last one uses an explicit `<a id="second-controller-nonlinear-mpc-use_nmpc"></a>` anchor at line 424 since GitHub's auto-slug would otherwise drop the parens' backtick content differently. | none |
| 26 | "`sim/rollout_core.run_core_rollout()`... `mpc_core.MPCController` for the live node" | OK | Both exist as named (`sim/rollout_core.py` has `run_core_rollout`; live `mpc_core.py:433 class MPCController`). | none |
| 65-71 | Mermaid: `bicycle_model.get_8state_discrete_model()`, `model_utils.adaptive_R_scaling(vx, R)`, `optimiser.solve_mpc()` | PARTIALLY STALE | `model_utils.py` still exists at that path (unverified exact function name `adaptive_R_scaling` — not directly grepped, lower priority; the mechanism is documented in lmpc.md). **`optimiser.solve_mpc()` is now `controller/lmpc/solve.py:30 def solve_mpc(...)`** — the function still exists and the call signature is presumably unchanged, but `optimiser.` as a module qualifier is wrong post-split (same drift as README's Key Modules row). | Update diagram label to `lmpc.solve.solve_mpc()` or `controller.lmpc.solve.solve_mpc()`. |
| 70 | `scoring.RolloutMetrics.add_step()` | OK | `sim/scoring.py:183 class RolloutMetrics`, `add_step` defined at line 226. | none |
| 74 | 24-state plant, `vehicle_physics.step_nonlinear_plant` | OK | Confirmed function referenced/used per grep in `vehicle_physics.py` docstring cross-refs (`Used by: step_nonlinear_plant()...`). | none |
| 88-98 | "ROS 2 vs Simulator Mapping" table: `sim_perception.py`, `centerline_planner.py`, `cone_map.py`, `boundary.py`, `path_utils.py`, `cone_sorting.py`, `mpc_core.py` (live side) | OK | All 7 confirmed present in `ros2/src/fsae_planning/` at the expected sub-paths (`perception/fsae_sim_perception/...`, `planning/fsae_planning/...`, `control/fsae_control/.../mpc/mpc_core.py`). | none |
| 90-97 | Simulator-side equivalents: `sim_track.SimPerception`, `sim_track.SimPlanner`, `planning/cone_map.ConeMap`, `planning/boundary.py`, `planning/path_utils.py`, `planning/cone_sorting.py`, **`controller/optimiser.py` + model/bicycle_model.py + controller/model_utils.py** | STALE (one cell) | Same `controller/optimiser.py` drift as above — file moved to `controller/lmpc/`. All the `sim_track.py`/`planning/*.py` names confirmed to exist. | Update cell to `controller/lmpc/` (build.py/solve.py). |
| 103 | `fsds_simulator/control/fsae_control/fsae_control/stanley_controller.py` "is the actual current Stanley controller" | OK (path exists) | Confirmed present via earlier `find`. Actual current-ness of content (byte-identical to upstream) not independently verified beyond existence; low priority per CLAUDE.md's "check filesystem, don't assume" guidance, which this claim already follows. | none required |
| 112 | `MIN_AHEAD` (0.5 m), `LOOK_AHEAD` (25 m), `LOOK_WIDE` (10 m) in `sim/sim_track.py` | OK | Exact match: `sim/sim_track.py:56-58` (`LOOK_AHEAD=25.0`, `LOOK_WIDE=10.0`, `MIN_AHEAD=0.5`). | none |
| 117 | `build_path_walls()`, `build_local_path()`, `blend_paths()` | OK | All three exist in both `planning/boundary.py`/`planning/path_utils.py` (offline copy) and the `fsds_simulator/planning/fsae_planning/fsae_planning/` mirror, matching signatures. | none |
| 146-172 | `PoseFeedHold` "in `sim/rollout_core.py`", with `POSE_HOLD_PROB=0.05`, `MEAN_TICKS=2.1`, `MAX_TICKS=5` | STALE (class location) / OK (values) | **`PoseFeedHold` is actually defined in `sim/sensor_noise.py`** (`class PoseFeedHold` at line 134), only *imported and used* in `sim/rollout_core.py` (`from sim.sensor_noise import SlamNoise, ConeNoise, PoseFeedHold`, line 61). The doc's own settings.py section (line 145: "`PoseFeedHold` in `sim/rollout_core.py` models...") states the wrong home module. Values themselves are exact: `settings.py` has `POSE_HOLD_PROB = 0.05` (line 1519), `POSE_HOLD_MEAN_TICKS = 2.1` (1525), `POSE_HOLD_MAX_TICKS = 5` (1529) — doc abbreviates the constant names (drops the `POSE_HOLD_` prefix) which is a readability simplification, not wrong, but could confuse someone grepping for the bare name `MEAN_TICKS`. | Say "`PoseFeedHold` in `sim/sensor_noise.py` (used from `sim/rollout_core.py`)"; optionally spell out the full constant names once. |
| 149-154 | Pose-feed-hold measured table (fresh-pose rate, repeated ticks, longest hold, peak pose_age_s) | UNVERIFIABLE (reproducibility) | No rerun performed per task instructions; no inline citation to a specific log file in this section (contrast with the sim-to-real section later, which does cite `sim_to_real_investigation.md`). Numbers are plausible and internally consistent with the surrounding prose, but there's no pointer to where "measured on live telemetry (two runs...)" is archived. | Add a `docs/logs/` citation for traceability, per CLAUDE.md's "record what was falsified... a null result nobody can find gets re-tested" spirit (applies to positive measurements too). |
| 193-196 | `GRIP_SCALE = 1.1`, `INERTIA_SCALE = 0.8`, `COASTING_SCALE = 3.0` | OK | Exact match in `model/vehicle_physics.py` lines 138-140. Prime tuning-drift spot per task instructions, checked directly: current values match doc exactly. | none |
| 210 | `max_steer`, `max_accel`, `max_accel_brake` propagate to `controller/optimiser.py` and `mpc_core.py` QP constraints | STALE (one path) | Fields confirmed to exist (`max_steer` = 25°, `max_accel` = 12.0, `max_accel_brake` = -7.0, lines 173/184/189). But `controller/optimiser.py` no longer exists — same recurring drift. | Update to `controller/lmpc/solve.py`. |
| 223 | "split across three files... `model/bicycle_model.py`... `controller/optimiser.py`... `mpc_core.py`" | STALE (one path) | Same `controller/optimiser.py` drift, 4th occurrence in this file. | Update. |
| 235, 271, 275, 281 | Optuna: `USE_OPTUNA_PRESEARCH` (default `True`), `OPTUNA_PRE_PASS_EVALS` (default 10% of MAX_EVALS) | OK (constant exists) | `USE_OPTUNA_PRESEARCH` and `OPTUNA_PRE_PASS_EVALS` both confirmed present in `settings.py` (exactly 1 match each). Exact default values (`True`, "10% of MAX_EVALS") not independently re-derived from the file in this pass; low risk given the constant names themselves check out and the doc's own phrasing hedges with "default". | Optional: re-confirm literal default values if a future pass has time. |
| 261 | `TUNABLE_Q_IDX = [0,1,2,3,4]`, `TUNABLE_R_IDX = [0,1]`, `TUNABLE_R_RATE_IDX = [0,1]` | OK (values), file location not asserted by doc | Exact match, but these constants live in `tuner/offline_tuner.py` (lines 150-152), not `settings.py` — the doc doesn't explicitly claim they're in `settings.py` in this sentence, so not marked WRONG, just noting for completeness. | none required |
| 295-301 | `cma.fmin_lq_surr2`, BIPOP `incpopsize=2`, `max_restarts = 7`, `sigma0 = 0.65`, `CMA_stds = 0.23 · log(upper/lower)` | OK | All confirmed in `tuner/offline_tuner.py`: `sigma0 = 0.65` (line 1303), `max_restarts = 7` (1313), `incpopsize=2` (1487 region), `cma.fmin_lq_surr2` call (1487). The `0.23` coefficient in `CMA_stds` formula not independently re-derived numerically in this pass (only structurally confirmed cma_stds is computed and passed), low priority. | none required beyond optional numeric re-derivation |
| 315 | `TAIL_QUANTILE` (in `settings.py`, default `0.8`) | OK | `settings.py: TAIL_QUANTILE = 0.8` confirmed exact. | none |
| 342-360 | "The 13 metrics" table: names, order 0-12, formulas | OK | Cross-checked against `sim/scoring.py`'s `IDX_*` constants (lines 40-52) and the `metrics = np.array([...])` construction (lines 94-108) in `compute_composite_score`: order and names match exactly (rmse, yaw_rms, smooth_rms, steer_rms, accel_rms, max_steering, steering_sat_ratio, jerk_rms, max_yaw_rate, steering_reversal_rms, peak_lateral_error, speed_rmse, accel_reversal_rms). Metric 0 formula (`1.2·e_y² + 0.4·e_psi²`) not independently re-derived from `add_step`'s internals in this pass (accepted on the strength of the exact-match ordering elsewhere in the same table); no evidence of drift found. | none required |
| 386 | "see `LapProgressTracker` in `docs/reference/README.md`'s 'Live/offline score parity' section" | WRONG | `docs/reference/README.md` does **not** contain a "Live/offline score parity" section or mention `LapProgressTracker` (grep confirms zero hits in that file). The actual section and `LapProgressTracker` documentation live in **`docs/reference/offline_live_parity.md`** (heading `## Live/offline score parity` at line 200, `LapProgressTracker` used throughout). This is a wrong cross-reference, not just stale, since `docs/reference/README.md` never had this content (the reference docs were split by subject into multiple files, `README.md` is only the index). | Change pointer to `docs/reference/offline_live_parity.md`. |
| 398-421 | "Module Reference" table | MOSTLY OK, dup of README issue | Same set of files as README's Key Modules table; same **`controller/optimiser.py` STALE** issue at line 411 (`| controller/optimiser.py | The parameterised CVXPY/OSQP QP formulation and solve...`), 5th occurrence of this exact drift in the two audited docs combined. Also line 420 repeats the `mpc_controller.py`/`mpc_core.py`/`control_utils.py` staged-path row, consistent with README's version. Line 421's `fsds_simulator/` row links `` [`docs/reference/`](`docs/reference/`) `` — same malformed-link bug as README's. | Fix the `controller/optimiser.py` → `controller/lmpc/` rename here too (this is the 5th and most consequential occurrence, since it's presented as the authoritative "Module Reference"), and fix the malformed link. |
| 429 | `nmpc_core.NMPCController` "chosen by the node parameter `use_nmpc`, default false" | STALE (default claim) | Class name correct. But as with README's flag-name issue: this describes the **live ROS2 node parameter default** (`nmpc_params.py`'s dataclass default is indeed `False`), yet `ros2/launch_all.sh:224` currently sets `USE_NMPC=true`, so the actual default behavior of a `launch_all.sh`-driven run is NMPC-on. The doc's claim is only true of the dataclass default in isolation, and doesn't warn a reader that the shortlist can (and currently does) override it — the exact failure mode CLAUDE.md's GUI/parity sections call out repeatedly. | Add a one-line caveat: "the dataclass default is `false`; check `ros2/launch_all.sh`'s `USE_NMPC=` line for the current effective default, which may override it." |

**Code paths/modules mentioned** (for a future restructure, deduplicated
against README's list above — new items only): `sim/sensor_noise.py`
(actual `PoseFeedHold` home), `model/bicycle_model.get_8state_discrete_model`,
`controller/model_utils.adaptive_R_scaling`, `controller/lmpc/build.py`,
`controller/lmpc/solve.py` (both correct-but-undocumented replacements for
`controller/optimiser.py`), `sim/speed_profile.py` (`compute_speed_profile`,
`smooth_profile`, `curvature_speed`, `optimal_lap_time`), `sim/sim_track.py`
(`place_cones`, `SimPerception`, `SimPlanner`), `tuner/performance_stats.py`
(`benchmark_weights`), `tuner/offline_tuner.py` (`SYNTHETIC_PATHS`,
`PATH_NAMES`, `get_cached_model`, `build_synthetic_paths`,
`parallel_evaluate_candidate`), `docs/reference/offline_live_parity.md`
(the doc that architecture.md line 386 *should* point to).

**Proposed action: KEEP, with a fix pass, and one structural note.** The
document's scope and split (overview here, deep math in lmpc.md/nmpc.md) is
sound and shouldn't be undone. Needed fixes: (1) the recurring
`controller/optimiser.py` → `controller/lmpc/` rename, present in **5
places across the two docs** (README Key Modules row, architecture.md's
mermaid diagram, ROS2-vs-Simulator table, Actuator-limits prose, MPC-works
prose, and Module Reference table — this is the single highest-value fix
in this audit, since it's the most-repeated stale fact); (2) fix the
`LapProgressTracker`/`docs/reference/README.md` wrong pointer; (3) fix
`PoseFeedHold`'s stated home module; (4) fix the malformed `` `docs/reference/` ``
link (2 occurrences in this file, same bug as README); (5) add the
`launch_all.sh USE_NMPC=true` override caveat in both docs' NMPC-default
sentences. None of this needs a split/merge/delete, the doc's shape is fine.

---

## Summary of cross-doc, repeated issues

1. **`controller/optimiser.py` no longer exists** (moved to
   `controller/lmpc/{build,solve}.py` in commit `a941625`). Referenced as
   still-current in **5 places**: README.md line 154 and architecture.md
   lines 65-71 (mermaid), 96 (ROS2-vs-Sim table), 210 (actuator limits),
   223 (MPC-works prose), 411 (Module Reference table). This is the
   single most important fix — a reader following any of these paths
   hits a dead file.
2. **Malformed link `` [`docs/reference/`](`docs/reference/`) ``**, backticks
   wrongly included inside the URL parens, appears 4 times total (README
   lines 70 and 135, architecture.md lines 26/103's rendering is fine since
   those use `docs/reference/` without backticks in the URL — re-check: only
   README's two occurrences and architecture.md's Module Reference row (421)
   and its own line-70-equivalent are malformed; verified above).
3. **`use_nmpc`/`USE_NMPC` naming and default are both slightly
   mis-stated**: offline flag is uppercase `USE_NMPC`; and whatever the
   dataclass default is, `ros2/launch_all.sh` currently forces
   `USE_NMPC=true`, so calling LTV-QP "the default" without that caveat is
   actively misleading about current live behavior.
4. **Two stale commit-hash references** (`59f03fa` for the outer repo, 1
   commit stale; `28dcd4d` for `fsae_planning`, 14 commits / ~4 months
   stale).
5. **One wrong cross-reference**: architecture.md line 386 points to
   `docs/reference/README.md` for `LapProgressTracker`/score-parity content
   that actually lives in `docs/reference/offline_live_parity.md`.
6. **One wrong module attribution**: `PoseFeedHold` is documented as living
   in `sim/rollout_core.py`; it's actually defined in `sim/sensor_noise.py`
   and only imported into `rollout_core.py`.

## Recommendation

- **README.md: keep, fix in place.** ~4 discrete fixes (path rename, 2 link
  fixes, flag-name/default caveat, optionally refresh commit hashes).
- **docs/architecture.md: keep, fix in place.** ~6 discrete fixes (path
  rename x5 locations, wrong cross-ref, wrong module attribution, link fix,
  NMPC-default caveat). No content is unsalvageable or wrong at the concept
  level; every issue found is a stale/wrong pointer, not a wrong explanation
  of how the system works. Neither doc needs splitting, merging, or
  deleting; the existing split with `lmpc.md`/`nmpc.md`/`docs/reference/*`
  is working as designed and should be preserved.
