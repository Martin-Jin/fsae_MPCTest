# CLAUDE.md audit (2026-09-29)

## Repo roles / existence

| claim | status | evidence | proposed fix |
|---|---|---|---|
| fsae_autonomous: "no MPC controller has landed there at all (only stanley_controller.py exists)" | WRONG | `ros2_autonomous/src/fsae_autonomous/control/fsae_control/fsae_control/mpc/nmpc_core.py` exists, plus 3 new test files (`test_nmpc_core_math.py`, `test_bench_scenarios.py`, `test_nmpc_signs_magnitudes.py`) referencing `fsae_control.mpc.nmpc_core`. All uncommitted (`??` in git status) as of this session — likely in-progress NMPC port matching MEMORY.md's `project_nmpc_implemented.md` note. | Update to note the NMPC port has landed (uncommitted) in `fsae_autonomous`, or reword to "as of this writing" with a caveat it may already be stale. |
| fsae_autonomous checkout moved to `ros2_autonomous/src/fsae_autonomous/` | OK | Confirmed exists at that path; old `fsae_autonomous/` path gone. | none |
| `fsae_MPCTest`/`fsae_planning`/`fsae_autonomous` have no own CLAUDE.md | OK | `find ... -iname CLAUDE.md` across all repos returns only the one at outer repo root. | none |
| "Testing" section: "no test files (no pytest/unittest) across any of the four repos" | WRONG | `fsae_autonomous` has real unit tests: `control/fsae_control/test/test_nmpc_core_math.py` (262 lines), `test_bench_scenarios.py`, `test_nmpc_signs_magnitudes.py` (346 lines), plus perception `test_fusion_time_sync.py`/`test_refinement.py`. (The `test_flake8.py`/`test_copyright.py`/`test_pep257.py` in `fsae_planning` are ROS2 ament boilerplate lint stubs, not real tests, and were presumably already present when this claim was written.) | Reword: "no test suite in fsae_planning/fsae_MPCTest/outer repo; fsae_autonomous has started accumulating NMPC unit tests (uncommitted as of 2026-09-29)." |

## Git layout table

| claim | status | evidence | proposed fix |
|---|---|---|---|
| Branch `main` current for `fsae_planning`, `fsae_MPCTest`, `fsae_autonomous` | PARTIALLY STALE | `fsae_planning`'s checkout is currently ON `feature/nmpc-and-controller-improvements`, not `main` (session-specific checkout state, not necessarily wrong per the table which documents intended-branch, but worth flagging since a concurrent session may be using the feature branch). `fsae_MPCTest` and `fsae_autonomous` are on `main` as documented. | Note table describes intended/default branch; current checkout may differ per session. |
| `feature/nmpc-and-controller-improvements` branch exists, narrow push exception | OK | Branch exists locally and on origin, with active commits (`ba0e580`, `e339901`, `b244b2e` — NMPC live_viz/weight work), consistent with being an active line of work. | none |
| All 3 inner repos have uncommitted local changes | OK (context) | `git status --short` in all three shows modified/untracked files — consistent with "concurrent sessions" warning elsewhere in CLAUDE.md. | none, just corroborates existing warning |
| No `fsae_MPCRos` local clone by default | OK | Not found anywhere under `/home`. | none |

## Single source of truth for MPC tuning

| claim | status | evidence | proposed fix |
|---|---|---|---|
| "106 dataclass fields total" (MPCParams + NMPCParams) | STALE | Counted directly: `MPCParams` = 69 fields, `NMPCParams` = 35 fields, total = **104**, not 106. CLAUDE.md itself says "count directly rather than trusting this number as it drifts" — it drifted. | Update to 104 (or re-verify at edit time per the file's own instruction). |
| Live path `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_params.py` | WRONG (path) | Actual path has an extra `mpc/` subdirectory: `control/fsae_control/fsae_control/mpc/mpc_params.py`. Same for `mpc_core.py`, `mpc_controller.py` — all now live under `.../fsae_control/mpc/`. `sync_mpc_params.py` itself already uses the correct `mpc/`-prefixed paths (see its `_MIRROR_ROOT`/dest dict), so the code is not confused, only CLAUDE.md's prose. | Add `mpc/` to every cited live path for `mpc_params.py`, `nmpc_params.py`, `mpc_core.py`, `mpc_controller.py`. |
| `mpc_controller_standalone.py` exists as separate file | WRONG | File no longer exists as source (only a stale `.pyc` remains: `control/fsae_control/fsae_control/__pycache__/mpc_controller_standalone.cpython-312.pyc`). Its functionality was merged into `mpc_controller.py` as a `standalone_output` ROS2 param (`MPCControllerNode`, see its module docstring: "This node has TWO output modes, selected by the `standalone_output` ROS2 parameter"). | Remove `mpc_controller_standalone.py` from every list that names it (Single-source-of-truth section, Third-copy file list); replace with "mpc_controller.py's `standalone_output` mode". |
| Example mapping `MPCParams.adaptive_r_rate_during_floor` ↔ `settings.ADAPTIVE_R_RATE_DURING_FLOOR` | WRONG | This field/mechanism was **deliberately removed** (not renamed) from both `MPCParams` and `settings.py` — documented in `fsae_MPCTest/docs/removed_mechanisms.md` ("Adaptive R_rate current-curvature floor" entry) and `docs/reference/control_mechanisms.md`, `docs/tuning.md`. Reason for removal: its computed value was always immediately overwritten by the corner-factor blend, so it never reached the QP live or offline despite being logged to telemetry. It still exists only in `.bak` files and in the (unsynced) `fsae_autonomous` checkout's `mpc_params.py`/`fsae_params.yaml`. | Replace the worked example with a currently-live field pair, e.g. `MPCParams.nmpc_corner_factor_k` ↔ a still-existing settings constant, and cross-reference `docs/removed_mechanisms.md` for why the old example was retired. |
| "See `fsae_MPCTest/docs/planning_control_sync.md`'s 'Numeric-parity constants' table" | WRONG (moved) | `planning_control_sync.md` is now a 16-line redirect stub ("This document has been split by subject into `docs/reference/`"). The actual "Numeric-parity constants" section is at `docs/reference/offline_live_parity.md` (confirmed heading present, line 121). | Update every cross-reference from `planning_control_sync.md` to `docs/reference/offline_live_parity.md`. |

## Git layout / fsae_MPCRos section

| claim | status | evidence | proposed fix |
|---|---|---|---|
| `tuner/tools/sync_mpc_params.py`'s `_AUTONOMOUS_CANDIDATES` list and mechanism | OK | Confirmed exact match: list contains both the old (`fsae_autonomous`) and new (`ros2_autonomous/src/fsae_autonomous`) paths, with a comment dated 2026-09-23 explaining exactly this history. | none |
| sync_mpc_params.py scope: `mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml` only | OK | Confirmed in source: `_MIRROR_FILES`-equivalent dict maps exactly `mpc/mpc_params.py`, `mpc/nmpc_params.py`, `common/fsae_bringup/config/fsae_params.yaml`. | none |
| Never syncs `fsae_MPCTest/settings.py` | OK | Confirmed by reading script; no reference to `settings.py` as a sync target. | none |

## Third copy: fsds_simulator mirror

| claim | status | evidence | proposed fix |
|---|---|---|---|
| `.gitignore` `/fsae_planning/` anchored pattern (fixed 2026-08-07) | OK | `fsae_MPCTest/.gitignore` line 5: `/fsae_planning/`, anchored. | none |
| Mirror includes full `common/`, `perception/`, `planning/` packages plus `control/fsae_control/` files named | OK | `find fsds_simulator -maxdepth 4` shows exactly this layout: `common/fsae_bringup`, `common/fsae_interfaces`, `control/fsae_control` (with `mpc/` subdir), `perception/fsae_sim_perception`, `planning/fsae_planning`. | none |
| Scoring parity: `sim/scoring.py` vs live copy, "verbatim copy... only difference is docstrings/inlined constants" | OK | Diffed `compute_composite_score`/`RolloutMetrics` bodies: only comments/docstrings differ; all formulas, weights, control flow are identical. | none |

## Model / plant (vehicle_physics.py)

| claim | status | evidence | proposed fix |
|---|---|---|---|
| `IDX_ALAT_LIM`, `alat_ceiling_enabled`, `alat_ceiling_mode='pi'`, `gain=450`, `tau=0.40` | OK | All confirmed present at stated values in `model/vehicle_physics.py` (lines 117, 205, 218, 222, 229). | none |

## NMPC solver internals / "danger zone" naming

| claim | status | evidence | proposed fix |
|---|---|---|---|
| "NMPC solver internals in `nmpc_optimiser.py`/`nmpc_core.py`" (appears 3x: model-usage danger-zone list, Third-copy file list, Testing section) | WRONG (retired) | `nmpc_optimiser.py` no longer exists in the live tree — it was moved to `fsae_MPCTest/deleted/controller/nmpc_optimiser.py`. Its functionality is now split into `controller/nmpc/{dynamics,layout,outputs,reference,solver,weight_schedule}.py`. `nmpc_core.py` itself still exists and is current. | Replace every `nmpc_optimiser.py` reference with `controller/nmpc/` (the package) or specifically `controller/nmpc/solver.py`, and note `nmpc_optimiser.py` is retired (see `deleted/controller/nmpc_optimiser.py` for history). |
| `controller/model_utils.py` still current, single file | OK | Confirmed present. | none |
| `controller/optimiser.py` for LMPC (implied by parallel structure) | STALE (recent split) | Not directly named in CLAUDE.md, but relevant: `controller/lmpc/` package (`build.py`, `solve.py`) now holds what used to be a single `optimiser.py`; latest commit "Split controller/optimiser.py into controller/lmpc/ package" dated 2026-09-29 (today). | If CLAUDE.md is edited soon, note this split explicitly since it's brand new. |

## Testing / CLI commands

| claim | status | evidence | proposed fix |
|---|---|---|---|
| `python -m tuner.recorded_map_rollout` | OK | `tuner/recorded_map_rollout.py` exists at root of `tuner/` package, parses fine. | none |
| `python -m tuner.nmpc_offline_check` | OK | `tuner/nmpc_offline_check.py` exists at root, parses fine. | none |
| `python -m tuner.plant_openloop_validation` | **WRONG** | Module has moved to `tuner/checks/plant_openloop_validation.py`. Confirmed the old import path raises `ModuleNotFoundError`. A stale `tuner/__pycache__/plant_openloop_validation.cpython-312.pyc` (no matching `.py`) is the fossil of the old location. | Change every occurrence (appears at least 3x: Model-usage list implicitly, "The offline sim does not yet fully predict the car" section, Testing section, Code-review checklist) to `python -m tuner.checks.plant_openloop_validation`. |
| `tuner/tools/sync_mpc_params.py` invocation `python -m tuner.tools.sync_mpc_params` | OK | Confirmed module exists at that exact path. | none |

## Docs cross-references (sim_to_real_investigation.md)

| claim | status | evidence | proposed fix |
|---|---|---|---|
| §12.8 "reference-heading lead" still open, "next thing to pursue" | STALE | Section §12.8 still exists at that number (confirmed), but per-memory-note `project_sim_to_real_gap.md`, §54 of the same doc (now superseded further by §60+) explicitly closes this lead as a dead end: "the one candidate tried (§29) already failed live... retrying the same fix isn't worth it." Doc has grown to §60 "Pose-rate mismatch" as of 2026-09-29 (commit `a941625`, same day). | Remove "this is the next thing to pursue" framing for §12.8; read current doc tail (§56-60+) before restating a recommendation. |
| §12.12 `alat_ceiling_tau` measured 0.40s, "did not close the gap" | OK | Section and content confirmed present and unchanged (tau=0.40 matches code). | none |
| Table's numbers (steering saturation 4.8%/21.1% etc.) frozen from an early investigation state | LIKELY STALE (unverified exact current numbers) | Doc has continued 12+ sections past the state CLAUDE.md's table reflects (up to at least §60 by 2026-09-29 vs a table implicitly dated around §37-38). Per memory note, "no clean before/after gap-closed number exists... live baseline itself keeps shifting between logs." | Flag the whole "does not yet fully predict the car" table as needing re-verification against the doc's latest section before trusting exact percentages; this was already flagged by memory as of 44 days ago and the doc has moved further since. |

## planning_control_sync.md — wholesale relocation

| claim | status | evidence | proposed fix |
|---|---|---|---|
| Doc still the "authoritative field-by-field mapping" / has "Numeric-parity constants", "Known planner defect: centreline curvature spikes", "MECHANISM: dynamically-enforced lateral-acceleration ceiling", "Live/offline score parity" sections | WRONG (moved, not present) | The file itself says it was split (self-documented, undated but clearly recent: "had grown to 38 sections and 2214 lines"). New locations confirmed by heading search: <br>• Numeric-parity constants + Live/offline score parity → `docs/reference/offline_live_parity.md` <br>• Known planner defect: centreline curvature spikes → `docs/reference/simulator_fidelity.md` (section literally titled "Known planner defect: centreline curvature spikes (OPEN — not fixed)") <br>• MECHANISM lateral-accel ceiling → `docs/reference/simulator_fidelity.md` ("The sim-to-real gap: a lateral-acceleration ceiling, partly closed" / "Root cause: FSDS enforces a sustained lateral-acceleration ceiling") | Every one of the ~6 CLAUDE.md cross-references to `planning_control_sync.md` needs its target file updated to the specific `docs/reference/*.md` file per the stub's own redirect table. This is the single biggest source of stale cross-references in the whole document. |

## WILL-GO-STALE items (restructure described in the audit prompt, not yet complete or partially complete)

| item | current state | what will break |
|---|---|---|
| `settings.py` split into `settings/` package | NOT YET DONE — still a single 100KB file (`settings.py`, plus stray `settings.py.bak`) | When split happens, every `settings.py`-relative path/import reference in CLAUDE.md ("Single source of truth" section, "GUI settings coverage" section) breaks. |
| `deleted/` folder removed, retired files deleted outright | NOT YET DONE — `deleted/controller/` still exists and currently holds `nmpc_optimiser.py` | CLAUDE.md's "Prefer moving a file to a sibling `deleted/<original path>` over an outright delete" instruction (Development practices) will become wrong the moment this happens; also erases the one concrete example this audit found (`nmpc_optimiser.py`) of that convention working as documented. |
| `fsds_simulator/` mirror restructured and copied over `fsae_planning`'s implementation files | NOT YET DONE — mirror still following live `fsae_planning`'s current (pre-restructure) file layout, e.g. `mpc/` subdir matches | Once this happens, "Third copy" section's whole framing (mirror trails live, PR-staging only) may partially invert if the mirror becomes the source pushed back into `fsae_planning`; re-check direction of sync. |
| `docs/logs/` labelled pre-refactor via README | NOT YET DONE — no README exists in `docs/logs/` today (only the log files themselves) | "Data retention" section's claim that `docs/logs/` are "curated... no retention concern" may need a caveat once a README marks some as pre-refactor/stale. |
| `docs/modules/offline_sim.md`, `docs/modules/fsds_ros2.md` | NOT YET DONE — no `docs/modules/` directory exists | Any future CLAUDE.md edit referencing these new docs should confirm they exist before citing headings inside them, same mistake as the `planning_control_sync.md` split above. |
| `docs/*.md` merges/renames beyond the sync-doc split already done | PARTIALLY DONE — `planning_control_sync.md`→`docs/reference/*` already happened; likely more consolidation coming | Re-run this same cross-reference check after any further doc merge, since it is the highest-yield error class found in this audit. |

---

# Memory notes audit

| note | status | evidence | fix |
|---|---|---|---|
| `MEMORY.md` top-line: "CLAUDE.md is ~36 sections stale" | STALE (undercounts, and imprecise) | The linked note `project_sim_to_real_gap.md` itself says "~44 sections" (not 36) as of 2026-08-10, and the real doc has since grown to at least §60 (2026-09-29 commit). The "36" figure in MEMORY.md doesn't match either the linked note's own number or the current state. Likely drifted further since the note was last touched. | Update MEMORY.md's one-line summary to not hardcode a section-gap count at all (it visibly can't stay current); point to "read the doc's tail + git log" as the note itself recommends. |
| `project_sim_to_real_gap.md` full content | STALE (dated content, correctly self-aware) | Frozen at §48-56 (2026-08-09/10). Real doc now at §60 (2026-09-29). The note's own final paragraph already tells the reader to re-check git log rather than trust its numbers — so the note is "stale but honestly labeled," not misleading if followed as instructed. Also carries the standard 49-day-old auto-disclaimer. | No action needed beyond what the note already prescribes; maybe append a pointer noting the doc reached §60 as of 2026-09-29 if convenient, but not required since the note self-flags. |
| `feedback_planning_repo_no_push.md` — "read-only reference only... report, don't commit/push" | OK, but incomplete vs current CLAUDE.md | Current CLAUDE.md has since added the narrow branch-push exception (`feature/nmpc-and-controller-improvements`, added 2026-09-16) which this memory note doesn't mention. Not wrong, just less complete than current CLAUDE.md. | Optionally append the exception so the memory doesn't read as more absolute than the current rule. |
| `project_track_storage_relocated.md` — tracks/ moved to fsae_planning | OK | Confirmed: `ros2/src/fsae_planning/tracks/` holds real per-track data (`cone_map.json`, `speed_profile.csv`, `centerline.csv`, `raceline.csv` for 3 tracks); `fsae_MPCTest/tracks/` holds only the Python package (`__init__.py`). Matches CLAUDE.md exactly. | none |
| `project_dynamic_speed_cap_disabled.md` | UNVERIFIABLE without deeper grep of launch_all.sh flags | Not specifically re-checked line-by-line this session; no contradicting evidence found. | Leave as-is; low priority since it's a "disabled" state note, cheap to re-verify with a single grep if ever relevant. |
| `project_exit_boost_timing_fix.md`, `project_r_a_underaccel_fix.md`, `project_corner_segmentation_implemented.md` | UNVERIFIABLE this session | Not directly re-checked; nothing found that contradicts them. Plausible given active/recent commit history around MPC weight tuning. | Re-verify only if a task specifically touches corner segmentation, exit-boost, or r_a. |
| `project_nmpc_implemented.md` — "now has fsae_MPCTest offline port" | OK, reinforced | This session's finding that `fsae_autonomous` now also has an (uncommitted) `nmpc_core.py`+tests is a direct continuation of this note's trajectory (NMPC moving from fsae_MPCTest → fsae_planning → now fsae_autonomous). | Consider adding a follow-up note: NMPC now also present (uncommitted) in fsae_autonomous as of ~2026-09-29, mirrors not yet pushed to fsae_MPCRos. |
| `project_mpcc_inspired_nmpc_features.md` — "REJECTED (both off)" | UNVERIFIABLE this session (would need explicit flag check) | Not directly re-checked. | Re-verify only if MPCC/speed-profile/friction-circle flags are touched. |
| `project_centreline_beats_raceline.md` | UNVERIFIABLE this session | Not directly re-checked. | Low priority. |
| `project_speed_error_dominance.md`, `project_nmpc_corner_overprediction_diagnosis.md`, `project_mpcc_progress_term_plan.md`, `project_nmpc_accel_authority_ceiling.md`, `project_deficit_max_was_accel_ceiling.md` | UNVERIFIABLE this session | Diagnosis-style notes about specific tuning behavior; not re-run/re-measured this session (would require actually running the rollout, out of scope for a read-only doc audit). | Re-verify via `tuner.recorded_map_rollout` before trusting for an active tuning decision, per CLAUDE.md's own testing discipline. |
| `feedback_*` notes (concurrent sessions, comment style, git fetch, onboarding voice, prefer live test, prose style) | OK / not falsifiable by code inspection | These are process/style conventions, not code-state claims; nothing in this session contradicts them. Concurrent-session note is directly corroborated (uncommitted changes found in all 3 inner repos this session). | none |

## Summary of highest-value fixes (ranked)

1. **`planning_control_sync.md` cross-references are uniformly wrong** — the doc was split into `docs/reference/*.md`; every specific-section citation needs redirecting per the stub's own table.
2. **`nmpc_optimiser.py` no longer exists** — retired to `deleted/controller/nmpc_optimiser.py`, replaced by `controller/nmpc/*` package. 3 CLAUDE.md citations need updating.
3. **`tuner.plant_openloop_validation` module path is wrong** — moved to `tuner.checks.plant_openloop_validation`; the documented command will fail as typed.
4. **`mpc_controller_standalone.py` no longer exists** — merged into `mpc_controller.py`'s `standalone_output` parameter.
5. **fsae_autonomous now has an NMPC controller + tests** (uncommitted) — contradicts both the "no MPC controller" and "no test suite" claims.
6. **106-field count is now 104** (69 + 35) — drifted as CLAUDE.md itself warned it would.
7. **Live paths missing `mpc/` subdirectory** — `mpc_params.py`/`mpc_core.py`/`mpc_controller.py`/`nmpc_params.py` all moved under `.../fsae_control/mpc/`.
8. **Worked parity example (`adaptive_r_rate_during_floor`) cites a deleted field** — mechanism was removed for being dead code; still lingers unsynced in `fsae_autonomous`.
9. **§12.8 "reference-heading lead... next thing to pursue" is stale** — later sections of the same doc (per memory note) already closed this off as tried-and-failed.
10. MEMORY.md's own "~36 sections stale" figure is itself wrong/outdated (linked note says ~44, doc has grown further since).
