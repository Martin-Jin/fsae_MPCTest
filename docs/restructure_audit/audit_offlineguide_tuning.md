# Documentation audit: offline_guide.md + tuning.md

Audited against: `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_params.py`,
`mpc/nmpc_params.py`, `fsae_MPCTest/settings.py`, `tuner/offline_tuner.py`, `gui/`, `ros2/launch_all.sh`,
`docs/reference/*.md`, `docs/removed_mechanisms.md`, `docs/logs/late_turn_in_investigation.md`, git log.

---

## 1. `docs/offline_guide.md` (253 lines)

**Purpose/audience**: how-to for the 2D GUI, offline CMA-ES tuner, and manual-drive tool. Audience is a
contributor running things locally, not someone deciding *what* to tune. Scope is respected: it defers to
`architecture.md` for "why" and to `tuning.md` for weight semantics, and does not duplicate weight values.

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 47 | 10 named synthetic paths (`PATH_SUDDEN_TURN` ... `PATH_MIXED`) | OK | `tuner/offline_tuner.py` `build_synthetic_paths()`, exactly these 10 keys, no more | — |
| 48 | `tracks/*/cone_map.json` + `fsds_simulator/cone_maps/*.json` fallback | OK | `ros2/src/fsae_planning/tracks/{comp_test_map_3,...}/cone_map.json` exist; `fsds_simulator/cone_maps/comp test map 3.json` exists | — |
| 140 | `mpc_params.py` path given as `ros2/src/fsae_planning/control/fsae_control/fsae_control/` | STALE | actual path is one level deeper: `.../fsae_control/fsae_control/mpc/mpc_params.py` (an `mpc/` package now holds `mpc_params.py`, `nmpc_params.py`, `mpc_core.py`, `nmpc_core.py`, `mpc_controller.py`) | add `/mpc/` to the path |
| 140 | `MPCParams` fields: `q_e_y`, `q_e_yd`, `q_e_psi`, `q_r`, `q_e_v`, `r_delta`, `r_a_accel`/`r_a_brake`, `r_rate_delta`, `r_rate_a` | OK | all 10 confirmed present in current `mpc_params.py` dataclass | — |
| 140 | "`mpc_core.py` builds its own `Q_diag`/`R_diag`/`R_rate_diag` from `self.params.*` ... no hardcoded weights" | OK | `mpc_core.py` line 543/548 builds `R_diag`/`R_rate_diag` from `self.params.r_delta` etc. | — |
| 181 | `tuner/checks/` vs `tuner/` root file placement (`steering_chatter_check.py`, `reference_heading_geometry_check.py`, `reference_excess_mechanism_check.py`, `nmpc_offline_check.py`, `recorded_map_rollout.py` at root) | OK | `ls tuner/` confirms all 5 are in root, not `tuner/checks/` | — |
| 165-181 | tuner/ layout tables (`offline_tuner.py`, `performance_stats.py`, `csv_log.py`, `recorded_map_rollout.py` at root; `plot_playback.py`, `export_speed_profile.py`, `raceline_optimizer.py`, `doc_lint.py` in `tuner/tools/`) | OK | all 8 files resolve at the claimed paths. Note `tuner/tools/` also now has `sync_mpc_params.py` (added later, CLAUDE.md-documented) — not in this table but not a wrong claim, just an incomplete one | add `sync_mpc_params.py` row if updating this table |
| 209-217 | Dependencies table versions (numpy≥1.24, scipy≥1.10, matplotlib≥3.7, cvxpy≥1.4, osqp≥0.6, clarabel≥0.6, cma≥3.3, optuna≥4.0) | UNVERIFIABLE | no `requirements.txt`/`setup.py`/`pyproject.toml` anywhere under `fsae_MPCTest/` to check pins against | leave as-is or note "unpinned, informational only" |
| 247-253 | `settings.USE_NMPC` flag, `run_core_rollout(use_nmpc=...)` param, `tuner.nmpc_offline_check` module | OK | `USE_NMPC = False` in settings.py confirmed; module imports cleanly (`python -m tuner.nmpc_offline_check` resolves) | — |
| 253 | links to `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section | OK | section exists at control_mechanisms.md:350 | — |
| 3-5 | links to `reference/simulator_glossary.md`, `architecture.md`, `debugging_tools.md` | OK | all three files exist | — |
| 40 | link to `debugging_tools.md#centralized-launcher-guilauncherpy` | OK | anchor exists at debugging_tools.md:35 | — |
| 91-95 | `VALIDATION_SUITE`, `MAX_EVALS`, `USE_PLANNER`, `USE_OPTUNA_PRESEARCH` names/defaults (`USE_PLANNER=False`, `USE_OPTUNA_PRESEARCH=True`) | OK | all four constants confirmed in settings.py with stated defaults (`MAX_EVALS=1500`, `USE_PLANNER=False`, `USE_OPTUNA_PRESEARCH=True`) | — |
| 185/177 | links to `fsds_integration_guide.md#csv-telemetry-logging`, `#2-export-the-speed-profile...`, `#recording-exporting-and-driving-a-track` | OK | all three headings exist verbatim in fsds_integration_guide.md | — |
| 196-201 | `gui/manual_drive.py` controls (W/S throttle/brake, A/D steer, SPACE full brake) | OK | matches manual_drive.py docstring/labels exactly | — |
| 241-245 | "Adding a new synthetic path" steps: `build_synthetic_paths()`, `_make_arc(cx, cy, radius, start_deg, end_deg, n)`, `_resample_path(wx, wy)` | OK | both function signatures match exactly (`_make_arc(cx, cy, radius, theta_start_deg, theta_end_deg, n=20)`, `_resample_path(waypoints_x, waypoints_y, n_points=...)`) | — |

**Overlap with other docs**: none problematic. It correctly defers weight semantics to `tuning.md` and
architecture to `architecture.md`, and doesn't restate numbers that would drift.

**Readability**: generally good, structured, numbered steps. One long run-on sentence in step 3 (line 48)
packs path-mode semantics, `USE_PLANNER` behavior, and a cross-reference into one dense paragraph — could
split into two sentences.

**Mechanism-without-WHY**: none egregious; this doc is intentionally a how-to, not a rationale doc, and
correctly delegates "why" to architecture.md throughout.

**Code paths/modules mentioned** (for a future restructure):
`gui/simulation.py`, `gui/launcher.py`, `gui/manual_drive.py`, `tuner/offline_tuner.py`,
`tuner/performance_stats.py`, `tuner/csv_log.py`, `tuner/recorded_map_rollout.py`,
`tuner/tools/plot_playback.py`, `tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py`,
`tuner/tools/doc_lint.py`, `tuner/steering_chatter_check.py`, `tuner/reference_heading_geometry_check.py`,
`tuner/reference_excess_mechanism_check.py`, `tuner/nmpc_offline_check.py`, `sim/rollout_core.py`,
`model/vehicle_physics.py`, `sim/track_io.py`, `settings.py`, `mpc_params.py` (path stale, see above),
`mpc_core.py`, `tracks/__init__.py` (implicitly via `tracks/*/cone_map.json`).

**Proposed action: KEEP, small fix.** Fix the one stale path (line 140, missing `/mpc/` segment). Otherwise
accurate and well-scoped; no need to merge/split.

---

## 2. `docs/tuning.md` (397 lines)

**Purpose/audience**: declared "single canonical reference for tuning the MPC" — every weight/gain/flag,
what it does, how to adjust it. Audience is someone about to change a weight and needing the known-constraints
history before doing so. This doc explicitly does NOT restate current numeric values (line 5) except in
§4b/§4c/historical-values callouts, which is where the stale numbers below live — a direct consequence of the
doc violating its own stated policy in a few places.

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 25-35 | Section 1 table fields (`q_e_y`, `q_e_yd`, `q_e_psi`, `q_r`, `q_e_v`, `r_delta`, `r_a_accel`, `r_a_brake`, `r_rate_delta`, `r_rate_a`, `terminal_q_scale`) | OK | all 11 confirmed on `MPCParams` | — |
| 50-54 | Section 2 fields (`delay_compensation_enabled`, `max_delay_compensation_steps`, `predict_epsi_clip`, `pose_age_lp_alpha`, `n_delay_hysteresis`) | OK | all confirmed on `MPCParams` | — |
| 56 | `DELAY_STEPS`/`DELAY_JITTER_STEPS`/`DELAY_JITTER_SEED` offline-only in settings.py | OK | `DELAY_STEPS=1`, `DELAY_JITTER_STEPS=0.2`, `DELAY_JITTER_SEED=12345` all present | — |
| 68-69 | `ref_heading_rate_limit_enabled`, `ref_heading_rise_rate_deg_s` fields; "do not re-enable" implies default False | OK | both fields exist; `ref_heading_rate_limit_enabled: bool = field(default=False, ...)` confirmed | — |
| 83-97 | §4.1-§4.3b field names (`adaptive_q_scaling_enabled`, `steer_rate_anti_hunt_enabled`, `anti_hunt_boost_max`, `corner_factor_k`, `q_ey_straight/corner`, `q_epsi_straight/corner`, `q_r_straight/corner`, `rrate_steer_straight/corner`, `r_steer_corner_mid`, `low_speed_corner_boost_v_half/max_extra`, `epsi_ra_half_rad/accel_boost_max/brake_floor`) | OK | every single field confirmed present on `MPCParams` | — |
| 99-101 | **§4.3 REMOVED**: `adaptive_r_rate_enable_in_corners`/`adaptive_r_rate_during_floor` no longer exist on `MPCParams` | OK | confirmed absent from current field list; matches recent commit `8b666eb "Remove dead adaptive-R_rate corner-softening mechanism"` and `docs/removed_mechanisms.md §10a` (dated removed 2026-09-29) | — |
| 101 | link `[removed_mechanisms.md]` (relative, i.e. `docs/removed_mechanisms.md`) | OK | file exists at `docs/removed_mechanisms.md`, all referenced anchors (§3,4,5,6,7,8,9,10) exist | — |
| 123-165 | §4.4-4.10 "historical/removed" fields genuinely absent: `use_precomputed_corner_map`, `CornerMap`, `_segment_corners`, `alat_ceiling_flat/_slope/_intercept` on MPCParams, `curvature_forcing_enabled/curvature_forcing_gain` | OK | none of these appear in the current `MPCParams` field list; `alat_ceiling*` moved to `nmpc_core.py`'s `_Plant` / `model/vehicle_physics.py` per doc's own claim, confirmed plausible from control_mechanisms.md cross-refs | — |
| 133-134 | precomputed corner segmentation section correctly separates "removed" from §4.5d's "inactive under NMPC" statement | OK | logically consistent, no contradiction found | — |
| 139-141 | §4.5c: `use_precomputed_heading_profile` is a **node-level launch param, NOT an `MPCParams` field**, default `false` | OK | confirmed: declared directly in `mpc_controller.py` (`('use_precomputed_heading_profile', False)`), absent from `MPCParams` dataclass | — |
| 140 | `HEADING_LEAD_AUTHORITY_FRAC` default 0.5, `SLIP_LIMIT_RAD` 5°, both in `tuner/tools/raceline_optimizer.py`, not `MPCParams` | OK | confirmed exact: `HEADING_LEAD_AUTHORITY_FRAC = 0.5`, `SLIP_LIMIT_RAD = math.radians(5.0)` | — |
| 171 | link to `docs/reference/README.md`'s "Nonlinear MPC (`use_nmpc`)" section | **WRONG** | `docs/reference/README.md` has no such section (only heading is "Where new content belongs"); the actual section lives in `docs/reference/control_mechanisms.md:350` ("Nonlinear MPC (`use_nmpc`): a second controller") | change link target to `control_mechanisms.md#nonlinear-mpc-use_nmpc-a-second-controller` |
| 171 | offline NMPC port is `controller/nmpc/` package | OK (and more current than several sibling docs) | confirmed: `controller/nmpc/{layout,reference,dynamics,outputs,weight_schedule,solver}.py` exist; several *other* docs (`error_state_reference.md`, `fsae_planning_pending_pr.md`, `nmpc.md`, `junior_project_mpc_docs.md`) still say `controller/nmpc_optimiser.py` (a stale single-file name, now only a leftover `.pyc`) — tuning.md is the one that's actually up to date here | no fix needed in tuning.md; flag the other docs separately |
| 176 | "tuning surface is therefore ~6 numbers, not ~56" | UNVERIFIABLE (rough figure) | plausible order-of-magnitude given ~69 total `MPCParams` fields minus the ~14+ inert-under-NMPC adaptive-gain fields, not worth pinning exactly | leave as rough color, not a hard count |
| 178 | NMPC weight overrides "live IN `MPCParams` itself (not a separate `NMPCParams`)" | OK | confirmed: `nmpc_q_e_y`, `nmpc_r_delta`, etc. are fields of `MPCParams` in `mpc_params.py`; `nmpc_params.py`'s `NMPCParams` holds only structural/solver fields (`nmpc_horizon`, `nmpc_sqp_iters`, ...) — this directly contradicts `docs/reference/control_mechanisms.md:450`, which says "**`NMPCParams`**... all 20 fields NMPC-only... `mpc_core.py` never imports or reads this dataclass at all" without qualifying that the *weight* overrides are elsewhere. **control_mechanisms.md is the stale one here, not tuning.md.** | flag control_mechanisms.md:450 for correction (out of this audit's file scope, but load-bearing) |
| 179 | `nmpc_horizon=20`, `nmpc_sqp_iters=1`, `nmpc_solve_budget_ms=25` | OK | all three confirmed exact in `nmpc_params.py` | — |
| 179 | `nmpc_rk_substeps=2` | **STALE** | actual current default is **4**, not 2 (`nmpc_rk_substeps: int = field(default=4, ...)`); matches git commit `40dc25f "Fix a second, distinct NMPC low-speed stall: nmpc_rk_substeps 2 -> 4"` | update to 4, and note the fix reason (low-speed stall) per code-review-checklist "why the earlier attempt failed" convention |
| 179 | `nmpc_jac_substeps=1` ("only sets the SQP step direction, never the prediction") | **STALE** | actual current default is **4**, not 1 (`nmpc_jac_substeps: int = field(default=4, ...)`); matches git commits `87347ca`/`13ab3e7` ("Speed-gate nmpc_jac_substeps to fix the jac=4 steering wobble") — note there are also `nmpc_jac_substeps_fast`/`nmpc_jac_gate_speed` fields now (speed-gated variant) not mentioned at all in tuning.md | update value to 4 and add a line on the speed-gated fast variant |
| 180 | `nmpc_alat_ceiling_enabled=true` default | OK | confirmed `default=True` | — |
| 181 | `nmpc_track_halfwidth=3.5` | **STALE** | actual current default is **3.35**, not 3.5 (`nmpc_track_halfwidth: float = field(default=3.35, ...)`) | update to 3.35 |
| 181 | `nmpc_slack_weight=10000`, `nmpc_curvature_dense_step=0.5`, `nmpc_curvature_smooth_w=3` | OK | all three confirmed exact | — |
| 182 | "Three MPCC-inspired flags, all NMPC-only": `nmpc_spline_reference_enabled` (default true), `nmpc_horizon_speed_profile_enabled` (default false), `nmpc_friction_circle_enabled` (default false) | **WRONG** (partial) | `nmpc_spline_reference_enabled=True` OK; `nmpc_friction_circle_enabled=False` OK; but **`nmpc_horizon_speed_profile_enabled` no longer exists as a field at all** — it was tried and removed (per-stage horizon speed profile), confirmed by `control_mechanisms.md:216` ("...was tried and removed") and by the only surviving reference being a code comment in `nmpc_core.py:813` ("it cannot reproduce the rejected nmpc_horizon_speed_profile_enabled") and `CHANGES.md:259` history. Listing it as a live, disableable flag is actively misleading — a reader could go looking for a checkbox/param that isn't there. | remove this field from the "three flags" list, replace with a note that it was tried and removed (link `removed`/`superseded_mechanisms.md`) |
| 182 | link to `docs/reference/README.md`'s "Three MPCC-inspired additions" subsection and "Which settings affect which controller" | **WRONG** | neither subsection exists in `docs/reference/README.md`; both live in `docs/reference/control_mechanisms.md` (lines 416, 430) | change link target to `control_mechanisms.md#three-mpcc-inspired-additions` / `#which-settings-affect-which-controller` |
| 194 | `R_rate_diag[0]` / `r_rate_delta` = **52.5** (from 2.8) | **STALE** | actual current live default is **100.0** (`mpc_params.py`), and offline `settings.py` also shows `R_rate_diag = [100.0, 2.0]` — both sides agree with each other (parity intact) but disagree with the doc's stated 52.5. Value has clearly moved on since this section was written; §4c's "smoke-test" mechanism note and the whole "steering smoothness" analysis (reversal ratios, DNF thresholds) may be stale relative to the current 100.0 too, not just the single number | update the "52.5 (from 2.8)" line to reflect current value and re-verify/re-date the qualitative claims built on top of it (chatter halved, reversal ratio 4.3x, etc.) since they were measured at 52.5, not 100 |
| 195 | `NMPC_RJERK_DELTA = 150.0` | OK | confirmed exact both in settings.py and mpc_params.py (`nmpc_rjerk_delta: float = field(default=150.0, ...)`) | — |
| 199 | "Untested pairing... `r_rate_delta=5.0` with `NMPC_RJERK_DELTA=250.0`" | UNVERIFIABLE (explicitly labeled untested) | no contradiction; flagged honestly as untested | — |
| 207-211 | `NMPC_RRATE_ZONE_ENABLED=true`, boost/ease/floor = 2.0/0.80/0.15, `NMPC_CORNER_FACTOR_K=27.0` | OK | all confirmed exact in both `settings.py` and `nmpc_params.py`/`mpc_params.py` | — |
| 216 | "`_EASE_APPROACH=0.35` is the intended value and DNFs offline; 0.80 ships instead" | OK (consistent with code comment) | `settings.py` line 757 code comment literally says "(0.35 DNFs offline -- see `docs/reference/`)" confirming this is a documented, intentional deviation | — |
| 222 | `CURVATURE_SPEED_A_LAT_MAX` 5.5 → 4.75 | not independently re-verified (numeric-history claim) | not re-run per instructions (no rerun requested); a reproducible source exists only if this constant's current value is checked | recommend a quick grep of current `CURVATURE_SPEED_A_LAT_MAX` value to confirm 4.75 is still current, not done in this pass — **flag as needing a fast follow-up check**, since other numeric constants in this same doc (r_rate_delta, nmpc_track_halfwidth, rk/jac substeps) were all found stale |
| 250-254 | measured saturation numbers for `rjerk_delta=150`/`r_rate_delta=52.5` (0 saturated ticks, 3 laps) | now **internally inconsistent** | this measurement is tied to `r_rate_delta=52.5`, but the *current* shipped `r_rate_delta` is 100.0 (see line 194 finding) — the doc doesn't flag that this specific number was measured at an since-superseded weight | note explicitly that this figure was measured at the historical r_rate_delta=52.5 and has not been re-measured at 100.0, or re-measure |
| 327-334 | `tuner/offline_tuner.py` searches **14** parameters: `Q_diag` 5, `R_diag` 2, `R_rate_diag` 2, `TUNABLE_NMPC` 5 | OK | confirmed exactly: `TUNABLE_Q_IDX`=5 entries, `TUNABLE_R_IDX`=2, `TUNABLE_R_RATE_IDX`=2, `TUNABLE_NMPC`=5 list entries (`rjerk_delta`, `corner_factor_k`, `rrate_zone_boost_straight`, `_ease_approach`, `_floor_corner`) — 5+2+2+5=14 | — |
| 338 | "Gate: NMPC block only does anything when `settings.USE_NMPC` is True, defaults to False" | OK | `USE_NMPC = False` confirmed | — |
| 356-360 | §5: `enable_dynamic_speed_cap`/`dynamic_cap_a_lat_max`/`dynamic_cap_safety` — "ENABLE_DYNAMIC_SPEED_CAP defaults to True in both settings.py and mpc_params.py" | **partially imprecise** | `ENABLE_DYNAMIC_SPEED_CAP=True` confirmed in `settings.py`. BUT these three fields are **NOT** `MPCParams` dataclass fields — they're declared directly as node-level ROS2 params inside `mpc_controller.py` (`declare_parameters([('enable_dynamic_speed_cap', True), ...])`), same pattern as `use_precomputed_heading_profile`. Saying "defaults to True... in mpc_params.py" implies a dataclass field that doesn't exist there. | reword to "...defaults to True in `settings.py` and as a `mpc_controller.py` node parameter (not an `MPCParams` field)" |
| 360 | `ros2/launch_all.sh` overrides the flag to `false` at runtime | OK | confirmed: `ros2/launch_all.sh:447` `ENABLE_DYNAMIC_SPEED_CAP=false`, and `DYNAMIC_CAP_A_LAT_MAX=3.2`/`DYNAMIC_CAP_SAFETY=0.9` both match `mpc_controller.py`'s and `settings.py`'s defaults (parity intact even though the flag itself is overridden off) | — |
| 360 | link "Full writeup: `docs/logs/late_turn_in_investigation.md`'s 'Dynamic speed cap' addendum" | OK (loose paraphrase, not a broken link) | actual heading is `## Addendum (2026-08-11): dynamic speed cap — closing the gap between the oracle profile and live tracking` at line 2912 — not a markdown link, just prose, and the paraphrase is close enough to find it | — |
| 366 | `sim/scoring.py` source of truth; live copy at `ros2/src/fsae_planning/control/fsae_control/fsae_control/scoring.py` | OK | live file confirmed present at exactly that path | — |
| 371 | 13 metrics listed (`rmse`, `yaw_rms`, ... `accel_reversal_rms`) | not independently re-verified field-by-field against `sim/scoring.py`'s docstring in this pass, but count and general shape (13 metrics, 2 reversal-RMS) is consistent with `METRIC_SCALES`/`SCORE_WEIGHTS` both existing in settings.py | OK (moderate confidence) | — |
| 366 | `METRIC_SCALES`, `SCORE_WEIGHTS` exist in settings.py | OK | both confirmed present (`METRIC_SCALES = np.array(...)` line 1236, `SCORE_WEIGHTS = np.array(...)` line 1343) | — |
| 381-384 | §7: `N_HORIZON`, `DELAY_STEPS`, `DELAY_JITTER_STEPS`, `SLAM_NOISE_ENABLED`, `SLAM_POS_JITTER_STD`/`SLAM_YAW_JITTER_STD` in settings.py | OK | all confirmed present (`N_HORIZON=35`, `DELAY_STEPS=1`, `DELAY_JITTER_STEPS=0.2`, `SLAM_NOISE_ENABLED=False`, `SLAM_POS_JITTER_STD=0.02`, `SLAM_YAW_JITTER_STD=radians(0.3)`) | — |
| 392-397 | §8 doc-pointer list: `architecture.md`, `offline_guide.md`, `fsds/fsds_integration_guide.md`/`fsds_settings.md`, `docs/reference/`, `docs/logs/sim_to_real_investigation.md`, `junior_project_mpc_docs.md` | OK | all files exist at stated paths | — |

**Overlap with other docs / canonical-source claim**: tuning.md's opening line claims to be *the* canonical
tuning reference and that other docs "link here instead of repeating this material." Spot-checked:
`docs/reference/control_mechanisms.md` **does** restate several exact numeric values inline (e.g. line 145
in the offline/live parity table quotes `nmpc_corner_factor_k` `27.0`, `nmpc_q_e_y` `7.5`, zone
`2.0`/`0.80`/`0.15`, `rjerk_delta` `150.0`) rather than only linking to tuning.md — this is a second place
these same numbers can drift independently of tuning.md, and in fact `docs/reference/offline_live_parity.md`
is the file that currently has the more complete/other set of currently-shipped numbers, not tuning.md
(tuning.md's stale 52.5/2/1/3.5 above sit right next to `offline_live_parity.md`'s already-correct 150.0
mentions). `architecture.md` was not exhaustively checked line-by-line in this pass but is referenced heavily
and would be worth a similar sweep. **The claim of sole canonicity is not fully honored in practice.**

**Readability**: dense, but that's largely intentional (research-codebase tuning reference, not a tutorial).
Section 4's numbering (4.1, 4.2, 4.3, 4.3b, 4.4, 4.5, 4.5b, 4.5c, 4.5d, 4.6, [no 4.7], 4.8, 4.9, 4.10) is
irregular and easy to mis-navigate — 4.7 is skipped entirely with no note why, and the historical entries
are interleaved with live ones (4.5c is LIVE-ONLY, sandwiched between three REMOVED sections). A future
restructure should separate "still tunable today" fields from "historical, removed" into two clearly
distinct groups rather than interleaving them by original discovery order.

**Mechanism without WHY** (recoverable from docs/logs or git log): §4.5d's rk/jac_substeps values are
stated as fact ("needed because tau_a is stiff...") without noting they've each changed at least once
for a *different* reason each time (a low-speed stall, then a steering wobble) — both explanations exist in
git log commit messages (`40dc25f`, `87347ca`/`13ab3e7`) but not in this doc. Worth pulling into the doc per
CLAUDE.md's own "when a fix corrects an earlier wrong attempt, say why the earlier attempt failed" rule.

**Code paths/modules mentioned** (for a future restructure): `mpc_params.py`, `nmpc_params.py`, `mpc_core.py`,
`nmpc_core.py`, `mpc_controller.py`, `settings.py`, `sim/rollout_core.py`, `controller/model_utils.py`,
`controller/nmpc/` (`layout.py`, `reference.py`, `dynamics.py`, `outputs.py`, `weight_schedule.py`,
`solver.py`), `tuner/offline_tuner.py`, `tuner/tools/raceline_optimizer.py`, `tuner/steering_chatter_check.py`,
`tuner/checks/ref_heading_limiter_suite_check.py`, `sim/scoring.py`, live `scoring.py`, `control_utils.py`,
`telemetry_logger.py`.

**Proposed action: KEEP, fix stale numbers + wrong links, then re-run the "measured" claims.** This doc is
correctly scoped and its structure is sound; the problems are (a) 4 stale numeric defaults
(`r_rate_delta`, `nmpc_rk_substeps`, `nmpc_jac_substeps`, `nmpc_track_halfwidth`), one of which
(`nmpc_horizon_speed_profile_enabled`) is not stale but flatly **wrong** (field no longer exists), and
(b) 2 dead links to `docs/reference/README.md` sections that actually live in `control_mechanisms.md`. None
of this rises to "restructure" — it's a straightforward resync pass, best done with
`tuner/tools/sync_mpc_params.py`-style discipline (read the live dataclass field-by-field, don't trust the
doc's own memory of a number).

---

## Summary of highest-value findings

1. **`nmpc_horizon_speed_profile_enabled` does not exist** (tuning.md:182) — the field was removed after being
   tried and rejected; the doc lists it as a live, disableable flag. This is the single most likely to send
   someone chasing a nonexistent knob.
2. **`r_rate_delta` is 100.0 live/offline today, not 52.5** (tuning.md:194) — and several qualitative
   "measured" claims in the surrounding paragraphs (§4b) were measured at the old value and have not been
   re-validated at the current one.
3. **`nmpc_rk_substeps=4` (not 2) and `nmpc_jac_substeps=4` (not 1)** (tuning.md:179) — both changed via
   documented, reasoned commits (a low-speed stall fix, then a steering-wobble fix) that aren't reflected here.
4. **`nmpc_track_halfwidth=3.35` (not 3.5)** (tuning.md:181) — minor numeric drift.
5. **Two dead links** to `docs/reference/README.md`'s "Nonlinear MPC"/"Three MPCC-inspired additions"/"Which
   settings affect which controller" sections (tuning.md:171, 182) — those sections live in
   `docs/reference/control_mechanisms.md` instead. `README.md` is now just an index page.
6. Confirms the CLAUDE.md/memory claim that `NMPCParams` in `control_mechanisms.md:450` ("all 20 fields
   NMPC-only, `mpc_core.py` never imports this dataclass") is itself the stale side — tuning.md's claim that
   NMPC weight overrides live inside `MPCParams` is the currently-correct one. This is a cross-doc
   contradiction worth resolving in `control_mechanisms.md`, not tuning.md.
7. offline_guide.md is in much better shape: only one stale path found (missing `/mpc/` path segment,
   line 140); everything else checked (10 synthetic paths, track file paths, tuner/ layout table,
   dependency claims, function signatures, anchors) is accurate.
8. tuning.md's "sole canonical source" claim is not fully honored: `docs/reference/offline_live_parity.md`
   independently restates several of the same numbers (and currently has *more accurate* ones than
   tuning.md for the exact fields found stale above).
