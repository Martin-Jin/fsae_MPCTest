# Audit: reference_path_and_speed.md, simulator_fidelity.md, simulator_glossary.md, superseded_mechanisms.md

All paths below are under `/home/Formula-Student-Driverless-Simulator/` unless stated. Verified against on-disk state 2026-09-29, using `ls`/`find`/`grep`/read-only `git` only.

---

## 1. `fsae_MPCTest/docs/reference/reference_path_and_speed.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 9-11 | related docs: `docs/fsds/fsds_integration_guide.md`, `docs/tuning.md`, `docs/reference/offline_live_parity.md` | OK | all three exist at those paths | — |
| 17-19 | `speed_profile.csv`←`tuner.tools.export_speed_profile`; `raceline.csv`/`centerline.csv`←`tuner.tools.raceline_optimizer` (`--mode centerline`) | OK | `tuner/tools/export_speed_profile.py`, `tuner/tools/raceline_optimizer.py` exist; `--mode` arg present | — |
| 21 | all three files live in `ros2/src/fsae_planning/tracks/<track>/` alongside `cone_map.json` | OK | `find` confirms `comp_test_map_3/{speed_profile.csv,raceline.csv,centerline.csv,cone_map.json,speed_profile_corner_test.csv}` present | — |
| 46 | `_newest_track` reimplemented in bash in `launch_all.sh`, "matching...`newest_track()`" | OK | `launch_all.sh` defines `_newest_track()` (line 65); `tracks/__init__.py` documents same "newest mtime" rule | — |
| 47 | `_track_geometry_name` prefers centerline over raceline, matching `tracks.geometry_path()` | OK (name exists) | `_track_geometry_name` defined at launch_all.sh:84; did not independently re-derive `tracks.geometry_path()`'s exact logic byte-for-byte (low-value re-check) | — |
| 52 | new `TRACK=` dated as `<name>_<YYYYmmdd>`, matches `tracks.dated_track_name()` | OK | `launch_all.sh:861` does `TRACK="${TRACK}_$(date +%Y%m%d)"`; `tracks/__init__.py` docstring confirms same scheme | — |
| 55 | `export_speed_profile`/`raceline_optimizer` both accept `--no-overwrite`, default overwrite=True | OK | both files: `"--no-overwrite", dest="allow_overwrite", action="store_false", default=True` | — |
| 62-68 | toggle resolution happens in `control.launch.py` via `IfElseSubstitution`→`effective_map_path`/`effective_path_map_path`; `stanley_controller.py` has no `use_precomputed_speed/path` param and never checks either toggle; same for MPC nodes | OK | `control.launch.py:89-95` defines both `IfElseSubstitution`s and passes them at lines 362-363/390-391; `stanley_controller.py` declares raw `map_path`/`path_map_path` params only, no `use_precomputed_*`; `grep` for `use_precomputed_speed`/`use_precomputed_path` across `fsae_control/` returns nothing (only an unrelated `use_precomputed_heading_profile`, a real, different, node-level param) | — |
| 82-93 | `compute_speed_profile()` in `sim/speed_profile.py`; 3-pass table; formulas | OK | function exists at `sim/speed_profile.py:220`; `curvature_speed()` at line 501 with `a_lat_max=CURVATURE_SPEED_A_LAT_MAX` default matching `CURVATURE_SPEED_A_LAT_MAX = 4.75` (line 112) and 24 m scan (`scan_end=24.0`) | — |
| 93 | `a_accel_max`/`a_brake_max` planning values 7.0/−5.0 vs plant 12.0/−7.0 | UNVERIFIABLE (not spot-checked numerically this pass) | plausible given surrounding code; not independently re-derived | low priority; verify against `sim/speed_profile.py` constants and `model/vehicle_physics.py` accel/brake caps if doc is edited next |
| 97-104 | `tuner/tools/raceline_optimizer.py` two modes (`raceline`/`centerline`), Kegel-style min-curvature search, cone-clearance check before writing | OK | `optimize_raceline(..., lateral_offsets=True/False)` confirmed at line 493; `lateral_offsets=False` path documented in the file's own docstring ("exports the CENTRELINE instead") | — |
| 119-131 | `--corner-slowdown KAPPA` → `compute_corner_slowdown_profile()`; `min()` clamp not overwrite; re-runs fwd/back passes; writes `speed_profile_corner_test.csv`; `0.10` flags 9 zones on `comp_test_map_3` | mostly OK, one UNVERIFIABLE | `compute_corner_slowdown_profile` exists (`sim/speed_profile.py:382`); CLI flags `--corner-slowdown`/`--corner-speed` confirmed in `export_speed_profile.py`; `speed_profile_corner_test.csv` **currently present on disk** as an untracked file in `comp_test_map_3/` (consistent with doc); the "9 corner zones at 0.10" / "p75=0.079" figures not re-run (per task instructions, no rerun) | — |
| 137, 144, 154-159 | 5-column `x,y,psi,psi_target,v_target` CSV for raceline/centerline; `speed_profile.csv` differs (4-column) | OK | header rows confirmed: `speed_profile.csv` = `# x,y,psi,v_target` (4 cols); `raceline.csv`/`centerline.csv` = `# x,y,psi,psi_target,v_target` (5 cols) — doc's text says "5-column...that the live controller consumes identically" for **both exporter outputs**, correctly distinguishing from the 4-col speed file | — |
| 148-161 | scored table: composite 0.752 (raceline) vs 0.488 (centreline), lap time 54.50/51.34s, etc. | UNVERIFIABLE (not rerun) | no rerun per task rules; matches user's own MEMORY.md note "centerline.csv scores 0.488 vs raceline 0.752" (2026-08-24), so internally consistent with project memory | — |
| 168 | `_candidate_score`'s `CURVATURE_SOFT_MAX` penalty thresholds absolute curvature (0.22) | OK | `CURVATURE_SOFT_MAX = 0.22` at `raceline_optimizer.py:205`; `_candidate_score` at line 358 confirmed to use `np.abs(kappa).max() - CURVATURE_SOFT_MAX` | — |
| 178 | `CURVATURE_SPEED_A_LAT_MAX` in `sim/speed_profile.py`, live `curvature_speed()` default too | OK | confirmed in both `sim/speed_profile.py` and live+mirror `control_utils.py` (`a_lat_max=4.75` default) | — |
| 191 | `raceline.csv`/`centerline.csv` use `alat_ceiling_at(v) × ALAT_MARGIN` (0.85) from `model/vehicle_physics.py` | OK | `ALAT_MARGIN = 0.85` at `tuner/tools/raceline_optimizer.py:180`; used at line 671 as `params.alat_ceiling_at(vi) * ALAT_MARGIN` | — |
| 195 | current files read 6.00-18.00 (`speed_profile.csv`) and 5.54-16.70 (`centerline.csv`) on `comp_test_map_3` | **STALE** | actual current on-disk `speed_profile.csv` v_target range is **5.9984 - 16.5**, not 6.00-18.00 (max is wrong by 1.5 m/s); `centerline.csv` range 5.5361-16.7 matches (5.54-16.70 OK). Track file has uncommitted local git changes (`git status` in `fsae_planning` shows `M tracks/comp_test_map_3/speed_profile.csv`), consistent with a re-export since the doc was last written | update the 18.00 figure to 16.5 (or re-verify after next export and note it can drift with re-tuning) |
| 203-219 | 5.5→4.75 steering-smoothness table, corner s0≈43→46, curvature ramp specifics | UNVERIFIABLE (not rerun) | numbers plausible, no reproducible log path cited in the doc itself to check against; recommend the doc cite the source CSV/log if one exists | cite backing log/run if available |
| 223-238 | `launch_all.sh`'s Docker/non-Docker branches both derive `map_path`/`path_map_path` from `$(basename "$SPEED_CSV"/"$PATH_CSV")` re-rooted at `$CONTAINER_TRACK_DIR` | OK | confirmed lines 885 (Docker) and 892 (non-Docker) in `launch_all.sh` | — |

**Purpose/audience:** operational reference for anyone changing which track/line/speed file the car drives, aimed at a session doing track/tuning work, not a first-read doc.

**Overlap:** Its "Related documents" list already correctly points to `offline_live_parity.md` for parity rules and `tuning.md` for weights — no duplication found with `simulator_fidelity.md`/`simulator_glossary.md`/`control_mechanisms.md` (skimmed headings only, per scope). `docs/reference/README.md`'s own table description of this doc ("the three-pass speed profile, the raceline/centreline exporters, and how to switch either") matches its actual content well.

**Readability:** Good bullet/table structure throughout, "Plain version" callouts used consistently and correctly (per CLAUDE.md's writing-style rule). One heading duplicates content unnecessarily: "Which file the car actually reads" (line 106) restates the launch-arg table already given in "The two files, and which does what" (line 13); could be merged/cross-referenced instead of re-tabling.

**Why-gaps:** Line 31 ("speed file and geometry file deliberately describe different lines... pointing both at the same file regresses the car badly") defers the "why" to `launch_all.sh`'s own comment — recoverable, not a real gap. The mechanism at line 176-197 (`CURVATURE_SPEED_A_LAT_MAX` being live-relevant despite driving `centerline.csv`) is explained in full; no gap found.

**Proposed action:** Keep. Update the stale `18.00` v_target figure (line 195) at next edit; consider trimming the duplicate "Which file the car actually reads" table.

**Code paths/modules mentioned (live + offline):**
- `tuner.tools.export_speed_profile` / `tuner/tools/export_speed_profile.py`
- `tuner.tools.raceline_optimizer` / `tuner/tools/raceline_optimizer.py` (`optimize_raceline`, `_candidate_score`, `CURVATURE_SOFT_MAX`, `ALAT_MARGIN`)
- `sim/speed_profile.py` (`compute_speed_profile`, `compute_corner_slowdown_profile`, `curvature_speed`, `CURVATURE_SPEED_A_LAT_MAX`)
- `model/vehicle_physics.py` (`alat_ceiling_at`)
- `ros2/launch_all.sh` (`_newest_track`, `_track_geometry_name`, `TRACK`, `SPEED_CSV`, `PATH_CSV`, `USE_DOCKER`, `CONTAINER_TRACK_DIR`)
- `fsae_MPCTest/tracks/__init__.py` (`TRACKS_DIR`, `newest_track()`, `dated_track_name()`, `geometry_path()`)
- `ros2/src/fsae_planning/common/fsae_bringup/launch/control.launch.py` (`effective_map_path`, `effective_path_map_path`)
- `ros2/src/fsae_planning/control/fsae_control/fsae_control/stanley_controller.py`
- `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_controller.py`, `mpc_controller_standalone.py`
- `ros2/src/fsae_planning/tracks/<track>/{cone_map.json,speed_profile.csv,raceline.csv,centerline.csv,speed_profile_corner_test.csv}`

---

## 2. `fsae_MPCTest/docs/reference/simulator_fidelity.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 3 | links to `simulator_glossary.md` | OK | file exists, anchor-free link resolves | — |
| 7 | "Full history in `docs/logs/sim_to_real_investigation.md`" | OK | file exists at that path | — |
| 15 | `SLAM_NOISE_ENABLED` default off | OK | `settings.py:230` `SLAM_NOISE_ENABLED = False` | — |
| 16 | `CONE_NOISE_ENABLED` default off | OK | `settings.py:296` `CONE_NOISE_ENABLED = False` | — |
| 17-19 | `pose_rate` default 20 Hz (was 10 Hz), `cone_rate` default 10 Hz | OK | `sim_perception.py`: `('pose_rate', 20.0)`, `('cone_rate', 10.0)`, fallback logic matches; warning fires `if pose_rate < 20.0` | — |
| 19 | steering slew `du_max`, 180 deg/s, "now on both sides" | OK (name/value found; "both sides" not independently re-diffed this pass) | value referenced consistently elsewhere (`control_mechanisms.md` heading "Slew-rate limit (`du_max`): on both sides, at 180 deg/s") | — |
| 24-38 | measurement-rate section: 50.5% frozen ticks, 9.9 Hz effective rate, specific CSV filename `mpc_standalone_control_1785976976.csv` | UNVERIFIABLE (log-backed, not rerun) | filename plausible per `fsae_logs/` naming scheme (`mpc_standalone_control_*.csv`, per CLAUDE.md); not found in this checkout (fsae_logs is gitignored/unbounded per CLAUDE.md, expected to not persist) | note in doc that source log may not persist locally (already implicit) |
| 32 | node logs a warning if `pose_rate < 20` | OK | confirmed at `sim_perception.py:156-158` | — |
| 40-64 | delay-jitter section, `settings.DELAY_JITTER_STEPS` default `0.2` | not independently re-verified this pass (low priority, name plausible) | — | — |
| 66-77 | steering saturation 4.8%/21.1%, e_psi 6.9/18.5 vs 15.9/42.0, a_lat max 11.24/12.34 | UNVERIFIABLE (not rerun); consistent with CLAUDE.md's own copy of this table | CLAUDE.md's "does not yet fully predict the car" section carries the identical current numbers (4.8%, 21.1%, 6.9/18.5, 15.9/42.0, 11.24, 12.34) | matches source of truth, OK by cross-reference |
| 86-95 | ceiling table: `alat_ceiling`=7.5 ("measured settled lateral acceleration"), `alat_ceiling_mode`='pi', `gain`=450, `tau`=0.40, `enabled`=True | **STALE** | `model/vehicle_physics.py` confirms `alat_ceiling=7.5` (now documented in-code as a **"Low-speed floor"**, not a flat setpoint), `alat_ceiling_mode='pi'`, `alat_ceiling_gain=450.0`, `alat_ceiling_tau=0.40`, `alat_ceiling_enabled=True` — BUT the code now also has `alat_ceiling_slope=0.47` and `alat_ceiling_intercept=2.46` making `alat_ceiling_at(vx)` **speed-dependent**: `max(7.5, 2.46 + 0.47*vx)`. This is a real model change (commit `6279c72 "Make alat_ceiling speed-dependent using the sweep's fit (S37)"` in `fsae_MPCTest` git history) that the doc's table does not reflect at all | add `alat_ceiling_slope`/`alat_ceiling_intercept` rows to the table; change "measured settled lateral acceleration" description of `alat_ceiling` to "low-speed floor" |
| 96 | "This closes part of the gap...but not the steering-saturation gap...residual narrowed to rate of entry...reference-heading lead is the next avenue" — implicitly still treats the ceiling as NOT speed-dependent (echoing the still-open item from CLAUDE.md) | **STALE / CONTRADICTS CODE** | `docs/logs/sim_to_real_investigation.md` §37 "Ceiling made speed-dependent, using the sweep's fit, validated against both checks" (line 2849) documents this exact fix as done, and the code confirms it is shipped. The doc's own "Still open" framing for speed-dependence (which CLAUDE.md also states almost verbatim: "the measured ceiling is speed-dependent and the model is not... Left unfitted on purpose") is now **out of date relative to the codebase**; CLAUDE.md itself may also need this update but that's out of this doc's scope | rewrite this section to state the ceiling is now speed-dependent (§37), and re-scope the "still open" residual to whatever §37 itself says remains (the reference-heading lead literature is presumably still separately open — verify against §37's own conclusion before rewriting) |
| 98 | repro command `python -m tuner.recorded_map_rollout [--mode p --gain 700 \| --tau 0.25 \| --no-ceiling]` | OK | all three flags (`--mode`, `--gain`, `--tau`, `--no-ceiling`) exist in `tuner/recorded_map_rollout.py`; `700`/`0.25` are the deliberately-historical values (old proportional law), consistent with CLAUDE.md's own account of the tuning history, not a live default being misquoted | — |
| 100-110 | `fsds_bridge.py` discards `a_cmd`, re-derives throttle via P-loop; `KP_THROTTLE`, `STICTION_KICK_SPEED=1.0`, `STICTION_KICK_THROTTLE=0.35` | OK | all three constants confirmed in `ros2/src/fsae_planning/control/fsae_control/fsae_control/fsds_bridge.py` with exact quoted values | — |
| 112-134 | centreline curvature-spike defect: workarounds in `curvature_speed()` (both `control_utils.py` and `sim/speed_profile.py`), `tracking_error_speed_gate()`, `GATE_RATE_LIMIT`, `SPEED_TARGET_RISE_RATE=7.0`, `SPEED_TARGET_DEFICIT_MAX=5.0` | OK (spot-checked `curvature_speed` presence in both; other constants not individually re-grepped this pass but consistent with MEMORY.md's "DEFICIT_MAX was the real accel ceiling" note, which independently confirms `SPEED_TARGET_DEFICIT_MAX` raised to 5.0) | — | — |
| 118 | min turn radius "≈3.7 m" at 25° lock, 1.55 m wheelbase | not independently recomputed | plausible (tan(25°)≈0.466, 1.55/0.466≈3.33, close but not exact to 3.7 — could include a track/CG correction not visible from the raw formula) | low priority, flag as UNVERIFIABLE rather than WRONG since bicycle-model turn radius formulas vary by convention |
| 130 | `fsae_autonomous/docs/NMPC_INTEGRATION_GAPS.md` gap E8 | not independently checked (file lives in a different repo not covered by this audit's primary paths; per task scope this is a secondary reference) | — | — |
| 146-150 | `_absorb()` bug: upstream `fsae_planning`'s version **still** appends same-frame duplicates; both `fsae_MPCTest`'s offline `planning/cone_map.py` and the `fsds_simulator` mirror's `cone_map.py` "already" check candidates against each other; "**Not ported upstream**" | **WRONG** | Direct read of `ros2/src/fsae_planning/planning/fsae_planning/fsae_planning/cone_map.py::_absorb()` shows the **same cross-candidate check already present** (the `if new_pts: ... new_dists ...` block), functionally identical to the offline/mirror copies — `diff` between offline and live copies shows only a two-line **comment** difference, no logic difference. The doc's claim that the live copy "still carries the same-frame-duplicate-prone version" and that porting the fix "is a resync TODO, not something this repo can apply directly" is factually wrong as of current disk state: **the fix is already present in `fsae_planning` live.** | Rewrite this subsection: state the fix has been ported/is present in all three copies (offline `planning/cone_map.py`, `fsds_simulator` mirror, and live `fsae_planning`), and remove the "not ported upstream" TODO framing, or re-verify with the user whether a most-recent local edit did this (git log for `fsae_planning`'s `cone_map.py` shows last real commit `d34ee79` on 2026-08-17, well before this doc's most recent edits on 2026-09-28/29, so the doc had every opportunity to reflect this and the "not ported" claim looks like it was simply never re-checked against disk) |
| 154-156 | `blend_paths()` used by both `centerline_planner.py` and `sim/sim_track.py`, `alpha=0.4`, `reset_dist=2.0` | OK | both call sites confirmed (`centerline_planner.py:165` live+mirror, `sim/sim_track.py:282` offline); defaults `alpha=0.4, reset_dist=2.0` confirmed in `path_utils.py:312-313` (present identically in offline, mirror, and live copies) | — |
| 158-162 | cone geometry section, reference to `sim_to_real_investigation.md`'s "## 11. Also verified..." | not independently re-checked this pass (low priority, doc-to-doc pointer only) | — | — |

**Purpose/audience:** the single source-of-truth doc for "does an offline/FSDS result predict the real car," aimed at anyone about to trust a tuning result. Correctly scoped and the most safety-critical of the four docs audited (a stale claim here directly risks re-litigating an already-closed investigation, exactly the failure CLAUDE.md warns about).

**Overlap:** Cleanly scoped per `docs/reference/README.md`'s table ("sim-vs-car discrepancy"). No unwanted duplication found with `simulator_glossary.md` (which explicitly defers gap detail here) or `superseded_mechanisms.md` (different subject: removed features, not measured gaps). Some internal near-duplication with CLAUDE.md itself (the ceiling table, the saturation-gap numbers) is expected and intentional per CLAUDE.md's own framing ("Full history in..." pointers), not a doc-hygiene problem.

**Readability:** Good structure, correctly uses "Plain version" callouts, tables for multi-quantity data. No heading misuse found (single H1, consistent H2/H3 nesting). One structural nit: the `>` blockquote-styled sub-findings under "Known planner defect" (lines 140, 144, 158+ in the raw file text) mix blockquote and heading conventions inconsistently with the rest of the doc — readable but a different visual register than the surrounding prose.

**Why-gaps:** None of consequence found — every mechanism here states its "why" (e.g., why integral not proportional, why 0.40s not 0.25s, why the stiction floor is needed). This doc is a model for the "why" discipline CLAUDE.md asks for.

**Proposed action:** Keep, but this is the **highest-priority doc to fix** of the four: it currently misstates (a) the ceiling's speed-dependence as unmodelled when it has been fitted (§37), and (b) the `_absorb()` cone-duplication bug as still unfixed upstream when it is not. Both are exactly the kind of stale claim CLAUDE.md warns could cause "re-discovering, re-litigating, or accidentally re-fixing" a settled question — recommend fixing both before the doc is read again for planner/cone-map work.

**Code paths/modules mentioned (live + offline):**
- `sim_perception.py` (`pose_rate`, `cone_rate`) — `ros2/src/fsae_planning/perception/fsae_sim_perception/fsae_sim_perception/sim_perception.py`
- `predict_ahead()`, `DELAY_JITTER_STEPS`, `POSE_HOLD_*`, `PoseFeedHold` — `settings.py` / live control nodes
- `mpc_controller.py`, `mpc_controller_standalone.py` (`/fsae/slam/car_odom` subscription)
- `sim/rollout_core.py`
- `model/vehicle_physics.py` (`alat_ceiling`, `alat_ceiling_mode`, `alat_ceiling_gain`, `alat_ceiling_tau`, `alat_ceiling_enabled`, `alat_ceiling_slope`, `alat_ceiling_intercept`, `alat_ceiling_at()`)
- `ros2/run_steering_sysid.sh`, `ros2/run_steering_step.sh`
- `tuner.recorded_map_rollout` / `tuner/recorded_map_rollout.py`
- `tuner/checks/plant_openloop_validation.py`, `tuner/checks/live_vs_sim_diagnostics.py`
- `fsds_bridge.py` (`KP_THROTTLE`, `STICTION_KICK_SPEED`, `STICTION_KICK_THROTTLE`) — live `ros2/src/fsae_planning/control/fsae_control/fsae_control/fsds_bridge.py` and `fsds_simulator` mirror copy
- `centerline_planner.py`, `boundary.py`, `cone_sorting.py` (`filter_cones_window`, `min_ahead=0.5`), `path_utils.py` (`smooth_centreline`, `pin_start`, `blend_paths`, `reset_dist`)
- `control_utils.py` / `sim/speed_profile.py` (`curvature_speed()`)
- `tracking_error_speed_gate()`, `GATE_RATE_LIMIT`, `SPEED_TARGET_RISE_RATE`, `SPEED_TARGET_DEFICIT_MAX`
- `cone_map.py` (`_absorb()`, `MERGE_DIST=0.8`) — offline `planning/cone_map.py`, `fsds_simulator` mirror, live `fsae_planning/planning/.../cone_map.py`
- `fsae_autonomous/docs/NMPC_INTEGRATION_GAPS.md` (external repo reference, gap E8)

---

## 3. `fsae_MPCTest/docs/reference/simulator_glossary.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 9 | FSDS lives at "Outer repo root (`Formula-Student-Driverless-Simulator`)" | OK | matches CLAUDE.md's repo-layout table | — |
| 9 | FSDS "used mainly to test and validate new features before they're trusted on the real car" | OK, matches CLAUDE.md's "Repo roles" section almost verbatim | — | — |
| 10 | Offline rollout = `sim/rollout_core.py` driving `model/vehicle_physics.py`'s 24-state nonlinear plant; run by `tuner/offline_tuner.py` (CMA-ES) and `tuner.recorded_map_rollout` | OK | `sim/rollout_core.py` exists, `run_core_rollout()` defined there; `model/vehicle_physics.py` docstrings elsewhere describe a 24-state plant (not independently recounted, but consistent with all other doc references); `tuner/offline_tuner.py` docstring literally says "Offline MPC Weight Optimisation via Surrogate-Assisted CMA-ES" and references `cma` library / BIPOP+lq-CMA-ES | — |
| 11 | 2D GUI = `gui/simulation.py`, "calls the same `rollout_core.run_core_rollout()`... physics fidelity is identical" | OK | `gui/simulation.py` imports `from sim.rollout_core import run_core_rollout, compute_step_budget` and calls `run_core_rollout(...)` at line 724; `sim/rollout_core.py`'s own module docstring states "tuner/offline_tuner.py and gui/simulation.py call run_core_rollout() and only differ in..." — directly confirms the claim | — |
| 21 | steering saturation 4.8% (offline) vs 21.1% (car), cross-referenced to CLAUDE.md and `simulator_fidelity.md` | OK | matches both target docs' current numbers | — |
| 26-29 | ordering diagram `2D GUI ≈ offline rollout < FSDS < real car` | OK, consistent with rest of doc's claims | — | — |
| 37-42 | "Which doc do I want" table: links to `../offline_guide.md`, `../fsds/fsds_integration_guide.md`, `../architecture.md`, `../tuning.md`, `../fsds/fsds_settings.md`, `../fsds/fsds_ros_integration.md`, `simulator_fidelity.md` (twice), `offline_live_parity.md` (twice), `../lmpc.md`, `../nmpc.md`, `../error_state_reference.md` | OK — all resolve | every listed path confirmed present via `ls` | — |

**Purpose/audience:** short disambiguation doc so other docs can say "the simulator" unambiguously; audience is anyone new to the repo confused by "FSDS" vs "offline sim" vs "2D GUI." Very tightly scoped, does exactly what it says (only points, doesn't restate).

**Overlap:** By design, zero content overlap with the other three docs; it exists specifically to be the single disambiguation point they all defer to (`simulator_fidelity.md` line 3 explicitly links here first). This is the cleanest-scoped of the four docs.

**Readability:** Excellent: short, one clear table for the three things, one clear "how trustworthy" section, one routing table. No issues found.

**Why-gaps:** None; a routing/glossary doc doesn't need mechanism "why," and it doesn't claim to have any.

**Proposed action:** Keep as-is. No changes needed. This is the doc other three should be modeled after for concision.

**Code paths/modules mentioned (live + offline):**
- `sim/rollout_core.py` (`run_core_rollout()`, `compute_step_budget`)
- `model/vehicle_physics.py`
- `tuner/offline_tuner.py`
- `tuner.recorded_map_rollout`
- `gui/simulation.py`
- (routing only, not "mentioned as code," but pointed at:) `docs/offline_guide.md`, `docs/architecture.md`, `docs/tuning.md`, `docs/fsds/fsds_settings.md`, `docs/fsds/fsds_ros_integration.md`, `docs/fsds/fsds_integration_guide.md`, `docs/lmpc.md`, `docs/nmpc.md`, `docs/error_state_reference.md`, `docs/reference/simulator_fidelity.md`, `docs/reference/offline_live_parity.md`

---

## 4. `fsae_MPCTest/docs/reference/superseded_mechanisms.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 7 | "A larger set of removals...has its own document: `docs/removed_mechanisms.md`" | OK | file exists at `fsae_MPCTest/docs/removed_mechanisms.md`, covers "The Lookahead Gain-Scheduling Family," 11 numbered sections | — |
| 9-13 | Exit-heading boost (`_lookahead_exit_boost`/`_update_lookahead_peak`/`dist_since_peak`) "no longer exists on either side"; replaced by corner-factor scheduler; see `control_mechanisms.md`'s "Corner-factor scheduler" section and `late_turn_in_investigation.md` | OK (removal confirmed) | `grep -rl` for all three symbol names across both `fsae_MPCTest` and `ros2/src/fsae_planning` returns **zero matches** — genuinely gone from both codebases; `control_mechanisms.md` does have a "Corner-factor scheduler: what replaced the lookahead gain-scheduling family" heading (line 16); `docs/logs/late_turn_in_investigation.md` exists | — |
| 15-19 | Accel effort weight superseded: `R_diag[1]`/`MPCParams.r_a` "no longer a single scalar," replaced by independent accel/brake weights; history in `sim_to_real_investigation.md` §59 | OK (§ number not independently re-verified, but `control_mechanisms.md` does have an "Accel/brake effort weight split" heading confirming the replacement exists and is current) | `control_mechanisms.md:218` "## Accel/brake effort weight split" | — |
| 21-23 | `_low_speed_steer_rate_boost` (`boost_max=2.5, k=0.35`) "does not exist in either codebase," removed with the rest of the lookahead family | **Function removal is TRUE, but claim needs a caveat** | `grep` for the literal function **definition** `_low_speed_steer_rate_boost` finds **no `def`** in either `mpc_core.py` copy — confirmed gone as a callable. However, the **name is still referenced twice in code comments** in the live `mpc_core.py` (and presumably the mirror, not independently re-diffed) explaining why the *new* `_low_speed_corner_boost` mechanism is safer than the old one. This is expected/healthy (a "why the old way failed" comment, exactly what CLAUDE.md's code-review checklist asks for) and does not contradict the doc — the function itself is gone, only its name survives in explanatory comments. Rating this OK on reflection, flagged only so a future reader doesn't mistake the comment hits for the function still existing. | none needed; doc is accurate. (Downgrading from a provisional WRONG to OK after full read of context — see evidence.) |
| 21-23 | (same section) also cross-referenced by `docs/removed_mechanisms.md`'s "## 9. Low-speed steering-rate boost: removed" | OK, and the two docs **agree** | `removed_mechanisms.md`'s §9 states the same fields (`boost_max=2.5, k=0.35`) and the same "regressed turn-in... removed" framing, and its final line explicitly says "See...`docs/reference/superseded_mechanisms.md`'s...section for the current-state pointer" — the two docs are cross-linked and consistent, not duplicative content forks | — |
| 25-33 | Curvature-forcing term (`curvature_forcing_enabled`/`curvature_forcing_gain`) "structurally unsound and disabled"; QP has no path-curvature term in `Ad`/`Bd`; do not re-enable "by flipping the flag alone"; see `late_turn_in_investigation.md` Part 6b | **WRONG / STALE — fields do not exist at all** | `grep -rn "curvature_forcing_enabled"` across `settings.py` and the live `mpc_params.py` returns **nothing**. The doc frames this as a disabled *flag* ("do not re-enable...by flipping the flag alone," implying the flag exists and is merely off), but no such flag or gain constant is present anywhere in the current codebase (checked `settings.py`, live `mpc_params.py`). Either the field was later fully deleted (making this indistinguishable from the fully-`removed_mechanisms.md`-style entries rather than a "disabled flag" entry) and the doc's phrasing is now misleading, or it never existed under this exact name and the doc describes a hypothetical/older iteration not reflected in current code. Either way, a reader following "flip the flag" instructions would find no flag to flip. | re-grep for the actual current field name (if renamed) and correct the field name, or reclassify this entry as a fully-removed mechanism (like `removed_mechanisms.md`'s style) if the flag itself no longer exists, rather than implying it's merely toggled off |
| 30 | "A clean, noise-free synthetic QP test across a full gain sweep reproduces this" | UNVERIFIABLE (test/log not independently reproduced, per task rules) | plausible, no rerun performed | cite the specific log/test file if one exists, for future re-verification |
| 33 | reference to `late_turn_in_investigation.md`'s "Part 6b" | not independently re-checked (section-number verification is low-value without full doc read) | file exists at that path | — |
| 35-38 | Precomputed corner segmentation (`CornerMap`) "removed," part of lookahead family, cross-referenced to `control_mechanisms.md`'s "Corner-factor scheduler" and `removed_mechanisms.md`'s "7. Precomputed corner segmentation (`CornerMap`)" | OK, and consistent between docs | `grep` for `class CornerMap`/`CornerMap(` across both `fsae_MPCTest` and live `fsae_planning` returns **zero matches** — genuinely removed; `removed_mechanisms.md` heading "## 7. Precomputed corner segmentation (`CornerMap`)" confirmed present (line 120) | — |
| 35-38 | *however*, MEMORY.md's own project note says "Corner segmentation implemented... 2026-08-12: CornerMap live-only... offline-validated, not yet live-tested" | **Contradiction worth flagging, not necessarily a doc bug** | The user's own MEMORY.md (auto-memory, separate from this repo's docs) records `CornerMap` as *implemented* on 2026-08-12; this doc records it as *removed* (as part of the wholesale lookahead-family removal). These are not necessarily inconsistent if `CornerMap` was implemented 2026-08-12 and later removed in the corner-factor rewrite — but flagging because a session relying on stale MEMORY.md content (which CLAUDE.md's own header for this project explicitly does not disclaim as stale) could easily reintroduce a mechanism this doc says is gone. Cross-check the corner-factor-scheduler rewrite's date against 2026-08-12 before trusting either source blindly. | no direct doc fix, but flag to the user: MEMORY.md's "Corner segmentation implemented" note is likely now superseded by this doc's "removed" entry and could itself be marked stale in the user's memory file |

**Purpose/audience:** a "graveyard" reference so a future session doesn't re-propose or re-implement a mechanism already tried, measured, and abandoned. Audience: anyone about to add a new gain-scheduling/lookahead-style mechanism to the controller.

**Overlap with `docs/removed_mechanisms.md` (explicit part of this audit):**
- This doc (`superseded_mechanisms.md`) and `removed_mechanisms.md` **do overlap in subject** for at least 3 of its 5 entries: exit-heading boost, low-speed steering-rate boost, and precomputed corner segmentation are all covered in both. In every overlapping case checked, **the two docs agree** with each other on outcome (removed, superseded, why) and are explicitly cross-linked (each points to the other for "current-state" vs. "full mechanism-level summary"). No factual disagreement found between the two docs themselves.
- The division of labor is real and stated: `removed_mechanisms.md` is scoped specifically to "the whole lookahead gain-scheduling family" (per its own title and this doc's line 7), a related but historically-grouped set of ~11 mechanisms removed together in one rewrite; `superseded_mechanisms.md` is the general-purpose "anything superseded or rejected" catch-all, and includes two entries (`accel effort weight` split, `curvature-forcing term`) that are **not** part of the lookahead family and have no `removed_mechanisms.md` counterpart at all.
- **Recommendation: do not merge.** The split is coherent (family-specific history vs. general graveyard) and both docs actively cross-reference each other correctly. Merging would re-bloat `superseded_mechanisms.md` with `removed_mechanisms.md`'s 11-section, single-family depth for no benefit; the current split matches CLAUDE.md's own stated intent for `docs/reference/` (per `README.md`: "these replace the former single `docs/planning_control_sync.md`, which had grown to 38 sections covering five unrelated topics").
- One structural improvement worth considering: `superseded_mechanisms.md`'s title ("Superseded and Rejected Mechanisms") and `removed_mechanisms.md`'s title ("Removed Mechanisms: The Lookahead Gain-Scheduling Family") are not parallel in scope-signaling — a first-time reader has no a priori way to know one is a subset-by-family and the other is catch-all without reading line 7. Consider a one-line "scope" note at the top of `removed_mechanisms.md` pointing back (symmetric to `superseded_mechanisms.md`'s existing line 7 pointing forward), if not already present [not independently checked in this pass — out of primary scope since only headings were to be skimmed for `removed_mechanisms.md`].

**Readability:** Short, consistent per-entry structure (what/why/pointer), good use of "Kept so a future session does not re-invent..." framing up front. No heading misuse. No walls of prose. This is a well-formed doc structurally; its problems are entirely factual/currency, not presentation.

**Why-gaps:** None found — every entry states why the mechanism was removed/superseded, matching CLAUDE.md's code-review checklist item ("say why the earlier attempt failed, not just what the new one does").

**Proposed action:** Keep, but fix the `curvature_forcing_enabled` entry (verify whether the flag/field still exists under any name; if fully gone, reframe like a `removed_mechanisms.md`-style "no longer exists" entry rather than "disabled, don't flip the flag"). Do not merge with `removed_mechanisms.md`; the split is correct and both docs are mutually consistent for the overlapping entries.

**Code paths/modules mentioned (live + offline):**
- `_lookahead_exit_boost`, `_update_lookahead_peak`, `dist_since_peak` (removed, both `mpc_core.py` copies)
- `R_diag[1]` / `MPCParams.r_a` (superseded by accel/brake split)
- `_low_speed_steer_rate_boost` (removed; replaced by `_low_speed_corner_boost`, gated on `corner_factor`)
- `curvature_forcing_enabled`, `curvature_forcing_gain` (claimed-disabled flag, **not found in current `settings.py` or live `mpc_params.py`**)
- `Ad`/`Bd` (QP dynamics model matrices)
- `adaptive_Q_lookahead`, `lookahead_steer_effort_relax`
- `anti_hunt_k_lookahead` (settled at `15.0`, per doc)
- `CornerMap` (removed, precomputed corner segmentation)
- `control_mechanisms.md`'s "Corner-factor scheduler" section (cross-reference target)
- `docs/logs/late_turn_in_investigation.md`, `docs/logs/sim_to_real_investigation.md` (§59), `docs/removed_mechanisms.md`

---

## Summary of counts

| doc | OK | STALE | WRONG | UNVERIFIABLE |
|---|---|---|---|---|
| reference_path_and_speed.md | ~18 | 1 (v_target range 18.00→16.5) | 0 | ~4 |
| simulator_fidelity.md | ~20 | 2 (ceiling speed-dependence table+narrative) | 1 (`_absorb()` "not ported upstream") | ~4 |
| simulator_glossary.md | 6 | 0 | 0 | 0 |
| superseded_mechanisms.md | 3 full entries + 2 partial | 0 | 1 (`curvature_forcing_enabled` field doesn't exist) | ~2 |
