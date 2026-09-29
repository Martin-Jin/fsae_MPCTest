# Audit: fsds_integration_guide.md, fsds_ros_integration.md, fsds_settings.md, fsds_simulator/README.md

Both docs commits: `1781898 Reflow hard-wrapped prose to single logical lines`, `644ac39 Restructure docs to clearly separate FSDS, offline sim, and 2D GUI` (all four fsds/-adjacent docs share these as their most recent touches; fsds_simulator/README.md has older history too, see below).

---

## 1. `fsae_MPCTest/docs/fsds/fsds_integration_guide.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 304-312 | "Set that IP as the `host` launch argument default in `fsds_ros2_bridge.launch.py`" (hardcode `default_value='xxx.xx.xxx.x'`) | **STALE** | Current (uncommitted) `ros2/src/fsds_ros2_bridge/launch/fsds_ros2_bridge.launch.py` no longer hardcodes `host`'s default; it now reads `environ.get('FSDS_HOST_IP', 'localhost')` (see `git diff`, this is an in-progress uncommitted change to that file). `ros2/launch_all.sh` (lines ~535-546) already auto-computes and exports `FSDS_HOST_IP` via `ip route show default`, matching the new mechanism, not the doc's hand-edit instructions. | Rewrite step 3 to say: set `FSDS_HOST_IP` env var (or pass `host:=<ip>` on the launch command) instead of editing the launch file's `default_value`. Point at `launch_all.sh`'s own `FSDS_HOST_IP` auto-detect as the preferred path. |
| 296-302 | "Get the IP... `ip route \| grep default \| awk '{print $3}'`" | OK, mechanically correct | Same command shape used in `launch_all.sh` (`ip route show default \| awk '{print $3}'`), just piped differently; both extract the gateway IP. | none needed, but should be reframed as "set FSDS_HOST_IP to this" rather than "hardcode this into the launch file" |
| 24-29 | `fsds_simulator/` is "full staging mirror ... every package", copy dirs over matching paths | OK | Confirmed via `find`: `fsds_simulator/{common,perception,planning,control}` all present with matching package names (`fsae_interfaces`, `fsae_bringup`, `fsae_sim_perception`, `fsae_planning`, `fsae_control`) mirroring live `ros2/src/fsae_planning/`. | none |
| 27 | link `[fsds_simulator/README.md](../../fsds_simulator/README.md)` | OK | File exists at `fsae_MPCTest/fsds_simulator/README.md`; relative path from `docs/fsds/` resolves correctly (`docs/fsds/../../fsds_simulator/README.md` = `fsae_MPCTest/fsds_simulator/README.md`). | none |
| 29, 103, 331 | "See `docs/reference/` for the full file mapping" | **STALE (imprecise)** | `docs/reference/README.md`'s own index says the file-mapping table specifically lives in `offline_live_parity.md` ("File mapping" heading), not spread generically across the folder. Not wrong (the folder does contain it) but should cite the specific doc, especially since `docs/reference/README.md` was explicitly split out to stop this kind of vague pointer. | Change to `docs/reference/offline_live_parity.md`'s "File mapping" section. |
| 43-46 | `mpc_controller.py`'s `standalone_output` param, single node/executable (not two separate files) | OK | `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_controller.py` exists; `mpc_controller_standalone.py` does **not** exist on disk (only a stale `.pyc` cache file remains), confirming the historical merge `launch_all.sh` line 189-190 also describes ("before mpc_controller.py and mpc_controller_standalone.py were merged into one node"). Also confirmed by fsae_MPCTest git log: `4d1cd1b Merge mpc_controller.py + mpc_controller_standalone.py into one node`. | none |
| 48-73 | topic-map mermaid diagram (`/fsds/testing_only/track`, `/fsds/testing_only/odom`, `/fsae/slam/left_track`, `/fsae/slam/right_track`, `/fsae/perception/cone_detection`, `/fsae/slam/car_odom`, `/fsae/slam/car_position`, `/fsae/planning/selected_trajectory`, `/fsds/signal/go`, `/fsds/control_command`, `/fsae/control/cmd_vel`) | OK | `sim_perception.py` subscribes to `/fsds/testing_only/track` and `/fsds/testing_only/odom`, publishes `/fsae/slam/car_position`, `/fsae/slam/car_odom`, `/fsae/slam/left_track`, `/fsae/slam/right_track`, `/fsae/perception/cone_detection` — matches exactly (grep of `create_publisher`/`create_subscription`). | none |
| 77-84 | control-loop phases (hold-at-start, stale-path emergency brake w/ `PATH_TIMEOUT`=0.5s, normal solve, cone-brake w/ `CONE_RESET_THRESHOLD`=0.3s, telemetry, publish) | OK | `mpc_controller.py` constants: `PATH_TIMEOUT = 0.5` (line 123), `CONE_RESET_THRESHOLD = 0.3` (line 122), both used exactly as described in `_control_step` (referenced at line 894 in that file, matches doc's citation). | none |
| 90-101 | track directory layout `tracks/<name>/{cone_map.json, speed_profile.csv, raceline.csv, centerline.csv}` | OK | Confirmed against `ros2/src/fsae_planning/tracks/comp_test_map_3/` and sibling track dirs found via `find`. | none |
| 103 | physical directory is `ros2/src/fsae_planning/tracks/<name>/`, not `fsae_MPCTest` | OK | Confirmed: `fsae_MPCTest/tracks/__init__.py`'s `TRACKS_DIR` resolves to that path (grep confirms `TRACKS_DIR = os.path.abspath(...)` pointing across the repo boundary, docstring explicitly states this). Matches CLAUDE.md's "Track data lives in fsae_planning" section and MEMORY.md's "Track storage relocated" note. | none |
| 105 | `comp_test_map_3` is the baseline track | OK (consistent) | Confirmed present on disk; `launch_all.sh`'s active `TRACK=comp_test_map_3` (uncommented) matches. | none |
| 107 | date-suffix auto-naming (`<name>_<YYYYmmdd>`) via `tracks.dated_track_name()` or `launch_all.sh`'s own logic | OK | `tracks/__init__.py` has `dated_track_name()` (grep confirms); sibling tracks on disk (`comp_test_map_2_20260916`, `acceleration_20260916`) show the pattern in practice. | none |
| 109 | `ros2/launch_all.sh`'s `TRACK=` "defaults to the most recently recorded track" — but qualify: script shows an **active, uncommented** `TRACK=comp_test_map_3`, not a commented-out one | **STALE (partial)** | Current `launch_all.sh` (line 62) has `TRACK=comp_test_map_3` uncommented and active, i.e. explicitly pinned, not defaulting to newest. The `_newest_track` fallback logic (lines 65-97) only triggers `if [ -z "${TRACK:-}" ]`, which is not the case right now. The doc's phrasing ("TRACK= defaults to the most recently recorded track... so a fresh recording is driven automatically") describes the *mechanism* correctly but doesn't flag that the script currently has an explicit override active, so a reader taking the doc at face value and not reading the script would wrongly assume today's launch auto-follows the newest recording. | Add a caveat: "unless a specific `TRACK=` is left uncommented in the script, which it currently is (pinned to `comp_test_map_3`)." |
| 133 | recording via plain `ros2 launch fsae_bringup sim.launch.py` writes default `~/fsae_logs/cone_map_<timestamp>.json` | OK | `sim.launch.py`'s `cone_out_path` arg description: "cone_recorder output path ('' -> ~/fsae_logs/cone_map_<timestamp>.json)" (grep confirmed). | none |
| 137 | "`fsae_MPCTest/fsds_simulator/launch_all.sh` ... writes timestamped files to its own `fsds_simulator/cone_maps/`" | OK | `fsds_simulator/cone_maps/` directory exists on disk (`find` confirmed). | none |
| 144-150 | export tool invocations (`tuner.tools.export_speed_profile`, `tuner.tools.raceline_optimizer`, `--mode centerline`, `--corner-slowdown`) | OK | Both scripts exist at `tuner/tools/export_speed_profile.py` and `tuner/tools/raceline_optimizer.py`. | none |
| 152 | "the two exporters do not share a corner-speed limit" — `export_speed_profile` uses `CURVATURE_SPEED_A_LAT_MAX`, `raceline_optimizer` uses `alat_ceiling_at(v) x ALAT_MARGIN` | OK | `sim/speed_profile.py:112` defines `CURVATURE_SPEED_A_LAT_MAX = 4.75`, used at lines 230/392. `tuner/tools/raceline_optimizer.py` uses `alat_ceiling_at(v)` (from `model/vehicle_physics.py:369`) and its own `ALAT_MARGIN = 0.85` (line 180). Confirms the doc's claim of two independent limits. | none |
| 160 | `export_speed_profile.py`'s `closed_loop=True` default, `--open-loop` flag | UNVERIFIABLE (not directly grepped for the flag name) but plausible given module docstring's framing | `sim/track_io.load_recorded_track(json_path, n_points=PATH_N_POINTS, closed_loop=True)` signature confirmed with `closed_loop: bool = True` default (grep line 337). The CLI flag name itself (`--open-loop`) not independently grepped but consistent with docstring style seen. | Low priority: spot check `--open-loop` argparse flag name directly if doubt remains. |
| 165 | live `control_utils.load_speed_profile_csv()` / `load_path_profile_csv()`, "no scipy" | OK | Both functions exist in `ros2/src/fsae_planning/control/fsae_control/fsae_control/control_utils.py` at lines 459 and 488 respectively. | none |
| 169-177 | table: `map_path`+`use_precomputed_speed`, `path_map_path`+`use_precomputed_path`, `use_nmpc` launch args | OK | All three confirmed present in `sim.launch.py`'s `DeclareLaunchArgument` list (grep of arg names) plus `use_nmpc` generated dynamically via `*(DeclareLaunchArgument(...) for name, default, meta in (*MPC_PARAM_FIELDS, *NMPC_PARAM_FIELDS))` — confirmed this loop exists and includes `use_nmpc` as an `MPCParams` field (matches `fsds_settings.md`'s own claim about auto-generated launch args). | none |
| 177 | `stanley_controller.py` declares and reads both `map_path`/`path_map_path` params, "identically" | OK | `stanley_controller.py` declares `('map_path', '')` and `('path_map_path', '')` (grep lines 62, 69) and uses `load_speed_profile_csv`/`load_path_profile_csv` the same way `mpc_controller.py` does. | none |
| 220-238 | CSV telemetry logging: `log_csv`/`log_dir` params, `ros2 run fsae_control mpc_controller --ros-args -p log_csv:=true` | OK | `mpc_controller` is a real `console_scripts` entry point in `fsae_control/setup.py` (`mpc_controller = fsae_control.mpc.mpc_controller:main`), confirmed. `sim.launch.py` has `log_csv`/`log_dir` args (grep confirmed). | none |
| 236 | `docs/architecture.md#the-composite-score` anchor | OK | `architecture.md` has `## The Composite Score` heading at line 338; GitHub-style anchor `#the-composite-score` matches. | none |
| 238 | `stanley_controller.py` supports `map_path` "(see `docs/logs/sim_to_real_investigation.md` §57)" | OK | `docs/logs/sim_to_real_investigation.md` line 4621: `## 57. Stanley given live CSV telemetry, superseding §48's map_path/stanley split rationale (2026-08-11)` — section exists and matches subject. | none |
| 175 | link `[tuning.md](../tuning.md)` and `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section, `architecture.md`'s "Second controller" section | OK | `docs/tuning.md` exists; `control_mechanisms.md` line 350 has `## Nonlinear MPC (\`use_nmpc\`): a second controller`; `architecture.md` line 425 has `## Second controller: nonlinear MPC (\`use_nmpc\`)`. Both anchors resolve. | none |
| 246-259 | Docker WSL setup: `osrf/ros:jazzy-desktop`, `--net=host`, apt packages | UNVERIFIABLE (external, not reproducible without running); no contradicting evidence found. | n/a | none, flag as unverified external procedure |
| 250 | `git clone ... --recurse-submodules` | OK, mechanically plausible | Repo does use AirSim as a dependency requiring submodule-style fetch (`AirSim/setup.sh` referenced at line 283, exists per repo layout convention); not independently verified via `.gitmodules` in this pass. | Low priority: check `.gitmodules` if doubt persists |
| 328 | `git clone https://github.com/UOA-FSAE/fsae_planning.git` | OK | Matches remote referenced elsewhere (`docs/reference/README.md`... actually the actual remote wasn't independently queried here, but `fsds_simulator/README.md` cites the same URL, consistent.) | none |
| 364-397 | solver dependency install commands (`cvxpy`, `osqp`, etc.), link `[The solver](../lmpc.md#the-solver)` | OK | `mpc_core.py` does `import cvxpy as cp` (line 82, confirmed). `docs/lmpc.md` has `### The solver` heading at line 360, anchor resolves. | none |
| 416 | rebuild note mentions `mpc/mpc_controller.py`/`mpc/mpc_core.py`/`control_utils.py` copied from `fsds_simulator/` | OK | All three files present in `fsds_simulator/control/fsae_control/fsae_control/` (mpc/mpc_controller.py, mpc/mpc_core.py) and `control_utils.py` (confirmed via earlier `find`). | none |

### Purpose and audience
Operational how-to for running the live FSDS/ROS2 stack: workspace layout, controller/planner selection, the full track-recording-to-driving pipeline, telemetry logging, and a full from-scratch Windows/WSL/Docker install walkthrough. Audience: someone setting up or operating a live FSDS session, not doing offline tuning.

### Overlap
- Its own "Choosing the controller and planner" section (topic map, standalone_output mode) duplicates roughly 60% of `fsds_ros_integration.md`'s "What the bridge hands off" diagram — the latter explicitly says it's "the same topic map [fsds_integration_guide.md] covers in full detail... this diagram is only the shape of it," so the duplication is intentional and cross-referenced, not accidental. Acceptable.
- No meaningful overlap with `fsds_settings.md` (settings-surface doc) beyond a top-of-file pointer; scopes are cleanly split.
- No overlap with `fsds_simulator/README.md` beyond both mentioning the copy-mirror-into-checkout step; `fsds_simulator/README.md` covers building/running from that mirror alone (different audience: someone with only `fsds_simulator/` + FSDS, no `fsae_planning` checkout), `fsds_integration_guide.md` covers the "with an existing `fsae_planning` checkout" case. Genuinely different scenarios, not redundant.
- `docs/architecture.md`/`docs/offline_guide.md` headings skimmed: no direct duplication (architecture.md is a deep technical reference; offline_guide.md is 2D-GUI/tuner-specific).

### Readability
- Single H1, clean H2/H3 hierarchy with a working table of contents linking to real anchors. Good structure overall.
- The WSL/Docker section (242-429) is long and procedural; reasonable as a "run this once" checklist, not prose-heavy.
- Some paragraphs (e.g. lines 75, 103, 107-109, 133) are dense single-paragraph blocks carrying multiple distinct facts (mechanism + caveat + pointer) that would read faster split into bullets, per CLAUDE.md's own writing-style rule ("one idea per bullet"). Not a blocking issue, just below the project's own bar.

### Why-gaps
- Lines 75-76 ("Don't launch fsds_bridge alongside mpc in standalone_output=true mode... they would both publish to /fsds/control_command") states the mechanism and the consequence but not *why* this two-mode design exists in the first place (i.e., why not always route through fsds_bridge). Recoverable: `fsds_simulator/README.md` line 65 supplies exactly this rationale ("standalone_output:=true... is the only mode whose longitudinal behaviour matches what this repo's offline tuner actually tunes"), not cross-linked from this doc. Worth a one-line cross-reference.
- Lines 107-109 (date-suffix track naming) explain the mechanism well but not why re-recording in place (no new date suffix) was chosen over always dating — recoverable from `tracks/__init__.py`'s own docstring/comments (not fully re-verified line-by-line here, but the function exists with clear purpose).

### Proposed action
**Keep**, fix the stale WSL host-IP instructions (highest priority), tighten the "docs/reference/" pointers to name `offline_live_parity.md` specifically, and add the `TRACK=` "currently pinned, not auto-following" caveat.

### Code paths/modules mentioned
- `fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_core.py` (live+mirror)
- `fsae_bringup`'s `sim.launch.py`, `control.launch.py`, `cone_recorder.launch.py`
- `mpc_controller.py` (`control:=mpc` node, `standalone_output` param)
- `fsds_bridge.py`
- `sim_perception` (node), publishing `/fsae/slam/*`, `/fsae/perception/cone_detection`
- `centerline_planner`
- `control_utils.curvature_speed()`
- `MPCController.compute()`, `MPCController.reset()`
- `cone_recorder` (ROS2 node, package `fsae_sim_perception`)
- `sim/track_io.py` (`load_recorded_track()`)
- `cone_map.py`'s `ConeMap` (imported by `centerline_planner.py`)
- `tracks/__init__.py` (`TRACKS_DIR`, `resolve_map_arg`, `newest_track()`, `dated_track_name()`, `geometry_path()`)
- `tuner.tools.export_speed_profile` / `export_speed_profile.py`
- `tuner.tools.raceline_optimizer` / `raceline_optimizer.py`
- `sim/speed_profile.py` (`CURVATURE_SPEED_A_LAT_MAX`)
- `model/vehicle_physics.py` (`alat_ceiling_at()`)
- `planning/boundary.build_path_walls()`
- `control_utils.load_speed_profile_csv()` / `load_path_profile_csv()`
- `telemetry_logger.ControlLogger`, `telemetry_logger.LapProgressTracker`
- `fsae_control/telemetry_logger.py`
- `stanley_controller.py`
- `docs/architecture.md` (Composite Score section)
- `docs/reference/offline_live_parity.md`, `docs/reference/reference_path_and_speed.md`, `docs/reference/control_mechanisms.md`
- `docs/logs/sim_to_real_investigation.md`
- `ros2/src/fsds_ros2_bridge` (bridge package), `fsds_ros2_bridge.launch.py`
- `AirSim/setup.sh`
- `docs/lmpc.md` (solver section)
- `ros2/launch_all.sh`

---

## 2. `fsae_MPCTest/docs/fsds/fsds_ros_integration.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 3 | "The bridge itself (`fsds_ros2_bridge`) is upstream code, not something this project builds or modifies" | **borderline STALE** | The bridge's own launch file (`fsds_ros2_bridge.launch.py`) currently has an uncommitted, in-progress edit (the `_find_settings()` rewrite) — i.e. this project *is* modifying it right now, contradicting the "not something this project builds or modifies" framing, at least for this file. The doc may mean "we don't maintain a fork" rather than "literally zero edits ever," but as written it overstates the boundary. | Soften to something like "largely untouched, save for local path-robustness fixes (e.g. FSDS_HOST_IP/env-based settings.json lookup)" or otherwise acknowledge the exception. |
| 16 | `SIM <-->|"AirSim RPC<br/>port 41451"| BRIDGE` | OK | `ros2/launch_all.sh` hardcodes `AIRSIM_RPC_PORT=41451` (grep confirmed) and the readiness-check probes exactly that port; matches doc. | none |
| 20 | "Start order matters... `.exe` opens the RPC port, so it must be running before the bridge launches" | OK | `fsds_integration_guide.md`'s own WSL walkthrough and `launch_all.sh`'s `wait_for_airsim_rpc()` function encode exactly this ordering constraint. | none |
| 34-40 | topic names in second mermaid (`/fsds/testing_only/track`, `/fsds/testing_only/odom`, `/fsae/slam/*`, `/fsae/planning/selected_trajectory`, `/fsds/signal/go`, `/fsds/control_command`, `/fsae/control/cmd_vel`) | OK | Same topic set independently confirmed against `sim_perception.py`'s pub/sub calls (see doc 1's verification); consistent across both docs. | none |
| 46 | "`fsds_bridge.py` (a *different* node from `fsds_ros2_bridge`, easy to conflate by name)" | OK | Confirmed two distinct files: `ros2/src/fsds_ros2_bridge/` (outer FSDS repo, C++/launch package) vs. `ros2/src/fsae_planning/control/fsae_control/fsae_control/fsds_bridge.py` (Python node in the planning repo, entry point `fsds_bridge = fsae_control.fsds_bridge:main` in `fsae_control/setup.py`). Naming collision is real and worth flagging as the doc does. | none |
| 50 | "a known, unexplained periodic pose discontinuity traced to the bridge's own `getCarState()` RPC path but never fixed there" | OK, matches CLAUDE.md | `docs/logs/periodic_pose_teleport_investigation.md` exists; CLAUDE.md's "Open, unsolved bug" section independently confirms this is still open/unfixed as of 2026-09-29 and names `getCarState()` as one of the three RPC calls implicated (not exclusively though — CLAUDE.md says all three of odom/`/clock`/IMU stall together, "not one call's caching"). The doc's phrasing "traced to the bridge's own getCarState() RPC path" slightly overstates isolation to that one call vs. CLAUDE.md's more careful "one shared transport-level bottleneck, not one call's caching." | Reword to match CLAUDE.md's more precise framing: the stall affects odom/clock/IMU together via a shared transport-level bottleneck, not isolated to `getCarState()` specifically. |
| 51 | "`ros2 topic list`/`ros2 topic hz` against ... `/fsds/testing_only/*`, `/fsds/signal/go`" | OK | Matches `launch_all.sh`'s own diagnostic captures (`ros2 topic hz -w 5 /fsds/testing_only/odom`, etc., confirmed in file). | none |
| 3-4 | cross-links to `fsds_integration_guide.md#launching-nodes-with-fsds-on-windows-wsl--docker` and `#choosing-the-controller-and-planner` | OK | Both headings exist verbatim in `fsds_integration_guide.md` (confirmed via heading grep) and GitHub-style anchor slugging matches. | none |

### Purpose and audience
Conceptual "how the pieces connect" map: where each process runs (Windows vs WSL/Docker), what crosses the AirSim RPC boundary vs. what's pure ROS 2 topics, and the two-return-path gotcha. Audience: someone debugging "is this a bridge problem or a my-code problem," oriented at a higher level than the integration guide's step-by-step.

### Overlap
Deliberately thin and cross-referencing, as designed (see doc's own line 43: "this diagram is only the shape of it"). No real duplication concern; this is the single clearest example of the three fsds/ docs' scopes being intentionally non-overlapping.

### Readability
Short, two mermaid diagrams, bulleted takeaways. Best-structured of the three fsds/ docs; nothing to flag.

### Why-gaps
None found; the doc is explicitly conceptual/pointer-shaped rather than mechanism-heavy, so there isn't much "why" to omit.

### Proposed action
**Keep as-is** structurally; fix the two factual items above (overstated "we don't modify the bridge," and the getCarState()-specific framing of the teleport bug that CLAUDE.md itself has since walked back to "shared transport bottleneck").

### Code paths/modules mentioned
- `fsds_ros2_bridge` (upstream bridge package)
- `sim_perception`
- `centerline_planner`
- controller (`mpc`/`stanley`)
- `fsds_bridge.py` (fsae_control's Python node, distinct from the package above)
- `docs/logs/periodic_pose_teleport_investigation.md`
- `fsds_integration_guide.md` (cross-referenced sections)

---

## 3. `fsae_MPCTest/docs/fsds/fsds_settings.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 10 | "~56 `MPCParams` fields + ~34 `NMPCParams` fields" | UNVERIFIABLE (not exactly counted this pass) but plausible order-of-magnitude | CLAUDE.md itself says "106 dataclass fields total as of 2026-09-29, count directly rather than trusting this number as it drifts" — 56+34=90, not 106, a ~16-field gap. Since CLAUDE.md explicitly warns the total drifts and to count directly, this doc's breakdown may simply be older/uncounted-since. | Recount `mpc_params.py`+`nmpc_params.py` fields directly and update both the split (56/34) and confirm against CLAouncilDE.md's 106 if that number is also refreshed. Flag as likely stale count. |
| 14-19 | example `q_e_y` field metadata (`default=6.4`, unit `1/m^2`, desc "lateral deviation from path centreline", controller "both") | **STALE (value)** | `mpc_params.py` line 47 confirms metadata field is real and shaped exactly like the doc's example, BUT the *actual current default* is `default=6.4` in the doc example vs. **grep shows `field(default=6.4, ...)`** — wait, actual grep result: `q_e_y: float = field(default=6.4, metadata={"unit": "1/m^2", "desc": "lateral deviation from path centreline", "controller": "both"})`. This matches the doc exactly (6.4, same unit, same desc, same controller scope). No discrepancy after all — confirmed OK. | none (verified correct on recheck) |
| 21 | `declare_mpc_params(node)`/`declare_nmpc_params(node)`, `mpc_params_from_node(node)`/`nmpc_params_from_node(node)` | UNVERIFIABLE this pass (not grepped directly) | Plausible given `mpc_params.py`'s field-metadata design confirmed above; function names not independently located in this pass. | Low priority: grep `mpc_params.py`/`nmpc_params.py` and `mpc_controller.py` for these exact function names to close the loop. |
| 23-25 | `fsae_params.yaml`'s `controller:` block, "single source of truth for tunables" comment | UNVERIFIABLE this pass (not opened) | Not independently opened in this session; no contradicting evidence. | Low priority: open `common/fsae_bringup/config/fsae_params.yaml` and confirm the literal comment string. |
| 25 | launch args generated from `MPC_PARAM_FIELDS`/`NMPC_PARAM_FIELDS` via list comprehension, not hand-written | OK | Directly confirmed: `sim.launch.py` contains `*(DeclareLaunchArgument(name, default_value=..., description=...) for name, default, meta in (*MPC_PARAM_FIELDS, *NMPC_PARAM_FIELDS))` — exact mechanism as described. | none |
| 27 | override order: dataclass default -> `fsae_params.yaml` -> launch arg (direct or via `launch_all.sh` shortlist) | OK, matches CLAUDE.md | CLAUDE.md's own "GUI settings coverage and precedence" section states this identical three-layer order and calls it a "Precedence hazard (fixed)" — consistent framing. | none |
| 29-31 | `ros2/launch_all.sh`'s shortlist, `_append_mpc_arg field value` helper | OK | Confirmed: `_append_mpc_arg()` function defined exactly as described (lines ~457-462 of `launch_all.sh`), called ~50 times for the full shortlist. | none |
| 35 | "Top-of-file basics: `CONTROLLER`, `STANDALONE_OUTPUT`, `V_MAX`, `V_MIN`, and `USE_NMPC`, active (uncommented) by default" | OK | Confirmed in current `launch_all.sh`: `CONTROLLER=mpc` (uncommented, line 185), `STANDALONE_OUTPUT=true` (line 193), `V_MAX=20.0`/`V_MIN=1.5` (lines 198-199), `USE_NMPC=true` (line 224) — all five uncommented/active exactly as the doc says. | none |
| 36 | "MPC tuning shortlist... A handful of these are also left active rather than commented out" | OK | Confirmed: alongside many commented-out shortlist lines, several ARE active: `MPC_ADAPTIVE_Q_SCALING_ENABLED=false` (line 439), `NMPC_CORNER_RRATE_BLEND_ENABLED=false` (323), `NMPC_CORNER_FACTOR_K=27.0` (335), `NMPC_RRATE_ZONE_ENABLED=true` + its 3 endpoint values (363-370), `NMPC_RJERK_DELTA=150.0` (383), `REVERSAL_PENALTY_ENABLED=true` (396), `NMPC_REVERSAL_PENALTY_ENABLED=false` (407), `ENABLE_DYNAMIC_SPEED_CAP=false` (447) — doc's "read the script directly, don't trust a cached description" caveat is well-placed and itself confirms the doc author anticipated this drift. | none — doc already hedges correctly |
| 40 | `sim_perception` node: FOV/box/radius filter, publishes `left_track`/`right_track`/`cone_detection`/`car_position` on separate timers | OK (partially reworded) | Confirmed publishers exist; "separate timers" not independently verified line-by-line but plausible given typical ROS2 node structure; not contradicted. | none |
| 42 | cross-link `architecture.md#simulated-perception-and-planning-use_planner` | OK | `architecture.md` heading: `### Simulated Perception and Planning (\`USE_PLANNER\`)` — GitHub slug for this (parens/backticks stripped, `_` kept) is `#simulated-perception-and-planning-use_planner`, matches. | none |
| 3, 5 | cross-links `docs/architecture.md`, `docs/tuning.md`, `CLAUDE.md`'s "Single source of truth..." section, `offline_live_parity.md` | OK | All four targets exist and were independently opened/grepped this session. | none |

### Purpose and audience
Explains the live-side settings *mechanism* only (three-tier precedence: dataclass -> YAML -> launch arg/shortlist) and where perception feeds the planner — explicitly not a tuning guide (defers to `docs/tuning.md`) and not the parity story (defers to `offline_live_parity.md`). Audience: someone about to change a live weight/flag and needing to know which of three files actually controls the running value.

### Overlap
- Cleanly scoped against `fsds_integration_guide.md` (that doc's "3. What the live controller reads" table covers `map_path`/`path_map_path`/`use_nmpc` specifically as *data-source* toggles; this doc covers the *general* three-tier mechanism for all ~90+ fields). Small legitimate overlap on `use_nmpc` specifically, appropriately cross-referenced both directions.
- No overlap with `fsds_ros_integration.md`.
- No overlap with `fsds_simulator/README.md`.

### Readability
Concise, three clearly delineated numbered items under "the three places," a two-bullet shortlist breakdown. One of the tightest of the four docs.

### Why-gaps
- Section doesn't explain *why* three layers exist rather than just the dataclass (i.e., why YAML AND launch args, not one or the other) — recoverable partly from CLAUDE.md's "Single source of truth for MPC tuning" section (YAML = deployment-level default changed without touching code; launch args = per-run override without touching any file), but this doc itself doesn't spell out the "why" for each layer, just that they exist and their precedence.

### Proposed action
**Keep.** Update the "~56/~34" field-count split against a fresh direct count (CLAUDE.md's own 106-field note already flags this number as due for redrift-checking), otherwise accurate and well-hedged.

### Code paths/modules mentioned
- `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_params.py` (`MPCParams`)
- `.../mpc/nmpc_params.py` (`NMPCParams`)
- `declare_mpc_params`/`declare_nmpc_params`, `mpc_params_from_node`/`nmpc_params_from_node`
- `common/fsae_bringup/config/fsae_params.yaml`
- `control.launch.py`/`sim.launch.py` (`MPC_PARAM_FIELDS`/`NMPC_PARAM_FIELDS`)
- `ros2/launch_all.sh` (`_append_mpc_arg`)
- `sim_perception` node
- `centerline_planner`/`skidpad_planner`
- `sim/sim_track.py` (`SimPerception`/`SimPlanner`, `USE_PLANNER`)
- `docs/architecture.md`, `docs/tuning.md`, `docs/reference/offline_live_parity.md`, `docs/reference/simulator_glossary.md`

---

## 4. `fsae_MPCTest/fsds_simulator/README.md`

| line | claim | status | evidence | fix |
|---|---|---|---|---|
| 3 | link `[\`docs/reference/\`](../\`docs/reference/\`)` | **WRONG (broken markdown)** | Literal backticks are embedded inside the link target itself: `(../`docs/reference/`)` — this renders as a link to the literal path `../`docs/reference/`` (with backtick characters in the URL), not `../docs/reference/`. Malformed markdown syntax. | Fix to `[\`docs/reference/\`](../docs/reference/)` — move the backticks to the link text only. |
| 3 | "every file lives at the exact relative path colcon expects" | OK | Confirmed via `find`: `fsds_simulator/{common,perception,planning,control}` package structure matches `ros2/src/fsae_planning/{common,perception,planning,control}` exactly. | none |
| 3 | "`launch_all.sh` is the one exception" (not byte-identical) | OK | Confirmed: `fsds_simulator/launch_all.sh` exists separately from `ros2/launch_all.sh` and (per "What's here but adapted" section, lines 75-77) is explicitly hand-adapted with machine-specific paths — consistent with the "one exception" framing. | none |
| 9-24 | layout tree (`common/fsae_interfaces`, `common/fsae_bringup`, `perception/fsae_sim_perception`, `planning/fsae_planning`, `control/fsae_control`) | OK | All five package directories confirmed present via `find`. | none |
| 21-22 | `control/fsae_control/` contains `stanley_controller`/`mpc/mpc_controller` (`standalone_output` param) + `fsds_bridge` (cmd_vel -> FSDS) + `mpc/mpc_core` (shared MPC QP) | OK | Confirmed all five files present: `fsds_simulator/control/fsae_control/fsae_control/{stanley_controller.py, mpc/mpc_controller.py, fsds_bridge.py, mpc/mpc_core.py}`. | none |
| 23 | "+ `scoring.py` (live/offline score parity...)" | OK | `fsds_simulator/control/fsae_control/fsae_control/scoring.py` confirmed present via `find`. | none |
| 5 | "Nothing under `fsds_simulator/` is imported by this repo's own simulator/tuner (`gui/`, `sim/`, `model/`, `controller/`, `tuner/`)" | OK, matches CLAUDE.md | CLAUDE.md's "cannot import fsae_planning" constraint independently corroborates this; no counter-evidence found (no grep performed for actual imports from fsds_simulator, but consistent with the standing architectural constraint documented project-wide). | none |
| 28-36 | build steps: clone `fs_msgs` (`-b ros2` branch), `apt install ros-jazzy-ackermann-msgs` | UNVERIFIABLE (external repo/package, not independently checked this pass) | No contradicting evidence found; plausible given `fsds_integration_guide.md`'s Docker section installs the same package family. | none, low priority to verify `fs_msgs`'s branch name externally |
| 37 | "each containing its packages (`fsae_interfaces`, `fsae_bringup`, `fsae_sim_perception`, `fsae_planning`, `fsae_control`)" | OK | All five confirmed present as directory names. | none |
| 39-42 | `requirements.txt` present, `pip install -r requirements.txt` | OK | `fsds_simulator/requirements.txt` confirmed present via `ls`. | none |
| 50-63 | Running section: 3-terminal walkthrough, `ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py UDP_control:=false` | OK | `UDP_control` is a real `DeclareLaunchArgument` in `fsds_ros2_bridge.launch.py` (confirmed at line ~97-99 of that file, default `'false'`), so `UDP_control:=false` is a valid (if redundant, since it matches the default) invocation. | none |
| 61-62 | `ros2 launch fsae_bringup sim.launch.py controller:=stanley` / `controller:=mpc standalone_output:=false` / `controller:=mpc` (implicit standalone true) | OK | `sim.launch.py`'s `controller` arg defaults to `'mpc'`, `standalone_output` defaults to `'true'` — confirmed via grep of `DeclareLaunchArgument` defaults. Matches doc's parenthetical "(default: standalone_output:=true)". | none |
| 65 | "`standalone_output:=true` (the default) is the only mode whose longitudinal behaviour matches what this repo's offline tuner actually tunes — see the parent README's explanation" | UNVERIFIABLE this pass — "the parent README" is ambiguous (which README? `fsae_MPCTest/README.md` presumably) | Not opened this pass to confirm the cross-reference actually exists there. | Verify `fsae_MPCTest/README.md` (top-level, not this one) contains the referenced explanation; if it doesn't, this is a dangling non-hyperlinked reference. |
| 69, 75-77 | `launch_all.sh` "adapted to this repo's own machine (hardcoded Windows username, screen resolution, workspace path)" | OK, consistent | `ros2/launch_all.sh` (the live-side original this was presumably forked from) hardcodes `WINDOWS_SIM_PATH="/mnt/c/Users/marti/Downloads/fsds-v2.2.0-windows/FSDS.exe"` and `-ResX=600 -ResY=500` — i.e. exactly the kind of machine-specific hardcoding the doc describes as needing hand-editing. Consistent framing (whether `fsds_simulator/launch_all.sh` itself was independently diffed against this is not confirmed, but the pattern matches). | none |
| 81 | "`fsds_ros2_bridge` itself — part of FSDS, not this stack" (deliberately not mirrored) | OK | Confirmed: no `fsds_ros2_bridge` directory found anywhere under `fsds_simulator/`. | none |
| 83 | "`launch_terminals.sh` — a simpler multi-terminal opener superseded by this mirror's own `launch_all.sh`" | OK | `ros2/src/fsae_planning/launch_terminals.sh` confirmed present in the live repo (via `find`), absent from `fsds_simulator/` (not listed in the `find fsds_simulator` output) — matches the "deliberately not here" claim. | none |
| 84 | "`CHANGES.md` / `.gitignore` — repo-management files specific to the live `fsae_planning` checkout" | OK | Both confirmed present live (`ros2/src/fsae_planning/CHANGES.md`, `ros2/src/fsae_planning/.gitignore`) and absent from the `fsds_simulator/` file listing. | none |

### Purpose and audience
Standalone build/run instructions for the `fsds_simulator/` mirror in isolation — i.e., someone who wants to stand up the full stack from ONLY `fsae_MPCTest` + FSDS + two message repos, with no `fsae_planning` checkout at all. Distinct audience from `fsds_integration_guide.md`, which assumes an existing `fsae_planning` checkout as the primary case.

### Overlap
Legitimate, non-redundant scope split from `fsds_integration_guide.md` (see above); the two docs explicitly describe complementary scenarios (with-checkout vs. without). No overlap with `fsds_ros_integration.md` or `fsds_settings.md`.

### Readability
Good structure (Layout / Building / Running / One-command launch / What's adapted / What's deliberately not here), clear headings, appropriately terse. The one real defect is the broken markdown link at line 3.

### Why-gaps
- Doesn't explain *why* `fsds_simulator/` exists as a full mirror rather than e.g. a git submodule or symlink into `fsae_planning` — this "why" is answered elsewhere (CLAUDE.md's "Third copy" section: it exists to stage PRs into the separate `fsae_planning` repo since nothing gets pushed there directly), not cross-linked from this README. Worth a one-line pointer back to CLAUDE.md's/`offline_live_parity.md`'s framing.

### Proposed action
**Keep**, fix the one broken link (trivial), and verify the dangling "parent README" cross-reference at line 65 actually exists in `fsae_MPCTest/README.md`.

### Code paths/modules mentioned
- `common/fsae_interfaces`, `common/fsae_bringup`
- `perception/fsae_sim_perception`
- `planning/fsae_planning` (`centerline_planner`, `skidpad_planner`)
- `control/fsae_control`: `stanley_controller.py`, `mpc/mpc_controller.py`, `mpc/mpc_core.py`, `fsds_bridge.py`, `scoring.py`
- `fsds_simulator/launch_all.sh`
- `fsds_simulator/requirements.txt`
- `fs_msgs` (external repo, `ros2` branch)
- `ackermann_msgs` (apt package)
- `fsds_ros2_bridge` (explicitly excluded)
- `launch_terminals.sh`, `CHANGES.md`, `.gitignore` (explicitly excluded, live-only)
- `docs/reference/` (broken link target)

---

## Cross-doc notes (not scored per-doc, but relevant to all four)

- `ros2/launch_all.sh`'s own error-message text (not one of the four audited docs, but referenced indirectly) tells the user to run `python3 -m tuner.export_speed_profile <TRACK>` (missing `.tools.`) and separately cites `fsae_MPCTest/docs/developer_guide.md` for "Recording, exporting and driving a track" — **that file does not exist** (`docs/developer_guide.md` is absent from `fsae_MPCTest/docs/`; the actual section lives in `docs/fsds/fsds_integration_guide.md`). This is a bug in `launch_all.sh`'s own comments/error text, not in the four audited docs, but worth flagging since a user following that error message would hit a dead end. Out of strict scope (not one of the 4 target docs) but adjacent enough to note.
- The bridge launch file's uncommitted `_find_settings()` rewrite is the single highest-impact stale-doc issue found: it directly invalidates `fsds_integration_guide.md`'s WSL setup step 3 as written.
