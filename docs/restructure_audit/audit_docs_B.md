# Docs Audit B: nmpc.md, lmpc.md, stanley.md, vehicle_physics_guide.md, error_state_reference.md, removed_mechanisms.md, junior_project_mpc_docs.md

Method: full read of all 7 docs, 6 parallel verification subagents (one per doc, nmpc.md+error_state_reference.md combined) cross-checking every claim against live code (`ros2/src/fsae_planning/`), offline code (`fsae_MPCTest/`), the `fsds_simulator/` mirror, `docs/reference/*.md`, `docs/logs/*.md`, and git log (read-only). Independently spot-verified the highest-value findings myself (state indices, settings.py defaults, module-split paths, broken cross-refs, mirror drift).

---

## Cross-cutting finding (affects nmpc.md, lmpc.md, error_state_reference.md, junior_project_mpc_docs.md)

**`controller/nmpc_optimiser.py` and `controller/optimiser.py` no longer exist.** Both were split into packages at some point after these docs were written:
- `controller/nmpc_optimiser.py` → moved to `fsae_MPCTest/deleted/controller/nmpc_optimiser.py`; the real offline NMPC port is now `controller/nmpc/` (`layout.py`, `reference.py`, `dynamics.py`, `outputs.py`, `weight_schedule.py`, `solver.py`), confirmed authoritative in `docs/reference/offline_live_parity.md` ("The offline NMPC is split into modules...").
- `controller/optimiser.py` → split into `controller/lmpc/build.py` (`init_parameterized_mpc`) + `controller/lmpc/solve.py` (`solve_mpc`), also confirmed in `offline_live_parity.md`.
- Both old names still work via `__init__.py` re-exports, which is likely why this drifted unnoticed (`from controller import nmpc as no` still runs).
- Affected files: `nmpc.md` (lines 7, 277), `error_state_reference.md` (table rows in §6), `lmpc.md` (4+ occurrences including a TOC anchor), `junior_project_mpc_docs.md` (§4, §6.4 table). Also affects out-of-scope docs `architecture.md`, `fsae_planning_pending_pr.md`, `settings.py`'s own comments — repo-wide drift, not unique to any one doc.

**`docs/reference/README.md` is repeatedly cited for sections that no longer live there.** After a doc split, `README.md` became a pure index (only "Where new content belongs"); the actual sections ("Nonlinear MPC (`use_nmpc`)", "Which settings affect which controller", "Three MPCC-inspired additions", "MPC weight/gain parity" table, "Live/offline score parity", slew-rate section) all moved to `docs/reference/control_mechanisms.md` or `offline_live_parity.md`. Confirmed broken in: `nmpc.md` (lines 254, 279), `tuning.md` (lines 171, 182 — outside scope but same pattern), `architecture.md`, `fsae_planning_pending_pr.md`, `steering_turn_in_upgrade_options.md`. This is a mechanical find-and-replace fix (`README.md` → `control_mechanisms.md` in most cases, `offline_live_parity.md` in a couple).

---

## 1. `docs/nmpc.md`

**Purpose/audience:** deep technical reference for the NMPC controller, split out of `architecture.md`, assumes familiarity with `lmpc.md`. Accurate to its stated scope.

| Claim | Status | Evidence |
|---|---|---|
| `use_nmpc` default false (`NMPCParams`) | OK | `nmpc_params.py`: `use_nmpc: bool = field(default=False, ...)` |
| Horizon 20 steps (1.0s) | OK | `nmpc_params.py: nmpc_horizon: int = field(default=20, ...)` |
| `nmpc_rk_substeps` default **2** | **STALE** | Live default is now **4** (`nmpc_params.py`); offline `settings.py: NMPC_RK_SUBSTEPS = 4`. Raised from 2 after a documented numerical-instability fix (`nmpc_low_speed_accel_stall_investigation.md`) — the doc cites this investigation by name but doesn't reflect its outcome. Independently confirmed via direct grep. |
| `nmpc_jac_substeps` default **1** | **STALE** | Live/offline default is now **4** (`nmpc_params.py`, `settings.py: NMPC_JAC_SUBSTEPS = 4`), for the same reason (instability below ~6.5-7 m/s at substeps=1). Independently confirmed. |
| Measured solve time mean 8.9ms / p95 11.6ms | **STALE** | Superseded by the same substep fix: `nmpc_low_speed_accel_stall_investigation.md` shows mean/p95/max **9.56/14.05/32.99 ms pre-fix** vs **18.63/23.61/38.86 ms post-fix (substeps=4, the shipped default)**. Doc's number doesn't match either row cleanly. |
| `nmpc_spline_reference_enabled` default true | OK | `nmpc_params.py` |
| `nmpc_friction_circle_enabled` default false | OK | `nmpc_params.py` |
| `nmpc_steer_rate_anti_hunt_enabled` exists | OK | Field actually lives on `MPCParams` (mpc_params.py), reused by NMPC — doc doesn't contradict this, just worth a restructure note |
| `nmpc_trust_delta_rad`/`nmpc_trust_a` exist | OK | `nmpc_params.py`, used in `nmpc_core.py` |
| `_DENOM_FLOOR = 0.25` | OK | `nmpc_core.py` |
| `v_blend_lo=1.0, v_blend_hi=2.5` | OK | `nmpc_core.py` |
| `dt = 0.05s` | OK | `nmpc_core.py` |
| Offline port = `controller/nmpc_optimiser.py` | **WRONG (moved)** | See cross-cutting finding above; real path is `controller/nmpc/` |
| `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section | OK | `control_mechanisms.md` line 350 |
| `docs/reference/README.md`'s "Which settings affect which controller" map | **WRONG FILE** | Lives in `control_mechanisms.md` (line 430), not `README.md` (which has no such heading). Repeated twice in this doc. |
| Link text `removed_mechanisms.md` in a couple of spots referring to the "superseded" content | Mostly OK — `removed_mechanisms.md` is this doc's real sibling file and exists; one or two internal mentions may conflate it with `superseded_mechanisms.md`, low-severity, both files exist so nothing is truly broken |
| `late_turn_in_investigation.md` Part 16, §16.6 etc. | OK | Headers confirmed present |
| Live A/B numbers (6.45%→0.58% saturation, 54.72s→52.35s) | UNVERIFIABLE in this pass (not found verbatim in `control_mechanisms.md`), but internally consistent with `junior_project_mpc_docs.md`'s own citation of the same pair — cross-doc consistent even if the primary source wasn't located |
| Three MPCC-inspired additions: only 2 flags remain (spline ref, friction circle), horizon speed profile removed | OK | Confirmed `nmpc_horizon_speed_profile_enabled` no longer exists as a live field anywhere (only survives in code comments describing its removal) |
| `_f()`/`_f_scalar()` machine-precision test via `test_nmpc_core_math.py::test_step_scalar_matches_step_vectorised` | OK (with clarification) | Test only exists in `fsae_autonomous` (now at `ros2_autonomous/src/fsae_autonomous/control/fsae_control/test/test_nmpc_core_math.py`), which the doc's own "Testing the math" section correctly separates from the offline `tuner.nmpc_offline_check` — no contradiction, just worth noting the path moved (CLAUDE.md's own note about `fsae_autonomous` moving once) |

**Overlap:** §3 ("State vector and Frenet metric factor") substantially re-derives the same `s_dot`/`e_psi_dot` mechanism as `error_state_reference.md` §4, at architecture-reference depth vs. that doc's from-scratch worked-example depth. Likely intentional (different altitude), but a restructure could have nmpc.md link out instead of re-deriving. The "Feature comparison" table explicitly discloses its overlap with `control_mechanisms.md`'s exhaustive map (good practice). The "Three MPCC-inspired additions" bullets restate `control_mechanisms.md`'s own section closely, including similar phrasing.

**Readability:** Good structure (TOC, mermaid diagram, tables), but a few 6+ sentence paragraphs (tyre-saturation rationale, blend-out bug) could be bulleted per house style.

**Why-recoverability:** Mostly present inline. The substep-count staleness is the one place the doc states an outdated "why" (old counts were "safe" via backtracking) without the newer, more serious why (numerical instability, not just coarseness) that's in the cited investigation doc.

**Code paths mentioned:** `nmpc_core.NMPCController`, `mpc_core.MPCController`, `controller/nmpc_optimiser.py` (stale), `_error_state()`, `PathReference.project()/_f()/_outputs()`, `_jacobians()`, `_output_jacobians()`, `_solve_step()`, `_rollout()`, `_cost()`, `mpc_controller.py`, `mpc_controller_standalone.py`, `control_utils.py`, `model_utils.py`, `tuner.nmpc_offline_check`, `test_nmpc_core_math.py`, `control_mechanisms.md`, `reference/README.md`, `superseded_mechanisms.md`/`removed_mechanisms.md`, `late_turn_in_investigation.md`, `nmpc_low_speed_accel_stall_investigation.md`, `nmpc_speed_limit_investigation.md`, `tuning.md`.

**Fix priority:** (1) substep defaults 2/1→4/4, (2) solve-time numbers now stale, (3) offline path `controller/nmpc_optimiser.py`→`controller/nmpc/`, (4) "README.md"→"control_mechanisms.md" ×2.

---

## 2. `docs/error_state_reference.md`

**Purpose/audience:** from-scratch, worked-by-hand pedagogical reference; explicitly distinct from architecture/status docs. Delivers on this well — strongest doc of the seven on the CLAUDE.md "plain English first" writing-style rule.

| Claim | Status | Evidence |
|---|---|---|
| `lf = 0.70m` in VehicleParams | OK | `vehicle_physics.py: self.lf = 0.70` (matches live `mpc_core.py`'s own `self.lf = 0.70`) |
| `MPCController._error_state()` lines ~833-945 | OK (close enough) | Function actually starts at line 838; doc hedges "as of this writing" |
| `PathReference.project()`/`_f()`/`_outputs()` in `nmpc_core.py` | OK | Confirmed present at expected relative positions |
| `vehicle_physics.plant_to_tracking_error()` exists | OK | `model/vehicle_physics.py` |
| Front-axle projection, `e_y`/`e_psi`/`e_y_dot` formulas | OK | Matches inline code comments in `mpc_core.py` verbatim in spirit |
| NMPC `PathReference`: CubicSpline x(s)/y(s), `psi_ref`, `kappa` formulas | OK | Matches `nmpc_core.py`'s own docstring |
| `controller/model_utils.py`'s `curvature_horizon_profile()` (historical) | OK, still present | Exists, referenced in a `CURVATURE_FORCING_ENABLED`-adjacent comment |
| §6 table's offline column: `controller/nmpc_optimiser.py`'s port (×3 rows) | **WRONG (moved)** | Same cross-cutting finding — actual path is `controller/nmpc/` |
| Worked-example arithmetic (§2.1, §4.1) | OK | Spot-checked by hand, all correct (front-axle position, `e_y_dot=0.698`, `e_psi_dot=-0.75`) |
| §3.2 curvature-forcing gain-sweep numbers (gain=1.0/6/20, `Ad[2,2]≈0.946`) | Internally consistent arithmetically; provenance not independently traced to a specific historical measurement in this pass, but corroborated in spirit by `superseded_mechanisms.md`'s existence |
| Link to `superseded_mechanisms.md`'s "Curvature-forcing term" section | OK | Section exists verbatim |

**Overlap:** §3 (why LTV-QP can't re-project) overlaps with `docs/reference/superseded_mechanisms.md`'s "Curvature-forcing term" section — this doc gives the pedagogical walkthrough, that one the terse index entry; correctly cited rather than silently duplicated. §4 overlaps with `nmpc.md` §3 as noted above.

**Readability:** Best of the seven docs for the "explain in plain English first" rule (the corridor-walking analogy, explicit "the step people most often get wrong" framing). No wall-of-text issues.

**Why-recoverability:** Excellent — this doc's entire purpose is explaining "why," and it succeeds throughout, especially the curvature-forcing failure mechanism.

**Code paths mentioned:** `mpc_core.py` (`_error_state`, `_discrete_model`, `_curvature`), `nmpc_core.py` (`PathReference`, `_f`, `_outputs`), `vehicle_physics.plant_to_tracking_error()`, `model/bicycle_model.get_8state_discrete_model()`, `controller/nmpc_optimiser.py` (stale), `controller/model_utils.py`, `superseded_mechanisms.md`, `lmpc.md`, `architecture.md`, `junior_project_mpc_docs.md`.

**Fix priority:** (1) three table rows citing `controller/nmpc_optimiser.py` → `controller/nmpc/`.

---

## 3. `docs/lmpc.md`

**Purpose/audience:** from-scratch technical reference for the LTV-QP controller's full math (state vector, every matrix entry, cost function, solver, adaptive features). Accurate to stated scope.

| Claim | Status | Evidence |
|---|---|---|
| `A_kin`/`A_dyn`/`B` matrix entries | OK | Exact match against `model/bicycle_model.py` |
| ZOH discretization via `scipy.linalg.expm` | OK | Matches `bicycle_model.py` structure |
| `1e-12` sparsity-pattern trick | OK | Matches code + docstring |
| Blend `clip((v_x-1.0)/(2.5-1.0),0,1)` | OK | Exact match |
| `controller/optimiser.py` as the QP module (4+ occurrences incl. TOC anchor) | **STALE (moved)** | Real path: `controller/lmpc/build.py` + `controller/lmpc/solve.py`, confirmed in `offline_live_parity.md` |
| Q/R/R_rate cost, `W_slack=10000.0`, ±3.5m soft boundary | OK | Matches `mpc_core.py` |
| `cp.Parameter` "parameterised" trick | OK | Matches `mpc_core.py` |
| `du_max` slew rate, OSQP+Clarabel, `OPTIMAL_INACCURATE` accepted | OK | Matches code structure |
| `adaptive_R_scaling`: `steer_scale=1+(1.5·vx)/(6.0+vx)` | OK | Matches `model_utils.py`/`mpc_core.py` |
| `adaptive_R_scaling`: `accel_scale=1+0.05·vx` | **WRONG** | Both live and offline now fix `accel_scale = 1.0` (deliberately disabled — a speed-dependent accel cost was found to fight corner-entry braking tuning, per both files' own docstrings). Independently confirmed via direct grep of both `model_utils.py` and `mpc_core.py`. |
| `adaptive_Q_scaling` floor/ey_lo=0.05/ey_hi=0.3 | OK | Matches both sides |
| `steer_rate_anti_hunt`: `boost_kappa=1/(1+60·|κ|)`, `boost_ey=1/(1+30·|e_y|)`, `boost_epsi=1/(1+23·|e_psi|)` | **STALE (values halved)** | Current constants are `k_kappa,k_ey,k_epsi = 30.0, 15.0, 11.5` in both `model_utils.py` and `mpc_core.py` (halved 2026-08-19, per inline comment: original values faded the boost out too fast on gentle curves). Independently confirmed via direct grep. Mechanism/shape description otherwise correct. |
| `corner_factor` formula, default `k=8.0` | OK | Matches `model_utils.py`, `mpc_params.py`, `settings.py` |
| `low_speed_corner_boost` formula | OK | Matches both sides |
| Heading-error accel/brake asymmetry `frac_epsi` formula and field names | OK | Matches `mpc_core.py` and `mpc_params.py` defaults |
| `MPCParams` fields (`q_ey_straight/corner`, etc.) | OK | All present with matching defaults |
| Delay-compensation fields | OK | All present in `mpc_params.py` |
| `TUNABLE_Q_IDX = [0,1,2,3,4]` in `tuner/offline_tuner.py` | OK | Exact match |
| `Q[5,5]/Q[6,6]/Q[7,7]` always 0 | OK | Matches `settings.py` and `mpc_core.py` |
| `N_HORIZON=35`, 1.75s, `dt=0.05` | OK | Matches `settings.py` |
| "Typical solve time is 1-5 ms at `N=25`" | **WRONG, but also present in source comment** | Actual `N_HORIZON=35`; the stale "N=25" phrase is copied verbatim into `controller/lmpc/solve.py`'s own docstring too — not unique to the doc, but still wrong in both places |
| Slew-rate 180 deg/s | OK | Matches `control_mechanisms.md`'s "Slew-rate limit" section and `mpc_core.py`'s hardcoded value |
| `ADAPTIVE_Q_SCALING_ENABLED = True` | OK | Matches `settings.py` |
| `STEER_RATE_ANTI_HUNT_ENABLED = True` | OK | Matches `settings.py` (note: `model_utils.py`'s own docstring says "default False", itself stale against the current `settings.py` value — a separate code-comment bug, not a doc bug) |
| "Removed: `adaptive_R_rate`'s current-curvature floor" | OK, slightly underclaims | `removed_mechanisms.md` confirms full deletion (function + params + telemetry column), not merely disconnected as this doc's wording could imply |
| Link `junior_project_mpc_docs.md#26-manual-tuning-guide` | **BROKEN** | Real anchor is `#55-manual-tuning-guide` (Section 5.5); "2.6" doesn't exist in that doc (Section 2 only goes to 2.3) |

**Overlap:** With `error_state_reference.md` — legitimate summary-with-pointer, not redundant (lmpc.md gives the premise, defers derivation). With `junior_project_mpc_docs.md` §2 — appropriate depth split (tutorial vs. reference), undercut only by the broken cross-link noted above.

**Readability:** Good, one H1, consistent nesting, working in-doc TOC. A few long paragraphs before code blocks but broken by subheadings.

**Why-recoverability:** Good for most mechanisms (Hill-function saturation, corner-factor blend). The `accel_scale`/anti-hunt staleness cases are doubly bad here: the doc's stated "why" (gentler linear scale) describes a formula that no longer exists.

**Code paths mentioned:** `model/bicycle_model.py`, `controller/optimiser.py` (stale → `controller/lmpc/`), `controller/model_utils.py`, `mpc_core.py`, `mpc_params.py`, `control_utils.py`, `vehicle_physics.py`, `sim/rollout_core.py`, `nmpc_core.py`, `tuner/offline_tuner.py`, `settings.py`, `nmpc.md`, `stanley.md`, `error_state_reference.md`, `removed_mechanisms.md`, `junior_project_mpc_docs.md`, `control_mechanisms.md`, `reference/README.md`, `architecture.md`.

**Fix priority:** (1) `accel_scale` formula wrong (should be fixed at 1.0), (2) anti-hunt constants stale (60/30/23 → 30/15/11.5), (3) `controller/optimiser.py` path stale everywhere, (4) broken link to junior doc §2.6→§5.5, (5) N=25 solve-time note.

---

## 4. `docs/stanley.md`

**Purpose/audience:** reference for the third selectable controller, written because live changes started landing on it and to untangle shared-vs-Stanley-specific plumbing. Accurate to scope, minimal unnecessary overlap with other docs.

| Claim | Status | Evidence |
|---|---|---|
| `StanleyController` class location (`control_utils.py`), used by `stanley_controller.py` | OK | Confirmed, doc's own header is self-consistent |
| Steering law `δ = θ_e + arctan(k_cte·e/(v+k_soft)) − k_d·ω` | OK | Code uses `atan2(k_cte*e, v+k_soft)`, mathematically equivalent |
| `k_cte`, `k_soft`, `k_d` defaults 1.0/1.0/0.1 | OK | Matches `control_utils.py.__init__` |
| Front-axle projection, `MAX_STEER_RAD` clip | OK | Matches code |
| No lookahead in nearest-point search | OK | Matches code (raw argmin) |
| `curvature_speed()`/`precomputed_speed_at()` shared, `map_path` param identical in both nodes | OK | Confirmed |
| No fixed timer (fires from `_pose_cb`) vs. MPC's `CONTROL_HZ` timer | OK | Confirmed |
| No stale-path/emergency-brake handling, only `len(path)<2` check | OK | Confirmed vs. MPC's `PATH_TIMEOUT`/`path_stale` |
| `V_CURV_FALL_RATE=7.0`, `GATE_RATE_LIMIT=2.0`, `SPEED_TARGET_RISE_RATE=7.0` | OK | Exact match in `stanley_controller.py` |
| Rate-limiter state resets to `None` on stale/short path | OK | Confirmed |
| Symptom narrative cross-checked against `docs/logs/planner_only_speed_target_oscillation.md` | **MISMATCH** | That log describes a 59s NMPC run stalling near the end (t≈40-45s), not a spin-out "within the first few seconds... within about 1.5 seconds." The doc's own timeline doesn't match its cited source — either a different, undocumented Stanley-specific incident is being loosely summarized alongside the NMPC one, or the citation is wrong. Worth tightening. |
| `fsae_logs/stanley_control_20260916-081934.csv` referenced | OK (file exists) | Confirmed present on disk with matching timestamp, among 6 sibling runs from the same day; contents not reproduced (out of scope, correctly) |
| `self.last_e_y = -e` sign convention matches `mpc_core.py`'s `last_telemetry['e_y']` | OK | Confirmed, same left-positive convention |
| `stanley_controller.py`/`control_utils.py` present, byte-identical in `fsds_simulator/` mirror | OK | Confirmed via `find`+`diff`, not `git status` |
| Doc's citation of `architecture.md`'s "Second controller" section for why the mirror covers the whole tree | **WRONG citation** | That section is entirely about NMPC. The actual whole-tree-mirror rationale sits in `architecture.md`'s Module Reference table, an unnamed row, not the section this doc points to |

**Overlap:** Minimal with `lmpc.md`/`junior_project_mpc_docs.md`, as intended — neither substantively duplicates Stanley internals.

**Readability:** Good — TOC, table for steering-law terms, mermaid flowchart, bold Symptom/Fix/Status labels.

**Why-recoverability:** Yaw-rate-damper reasoning is in the doc AND corroborated by a code comment. "Why no lookahead in the nearest-point search" is doc-only synthesis, not traceable to any code comment — plausible but not independently grounded.

**Code paths mentioned:** `control_utils.py` (`StanleyController`, `curvature_speed`, `precomputed_speed_at`, `tracking_error_speed_gate`, `MAX_STEER_RAD`), `stanley_controller.py`, `mpc_controller.py` (`CONTROL_HZ`, `PATH_TIMEOUT`, `path_stale`, `map_path`), `mpc_core.py` (`last_telemetry`), `fsds_simulator/` mirror copies, `planner_only_speed_target_oscillation.md`, `lmpc.md`, `nmpc.md`, `error_state_reference.md`, `architecture.md`.

**Fix priority:** (1) symptom-narrative/log-timeline mismatch, (2) wrong `architecture.md` section citation.

---

## 5. `docs/vehicle_physics_guide.md`

**Purpose/audience:** plain-English companion to `vehicle_physics.py`, no prior vehicle-dynamics knowledge assumed, for tuning lookups. Matches its content and tone well; heavy, effective use of tables.

| Claim | Status | Evidence |
|---|---|---|
| 25-state vector, indices 0-24 per doc's table | OK | Exact match to `IDX_*` constants (`IDX_X=0...IDX_ALAT_LIM=24`, `N_STATES=25`) |
| "Positions 0-7 match the MPC's own 8 states" | **FALSE / MISLEADING** | Plant indices 0-7 are raw `[X, Y, psi, vx, vy, r, delta_act, a_act]`. The MPC's actual 8-state vector (per `lmpc.md`) is path-relative **error** states `[e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]` — different physical quantities in the same 8 slots. Only positions 6-7 (`delta_act`, `a_act`) are genuinely the same quantity. A dedicated conversion function, `plant_to_tracking_error()`, exists specifically because the two orderings are NOT interchangeable — its existence is itself proof against the doc's claim. `gui/simulation.py`/`tuner/offline_tuner.py` read the **converted** error state, not raw plant indices 0-5 directly, contradicting the doc's rationale for why this alignment supposedly helps. This same false claim also appears in `vehicle_physics.py`'s own module docstring — a repo-wide error, not unique to this doc, but it should be corrected in both places. |
| `VehicleParams` fields (lf, lr, m, Iz, tf, tr, h_cg, max_steer, max_accel, max_accel_brake, max_v, m_us, r_eff, I_wheel, I_drivetrain, k_susp_f/r, c_damp_f/r, k_arb_f/r, z_max/z_min, camber_gain, camber_stiff_f/r) | OK | All present |
| Pacejka MF94 functions, B/C/D/E/Sv/Sh | OK | Both `pacejka_lateral_mf94`/`pacejka_longitudinal_mf94` exist as described |
| `mu`, `k_sens`, `road_mu` | OK | All present with matching roles |
| `sigma_y_f`/`sigma_y_r` relaxation | OK | Matches code |
| Friction ellipse | OK | Matches code exactly |
| `Cd_A`, `Cl_A_f/r`, `Cl_pitch_sens` | OK | Present |
| `Crr`, `F_stiction` | OK | Present |
| `tau_delta`, `tau_a` | OK | Present |
| `tv_gain` default 0 | OK | Confirmed |
| `alat_ceiling*` mechanism (restoring yaw moment, first-order lag, not hard clip, 'pi' vs legacy 'p') | OK | All fields present, mechanism matches, even the overshoot rationale is duplicated verbatim between the code comment and the doc |
| "~14.5 m/s² unaided" | **MINOR DRIFT** | Current settled figure in `sim_to_real_investigation.md` is 14.06 (cited 3× in the log at its most recent stage); "~14.5" traces to an earlier round-number citation from an earlier investigation stage. Not a contradiction, just imprecise/dated. |
| "7-9 m/s² depending on speed" / 4.8%-21.1% saturation / a_lat max 11.24/12.34 | OK | Matches `docs/reference/simulator_fidelity.md` and CLAUDE.md's own table exactly |
| Pointer to `simulator_fidelity.md`'s "lateral-acceleration ceiling, partly closed" section | OK | Section exists verbatim |

**Overlap:** With `docs/reference/simulator_fidelity.md` — deliberate, disclosed 2-paragraph summary that explicitly defers to that doc for the full derivation; `vehicle_physics_guide.md` isn't part of the `docs/reference/` family at all (confirmed via that folder's own README), it's a separate, more basic companion doc. No unlisted duplication.

**Readability:** Best table usage of the seven docs; very little unstructured prose.

**Why-recoverability:** Strong for `alat_ceiling` — explicitly states FSDS's internal cause is unknown and this only reproduces the external symptom, correctly deferring deeper history to `sim_to_real_investigation.md` rather than overclaiming.

**Markdown links:** None (references other docs by filename in prose only, nothing to break). Note: `vehicle_physics.py`'s own code comments point at `docs/reference/README.md`'s now-nonexistent "MECHANISM" section — a code-comment bug, not this doc's.

**Code paths mentioned:** `vehicle_physics.py`, `bicycle_model.py`, `gui/simulation.py`, `tuner/offline_tuner.py`, `docs/reference/simulator_fidelity.md`.

**Fix priority:** (1) "positions 0-7 match the MPC's own 8 states" is genuinely wrong and should be corrected (in both this doc and the code docstring it's copied from), (2) "~14.5" → 14.06 for precision.

---

## 6. `docs/removed_mechanisms.md`

**Purpose/audience:** historical/reference doc preserving why a whole family of lookahead gain-scheduling mechanisms was deleted, explicitly to prevent re-discovery/re-litigation. Delivers on this purpose very well — the cleanest doc of the seven, essentially zero factual errors found.

Every mechanism claimed **removed** was independently confirmed absent from live (`mpc_core.py`, `mpc_params.py`), offline (`model_utils.py`, `settings.py`), AND the `fsds_simulator/` mirror:
1. `adaptive_Q_lookahead`/`ADAPTIVE_Q_LOOKAHEAD_ENABLED`, `lookahead_curvature_profile` — confirmed absent
2. `lookahead_steer_effort_relax` — confirmed absent
3. `ADAPTIVE_Q_DEMAND_NORMALISED`, demand normalisation — confirmed absent
4. U-turn detection (60°/120° thresholds) — confirmed absent
5. Straight-line adjustments (`adaptive_q_straight_*`, `steer_effort_straight_boost`) — confirmed absent
6. `CornerMap`/`use_precomputed_corner_map`/`_segment_corners` — confirmed absent
7. `curvature_forcing_enabled`/`CURVATURE_FORCING_ENABLED` — confirmed absent
8. `low_speed_steer_rate_boost` — confirmed absent
9. §10a `adaptive_R_rate` current-curvature floor (`adaptive_r_rate_enable_in_corners`, `_during_floor`, `m_Rrate_corner`) — confirmed absent live+offline+mirror

And confirmed **still present** where the doc says so: `alat_ceiling_at()` (vehicle_physics.py), `_Plant` class in `nmpc_core.py`, `_corner_factor`/`_low_speed_corner_boost`/`_blend` (both sides + mirror), `steer_rate_anti_hunt` (both sides, reused by NMPC).

Git log spot-check: commit `8b666eb` (2026-09-29) matches §10a's removal exactly, touching all the files the doc names on all sides including the mirror and `fsae_params.yaml`/`launch_all.sh`/GUI/telemetry_logger. Commit `bb8863d` (2026-08-13) matches the corner-factor rewrite date.

All cross-references resolve: `control_mechanisms.md`'s "Corner-factor scheduler", `superseded_mechanisms.md`'s "Curvature-forcing term" and "Low-speed steering-rate boost" sections, `late_turn_in_investigation.md`'s Appendix, `tuning.md` §4.3b, `error_state_reference.md` §3.

**Overlap with `docs/reference/superseded_mechanisms.md`:** Deliberate and disclosed (superseded_mechanisms.md's own intro says the lookahead family "has its own document" here). Within the overlap: "Low-speed steering-rate boost" and "Curvature-forcing term" appear in both, but as summary-vs-detail (acceptable layering). "Precomputed corner segmentation" in `superseded_mechanisms.md` is nearly a strict subset of this doc's §7 with no new information — candidate for trimming to a one-line pointer. Minor cross-reference nit found in passing: this doc's §8 says the "full numeric gain-sweep trace" lives in `superseded_mechanisms.md`, but that target section only has a qualitative description; the actual numeric trace is in `late_turn_in_investigation.md` "Part 6b" — the pointer should go there directly.

**Readability:** Good TOC, effective summary table in §2, "show don't tell" code-snippet evidence in §10a. One numbering oddity (10a inserted between 9 and 10) but functional and dated in its own heading.

**Why-preserved-ness:** The doc's whole point is preserving "why," and it succeeds for essentially every mechanism (quantified before/after tables in §4/§5, exact overwrite-order code evidence in §10a). §7 (CornerMap) is the one thin entry — it wasn't rejected on its own merits, just deleted alongside its family — but the doc doesn't falsely claim otherwise.

**Code paths mentioned:** `mpc_core.py`, `mpc_params.py`, `nmpc_core.py` (`_Plant`), `model_utils.py`, `settings.py`, `vehicle_physics.py` (`alat_ceiling_at`), `sim/rollout_core.py`/`rollout_phases.py`, `fsds_simulator/` mirror, `control_mechanisms.md`, `superseded_mechanisms.md`, `offline_live_parity.md`, `late_turn_in_investigation.md`, `tuning.md`, `error_state_reference.md`, `architecture.md`.

**Fix priority:** None load-bearing. Optional: retarget §8's gain-sweep-trace pointer to `late_turn_in_investigation.md` Part 6b instead of `superseded_mechanisms.md`.

---

## 7. `docs/junior_project_mpc_docs.md`

**Purpose/audience:** onboarding tutorial (student project writeup), explicitly exempt from the "no you" rule per project convention — correctly written in second person throughout, appropriately so.

| Claim | Status | Evidence |
|---|---|---|
| 9 tunable numbers (`Q_diag[0:5]`, `R_diag[0:2]`, `R_rate_diag[0:2]`) | OK | Matches `tuner/offline_tuner.py`'s `TUNABLE_Q_IDX`/`_R_IDX`/`_R_RATE_IDX`, whose own docstring literally states "5+2+2=9" |
| 13 raw metrics, names/order | OK | Exact match to `sim/scoring.py`'s `RunAccumulator.finalize()` and independently confirmed against `settings.py`'s `METRIC_SCALES` comment block (same 13 names, same order) |
| `METRIC_SCALES`/`SCORE_WEIGHTS` sum to 1.0, `CONSTRAINT_FLOOR=10.0` backs ">10.0 = crashed" | OK | `settings.py` asserts len==13 for both; `CONSTRAINT_FLOOR=10.0` confirmed |
| `tuning history.txt` exists, format as claimed | **MOSTLY OK, one nuance** | File exists, format matches, but the "Overall score" field is pre-filled with literal text `"Haven't been tested."` in every entry, not literally blank as the doc says. Minor. Doc also doesn't mention the file's own header caveat that pre-2026-08-06 entries are a "closed book," not comparable. |
| `python -m tuner.offline_tuner` resolves | OK | Valid module with `__main__` guard |
| `build_synthetic_paths()`, `VALIDATION_SUITE` | OK | Both present |
| §6.5 settings table (most rows) | OK | `N_HORIZON=35`, `USE_PLANNER=False`, `DELAY_STEPS=1`/`DELAY_JITTER_STEPS=0.2`, `SLAM_NOISE_ENABLED=False`, `MAX_FAILS=5`, `OFFTRACK_LIMIT=TRACK_HALF_WIDTH*1.3` (`TRACK_HALF_WIDTH=1.75`), `ROLLOUT_EPS=1e-4`/`ROLLOUT_MAX_ITER=8000`, `PATH_N_POINTS=1000`, `TIME_OBJECTIVE_WEIGHT=1.0`/`QUALITY_WEIGHT=0.35`, `CONSTRAINT_FLOOR=10.0`/`DNF_PENALTY=3.0`/`DNF_OFFTRACK_PENALTY=3.0`, `FAST_TEST_MODE=False` all confirmed matching |
| `MAX_EVALS` "2500 by default" | **WRONG** | Current value is **1500** (`settings.py` line 1123). Independently confirmed via direct grep. The setting's own inline comment still uses "2500 → 5000" as its illustrative example, itself now stale relative to the live value — looks like both the doc and the code comment were written when `MAX_EVALS` was 2500 and it was since retuned down without updating either reference. |
| §7: "The live ROS 2 side of this project lives... under `fsds_simulator/control/fsae_control/`" | **WRONG / MISLEADING, most significant finding in this doc** | Directly contradicts CLAUDE.md's explicit statement that `fsds_simulator/` is a PR-staging snapshot, not the live/always-in-sync runtime — the actual live tree is `ros2/src/fsae_planning/`. The doc never once names `ros2/src/fsae_planning/` as a concrete path anywhere in all of Section 7. Independently confirmed the two copies have **already drifted**: `diff -q` between the live `mpc_controller.py` and the mirror's copy shows they differ. A reader following Section 7 to "run against the real FSDS simulator" is pointed at the wrong package location, and the code walkthrough that follows may describe stale mirror behavior rather than what's actually running. |
| `controller:=stanley` default, `standalone_output` default `true` | OK | Confirmed against the live (not mirror) `mpc_controller.py` |
| Control loop phases 1-6 | Mostly OK, numbering off | Code labels phases 1/2/3/4/**4a**/5, not a clean 1-6; content of each phase description is accurate, just the doc's own numbering (labeling telemetry-logging "5" and publish "6") doesn't match the code's own phase labels |
| "NMPC and Stanley currently perform similarly overall... LMPC has a structural disadvantage" | **Uncorroborated** | This specific three-way ranking claim has no citable source anywhere else in the docs corpus (searched `nmpc.md`, `tuning.md`, `lmpc.md`, `control_mechanisms.md`). LMPC's turn-in disadvantage alone is well-supported; the NMPC≈Stanley>LMPC ranking is this doc's own assertion only. |
| §3.3 numbers cross-check against `nmpc.md` | OK | 6.45%→0.58%, 54.72s→52.35s (2.37s, doc rounds to "about 2.4") match exactly; offline 12.5%→0.8% independently corroborated in `late_turn_in_investigation.md` |
| 16 absolute `github.com/Martin-Jin/fsae_MPCTest/blob/main/...` links, 0 relative links | **Maintainability defect** | Every other doc in this audit uses exclusively relative `.md` links (nmpc.md: 0 absolute/11 relative; lmpc.md: 0/9; error_state_reference.md: 0/10). This doc's links resolve against whatever was last pushed to GitHub `main`, not the local working tree — any local edit not yet pushed serves stale content to a reader who clicks through, unlike every sibling doc. |

**Overlap:** Substantial but appropriate — Sections 1-4 re-derive `lmpc.md`/`nmpc.md`/`error_state_reference.md` material at tutorial depth, consistently pointer-ifying to the canonical doc for depth. Right shape for onboarding material.

**Readability:** Single H1, clean heading hierarchy, TOC matches headings 1:1. Good tables (§5.3.1 metrics, §6.5 settings).

**Why-explained:** Good throughout (receding horizon, why linear is good enough, why an automatic tuner).

**Code paths mentioned:** (long list) `sim/scoring.py`, `error_state_reference.md`, `control_mechanisms.md`, `removed_mechanisms.md`, `model/bicycle_model.py`, `nmpc_core.py`, `controller/nmpc_optimiser.py` (stale), `tuner/offline_tuner.py`, `settings.py`, `mpc_core.py`, `tuning history.txt`, `gui/simulation.py`, `gui/manual_drive.py`, `offline_guide.md`, `sim/rollout_core.py`, `sim/speed_profile.py`, `sim/sim_track.py`, `architecture.md`, `vehicle_physics.py`, `vehicle_physics_guide.md`, `controller/optimiser.py` (stale), `controller/model_utils.py`, `tuner/performance_stats.py`, `planning/*`, `fsds_simulator/` (whole tree, framed incorrectly as "the live side"), `reference/README.md`, `stanley_controller.py`, `mpc_controller.py`, `fsds_bridge`, `fsds_integration_guide.md`, `ros2/launch_all.sh`.

**Fix priority:** (1) Section 7's "fsds_simulator IS the live side" framing is backwards and should be corrected to name `ros2/src/fsae_planning/` as the actual live runtime, (2) `MAX_EVALS` 2500→1500, (3) convert 16 GitHub links to relative links, (4) minor: phase numbering, "Overall score" field wording, unsourced NMPC≈Stanley ranking claim.

---

## Overall summary of merge/split/rename recommendations

- **No doc needs deleting or merging wholesale.** Each of the seven has a distinct, justified purpose (deep reference vs. tutorial vs. historical-preservation vs. plain-English-physics-companion vs. single-controller-reference), and the overlaps found are mostly disclosed/deliberate summary-with-pointer relationships, not duplication.
- **`docs/reference/superseded_mechanisms.md`**: trim its "Precomputed corner segmentation" entry to a one-line pointer to `removed_mechanisms.md` §7 (near-total duplicate today).
- **`junior_project_mpc_docs.md`**: convert its 16 absolute GitHub links to relative links for consistency with all six sibling docs, and fix the Section 7 "live side" framing — this is the single highest-value fix in the whole batch, since it actively misdirects a new contributor trying to run the project.
- **Mechanical, repo-wide fix** (touches nmpc.md, lmpc.md, error_state_reference.md, junior_project_mpc_docs.md, and files outside this audit's 7): replace every `controller/nmpc_optimiser.py` reference with `controller/nmpc/`, every `controller/optimiser.py` reference with `controller/lmpc/{build,solve}.py`, and every `docs/reference/README.md`-cited-section with the actual file it moved to (mostly `control_mechanisms.md`).
