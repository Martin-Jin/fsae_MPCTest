# Planning/Control Upstream Sync

Reference for re-syncing this repo's `planning/` package and `fsds_simulator/`
ROS 2 node files against a newer clone of the upstream
[`fsae_planning`](https://github.com/UOA-FSAE/fsae_planning) repo. Read this
before touching either directory during a resync — it records which files
mirror which upstream files, which pieces are deliberately *not* mirrored (and
why), and the numeric-parity constants that must stay matched across the
offline/live boundary.

## File mapping

| This repo | Upstream (`fsae_planning`) | Notes |
|---|---|---|
| `planning/boundary.py` | `planning/fsae_planning/fsae_planning/boundary.py` | Direct mirror |
| `planning/cone_map.py` | `planning/fsae_planning/fsae_planning/cone_map.py` | Direct mirror |
| `planning/cone_sorting.py` | `planning/fsae_planning/fsae_planning/cone_sorting.py` | Direct mirror |
| `planning/path_utils.py` | `planning/fsae_planning/fsae_planning/path_utils.py` | Direct mirror |
| — (no file counterpart) | `planning/fsae_planning/fsae_planning/centerline_planner.py` | The ROS 2 planner node itself. Its behaviour (temporal centreline blending via `blend_paths()`, called every planning tick) is reproduced inline by `sim/sim_track.py`'s `SimPlanner.update()` rather than as a ported file — the simulator has no separate planner *node*, `SimPlanner` plays that role directly. |
| `fsds_simulator/mpc_core.py` | `control/fsae_control/fsae_control/mpc_core.py` | The `MPCController` class — QP-based MPC, kept in byte-for-byte parity with `sim/rollout_core.py`/`controller/optimiser.py` per this repo's own numeric-parity rule (see `CLAUDE.md`). |
| `fsds_simulator/control_utils.py` | `control/fsae_control/fsae_control/control_utils.py` | **Partial mirror.** This repo's copy contains only `curvature_speed()`. Upstream's file also contains `StanleyController` — deliberately not ported here (see below). |
| `fsds_simulator/control_node.py` | `control/fsae_control/fsae_control/mpc_controller.py` | **Pattern mirror, not a 1:1 file mirror.** Shares the core idea of computing `desired_speed` locally every tick via `curvature_speed()` instead of subscribing to a speed topic, but `control_node.py` carries substantially more responsibility that upstream's file doesn't have (GO-signal hold, stale-path emergency brake, cone-proximity braking, CSV telemetry logging). Don't expect these two files to ever converge to identical. |

`planning/` and the `MPCController` half of `fsds_simulator/mpc_core.py` are
shared algorithm code and should track upstream closely. `control_node.py` is
this repo's own integration layer built around that shared algorithm and is
expected to diverge.

## Deliberately not mirrored

- **`StanleyController`** (in upstream's `control_utils.py`) — this repo
  already has its own frozen Stanley reference implementation at
  `fsds_simulator/stanley_controller/stanely_control_utils.py` +
  `stanley_control.py`, documented (in `README.md` and in-code) as a
  previous-controller reference only, not the active controller and not kept
  in sync with upstream. Do not port upstream's `StanleyController` into
  `fsds_simulator/control_utils.py`.
- **`control_node.py`'s extra safety phases** (GO-signal hold, stale-path
  emergency brake, cone-proximity braking, telemetry logging) — these are
  this repo's own additions on top of the shared MPC pattern, with no
  upstream counterpart to sync against. Preserve them across any resync of
  `control_node.py`'s core loop structure.
- **The offline oracle speed-profile array** (`sim/speed_profile.py`'s
  `compute_speed_profile()` / `smooth_profile()`) — used only for the
  synthetic/oracle path in `tuner/offline_tuner.py` and `gui/simulation.py`'s
  exact-path mode. Upstream replaced the equivalent live-path logic with a
  scalar `curvature_speed()` call, but this repo deliberately keeps the
  precomputed-array version for the oracle path: it's static across a whole
  rollout, so precomputing it once is a performance win with no accuracy
  cost. The live-path `curvature_speed()`'s dense-resample-and-denoise step
  exists specifically to combat frame-to-frame replanning jitter — a static
  oracle path never has that problem, so there is no equivalent upstream
  logic to port back for the oracle branch. `sim/sim_track.py`'s `SimPlanner`
  correspondingly emits only `.centreline` (no `.v_profile`), matching
  upstream's planner-emits-path-only design; the live-planner branch derives
  its speed on demand via `speed_profile.curvature_speed()` instead.

## Numeric-parity constants

These pairs must stay numerically identical across the offline/live
boundary, or offline-tuned weights will not transfer faithfully to the live
controller. Line numbers below were confirmed by grep at the time of
writing — re-confirm before relying on them, since a resync can move them.

| Constant | Offline copy | Live copy | Current value |
|---|---|---|---|
| `curvature_speed()`'s `a_lat_max` | `sim/speed_profile.py:328` (function default) | `fsds_simulator/control_utils.py:37` (function default) | `4.0` |
| Planner top/bottom speed clamp | `sim/rollout_core.py:55-56` (`PLANNER_V_MAX`, `PLANNER_V_MIN`) | `fsds_simulator/control_node.py:76-77` (`V_MAX`, `V_MIN`) | `20.0` / `1.5` |

Notes on how these are actually used:

- `sim/speed_profile.py`'s `curvature_speed()` (the offline mirror of
  `fsds_simulator/control_utils.py`'s `curvature_speed()`) is called by
  `sim/rollout_core.py`'s `use_planner=True` branch at `sim/rollout_core.py:232-234`,
  which passes `v_max=PLANNER_V_MAX, v_min=PLANNER_V_MIN` explicitly —
  overriding that function's own `v_max=15.0, v_min=1.5` defaults. The live
  side calls the same function the same way: `control_node.py:316` passes
  `v_max=V_MAX, v_min=V_MIN` (also overriding `control_utils.py`'s function
  defaults). So it's the **call-site keyword arguments**
  (`PLANNER_V_MAX`/`PLANNER_V_MIN` vs. `V_MAX`/`V_MIN`), not the functions'
  own default parameter values, that must be kept matched — the function
  defaults themselves are never hit in either the offline or live path.
- `a_lat_max=4.0` is the value actually used (as each function's default,
  not overridden at either call site) and must stay identical between
  `sim/speed_profile.py`'s and `fsds_simulator/control_utils.py`'s
  `curvature_speed()` — see the explicit callout in
  `sim/speed_profile.py`'s `curvature_speed()` docstring, which also notes this
  is deliberately *different* from `compute_speed_profile()`'s own
  `a_lat_max = mu * g` (≈5.886) convention, since that function has no live
  counterpart to stay matched to.

If a resync changes any of these values or call sites, update both sides in
the same change and re-grep this table's line numbers.

## MPC prediction horizon: frozen target speed

`sim/rollout_core.py`'s (and `mpc_core.py`'s) MPC formulation bakes
`desired_speed` into `x0[4]` (`e_v`) as a single scalar frozen for the whole
prediction horizon — this is a pre-existing architectural characteristic,
not something introduced by this sync, and not a bug. See `README.md`'s
state-vector section (search "e_v's target speed is frozen for the whole
horizon") for the full explanation; not repeated here to avoid duplication.

## Resync procedure

1. Clone/update the sibling `fsae_planning` repo checkout used as the sync
   source (gitignored in this repo, e.g. at `fsae_planning/` in the repo
   root).
2. For each file in the mapping table above, read **both** the old (already
   ported) version and the new upstream version in full before porting
   anything — don't diff-and-patch blind. The one mechanical, unavoidable
   difference: upstream's `planning/` files import each other as
   `from fsae_planning.xxx import yyy` (their ROS 2 package is named
   `fsae_planning`); this repo's package is named `planning`, so every ported
   file needs its intra-package imports rewritten from `fsae_planning.xxx` to
   `planning.xxx` — a one-line-per-import search/replace, not a content
   change. `fsds_simulator/mpc_core.py` and `control_utils.py` have no such
   rewrite to do: both are standalone with zero cross-package imports by
   design (see each file's own docstring), so those two are already a literal
   copy-paste once you've re-applied the `control_utils.py`/`mpc_core.py`
   filename split documented in the mapping table above.
3. Port algorithm changes only, preserving this repo's existing import style
   and the deliberate non-mirrors listed above (don't reintroduce
   `StanleyController` into `control_utils.py`, don't strip
   `control_node.py`'s extra safety phases, don't restore a `.v_profile` on
   `SimPlanner`).
4. If the change touches `planning/` or `fsds_simulator/mpc_core.py`, check
   per `CLAUDE.md`'s numeric-parity rule whether `sim/rollout_core.py` needs a
   mirrored change — `rollout_core.run_core_rollout()` and
   `fsds_simulator/mpc_core.MPCController` (formerly `control_utils.py`) are
   two implementations of the same control loop kept in deliberate numeric
   parity. Call this out explicitly in the resync notes if a mirrored change
   is or isn't needed.
5. Re-check the numeric-parity constants table above — if upstream changed
   `curvature_speed()`'s `a_lat_max` or the planner speed clamp values, both
   the offline (`sim/speed_profile.py`, `sim/rollout_core.py`) and live
   (`fsds_simulator/control_utils.py`, `fsds_simulator/control_node.py`)
   copies need the same update.
6. Run the smoke-test pattern from `CLAUDE.md`'s Testing section: confirm
   changed files import cleanly, then run `python -m gui.simulation` (or a
   short `python -m tuner.offline_tuner` run with `FAST_TEST_MODE = True` in
   `settings.py`) against one synthetic path and check the rollout still
   converges and tracks correctly. For any change touching
   `fsds_simulator/mpc_core.py` or `control_node.py`, there is no way to test
   against the real/FSDS car from this repo directly — reason through the
   change against `sim/rollout_core.py` instead and flag it for live testing
   by a human.
