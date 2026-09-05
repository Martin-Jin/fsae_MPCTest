# CLAUDE.md

Formula Student Driverless Simulator: an AirSim/UE4-based sim plus a ROS2
autonomy stack (perception, planning, MPC control), tuned offline against a
Python plant model before being run on the real/simulated car. There is no
single build command across the four repos below; the closest thing to a
correctness bar after a planning/control change is `python -m
tuner.recorded_map_rollout` (offline scoring reproduction, ~2 min) and, for
NMPC-specific numerical changes, `python -m tuner.nmpc_offline_check`, both
run from `fsae_MPCTest/`. Neither is a real test suite; see "Testing" below.

**This is the only `CLAUDE.md` for this project.** `fsae_MPCTest/` and
`fsae_autonomous/` do not get their own. If either one now has a
`CLAUDE.md`, that's drift, not intent: check for it, and if found, remove
it and fold anything load-bearing from it into this file instead.

## Repo roles: sim, staging, and the actual car

Four repos are in play, and they are not interchangeable. See "Git layout"
below for paths/branches and "Git workflow per repo" for what's pushable
where:

- **`fsae_autonomous`** is the **production repo**, what actually gets put
  on the real car for testing. It is smaller than the sim tree: no FSDS
  bridge, no offline scoring/telemetry glue, and as of this writing no MPC
  controller has landed there at all (only `stanley_controller.py` exists
  under its `control/fsae_control/`). It lags the sim repos by design,
  because it only ever receives a deliberate, tested push from the user
  (see "Git workflow per repo" below, agents never push here), not a
  continuous mirror.
- **The outer FSDS repo + `fsae_planning`** are **purely for simulation**,
  primarily where the MPC controller itself gets developed, but also usable
  to validate other controllers (Stanley, LTV-QP, NMPC) and planning
  algorithms (`centerline_planner.py`, boundary/cone-sorting logic) against
  FSDS before anything is trusted enough to move toward the car.
- **`fsae_MPCTest`** is where offline tuning happens, and it doubles as the
  **change ledger for the sim ROS2 tree**: since nothing is ever pushed to
  `fsae_planning` (see below), `fsae_MPCTest/fsds_simulator/` is what
  actually stores the history of every planning/control change made in
  `ros2/src/fsae_planning/`, mirroring changed files at their **same
  relative path** (e.g. a change to
  `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_core.py`
  is mirrored to
  `fsae_MPCTest/fsds_simulator/control/fsae_control/fsae_control/mpc/mpc_core.py`).
  See "Third copy" below for the exact mirroring rules (don't add files
  that were never there, don't fix unrelated drift).
- **`fsae_MPCTest` also contains a second, simpler simulator**, a 2D
  matplotlib-based tool (`gui/simulation.py`, distinct from the headless
  `sim/rollout_core.py` FSDS-facing rollout) for visualization and automatic
  MPC tuning. Useful for quickly prototyping a concept, but its dynamics do
  not match FSDS. Treat any result from it as a rough signal, not a
  validated one, and re-check anything that matters against the FSDS-facing
  offline sim (`tuner.recorded_map_rollout`) or FSDS itself before trusting
  it.

## Model usage (token efficiency)

Default to a mid-tier model for the main session; this project's day-to-day
work (weight tuning, bug investigation, doc updates) doesn't need the most
expensive tier by default.

For subagents:

- **Cheap/low-effort**: grepping for a symbol or field across the repos,
  reading files to answer "does X exist / what's its current value," tracing
  a definition/usage, mechanical rename/formatting edits.
- **Default tier**: most bug fixes, doc updates, and single-side
  planning/control edits.
- **Expensive/high-effort, last resort.** Reserve for the specific danger
  zones named throughout this file, where a wrong first answer is expensive
  to unwind: the vehicle/plant model in `model/vehicle_physics.py`
  (`alat_ceiling*` and friends, a wrong physical mechanism can pass offline
  validation and still be wrong, as the tyre-parameter dead ends recorded in
  `sim_to_real_investigation.md` show), any edit that touches **both** sides
  of the live/offline parity boundary at once, and the NMPC solver internals
  in `nmpc_optimiser.py`/`nmpc_core.py` (state-anchoring, Hessian sparsity,
  see the `nmpc_rjerk_delta` implementation notes). Plain MPC weight
  retuning is not in this tier by itself, it's default tier plus the
  parity discipline below.
- **Effort ceiling:** keep parity-check and doc-sync subagents at low/medium
  reasoning effort. The failure mode there is reading only one side's
  *default* value and missing a runtime override (e.g. a `launch_all.sh`
  shortlist), which is a thoroughness problem, not a reasoning-depth one.

Don't spawn a subagent to check one known field in one known file, just
read it.

## Cross-cutting concerns

This project's interconnected systems, and what "done" means for a change
touching each. The sections below cover them in detail; this is the index.

- **Live/offline MPC parity** (next section, and "Single source of truth for
  MPC tuning" below it): two numerically-identical copies of the planning
  stack. Silently breaks: weights tuned offline stop being valid on the car.
  Watch for: any edit to `mpc_params.py`/`settings.py`, the vehicle/plant
  model, or delay/tracking-error math that only touches one side.
- **Git layout** ("four independent repos" below): four repos, two of them
  never pushed to by an agent at all (`fsae_planning`, `fsae_autonomous`).
  Silently breaks: an agent commit landing in either of those two repos
  when it should have stayed local; `git status` at the wrong root not
  showing the other repos' changes at all.
- **Track data placement** ("Track data lives in `fsae_planning`"): track
  files must ship with `fsae_planning`, not `fsae_MPCTest`. Silently breaks:
  a fresh `fsae_planning`-only checkout can't drive because a new track was
  only ever written to the `fsae_MPCTest` side.
- **`fsds_simulator/` PR-staging mirror** ("Third copy" below): a snapshot,
  not an always-in-sync mirror, but files present in both copies must match.
  Silently breaks: a live-side fix ships without its mirror counterpart;
  `git status`/`.gitignore` interactions can make a present, identical file
  look "missing" (see that section's `.gitignore` note). Verify with
  `ls`/`find`, not `git status`, before concluding a file needs adding.
- **Scoring parity** ("Scoring parity" near the end): `sim/scoring.py` is
  the source of truth, the live copy is inlined but must match numerically,
  and the live *caller* has to feed it real progress/completion data or a
  parity-correct formula still produces a meaningless constant score.

## Planning/control parity: sim and live must stay in sync

The MPC and planning stack exists in two places, and they must be kept
numerically identical:

- **Live ROS2 nodes**: `ros2/src/fsae_planning/` (e.g.
  `control/fsae_control/fsae_control/mpc_core.py`, `mpc_controller.py`,
  `mpc_controller_standalone.py`). This runs on the real/simulated car.
- **Offline simulator/tuner**: `fsae_MPCTest/` (e.g. `sim/rollout_core.py`,
  `settings.py`). This is where weights and models are tuned and validated
  before being copied into the live nodes.

Any change to planning/control logic, MPC weights, the vehicle/plant model,
delay handling, tracking-error math, tuning parameters, etc., must be
applied to **both** copies. Weights and behavior tuned offline are only
valid on the real car if the live code matches the simulator exactly.
Treat a one-sided edit to either side as incomplete.

### Single source of truth for MPC tuning, per side

Every MPC weight/gain/flag (Q/R/R_rate weights, adaptive-gain shape
constants, feature-enable flags, ~56 values total) is centralized, one
place per side, rather than scattered across hardcoded literals:

- **Live**: `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_params.py`'s
  `MPCParams` dataclass. `mpc_core.py`'s `MPCController` accepts an
  `MPCParams` instance (defaulting to `DEFAULT_MPC_PARAMS`); the ROS2 nodes
  (`mpc_controller.py`/`mpc_controller_standalone.py`) build one from
  declared ROS2 parameters (see `declare_mpc_params`/`mpc_params_from_node`
  in that file), which are in turn exposed as launch args in
  `control.launch.py`/`sim.launch.py` (generated from `MPCParams`' own field
  metadata, not hand-written) and as YAML defaults in
  `common/fsae_bringup/config/fsae_params.yaml`'s `controller:` block.
  `ros2/launch_all.sh` carries a small shortlist of the most commonly
  retuned fields as commented-out overrides.
- **Offline**: `fsae_MPCTest/settings.py` remains the one file to edit,
  every `MPCParams` field has a matching uppercase constant there (e.g.
  `MPCParams.q_e_y` ↔ `settings.Q_diag[0]`, `MPCParams.adaptive_r_rate_during_floor`
  ↔ `settings.ADAPTIVE_R_RATE_DURING_FLOOR`), imported and threaded through
  `sim/rollout_core.py`'s per-tick calls into `controller/model_utils.py`'s
  adaptive-gain functions.

`fsae_MPCTest` cannot import anything from `fsae_planning` (the live node's
runtime is not on `fsae_MPCTest`'s Python path, and vice versa, the
standing "no settings.py-on-the-car" constraint), so the two are kept
numerically identical **by hand**, the same as `Q_diag`/`R_diag`/`R_rate_diag`
always have been. Extend both sides together, never one without the other.
See `fsae_MPCTest/docs/planning_control_sync.md`'s "Numeric-parity
constants" table for the authoritative field-by-field mapping.

These weights are **not unit-normalized**, the MPC's own QP cost applies
each raw weight directly to its raw-unit error term (`q_e_y` on metres²,
`q_e_psi` on radians², etc.), unlike `fsae_MPCTest/sim/scoring.py`'s
`METRIC_SCALES`, which explicitly divides each scoring metric by a typical
magnitude before weighting. Don't assume two `MPCParams`/`settings.py`
values are on a comparable scale just because they're both "a weight."

## Git layout: four independent repos

`git` at the FSDS root does **not** see the planning/control work. There are
four separate repos:

| Path | Repo | Branch | Role |
|---|---|---|---|
| `/` | upstream `Formula-Student-Driverless-Simulator` | `master` | sim host |
| `ros2/src/fsae_planning/` | `fsae_planning` (own `.git`) | `main` | sim-side planning/control dev |
| `fsae_MPCTest/` | `fsae_MPCTest` (own `.git`) | `main` | offline tuning + change ledger for `fsae_planning` |
| `fsae_autonomous/` | `fsae_autonomous` (own `.git`) | `main` | **production, goes on the real car** |

`fsae_planning` and `fsae_MPCTest` are nested inside the outer tree;
`fsae_autonomous` is a sibling checkout, not nested under the outer repo at
all. But all three inner repos are untracked from whichever repo you'd
otherwise expect to see them from, so `git status`/`git log` run from the
wrong root shows them only as `??` untracked directories and cannot resolve
their commits. **Run git inside the actual repo directory** to inspect its
history, and note that `git worktree` on the outer repo produces a tree
containing *none* of the planning/control code.

### Git workflow per repo

These repos are not equally writable and don't follow one uniform branch
policy. Check which repo you're in before applying a rule:

- **`fsae_planning`**: read-only for agents, and never used to store
  history: nothing is ever pushed here at all (that's what
  `fsae_MPCTest/fsds_simulator/` is for, see "Repo roles" above). Never
  commit or push here, even for a confirmed, offline-validated fix; make
  the local edit if asked, then report the change back to the user instead
  of committing. This is a hard rule, not a default, see the standing note
  on this.
- **`fsae_MPCTest`** and the outer FSDS repo: day-to-day tuning/doc commits
  have been going straight to `main`/`master` (no feature branch, no squash
  step) and that is the established pattern here; don't unilaterally start
  branching for routine parity/doc/weight commits. Do use a throwaway branch
  for anything you'd want to abandon cleanly (an experimental model change,
  a multi-commit investigation that might not pan out).
- **`fsae_autonomous`**: the production repo. **Never push here as an
  agent, full stop, the same hard rule as `fsae_planning` above, not a
  wait-for-explicit-ask gate.** Pushes to this repo happen deliberately and
  rarely, only to a branch (never `main` directly), and only once a major
  version upgrade is fully tested and working, and only the user does that
  push themselves. Make local edits here if asked, and report back what
  changed instead of committing or pushing. Offline validation is not the
  bar for this repo, a full tested car-ready upgrade is. **Pull `main`
  before working in this repo each session.** It moves over time
  independently of the other three (other contributors, or the user's own
  past pushes merged upstream), so a local checkout can go stale between
  sessions in a way that isn't true of the same-session-owned tuning repos.
- **Authorship**: commits pushed on the user's behalf use the user as sole
  author, no `Co-Authored-By` trailer unless asked. (Some existing commits
  in `fsae_MPCTest` do carry one from before this was clarified; don't treat
  that history as the current convention.)
- **Concurrent sessions**: more than one agent/session may be working in
  these checkouts at once, see the standing note on this. Check `git
  status`/`git worktree list` for changes you don't recognize before
  branching or committing, and untangle any mixup with a targeted `git
  stash push -- <paths>` rather than a blanket reset.
- **Never force-push.** None of these four remotes are known to have a
  separately-diverged deploy history requiring cherry-pick reconciliation;
  if that ever changes (e.g. a remote gets rewritten independently), get
  explicit confirmation before any push that would rewrite it back.

## Track data lives in `fsae_planning`, not `fsae_MPCTest`

`fsae_planning` (+ the outer FSDS repo) must be fully drivable with **no**
`fsae_MPCTest` checkout at all. Clone just those two, run
`ros2/launch_all.sh`, and an already-recorded track drives immediately. That
requirement means the track data itself, not just the code that reads it,
has to ship with `fsae_planning`.

Canonical storage is `ros2/src/fsae_planning/tracks/<name>/` (`cone_map.json`
+ `speed_profile.csv` + `raceline.csv`), committed inside that repo.
`control.launch.py`/`sim.launch.py`'s `map_path`/`path_map_path` defaults and
`ros2/launch_all.sh`'s `TRACK=` variable all point there directly.

`fsae_MPCTest` is only where **new** tracks get produced: record a lap,
then run `tuner.export_speed_profile`/`tuner.raceline_optimizer`.
`fsae_MPCTest/tracks/__init__.py`'s `TRACKS_DIR` constant points across the
repo boundary at `ros2/src/fsae_planning/tracks/`, so those tools (and
`gui/simulation.py`'s **Load Recorded Track**, and every offline tuner script
that imports `recorded_map_rollout.DEFAULT_MAP`) write/read there directly
when both repos are checked out side by side, no manual copy step needed in
that case. A capture made from `fsae_MPCTest` alone with no `fsae_planning`
sibling present has nowhere real to land; `fsae_MPCTest/tracks/` itself holds
only the `tracks` package's Python code, no data.

Editing anything under `ros2/src/fsae_planning/tracks/` is an edit to the
**`fsae_planning` checkout**, subject to [[Never push to fsae_planning
repo]]: local-only, never committed/pushed there by an agent, report changes
instead.

## Third copy: `fsae_MPCTest/fsds_simulator/` is a PR-staging area, not a mirror

`fsae_MPCTest/fsds_simulator/` holds a snapshot of the `fsae_planning` ROS 2
workspace. Its purpose is to stage changes that get pull-requested into the
separate `fsae_planning` repo later. It is not meant to be a live,
always-in-sync mirror like the parity rule above.

It has since grown into a mirror of the **whole** workspace, not the partial
control-file subset this section used to describe: `control/fsae_control/`
now includes `mpc_controller.py`, `telemetry_logger.py`, `scoring.py`,
`fsds_bridge.py` and `stanley_controller.py`, and there are full
`common/`, `perception/` and `planning/` packages too (including
`centerline_planner.py`). See `fsae_MPCTest/docs/planning_control_sync.md`
for the authoritative file-mapping table, trust that over any list here.

When a change lands in a file that already exists in `fsds_simulator/`,
apply the same change there too. But:
- **Do not add files** to `fsds_simulator/` that were never there, check
  first rather than assuming from the list above.
- **Do not fix unrelated pre-existing drift** you happen to notice there
  (e.g. stale tuned weights from an older tuning run) unless asked,
  only propagate the specific change you're actually making.

**When checking "does this file already exist in `fsds_simulator/`," use
`ls`/`find`, not bare `git status`.** `fsae_MPCTest/.gitignore` had a bare
`fsae_planning/` pattern (fixed 2026-08-07 to `/fsae_planning/`, anchored to
repo root) that matched *any* directory named `fsae_planning` at any depth,
including the mirror's own `fsds_simulator/planning/fsae_planning/`, so
`git status` silently showed that entire package (not just one file) as
absent, when in fact `boundary.py`, `centerline_planner.py`,
`cone_sorting.py`, `path_utils.py`, and more were present on disk and
byte-identical to the live copies the whole time. A gitignore rule can hide
"this file already exists" just as effectively as the file actually being
missing. Check the filesystem, or `git status --ignored`, before concluding
a file was "never there."

## Known, deferred issues

Real, understood bugs left deliberately unfixed, so they aren't
re-discovered, re-litigated, or accidentally re-fixed in a way that
contradicts what was already decided. Add to this list whenever a fix is
knowingly deferred, not just when asked to.

- **Centreline curvature spikes** (next section): known planner defect;
  the controller carries workarounds for it. Not fixed because the
  workarounds are working defence-in-depth and no one has re-derived the
  planner's smoothing to remove the root cause yet.
- **Periodic car-position teleport** ("Open, unsolved bug" below): ~31.7s
  periodic pose discontinuity, root cause narrowed to `fsds_ros2_bridge`'s
  `getCarState()` RPC path but not confirmed. Not fixed because the next two
  diagnostic experiments (a `/clock` arrival-rate capture, instrumenting
  `odom_cb` directly) haven't been run yet, see that section for exact
  next steps.
- **Sim-to-real gap, partially closed** ("does not yet fully predict the
  car" below): an `alat_ceiling` model closes most of the gap but a
  residual remains (sim 4.8% steering saturation vs. live 21.1%). Not fixed
  because the leading candidate (planner reference-heading lead, §12.8 of
  `sim_to_real_investigation.md`) is a new investigation, not a known fix
  waiting to be applied. Read that doc before proposing a cause, several
  plausible-looking ones are already eliminated by measurement there.

## Before changing the planner: read the open centreline-quality issue

There is a **known, unfixed defect in the planner's centreline** documented in
`fsae_MPCTest/docs/planning_control_sync.md` ("Known planner defect: centreline
curvature spikes"). Read that section before touching `centerline_planner.py`,
`boundary.py`, `cone_sorting.py`, `path_utils.py`, or the planner's smoothing
parameters, the controller currently carries **workarounds** for it
(curvature smoothing, a tracking-error speed gate, and a speed-target rise
limiter in `control_utils.py` / `mpc_controller*.py` / `sim/rollout_core.py`).

Two consequences:
- If you fix the planner, those workarounds may become unnecessary or
  mis-tuned. Re-measure before removing any of them, they are defence in
  depth, not redundant.
- If you change the planner without reading it, you may re-introduce or mask
  the defect and misattribute the resulting control behaviour.

## Open, unsolved bug: periodic car-position teleport (not a controller issue)

There is a **known, unexplained failure mode** where the car's reported
`(x, y, yaw)` discontinuously jumps several metres in a single 50 ms tick,
roughly every ~31.7 s, fully documented in
`fsae_MPCTest/docs/logs/periodic_pose_teleport_investigation.md`. Read that
doc in full before investigating any "car randomly steers off" / "drives off
at certain places" report. It is easy to mistake this for a controller or
planner bug and re-tune around a symptom with a perception-layer cause.

**What's already ruled out** (two live capture rounds): `sim_perception.py`
(publishes a perfectly clean 20 Hz throughout, both captures), the LTV-QP vs.
NMPC vs. Stanley controller choice (reproduces on all three), the planner
(reproduces with a precomputed static path, no planner running), and FSDS's
own simulation clock (`clock_drift_check.py` shows sim-time/wall-time ratio
~0.998 with no dip at any of the teleport events themselves). The leading
candidate is `fsds_ros2_bridge`'s `getCarState()` RPC path specifically, see
the doc's "Second capture result" and "Next step" sections for why, and for
the two concrete next experiments (a `/clock` arrival-rate capture, and
instrumenting `odom_cb` directly) neither of which has been run yet.

`ros2/launch_all.sh` already launches the diagnostic captures this needs
(search for `TEMPORARY (2026-08-19)`), logging to
`fsae_logs/topic_hz_diagnostics/`, so a fresh investigation can start
capturing immediately without adding new instrumentation first, other than
the `/clock`-rate capture the doc's "Next step" section calls for.

## The offline sim does not yet fully predict the car, read before tuning

The main cause of the `fsae_MPCTest`-vs-car gap has been found and **modelled**,
but the gap is only **partly** closed. Same map (`comp test map 3`), same gains:

| | sim (no ceiling) | sim (now) | live car |
|---|---|---|---|
| steering saturation | 4.4% | **4.8%** | **21.1%** |
| \|e_psi\| mean / p90 | 6.3 / 14.2 | **6.9 / 18.5** | **15.9 / 42.0** |
| a_lat max | 14.06 | **11.24** | 12.34 |
| a_lat > 7.5 | 14.2% | **10.9%** | 9.8% |

**Still do not trust an offline score on its own.** Live saturates 4× more
often than the sim and carries 2× the heading error. Always validate on the car.

Reproduce this table any time with
`python -m tuner.recorded_map_rollout` (headless, ~2 min).

**CAUSE (2026-08-06): FSDS enforces a sustained lateral-acceleration ceiling of
~7.5 m/s².** Measured by open-loop system-ID (fixed steering at fixed speeds,
MPC bypassed) plus a step-input test:

| speed | commanded steer | achieved | ratio |
|---|---|---|---|
| 3–5 m/s | 25° | 25° | **1.00** |
| 8 m/s | 25° | **8.5°** | 0.34 |
| 14 m/s | 25° | **4.1°** | 0.17 |

Below ~6 m/s the car never reaches the ceiling and FSDS delivers the commanded
angle exactly; above it the yaw response collapses. The cap is far below the
12.3 m/s² the same car reaches on a lap, so it is **not** tyre saturation. A
grip limit does not depend on speed. It is a *sustained* ceiling, not a wall:
the live car exceeds it on 9.8% of ticks (peak 12.34) in bursts of ~0.05 s.

That explains the closed-loop symptom: the MPC plans a corner, FSDS clamps the
yaw, heading error builds, the controller demands more steer and hits the 25°
stop → 21% steering saturation on every lap.

**Modelled** in `model/vehicle_physics.py` as `alat_ceiling*`, a restoring yaw
moment with a first-order lag (state `IDX_ALAT_LIM`), *not* a clip. Set
`alat_ceiling_enabled=False` to recover the unconstrained plant for
real-vehicle work.

**The law was corrected on 2026-08-07** from proportional-on-excess to a leaky
**integral** (`alat_ceiling_mode = 'pi'`, `gain` 700 → 450). A proportional term
needs a finite error to produce output, so its equilibrium *must* sit above the
setpoint, which is why fitting the peak left sustained cornering 13% high and
fitting the settled value flattened the excursions and DNF'd. Those were not two
bad guesses but **two ends of one structural flaw**. The integral form pins the
settled value at the ceiling by structure for any gain, so `gain` is fitted to
the measured peak alone and the settled value falls out correctly; on the
held-out sweep the capped-point error improves 5×.

**`alat_ceiling_tau` was measured on 2026-08-07** (0.40 s, was 0.25 behavioural)
with a longer `step_s=8.0` (see `planning_control_sync.md` for the command).
Under the integral law it affects *only* the transient. **It did not close the
saturation gap** (moved the *wrong* way, 6.7% → 4.8%, away from live's 21.1%).
The residual is a planner/reference problem, not this parameter. Do not
re-litigate `tau` without new evidence; read
`sim_to_real_investigation.md` §12.12 first.

**Still open:**
- **The measured ceiling is speed-dependent and the model is not** (6.45 @ 8 m/s,
  7.54 @ 11, 9.26 @ 14 vs a flat 7.5). Left unfitted on purpose, 16 points, one
  run.
- **The reference-heading lead (§12.8).** In both stacks the planner's
  reference heading swings faster than either car can ever yaw, and drives
  78–100% of heading-error growth. This is the next thing to pursue, and it is
  testable **offline**, no FSDS session required.

Re-run the measurements any time with `ros2/run_steering_sysid.sh` (sweep) and
`ros2/run_steering_step.sh` (transient); details in
`fsae_MPCTest/docs/planning_control_sync.md` under **"MECHANISM: a
dynamically-enforced lateral-acceleration ceiling"**.

**Before re-testing any candidate cause, read
`fsae_MPCTest/docs/logs/sim_to_real_investigation.md`**, the full history of what
was tried and eliminated (plant grip, planner quality, SLAM noise, latency,
tyre understeer, `MAX_STEER_RAD` scaling, actuator lag, longitudinal scaling,
and more). Several dead ends were individually convincing; one produced an
*exact* numerical match and was still wrong.

Consequences:

- **Do not imitate the cap with tyre parameters.** Scaling `mu` was already
  tried and failed (μ=1.455 matches the understeer gradient, then fails the
  full-lock and closed-loop checks); reaching the measured `K_us` through
  cornering stiffness needs `C_f` at 10% of a physical value. Both would wreck
  the plant's genuine grip while imitating a yaw limit. The `alat_ceiling*`
  model is the right mechanism, tune that, not the tyres.
- **A residual gap remains and is not yet explained** (sim 6.7% saturation vs
  live 21.1%). Do not assume the remaining difference is tuning; it was not
  last time. Investigate it with the same open-loop discipline before trusting
  a tuned weight set. Two candidates are now **eliminated by measurement**:
  cornering capability (lowering the ceiling DNFs offtrack, the wrong failure
  mode) and `fsds_bridge` discarding `a_cmd` (inside saturation the sim already
  arrives *hotter* than live). Read `sim_to_real_investigation.md` §12 before
  re-testing either.

**Before changing the plant, check it still reproduces the measurements:**
`python -m tuner.plant_openloop_validation` replays both open-loop experiments
through the offline plant at matched (speed, steering). The absence of this check
is exactly how the 13% sustained-cornering surplus survived a refit. Its
low-speed rows are a known rig confound, not a finding, see the caveat it prints.

## Writing style for docs, logs and comments

Keep docs and logs **concise, separated into clear sections and/or bullet
points.** Prose that runs for paragraphs without structure does not get read,
and these files exist to be read by the next session under time pressure.

Applies to every edit to `docs/*.md`, `docs/logs/*.md` and this file:

- Lead with the conclusion, then the evidence. A reader who stops after two
  lines should still have the finding.
- Prefer a table for anything with more than two measured quantities.
- One idea per bullet; split a bullet that needs an "and also".
- Give a section a heading that states its claim, not its topic
  ("`k=27` is load-bearing for the zone", not "About `corner_factor_k`").
- Record what was **falsified** as explicitly as what worked, with the
  numbers, a null result nobody can find gets re-tested.

**Explain every finding in plain English as well, without losing the detail.**
Assume the reader does not know what a QP, a Frenet frame or a second
difference is. Give the plain-language version first (what happens to the car,
and why it matters), then the precise mechanism and the numbers. Do not drop
the technical content to achieve this, a doc that is readable but no longer
sufficient to act on has failed differently. A symbol or term used for the
first time gets a short gloss in place.

**Write these as standalone documents, not as transcripts.** They should read
as something a person sat down and wrote. Concretely:

- No conversational framing: no "as we found", "let's look at", "I measured",
  "you asked about", no addressing a reader as "you".
- No session or chat scaffolding: no "Session N", no turn-by-turn narrative of
  what was tried in what order unless the order is itself the finding.
- State findings in the present tense as properties of the system ("the zone
  never engages at `k=8`"), not as events that happened to someone.
- Attribute measurements to the measurement, not the measurer ("measured live
  over three laps", not "I ran three laps and found").

Code comments follow the separate rule already in force: explain *why* and
*what to be careful of* in the current state. No dates, no changelog, no
score history, those belong in `docs/logs/`.

**No em dashes.** Use a comma, a period, or "and"/"but" instead. Plain
sentences read faster and are easier to skim under time pressure than a
sentence broken up with dashes.

**Don't editorialize with intensifiers.** Words like "genuinely,"
"actually," "really," or "crucially" don't add information, they just make
a plain fact sound more dramatic than it is. State the fact directly. If a
fact needs emphasis, use bold or a heading, not an adverb.

## Scoring parity: live runs are scored by the offline formula

`fsae_MPCTest/sim/scoring.py` is the single source of truth for the composite
score. `ros2/src/fsae_planning/control/fsae_control/fsae_control/scoring.py`
is a **verbatim copy** of it, so a score logged on the car is directly
comparable to one from `tuner/offline_tuner.py`. Change `sim/scoring.py`
first, then re-copy, never edit the live copy directly. The one intentional
difference is that the live copy inlines `SCORE_WEIGHTS` and the
bonus/penalty constants (no `settings.py` on the car's PYTHONPATH); those
must be kept numerically identical.

**`scoring.py` parity alone does not make a live score meaningful, the
caller has to feed it real inputs too (fixed 2026-08-11).** Until this fix,
both live controller nodes called `telemetry.close()` with no arguments,
so `progress` defaulted to `0.0` and `reached_end` to `None`;
`compute_composite_score()` reads that as "never finished" and returned
`CONSTRAINT_FLOOR + DNF_PENALTY = 13.0` on every single run, regardless of
driving quality. `telemetry_logger.LapProgressTracker` now derives real
`progress`/`reached_end`/`time_bonus` from the car's position against a
precomputed track path (when one is loaded via `map_path`), see
`fsae_MPCTest/docs/planning_control_sync.md`'s "Live/offline score parity"
section for the mechanism. A run against the live planner topic instead of a
precomputed path still has no known path end, so it still scores
`score_is_partial=1`.

## Changelog / versioning

No user-facing changelog or version number exists in this project, it's a
research/tuning codebase, not a shipped product with an end-user release
surface. Don't add one speculatively; if that ever changes, state the
scheme here at that point.

## Development practices

- Read the actual file before editing it. Several of the sections above
  exist precisely because a plausible-looking value (a dataclass default, a
  doc's parity table) turned out not to be what's actually running; don't
  reconstruct current values from memory or from one file when a runtime
  override (a launch arg, a YAML default) can supersede it.
- No linter or formatter is configured in any of the four repos, match
  the surrounding file's style by eye.
- Before editing in `fsae_autonomous`, pull `main` first (see "Git workflow
  per repo" above). Unlike the tuning repos, it moves independently of
  this session between visits.
- Match effort to scale: this is a small/medium research project (a handful
  of contributors, no external users), not a large production system.
  Don't add config plumbing, feature flags, or speculative abstraction for
  hypothetical future needs. See "Single source of truth for MPC tuning"
  above for the one place this project *has* deliberately centralized
  config, and why (56 values that must track 1:1 across a hand-maintained
  boundary, that's a real, present need, not a speculative one).
- Never commit credentials/secrets in plaintext. This project has no auth
  layer today; if one is added, check before staging anything touching it.
- Before changing a shared file with dependents (`mpc_params.py`,
  `scoring.py`, anything under `tracks/`, the `MPCParams` dataclass shape),
  check every consumer named in the cross-cutting-concerns sections above,
  not just in-repo callers; `fsae_MPCTest` cannot import `fsae_planning` (or
  vice versa) so a dependent on the other side of that boundary will not
  show up in an in-repo search.
- **Be careful about deleting files**, especially inside `fsae_planning`
  (read-only for agents, report a proposed deletion rather than making it)
  and inside `tracks/` (a track being removed may still be referenced by a
  `TRACK=` default elsewhere). Prefer moving a file to a sibling
  `deleted/<original path>` over an outright delete so it's recoverable.

## Data retention

`fsae_logs/` (gitignored, root of the outer repo) holds timestamped
per-run CSVs (`mpc_standalone_control_*.csv`, `stanley_path_*.csv`, the
`topic_hz_diagnostics/` captures, etc.) with **no retention policy and no
cleanup step today**, it grows without bound as runs accumulate. This is a
known gap, not a deliberate "unbounded because X already bounds it
elsewhere". Don't assume it's handled. If you add a new category of logged
run data, at minimum name that here; implementing rotation/pruning is worth
doing opportunistically but hasn't been asked for yet.

`fsae_MPCTest/docs/logs/` is different: those are curated, hand-written
investigation logs (small, text, committed to git), not raw run output,
no retention concern there, they're meant to accumulate as project history.

## Testing

There is no formal test suite (no `pytest`/`unittest` test files) across any
of the four repos. The closest things to a correctness bar:

- **`python -m tuner.recorded_map_rollout`** (from `fsae_MPCTest/`,
  ~2 min, headless): replays a recorded map and reproduces the
  live-vs-offline comparison table used throughout this file. Run this
  after any change to MPC weights, the plant model, or scoring, it is the
  standard way an offline claim in this file gets re-verified.
- **`python -m tuner.plant_openloop_validation`**: replays the two
  open-loop system-ID experiments (steady-state sweep, step input) through
  the offline plant and checks it still reproduces the measured numbers.
  Run this after any change to `model/vehicle_physics.py`, its absence is
  exactly how a bad refit survived undetected once before (see "The offline
  sim does not yet fully predict the car" above).
- **`python -m tuner.nmpc_offline_check`** (from `fsae_MPCTest/`): solver
  self-consistency checks (`_step_scalar == _step`, SQP monotonic
  convergence, turn-in sign checks) for the NMPC path specifically. Run
  after touching `nmpc_optimiser.py`/`nmpc_core.py`.

None of these substitute for a live/sim run, see "The offline sim does not
yet fully predict the car" above. **Always validate a planning/control
change on the car (or at least a full FSDS sim session) before trusting an
offline score on its own**; the documented live-vs-sim gap in that section
is exactly the failure mode of stopping at the offline check.

If a failing run looks flaky rather than truly broken, re-run it at
most three times total before concluding either way, don't loop
indefinitely chasing a clean signal, particularly given the known periodic
teleport bug above, which can itself look like flakiness.

## Design direction

No UI in this project (ROS2 nodes, an offline Python tuner/sim, and the
upstream AirSim/UE4 simulator's own existing UI, which this project doesn't
modify). This section doesn't apply; if a UI is ever added here, state the
responsiveness/platform bar at that point.

## Code review checklist

- Maintainable, efficient, and idiomatic Python for the stack in use (ROS2
  nodes / offline numpy-based sim).
- Comments explain *why*, not *what*, see "Writing style for docs, logs
  and comments" above; the same non-obvious-only rule applies to code
  comments, and dates/tuning-history/scores belong in `docs/logs/`, not
  inline.
- When a fix corrects an earlier wrong attempt, say why the earlier attempt
  failed, not just what the new one does. Several sections above
  (`alat_ceiling_mode`, `corner_factor_k`) exist specifically to stop a
  previously-eliminated cause from being re-tried.
- No duplicated logic where an existing utility already does it, check
  `controller/model_utils.py` / `control_utils.py` before writing a new
  helper on either side.
- **Every cross-cutting concern the change actually touches has been
  updated on every side it applies to**, not just the side you were
  originally asked to change: live+offline MPC params, live+offline
  scoring, and (when the change lands in a file the mirror already carries)
  the `fsds_simulator/` copy.
- Any new persisted state (a new log file category, a new cached artifact)
  has a stated retention answer (see "Data retention" above) or is
  explicitly noted as deferred.
- The correctness bar for the area touched has been run (not just
  "should pass"), see "Testing" above, and, for anything planning/control
  related, validated live or in a full sim session, not just offline.
- No secrets committed in plaintext.
- No new files added to `fsds_simulator/` that weren't already there (see
  "Third copy" above), and nothing committed/pushed to `fsae_planning` or
  `fsae_autonomous`.
