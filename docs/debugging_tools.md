# Debugging Tools

Catalog of the diagnostic/debugging tools across this repo and the outer
`ros2/` scripts: which question each one answers and how to run it. For
*why* a tool was built, or the investigation it came out of, see
`docs/logs/`, this document only covers current usage.

## Which tool for which question

| Tool | Question it answers | Run with |
|---|---|---|
| `gui/launcher.py` | Where's the fastest way to launch the sim, debug a log, run the offline sim, or retune a weight, without editing a script or remembering a CLI? | `python -m gui.launcher` |
| `live_viz.py` | Which tracking error or control-law term is actually driving the car's steering/throttle right now, live, whichever controller is running? | launched automatically by `ros2/launch_all.sh` (or the launcher's Launch Sim tab); standalone: `ros2 run fsae_control live_viz` |
| `tuner/recorded_map_rollout.py` | Does a plant/weight change still reproduce the published closed-loop baseline on the recorded map? | `python -m tuner.recorded_map_rollout` |
| `tuner/checks/plant_openloop_validation.py` | Does the offline plant model reproduce FSDS's measured open-loop behaviour? | `python -m tuner.checks.plant_openloop_validation [--ab] [--robustness]` |
| `tuner/nmpc_offline_check.py` | Is the offline NMPC (model parity, Jacobians, SQP convergence, LTV-QP-vs-NMPC A/B) still internally consistent? | `python -m tuner.nmpc_offline_check` |
| `ros2/src/fsae_planning/.../test/nmpc_offline_check.py` | Same four checks, for the **live** NMPC port | `python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py` |
| `tuner/checks/live_vs_sim_diagnostics.py` | Where does a live run diverge from an offline run on speed-tracking error and saturation-episode structure? | `python -m tuner.checks.live_vs_sim_diagnostics` |
| `tuner/checks/analyze_adaptive_log.py` | Which adaptive-gain feature is responsible for the tracking error in a specific corner of a live control CSV? | `python -m tuner.checks.analyze_adaptive_log <control_csv>` |
| `ros2/run_steering_sysid.sh` + `tuner/checks/steering_sysid_analysis.py` | What does FSDS's steering→yaw response actually look like across speed (sustained cornering)? | `cd ros2 && ./run_steering_sysid.sh` |
| `ros2/run_steering_step.sh` + `tuner/checks/steering_step_analysis.py` | Which mechanism caps FSDS's yaw rate (hard limit / scaled authority / active damping), from a step-input transient? | `cd ros2 && ./run_steering_step.sh` |
| `tuner/checks/steering_response.py` | What understeer coefficient and full-lock deficit does a live control CSV imply? | `python -m tuner.checks.steering_response <control_csv>` |
| `tuner/tools/plot_playback.py` | What did this run (or these runs, compared) actually do, signal by signal and on the map? | `python -m tuner.tools.plot_playback [csv ...]` |
| `tuner/steering_chatter_check.py` | Does a weight/setting change make tick-to-tick steering chatter better or worse? | `python -m tuner.steering_chatter_check [--controller nmpc\|ltv] [--set NAME=VALUE ...]` |
| `tuner/reference_heading_geometry_check.py` | Is a reference-heading swing caused by the online planner's rebuild, or is it just what the track geometry demands even offline? | `python -m tuner.reference_heading_geometry_check` |
| `tuner/reference_excess_mechanism_check.py` | Are fast-reference-heading ticks explained by the boundary-planner's seed-midpoint anchor jump? | `python -m tuner.reference_excess_mechanism_check` |
| `tuner/checks/ref_heading_limiter_ab.py` / `ref_heading_limiter_suite_check.py` | Does `REF_HEADING_RATE_LIMIT` help or hurt, on one map or across the whole validation suite? | `python -m tuner.checks.ref_heading_limiter_ab` / `python -m tuner.checks.ref_heading_limiter_suite_check` |
| `ros2/clock_drift_check.py` | Is FSDS's own simulation clock falling behind wall time (a genuine sim-side slowdown), separate from message-delivery timing? | launched by `ros2/launch_all.sh`, or standalone: `python3 clock_drift_check.py <output_csv_path>` |
| `ros2 topic hz` capture block in `ros2/launch_all.sh` | Are `/fsds/testing_only/odom`, `/fsae/slam/car_position`, `/clock`, `/fsds/imu` arriving at their expected rate? | launched automatically by `ros2/launch_all.sh`, logs to `fsae_logs/topic_hz_diagnostics/` |
| `tuner/tools/doc_lint.py` | Does a doc break this project's own writing conventions (long unstructured prose, stray AI-instruction-file references)? | `python -m tuner.tools.doc_lint [--max N]` |
| `tuner/tools/sync_mpc_params.py` | After a param change is live-tested and confirmed good, is `fsae_autonomous` (and the `fsds_simulator` mirror) still running the OLD weights? | `python -m tuner.tools.sync_mpc_params [--apply]`, or the Settings tab's "Overwrite All Params..." button |

`tuner.offline_tuner` (the CMA-ES weight search) and `sim/`'s modules are
core simulation infrastructure, not diagnostic tools, and are covered in
[developer_guide.md](developer_guide.md) instead.

**Two different scripts share the name `nmpc_offline_check.py`.** One lives
in this repo (`tuner/nmpc_offline_check.py`, offline plant), the other in
the live `fsae_planning` checkout
(`control/fsae_control/test/nmpc_offline_check.py`, mirrored under
`fsds_simulator/control/fsae_control/test/`). They run the same four
checks (scalar/vectorised model parity, Jacobian finite-difference
cross-check, SQP cost-monotonic convergence from a cold start, and an
independent CasADi+IPOPT solver cross-check) against their own side's
implementation, and are meant to be read side by side, not interchanged.
The live version's closed-loop A/B section optionally imports this repo
for an extra cross-check when checked out alongside, and degrades to
synthetic-state checks only when it isn't.

**Some investigation scripts named in `docs/logs/sim_to_real_investigation.md`
no longer exist** (`gap_attribution_ledger.py`, `blend_reset_diagnostics.py`,
`reference_heading_vs_rebuild.py`, `combined_factors_sweep.py`,
`plot_control_log.py`): they were one-off scripts for a since-concluded
question, deleted once concluded. Only a stale `.pyc` remains for these.
Don't try to run them; if the same question comes up again, treat the
surviving tools above (particularly `plot_playback.py`, which superseded
`plot_control_log.py`) as the current equivalent.

## Centralized launcher: `gui/launcher.py`

**The main entry point for this project.** One tkinter app, tabbed by tool,
wrapping every entry point below into a single window instead of a script
edit plus a separate terminal command each time. It contains no simulation/
plotting/tuning logic of its own: every button rewrites a config file in
place (same effect as hand-editing it) and then shells out to the existing
tool via subprocess. It does not replace any of the CLI usage documented
elsewhere on this page, it's a faster path to the same tools, not a
different implementation of them — everything it does remains directly
reachable the manual way too.

**Required checkout location: `fsae_MPCTest/` must be cloned directly
inside the outer FSDS simulator repo's root**, i.e.
`<FSDS repo root>/fsae_MPCTest/`, a sibling of that repo's own `ros2/`
folder — the same layout this project's `CLAUDE.md` "Git layout" section
already documents for every other tool here. The launcher locates
`ros2/launch_all.sh` and `ros2/src/fsae_planning/tracks/` by walking up
from its own file location (`gui/launcher.py` → `fsae_MPCTest/` → its
parent), so a `fsae_MPCTest` checked out anywhere else (a sibling
directory instead of nested inside, a different drive/path entirely) will
fail to find the live sim to launch, or warn that `mpc_params.py` isn't
where it expects (see the Settings tab's live-sync note below).

```bash
cd fsae_MPCTest && python -m gui.launcher
```

**Config files this tool writes to** (all in place, no new files, `.bak`
backups as noted below):

| File | Written by | What changes |
|---|---|---|
| `ros2/launch_all.sh` | Launch tab's Launch button | `TRACK`, `CONTROLLER`, `USE_NMPC`, `STANDALONE_OUTPUT`, `USE_PRECOMPUTED_SPEED`, `USE_PRECOMPUTED_PATH`, `V_MAX`, `V_MIN`, and (NMPC only, when checked) `NMPC_PROGRESS_ENABLED` |
| `fsae_MPCTest/settings.py` | Settings tab's Save button (also Profiles tab's Load, see below) | `Q_diag`, `R_diag`, `R_rate_diag`, `R_A_ACCEL`, `R_A_BRAKE`, `SPEED_TARGET_DEFICIT_MAX`, every `NMPC_*` weight override, every feature-enable flag listed below |
| `ros2/src/fsae_planning/.../mpc_params.py` / `nmpc_params.py` (live) | Settings tab's Save button (also Profiles tab's Load) | the matching field for every one of the above that has a live counterpart (see "Settings tab, and live/offline sync" below for the handful that don't) |
| `ros2/src/fsae_planning/common/fsae_bringup/config/fsae_params.yaml` (live) | Settings tab's Save button (also Profiles tab's Load) | the same fields as the live dataclass row above. This is the file ROS actually loads a field's RUNTIME default from at node startup (it OVERRIDES the dataclass default), so a save that skipped this file could leave the car running the OLD value indefinitely with no visible sign of it -- this happened twice (`r_a_accel`, `nmpc_track_halfwidth`) before this file was added to the write path |
| `fsae_MPCTest/fsds_simulator/.../mpc_params.py` / `nmpc_params.py` / `fsae_params.yaml` (mirror) | Settings tab's Save button (also Profiles tab's Load) | the same fields again, kept identical to the live copies above per CLAUDE.md's "Third copy" change-ledger rule |
| `fsae_MPCTest/settings_profiles/<name>.json` | Profiles tab's Save/Delete | a full snapshot of every field above, see "Profiles tab" below |
| `ros2/src/fsae_planning/tracks/<name>/` | Launch tab's Export & Save Track button | writes `speed_profile.csv`/`raceline.csv`/`centerline.csv` for a newly recorded track (only after explicit confirm if the name already exists) |
| `fsae_MPCTest/fsds_simulator/tracks/<name>/` | same Export & Save Track button | copy of the same new track's files |
| `<FSDS repo root>/fsae_logs/*.csv` → `fsds_simulator/recorded_runs/<Controller>/` | Launch tab's Stop button (moves, doesn't create) | only after the confirmation prompt it shows is accepted |

Nothing else in this repo or the outer `ros2/` tree is touched by any tab.

| Tab | What it does |
|---|---|
| **Launch Sim** | Rewrites `ros2/launch_all.sh`'s `TRACK`, `CONTROLLER`, `USE_NMPC`, `STANDALONE_OUTPUT`, `USE_PRECOMPUTED_SPEED`, `USE_PRECOMPUTED_PATH`, `V_MAX`, `V_MIN` from a form (with a preview/confirm before writing), then runs it. A **Stop** button sends the same signal a terminal Ctrl+C would (`launch_all.sh`'s own `trap cleanup SIGINT SIGTERM` handles the actual teardown). NMPC-only: a "Progress term (experimental)" checkbox, visible only when NMPC is selected, toggles the commented-out `NMPC_PROGRESS_ENABLED` shortlist line; it does NOT set `NMPC_SLACK_LINEAR_WEIGHT` for you (tune that from the Settings tab), even though the progress term is measured to need it set well above 0. See "Record a new track" below for its recording mode. |
| **Debug a Log** | File browser over `<FSDS repo root>/fsae_logs/` (matching `launch_all.sh`'s own `log_dir:=` — NOT `~/fsae_logs`, `ControlLogger`'s fallback default when no `log_dir` is given) and `fsds_simulator/recorded_runs/` (including per-controller subfolders); select one or more `*_control_*.csv` files and run `tuner.tools.plot_playback` on them, or use "Debug Latest" for that tool's own auto-load-newest behaviour with no selection needed. |
| **Run Offline Sim** | Launches `gui/simulation.py`. Carries forward its "rough signal only" caveat (see "The offline sim does not yet fully predict the car" in the root `CLAUDE.md`) directly in the tab. |
| **Settings** | Edits the commonly-retuned `settings.py` constants in place, described in full below. |
| **Profiles** | Named snapshots of every field the Settings tab manages, described in full below. |

### Record a new track

Checking **"Record new track"** on the Launch tab and typing a name switches
the form into the recording setup this project's own docs already
recommend (`launch_all.sh`'s "Set BOTH to false (with CONTROLLER=stanley
below) when recording a NEW track" comment, and
[developer_guide.md](developer_guide.md#recording-exporting-and-driving-a-track)'s
step 1): `CONTROLLER=stanley`, `USE_PRECOMPUTED_SPEED=false`,
`USE_PRECOMPUTED_PATH=false`. Prior values for those three are remembered
and restored when the checkbox is unchecked, so toggling recording mode on
and off never clobbers an otherwise-normal drive setup. `launch_all.sh`
always writes `cone_map.json` for whatever `TRACK` is set to as a side
effect of driving, recording mode just makes sure it lands in a *new*
track's folder with a live (not precomputed) drive behind it.

Once stopped (the **Stop** button, or Ctrl+C in the terminal it opened),
**Export & Save Track** runs the same two exporters
[developer_guide.md](developer_guide.md#recording-exporting-and-driving-a-track)'s
step 2 does by hand (`tuner.tools.export_speed_profile`, then
`tuner.tools.raceline_optimizer` in both `raceline` and `centerline`
modes), which write directly into
`ros2/src/fsae_planning/tracks/<name>/` as they already do outside the
GUI. It then additionally copies that whole track folder into
`fsae_MPCTest/fsds_simulator/tracks/<name>/`, since that mirror has no
automatic resync (see "Third copy" in the root `CLAUDE.md`). Re-exporting
over an existing track name asks to confirm the overwrite first.

Clicking **Stop** on any run (recording or not) also offers to move that
run's just-written CSV pair from `<FSDS repo root>/fsae_logs/` (matching
`launch_all.sh`'s own `log_dir:=`) into
`fsds_simulator/recorded_runs/<Controller>/` — the same manual copy step
described under "Telemetry playback" below, done for you. It only offers
logs written after the current launch started, so an unrelated older file
sitting in `fsae_logs/` is never swept up by mistake. Since the node's own
telemetry file isn't necessarily flushed and closed the instant Stop is
clicked (`ControlLogger.close()` runs from the node's own signal handler,
asynchronously with the click), the launcher polls for up to 5 seconds
before giving up silently, rather than checking once and missing a file
written moments later.

Accepting the move prompt then asks for an optional label for the run.
Leaving it blank keeps the original filename; typing one splices it in
between the tag and `_control_`/`_path_`
(`mpc_standalone_<label>_control_<stamp>.csv`), the same hand-labelled-run
convention `recorded_runs/` already uses for runs saved for later
comparison (see "Telemetry playback" below) — `plot_playback.py`'s own
discovery already tolerates this, it only looks for `_control_`/`_path_`
plus the trailing stamp.

### Settings tab, and live/offline sync

Edits the commonly-retuned `settings.py` constants in place: `Q_diag`,
`R_diag`, `R_rate_diag`, `R_A_ACCEL`/`R_A_BRAKE`, every `NMPC_*` weight
override (with an explicit "override vs. inherit (-1.0)" checkbox per
field, matching `settings.py`'s own sentinel convention), and every
feature-enable flag, grouped exactly the way `mpc_params.py`'s own
per-field `"controller"` metadata already classifies them:

- **Both controllers**: `delay_compensation_enabled` (live-only, no
  `settings.py` equivalent — the offline rollout always has it on).
- **LTV-QP only**: adaptive Q scaling, steer-rate anti-hunt, adaptive
  R-rate in corners, reference-heading rate limit, reversal penalty.
- **NMPC only**: its own steer-rate anti-hunt, reversal penalty, rate-cost
  stage ramp, rate-cost 3-zone schedule, and corner rate-blend
  (experimental, all default off).

Every field shows a short description (and unit, where one applies)
pulled live from `mpc_params.py`'s own field metadata (or, for the 3
weight vectors, a hand-written per-index breakdown), so the tab's text can
never drift out of sync with what the field actually means.

**Saving also updates the live simulator**, not just this repo: every
weight/override/flag that has a matching field in
`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc/mpc_params.py`
or `nmpc_params.py` is rewritten there too, in the same click, per
CLAUDE.md's "Single source of truth for MPC tuning" numeric-parity rule
(`settings.py`'s `Q_diag[0]` ↔ `mpc_params.py`'s `q_e_y`, and so on). A
handful of fields have no live counterpart and are settings.py-only:
`R_diag[1]` (nominal-only, superseded by `R_A_ACCEL`/`R_A_BRAKE`) and
`Q_diag`'s last three entries (`e_a`/`delta_act`/`a_act`, always 0.0). If
the live file isn't found at the expected path (an unusual repo layout),
Settings still saves to `settings.py` alone and says so with a warning,
rather than silently only updating one side.

**The same click also writes `fsae_params.yaml` and the `fsds_simulator/`
mirror** (both dataclasses, both YAMLs), not just the live dataclass
default. This matters because `fsae_params.yaml` is what a launched node
actually reads its runtime value from -- it OVERRIDES the dataclass
default at ROS param declaration time, so writing only the dataclass could
leave the car silently running an old value with the GUI showing the new
one and nothing to say they'd diverged. This is not a hypothetical: it
happened twice before this file was added to the write path (`r_a_accel`
stuck at 2.25 instead of a corrected 1.0, `nmpc_track_halfwidth` stuck at
3.0 instead of a reverted 3.5), each time discovered only by reading the
YAML directly rather than trusting the dataclass. A field missing from
`fsae_params.yaml` entirely (20 experimental NMPC fields were, until this
was fixed) is reported the same way a missing dataclass field is, not
silently skipped.

**What it does NOT expose**: the full commented-out `MPC_*`/`NMPC_*`
per-tick weight-override shortlist further down `launch_all.sh` (structural
solver settings like `NMPC_HORIZON`, `NMPC_SQP_ITERS`) — those stay a
manual edit, deliberately, since they're touched far less often than the
Launch tab's fields and the enable/disable-by-comment mechanic for that
whole block isn't worth the UI surface it would need.

**File safety**: the first time a session rewrites `ros2/launch_all.sh`,
`settings.py`, or the live `mpc_params.py`, it saves a `.bak` copy
alongside the original (e.g. `launch_all.sh.bak`) before writing, so a bad
edit has a one-command recovery (`mv launch_all.sh.bak launch_all.sh`)
independent of git.

### "Overwrite All Params..." button

A different sync direction from everything else on this tab. Save (above)
writes what THIS GUI session's widgets currently hold, outward, to
`settings.py`/the live dataclasses/YAML/the `fsds_simulator` mirror.
"Overwrite All Params" instead runs `tuner/tools/sync_mpc_params.py` (its
own section further down this doc has the full mechanism; a plain
subprocess call, matching this file's "every button shells out" design),
pushing the LIVE checkout's CURRENT on-disk `mpc_params.py`/
`nmpc_params.py`/`fsae_params.yaml` into `fsae_autonomous` and the
`fsds_simulator` mirror.
It does not read this tab's own widgets at all, and works regardless of
whether the Settings tab has unsaved edits pending (those are a separate,
earlier step: save/push live first, run this after).

Two-step, both against a background thread so the window stays responsive:
1. A dry run first, to find out which destinations actually differ.
   "Already in sync" short-circuits to an info dialog with nothing written.
2. If anything differs, a confirmation dialog names exactly which
   destinations (by the script's own labels, e.g. `fsae_autonomous`) will
   be overwritten, states plainly that local edits to those 3 files there
   are lost except for a `.bak` backup, and that `fsae_autonomous` itself
   is never committed or pushed by this tool, only its local tree is
   touched. Declining leaves every file untouched.

### Profiles tab

Named snapshots of every field the Settings tab manages (the same set the
"Saving also updates the live simulator" section above describes, derived
from the Settings tab's own field tables so a profile can never drift out
of covering less than a Settings-tab Save does), stored one JSON file per
profile under `fsae_MPCTest/settings_profiles/<name>.json`. Tracked in git
like any other project file, not gitignored -- a profile is meant to be a
shareable, reusable tuning configuration, not a personal scratch file.

- **Save Current As Profile...** prompts for a name, reads the CURRENT
  value out of every Settings-tab widget (not `settings.py` -- this
  captures an unsaved in-progress edit too), and writes it as
  `{"name": ..., "values": {...}}`. An existing profile with the same name
  asks to confirm the overwrite first.
- **Load Selected** pushes a profile's values into every matching
  Settings-tab widget, then runs the exact same save routine the Settings
  tab's own Save button uses -- settings.py, both live dataclasses, both
  `fsae_params.yaml` copies, and both `fsds_simulator/` mirrors are all
  rewritten immediately, precisely as if every field had been retyped by
  hand and Save clicked. There is no separate, second write path to keep
  in sync with the Settings tab's own. A name in the profile that no field
  table currently recognises (e.g. a profile saved by an older GUI version
  before a field was renamed or removed) is skipped silently rather than
  reported as an error, since a profile is a convenience snapshot, not a
  strict schema every version must satisfy.
- **Delete Selected** removes the profile's JSON file. Not recoverable
  except via git history if the file had already been committed.

Loading a profile only writes files; it does not restart a running sim.
The confirmation dialog says so, matching the Settings tab's own "restart
the sim to pick up the live change" reminder.

## Pushing live-tested params to `fsae_autonomous` and the `fsds_simulator` mirror: `tuner/tools/sync_mpc_params.py`

**What it answers**: a param has been retuned and validated on a live/FSDS
run in `fsae_planning` (the ONLY place params get tuned, per this
project's stated workflow) — is `fsae_autonomous` (the production repo)
or the `fsds_simulator` mirror still running the old value?

```bash
python -m tuner.tools.sync_mpc_params            # dry run, prints a diff per file per destination
python -m tuner.tools.sync_mpc_params --apply     # actually overwrite
```

Or from the GUI: Settings tab's "Overwrite All Params..." button, see
that tab's own section above.

**Scope**: exactly the 3 files CLAUDE.md's "Single source of truth for
MPC tuning" section names as the live side of the parity boundary —
`mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`. Nothing else; a
code change to `mpc_core.py`/`nmpc_core.py`/`mpc_controller.py`/
`live_viz.py` still needs the ordinary manual propagation step (see
CLAUDE.md's "Third copy" section for the mirror, and the standing
`fsae_autonomous`/`fsae_MPCRos` workflow for that side).

**Direction: one-way, from `fsae_planning` only.** `ros2/src/fsae_planning/`
is always the source; `fsae_autonomous` and `fsae_MPCTest/fsds_simulator/`
are always the destinations. This script never reads either destination's
current values as a source for anything — it does not matter what they
currently hold, only what live currently holds.

**Why `fsae_params.yaml` is in scope, not just the two `.py` files**: the
YAML overrides the dataclass `default=` at ROS param declaration time (see
the Settings tab's own writeup above for the two times this caused a
value to go silently stale), so syncing only the `.py` files would leave
`fsae_autonomous`/the mirror running an old number even after their
`mpc_params.py` looked updated.

**Safety**: dry run by default, nothing written until `--apply`. Each
destination file gets a one-time `.bak` backup (skipped if one from this
run already exists) before being overwritten, same convention as
`gui/launcher.py`'s own file-safety mechanism. `fsae_autonomous` is a
production repo this project's CLAUDE.md never lets an agent commit or
push — this script only ever writes into its LOCAL working tree; review
and commit there stays a separate, deliberate, human step.

**`fsae_autonomous`'s actual checkout location isn't fixed** — it moved at
least once (CLAUDE.md's documented sibling-checkout path
`fsae_autonomous/` was found stale on 2026-09-23; the real checkout was at
`ros2_autonomous/src/fsae_autonomous/`). The script searches a short list
of known-observed locations and prints a clear warning (skipping that
destination, not failing outright) if neither is found, rather than
silently doing nothing or hardcoding a path that can go stale again.

## Live debug window: `live_viz.py`

Sim-only debug visualiser (car, cone map, reference path, driven trail,
and a live weighted-error breakdown), redrawn from live ROS2 topics on a
timer. Never launched by `fsae_autonomous`, only by `ros2/launch_all.sh`
alongside the sim, or the launcher's Launch Sim tab.

```bash
ros2 run fsae_control live_viz
```

**Adapts to whichever controller is actually running.** MPC and Stanley
share one ROS node name and, in `cmd_vel` output mode, one output topic,
so `live_viz.py` cannot tell them apart by topic alone — it identifies the
active controller by which debug topic last published
(`/fsae/control/debug_weights` for MPC, `/fsae/control/debug_stanley` for
Stanley), and switches both the main window's title/stats text and which
debug figure is shown accordingly:

- **MPC active**: the weighted-cost breakdown described under "Centralized
  launcher" above — grouped step-0 panels (tracking/effort/rate) plus the
  horizon-summed panel, all against `total_cost`.
- **Stanley active**: a separate two-panel figure —
  1. **Heading vs. lateral error**: Stanley's own two tracking errors
     (`e_y`, `e_psi`), one shared scale, share of `|e_y| + |e_psi|`.
  2. **Control-law term breakdown**: Stanley's three additive terms
     (`δ = heading_error + atan2(k_cte·e, v+k_soft) − k_d·yaw_rate`, see
     `control_utils.py`'s `StanleyController`), each as a share of the sum
     of their absolute values — "how much of this tick's steering command
     came from which term."

Both figures are built once at startup and shown/hidden as a whole rather
than rebuilt each frame, so switching controllers mid-session (stopping
one run and launching the other) updates the display without a restart.
"Shown/hidden" means the actual OS window, not just the figure's own
artists: `Figure.set_visible()` alone only controls whether a figure's
contents draw onto ITS OWN canvas, it does not touch the window the TkAgg
backend opened for it, so the inactive controller's debug figure was left
on screen the whole time showing nothing -- a third, unlabelled, empty
window with no visible reason to be there. Fixed by calling
`fig.canvas.manager.window.withdraw()`/`.deiconify()` (Tk's own window
object) on top of `set_visible()`, gated to fire only on an actual
controller-switch transition rather than every redraw frame, so a
manually moved/resized debug window isn't fought back into place ~50
times a second by the two figures' animations.

**Every bar-graph panel's row order is fixed for the lifetime of the
window**, not recomputed each frame. Previously, a panel's y-axis
category list was built by filtering a fixed name list down to only the
terms with data THIS tick (`[n for n in names if n in terms]`), and the
horizon panel additionally re-sorted that filtered list by current value,
descending, every frame. Both meant the list `ax.barh()` drew from could
change length or order tick to tick even though the values themselves
were moving smoothly: a term temporarily absent (e.g. `progress`/
`v_cap_hinge` outside progress mode) collapsed every row below it upward
by one slot, and the horizon panel's own sort visibly swapped two rows
the instant one term's cost share crossed another's. Fixed by always
drawing one row per name in the full declared table (`DEBUG_BAR_GROUPS`,
`DEBUG_HORIZON_TERMS`, `STANLEY_ERROR_TERMS`, `STANLEY_LAW_TERMS`), with
an absent term shown as an empty grey row at its own fixed position
rather than omitted, and the horizon panel's per-frame sort removed
entirely in favour of its declared order.

**`fig_dbg`/`fig_stanley` use `layout='constrained'`, not a one-shot
`tight_layout()` call.** `tight_layout()` computes fixed axes-position
fractions once, right before `plt.show()`, and never again -- a window
later resized smaller than its requested `figsize` (dragged by the user,
or placed smaller by the window manager) has no way to re-reserve margin
for the y-axis category labels at the new size, and the longer ones get
clipped by the figure's own left edge. Measured directly against a
screenshot showing exactly that on both figures (`"g error (e_psi)"`,
`"eral error (e_y)"`, MPC panel names losing their first several
characters). `constrained_layout` re-solves the whole layout on every
draw, including a resize, so labels always get the margin the CURRENT
window size actually needs; confirmed offline at both the intended
figsize and a synthetic resize to under half of it, no label clipped
either way. The stacked panels' gridspecs were also given explicit
`hspace`/`wspace` (0.6/0.35), since once labels started reserving real
margin instead of being clipped away, the default spacing let adjacent
panels' rows visually run into each other.

## Steering system-ID harness: `run_steering_sysid.sh` / `run_steering_step.sh`

Isolating the plant from the controller, commanding fixed steering angles
at fixed speeds on an empty map and recording the achieved yaw rate, is
how FSDS's lateral-acceleration ceiling was found (see
[simulator_fidelity.md](reference/simulator_fidelity.md)'s "Root cause"
section). Reuse this methodology whenever a plant-vs-car discrepancy is
suspected: a closed-loop lap log alone cannot separate a plant defect from
a controller/reference one.

**Where the pieces live** (the ROS 2 node and the harness script are
working-tree-only files in the live ROS 2 workspace, no mirror in this
repo):

| File | Repo | Role |
|---|---|---|
| `control/fsae_control/fsae_control/steering_sysid.py` | `fsae_planning` (live ROS 2 ws) | the node, drives FSDS directly |
| `ros2/run_steering_sysid.sh` | FSDS repo root, next to `launch_all.sh` | one-command harness |
| `tuner/checks/steering_sysid_analysis.py` | `fsae_MPCTest` | reads the log, names the mechanism |
| `fsae_control/steering_step.py` / `ros2/run_steering_step.sh` / `tuner/checks/steering_step_analysis.py` | same split | the step-input companion test (50 Hz, isolates the transient) |

`steering_sysid.py`/`steering_step.py` and their harness scripts are
**deliberately not mirrored** into `fsds_simulator/`, they never existed in
`fsae_planning`'s committed git history; see
[offline_live_parity.md](reference/offline_live_parity.md).

**Run it with one command** (starts FSDS, waits for RPC, starts the
bridge, waits for odom, runs the sweep, analyses the log, tears everything
down, including on Ctrl+C):

```bash
cd <FSDS repo>/ros2 && ./run_steering_sysid.sh
```

Flags: `--no-sim` (FSDS already running), `--quick` (fewer points), and
any `-p name:=value` passes through to the node. **Run it on an empty
map**: it circles at up to 14 m/s and does not brake for cones. The
harness refuses to start if `mpc_controller`, `fsds_bridge`, or `stanley`
is already running, since two publishers on `/fsds/control_command` would
interleave and corrupt the log.

**Geometry is bounded automatically.** The node reaches target speed
while already turning (so it orbits rather than travelling), checks a
geofence (`home_radius`/`max_radius`) from every phase, and predicts each
point's orbit size in advance (using a deliberately pessimistic
`K_US_ESTIMATE = 0.05`) to skip any (speed, steering) pair whose orbit
won't fit in the geofence, logging what it dropped. Default steering
commands (`[0.5, 0.65, 0.8, 1.0]`) are biased high since low-angle,
high-speed points are both the least informative and the least likely to
fit.

**Reading the log.** It records the raw normalised `cmd.steering`
alongside the roadwheel angle it's assumed to map to, since recording only
the assumed angle would beg the question the test exists to answer. A
falling `s = δ_ach/δ_cmd` is not by itself diagnostic (a speed-scaled
rack, genuine understeer, and grip saturation all produce one), so the
analyser fits all five candidate mechanisms to achieved yaw rate and
reports the margin to the runner-up:

| Winning model | Meaning |
|---|---|
| neutral (s≈1) | steering path is fine, look at the controller/reference |
| constant scale | `MAX_STEER_RAD` wrong, fix in all three copies |
| speed-scaled rack | FSDS reduces lock with speed, model it in the plant |
| understeer (v²) | real vehicle dynamics |
| grip saturation | yaw capped by lateral grip |

The default speed sweep is **3–14 m/s**, wide enough to separate
speed-scaled rack from understeer (near-degenerate over a narrow band). If
the analyser prints a margin warning, widen the speed range and re-run
rather than trusting the verdict; it also refuses a verdict when fewer
than 3 windows contain real motion (a car wedged against a wall otherwise
reports a confident, meaningless answer from all-zero data).

`run_steering_step.sh` is the transient companion: same one-command
pattern, same `--no-sim`/`--quick`/`-p name:=value` flags, but a 50 Hz
step-input hold instead of a speed sweep, isolating *which* mechanism caps
yaw rate rather than just confirming that one exists.

Both harnesses are **diagnostic-only, deliberately separate from
`launch_all.sh`**: they must not run alongside the planning/control stack,
since any other node publishing to `/fsds/control_command` would
interleave with the sweep's commands and corrupt the measurement. If
either one is suspected to be measuring something stale, re-measure, don't
guess, this is a measured property of the simulator, not a tuning knob
(see [tuning.md](tuning.md) §4.8).

## Telemetry playback: `tuner/tools/plot_playback.py`

Turns one or more control-telemetry CSVs (see
[developer_guide.md](developer_guide.md#csv-telemetry-logging) for how
those get written) into an interactive matplotlib figure that answers both
"what did this signal do over the whole run" and "where was the car, and
what did the path look like, at this specific moment" at once:

- **left:** the scored signals (`e_y`, `e_psi_deg`, `kappa`, `steer_deg`,
  `v`) stacked on a shared time axis, one line per signal per run when
  comparing multiple logs, with a vertical cursor marking "now"
- **top right:** each run's full driven trajectory, plus the planner's
  most-recent path snapshot at "now", with a triangle marking that run's
  car position/heading
- **bottom right:** the same scene zoomed tightly to the car's current
  section of track (with `e_y`/`e_psi` in its title)

A slider under the metrics panel scrubs a shared "now" time through the
run; dragging it updates every run's cursor, triangle, and path overlay
together. Each run gets its own colour, used consistently for its signal
lines, driven trajectory, path overlay, and car marker, and, when more
than one log is given, its own checkbox to show/hide it everywhere at
once. Built for eyeballing a single run or comparing two controllers
head-to-head (e.g. an MPC log against a Stanley log recorded on the same
`map_path`) without writing a one-off script each time.

```bash
# no CSV given -> auto-loads and overlays every run in
# fsds_simulator/recorded_runs/ (one CSV -> single-run playback,
# several -> automatic comparison with a checkbox per run)
python -m tuner.tools.plot_playback

# same, but only the newest run if recorded_runs/ has several and you
# just want the latest one
python -m tuner.tools.plot_playback --latest-only

# default signal set: e_y, e_psi_deg, kappa, steer_deg, v (actual + desired)
python -m tuner.tools.plot_playback ~/fsae_logs/mpc_standalone_control_<ts>.csv

# overlay two explicit runs -- each gets its own colour, signal lines,
# marker, trajectory, and path overlay, plus a checkbox to hide/show it
python -m tuner.tools.plot_playback \
    ~/fsae_logs/mpc_standalone_control_<ts>.csv \
    ~/fsae_logs/stanley_control_<ts>.csv

# choose your own signals (any numeric column the log has)
python -m tuner.tools.plot_playback run.csv --signals e_y,yaw_rate,solve_ms
```

On Windows PowerShell, drop the `\` line continuations (use backtick `` ` ``
or put everything on one line) and don't rely on `~`, PowerShell doesn't
expand either the way bash does, and a bad path there fails with a raw
`FileNotFoundError` from `csv_log.py`'s `open()`, not a friendlier CLI error.
The multi-run examples above are bash syntax; on PowerShell write e.g.
`python -m tuner.tools.plot_playback $HOME\fsae_logs\mpc_standalone_control_<ts>.csv $HOME\fsae_logs\stanley_control_<ts>.csv`
on one line, or use the backtick continuation character in place of `\`.

Run from `fsae_MPCTest/` (so `tuner` resolves as a package). A signal
missing from a given log (e.g. the `m_Q_*`/`m_R_*` adaptive-weight columns,
`solve_ms`, on a Stanley run) is skipped for that run with a warning rather
than plotting an empty line, runs don't need identical columns to overlay
the ones they share. The figure title and each line's legend label include
the run's short label (its controller subfolder name, e.g. `LMPC`/`NMPC`/
`Stanley`), so a comparison plot is self-labelled without cross-referencing
the raw CSV.

Each run's sibling `<tag>_path_<stamp>.csv` (same directory, same timestamp,
the file `ControlLogger` writes alongside every control CSV) is loaded
automatically if present, to draw that run's path as it looked at each
moment, copy both files together into `recorded_runs/`, not just the
`_control_` one, or that run's map/zoom views fall back to showing only its
own driven trajectory with no live path overlay. The path CSV is a time
series of path snapshots (see `telemetry_logger.py`'s `log_path()`); the
slider always shows the most recent snapshot at or before the selected
time, not an interpolation between two snapshots.

Runs may have different `t` sampling or length (e.g. an 80-sample Stanley
log next to a 50-sample MPC log), the slider drives one shared time
value, and each run independently looks up its own nearest sample, so
mismatched logs still overlay correctly. The slider itself still scrubs
the full range up to the **longest** run's end (so the map/zoom views can
follow it to completion), but the left-hand signal plots' x-axis is
clipped to the **shortest** run's end, past that point only one run has
data left, which would otherwise dwarf the overlapping (comparable) part
of the plot with a stretch that isn't a comparison anymore.

**Auto-search folder: `fsds_simulator/recorded_runs/`.** Running the script
with no CSV argument searches this folder **recursively** for
`*_control_*.csv` files, including one level of per-controller subfolders,
e.g. `recorded_runs/LMPC/`, `recorded_runs/NMPC/`, `recorded_runs/Stanley/`,
by the timestamp `ControlLogger` stamps into the filename (not file mtime).
That stamp is either the current local `%Y%m%d-%H%M%S` form or the older
epoch-seconds form. `plot_playback.py`'s `_stamp()` decodes both to epoch
seconds so a folder holding runs from either era sorts correctly as one set.

By default it loads just the **newest run from each subfolder** (one
representative LMPC run, one NMPC run, one Stanley run, ...; runs left flat
directly in `recorded_runs/` are grouped as one "folder" for this purpose),
pass `--all` to overlay every run in every subfolder instead, or
`--latest-only` to load only the single newest run across the whole tree
(which may leave other controllers unrepresented).

Each run's plot label is its `recorded_runs/<folder>/` name (e.g. `LMPC`,
`NMPC`, `Stanley`) rather than the raw filename tag, since the tag alone is
often ambiguous (both LMPC and NMPC logs use the same `mpc_standalone`
tag). Runs left flat directly in `recorded_runs/` fall back to the
filename tag; if a folder has multiple loaded runs (e.g. under `--all`),
duplicates get a ` #2`, ` #3`, ... suffix. The CSVs under this folder are
tracked in git (not gitignored) so reference runs for each controller
travel with the repo.

A live run's actual output location is `log_dir` (default `~/fsae_logs`,
or whatever `ros2/launch_all.sh`'s `log_dir:=` argument points at, in the
outer `fsae_planning`-adjacent launch script, outside this repo, not
modified by this feature), so after a run, the CSV pair needs to be copied
or moved into the right controller subfolder manually:

```bash
cp ~/fsae_logs/mpc_standalone_control_<stamp>.csv \
   ~/fsae_logs/mpc_standalone_path_<stamp>.csv \
   fsds_simulator/recorded_runs/LMPC/
python -m tuner.tools.plot_playback       # picks up the file just copied in
```

The recorded filenames under `recorded_runs/` may also carry a descriptive
topic segment between the tag and the stamp (e.g.
`mpc_standalone_postjitterfix_best_control_1787527398.csv`), added by hand
when a run is stored specifically for later comparison. `plot_playback.py`
only looks for `_control_`/`_path_` and the trailing stamp, so an inserted
topic segment does not affect discovery or sibling pairing.

**Curated drop zone: `recorded_runs/graph/`.** If this folder contains any
`*_control_*.csv` files (directly, it's not scanned for further
subfolders), auto-load uses **only** what's in `graph/` instead of scanning
`LMPC/`/`NMPC/`/`Stanley/`/etc. This is the easiest way to control exactly
what a plain `python -m tuner.tools.plot_playback` shows: move (or copy) the
specific run(s) of interest into `graph/`, without deleting them
from their controller subfolder or passing a path on the command line each
time. It's empty by default (tracked via `.gitkeep`), drop files in, run
the command, and it just works:

```bash
cp fsds_simulator/recorded_runs/NMPC/mpc_standalone_control_<ts>.csv \
   fsds_simulator/recorded_runs/NMPC/mpc_standalone_path_<ts>.csv \
   fsds_simulator/recorded_runs/graph/
python -m tuner.tools.plot_playback       # loads every run in graph/, overlaid
```

`graph/` is flat by design (no per-controller subfolders of its own), so
**every** run dropped into it loads and overlays, unlike the full-tree
default, which keeps only the newest run per controller subfolder. This is
what makes it useful for comparing runs across different controllers (e.g.
an NMPC run against a Stanley run) without `--all`. `--latest-only` still
narrows a populated `graph/` down to its single newest run when that's the
desired result instead. Each run's label falls back to its filename tag rather
than the folder name `graph` (which would be true of every run in it and so
useless for telling them apart), same rule as a run left loose directly in
`recorded_runs/` itself.

Point the search elsewhere with `--recorded-runs <dir>` (e.g. to auto-load
straight out of `~/fsae_logs` without copying, or to compare two specific
takes kept in their own directories). This bypasses the `graph/`
override too, since it changes the root being searched. When two or more
runs are loaded, a **"Zoom focus"** radio-button widget appears bottom-left of the figure,
pick a run there to change which one the bottom-right zoomed view tracks
(it defaults to the first-loaded run). The separate **"Show/hide"**
checkbox widget above it toggles each run's visibility everywhere
(signals, map, zoom) without changing zoom focus.

## Live pose/RPC diagnostics: `clock_drift_check.py` and the `topic_hz` capture block

**Temporary instrumentation** for the open
[periodic car-position teleport bug](logs/periodic_pose_teleport_investigation.md),
not a permanent part of the launch flow. `ros2/launch_all.sh` launches it
automatically right after the bridge comes up (search for `TEMPORARY
(2026-08-19)`), and it should be removed (this block, its `cleanup()`
teardown, and `ros2/clock_drift_check.py` itself) once that bug's root
cause is found.

What it captures, all logged into `fsae_logs/topic_hz_diagnostics/`:

| Capture | What it checks |
|---|---|
| `ros2 topic hz -w 5 /fsds/testing_only/odom` | bridge's raw 250 Hz odom output, arrival rate |
| `ros2 topic hz -w 5 /fsae/slam/car_position` | `sim_perception.py`'s 20 Hz relay, arrival rate |
| `ros2 topic hz -w 5 /clock` | sim clock's arrival rate |
| `ros2 topic hz -w 5 /fsds/imu` | IMU arrival rate |
| `clock_drift_check.py` | whether `/clock`'s **value** (not just its arrival rate) falls behind wall time |

`clock_drift_check.py` subscribes to `/clock` and logs `(wall_time,
sim_time)` on every message to a CSV. Unlike `ros2 topic hz`, which only
shows how often a message *arrives*, this shows whether the sim clock's
*value* is falling behind wall time, directly testing whether FSDS/Unreal's
own simulation is running slower than real time (a genuine frame/tick-rate
problem), as distinct from a message-delivery problem on the ROS2/network
side. `d(sim_time)/d(wall_time)` near 1.0 means the simulation is keeping
up; a sustained value below 1.0 means FSDS itself is the bottleneck.

```bash
# launched automatically by ros2/launch_all.sh; to run standalone
# (after sourcing the workspace):
python3 clock_drift_check.py <output_csv_path>
```

See
[periodic_pose_teleport_investigation.md](logs/periodic_pose_teleport_investigation.md)
for what these captures have found so far and what's still open, this
document only covers what the tools do and how to run them.

## Doc conventions: `tuner/tools/doc_lint.py`

Flags docs that break this project's writing conventions (see CLAUDE.md's
"Writing style for docs, logs and comments"): prose blocks longer than a
line ceiling (default 12, lists/tables/headings/code exempt), and stray
references to AI-assistant instruction files, which aren't project
documentation.

```bash
python -m tuner.tools.doc_lint            # report only
python -m tuner.tools.doc_lint --max 10   # stricter paragraph ceiling
```

## Debugging solver failures

If the live simulator reports `consecutive_solver_failures` or the console
frequently shows `OPTIMAL_INACCURATE`:

- **Weight scaling**: OSQP is sensitive to poorly-conditioned matrices. If
  any entry of `Q`, `R`, or `R_rate` exceeds `1e4` or drops below `1e-4`,
  convergence can suffer. Check `controller/model_utils.py`'s
  `adaptive_R_scaling()`'s output at your test speed isn't blowing up the
  steering cost unexpectedly.
- **Kinematic vs. dynamic gap**: if the car consistently fails at tight
  hairpins, `sim/speed_profile.py` may be commanding a speed that demands
  more lateral force than the Pacejka friction circle can supply at that
  curvature. Lower `mu` in `compute_speed_profile()` to force more
  conservative corner-entry speeds.
- **Model-plant mismatch at extremes**: remember the MPC's internal model
  is linear and only blends kinematic/dynamic behaviour between 1-2.5 m/s;
  well outside that (very low speed under load, or very high lateral
  acceleration near the tyre limit) is where the biggest prediction error
  will show up, and where `adaptive_R_scaling`/`adaptive_R_rate` matter most.

If the NMPC's SQP misbehaves instead (non-improving steps, oscillation),
the usual suspects are the same as above, plus two NMPC-specific ones:
`nmpc_solve_budget_ms`/`nmpc_sqp_iters` too tight for the horizon, or a
weight override (`NMPC_Q_E_Y` etc. in `settings.py`, `-1` inherits from the
base weight) pushing the cost badly out of scale. See
[control_mechanisms.md](reference/control_mechanisms.md)'s "Nonlinear MPC
(`use_nmpc`)" section for the model and weight-mapping details, and
[tuning.md](tuning.md) §4.5d for the tuning surface.

For the investigation narrative behind any of these tools, see
[docs/logs/](logs/).
