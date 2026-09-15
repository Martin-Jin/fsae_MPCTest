# NMPC in planner-only mode loses heading control when the live planner path truncates mid-corner

## Summary

A single live run of NMPC with no precomputed track (`map_path`/
`path_map_path` both unset, live planner path and live curvature-derived
speed target only) shows a severe transient heading-tracking failure at one
tight corner: heading error runs from -23 to -88 degrees in 0.3 s while
steering is pinned at the 25 degree lock, then the sign flips. **Root cause
is confirmed**: at `t=23.62 s`, the live planner published a path snapshot
where only the first 20 of 51 points are distinct, the remaining 30 are the
last real point repeated (a frozen, zero-length tail). NMPC solved against
that snapshot with a reference that stops tracking the corner partway
through the horizon, so its own optimum drove the car toward a target the
real corner had already diverged from. The car does not go off-track by
lateral error and does not stall; it recovers once a subsequent, complete
planner snapshot arrives and finishes the run normally.

## Plain-language version

The car had no pre-recorded map for this run, so it built its own picture of
the track live from the planner as it drove, working out its own speed
target for each corner rather than reading both off a file made in advance.
For most of a one-minute run this worked fine. At one corner, the planner's
live output briefly stopped updating partway along its own predicted path,
so instead of a full picture of the upcoming corner, roughly the back 60% of
what the car had to steer against was just the same single point copied
over and over, an artificial dead end sitting short of where the road
actually goes. The controller trusted this and steered toward the dead end
convincingly, its own math said it had found the best plan against what it
was given. But because the corner kept curving past that frozen point, the
car's real heading swung wildly away from where the track actually pointed,
close to being turned side-on to it, over about a third of a second, while
steering was already turned as far as it goes and braking hard for the same
corner, leaving no spare authority to correct with. The car did not run off
the track and drove normally again as soon as a fresh, complete path arrived
from the planner, but this is exactly the kind of moment that would be
dangerous with less margin (a tighter track, a faster approach, or slightly
worse luck on timing).

## What is confirmed by telemetry

Run: `mpc_standalone_control_20260915-084943.csv` /
`mpc_standalone_path_20260915-084943.csv` (`comp_test_map_3`, `use_nmpc=1`,
`map_path=''`, `path_map_path=''`, confirming genuine planner-only mode, no
precomputed path or speed profile in play).

| | value |
|---|---|
| run duration | 62.7 s |
| `\|e_y\|` max | 1.49 m (track half-width 3.5 m, never approached) |
| `v_actual` max | 17.9 m/s |
| steering at/near 25 deg lock | 3.9% of ticks |
| solver failures | 0 (`nmpc_status=1`, converged, throughout) |
| run outcome | completes, no stall, no off-track by `e_y` |

### The path snapshot at the moment of failure is truncated

The path log records one full path snapshot roughly once per second. Every
snapshot in the 62.7 s run has all-distinct points except one:

| snapshot `t` (s) | total points | trailing duplicate points |
|---|---|---|
| 21.55 | 27 | 0 |
| 22.60 | 23 | 0 |
| **23.62** | **51** | **30 (59%)** |
| 24.65 | 13 | 0 |

At `t=23.62`, points `idx=0..19` are distinct and trace the approach to the
corner; from `idx=20` onward every point is the identical `(34.545,
41.618)`, repeated to pad the array to its usual length. This is the one
and only such snapshot logged in the entire run. Naively measuring the
turn angle across that frozen tail reads as a -147 degree kink; that number
is an artifact of computing a heading through a zero-length segment, not a
real geometric turn. The actual defect is simpler and is real: the planner
delivered a short path and padded it, rather than a full one, for that one
tick.

### Control telemetry lines up exactly with that snapshot

The control log's `path_age_s` column (time since the last planner path
update was received) drops to an anomalously low value only at the ticks
where a fresh planner snapshot has just landed (0.02-0.03 s vs. a typical
0.04-0.06 s), and the controller emits a one-tick `steer=0.0` at each such
moment, including at `t=23.62` itself. The corner-entry sequence:

| t (s) | v_actual | v_desired | e_y | e_psi (deg) | steer (deg) | path_age (s) | note |
|---|---|---|---|---|---|---|---|
| 23.59 | 4.27 | 4.63 | -0.33 | -12.5 | -10.4 | 0.049 | approaching corner normally |
| 23.62 | 4.22 | 4.82 | -0.33 | -12.5 | 0.0 | 0.023 | truncated snapshot lands |
| 23.70 | 4.15 | 4.79 | -0.57 | -20.9 | -7.8 | 0.076 | reference now stale relative to real corner |
| 23.75 | 4.10 | 4.47 | -0.62 | -23.3 | -3.4 | 0.045 | |
| 23.95 | 3.64 | 3.09 | -0.92 | -50.5 | 16.6 | 0.050 | |
| 24.00 | 3.84 | 2.59 | -1.06 | -59.1 | 21.8 | 0.043 | |
| 24.05 | 3.95 | 2.10 | -1.06 | **-87.9** | **25.0** | 0.057 | steering at lock, near side-on |
| 24.10 | 4.13 | 1.65 | -0.94 | +22.6 | 18.3 | 0.045 | heading crosses through path tangent |

Two solver-internal columns confirm the controller was not failing to
solve, it was solving correctly against a bad reference: `nmpc_status`
stays `1` (converged) at every tick in this window, while
`nmpc_pred_ey_end` (the solver's own predicted lateral error at the far end
of its horizon) grows from -0.07 m at 23.59 s to -1.59 m at 24.05 s, and
`nmpc_pred_ey_max_abs` reaches 2.02 m, both far outside anything seen
elsewhere in the run. The optimizer is reporting, correctly, that its own
plan ends up 1.6-2.0 m off the path it was given, because the far end of
that path was a stale, frozen point rather than the real corner. `e_psi`
grows steadily and monotonically while steering saturates, then jumps from
-87.9 to +22.6 degrees in one 50 ms tick, consistent with the car's actual
heading crossing through the path tangent direction (a genuine physical
near-spin, not a telemetry glitch: `e_y` and `v_actual` both stay continuous
and physically reasonable across the same tick). `v_desired` collapses hard
(4.47 to 1.65 m/s over 0.35 s) as the live curvature-based speed target
reacts to the approaching corner independently of the truncated path
defect.

## What this rules out

- **Not a solver failure.** `nmpc_status=1` throughout; the SQP converged
  to a correct optimum for the (bad) reference it was given.
- **Not the documented centreline curvature-spike defect** (CLAUDE.md,
  "Before changing the planner"). That defect is a smoothing artifact in
  otherwise-complete path geometry; this is a path that is missing data
  entirely for part of its length. Different failure mode, same general
  area of the planner.
- **Not a `curvature_speed()` lookahead-timing fault.** `v_desired` reacts
  correctly and continuously to the approaching corner across this window;
  the failure is in the geometry the heading/lateral error are computed
  against, not the speed target.

## What is not established

- **Why the planner emitted a short path at this one tick.** Whether this
  is a transient perception/planning glitch (e.g. a momentarily short cone
  detection range cutting the planned centreline short) or a systematic
  planner behavior at this corner shape is not known; the planner's own
  code was not instrumented for this investigation.
- **Whether NMPC should detect and reject/hold on a truncated path.** No
  validation currently distinguishes "planner published fewer real points
  than usual" from "planner published its normal full-length path." Adding
  such a check (e.g. detect a run of repeated trailing points, or a
  minimum path arc length before trusting a new snapshot) is a plausible
  fix, not yet designed or tested.
- **Single occurrence.** One corner, one run. Not yet checked whether the
  same planner-side truncation reproduces on a repeat run or at other
  corners.

## Status

Root cause confirmed (truncated live planner path snapshot at `t=23.62 s`,
verified against both the control and path logs). Not yet fixed: no guard
against a short/truncated planner path exists in NMPC or in the planner
output today. Planner-only NMPC is otherwise unaffected by this failure
mode in this run: the run completes normally and this is the only
truncated snapshot in 62.7 s of driving (one per second, ~63 snapshots
total).
