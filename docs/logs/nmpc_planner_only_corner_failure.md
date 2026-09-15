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
- **Single occurrence.** One corner, one run. Not yet checked whether the
  same planner-side truncation reproduces on a repeat run or at other
  corners.
- **Not fixed at the source.** The fix below stops NMPC from being misled
  by a truncated path; it does not explain or address why the planner
  emitted one in the first place.

## Fix: `PathReference` now detects and drops a frozen trailing tail

`PathReference.__init__` (`nmpc_core.py`) scans the incoming path's segment
lengths from the end and drops any trailing run of near-zero-length
segments (repeated last-point padding) before computing arc length,
curvature or reference heading. The path's real length (`self.total`) then
reflects only the genuine data, and the existing edge-hold behaviour in
`kappa_at`/`psi_ref_at` (see their docstrings, unchanged) takes over from
the true last point: a horizon that runs past real data holds the last
*real* curvature/heading sample, rather than reading a frozen duplicate
point as flat, stopped geometry. This directly matches the mechanism
confirmed above (`nmpc_pred_ey_end`/`nmpc_pred_ey_max_abs` blowing out
because the reference stopped moving with the corner).

Verified against a synthetic padded path (20 real points on a curve plus 30
duplicated final points): `PathReference.total`/`.path` correctly reflect
only the 20 real points, and `kappa_at` beyond that range returns the last
real curvature rather than the padded value. `python -m
tuner.nmpc_offline_check` passes unchanged (model parity, SQP convergence,
turn-in sign checks, closed-loop DNF check), confirming the truncation
detection does not alter behavior on any normal, non-truncated path (every
recorded/precomputed path check in that suite has no trailing duplicates,
so `real_n` never shrinks and the new code path is a no-op there).

Mirrored to `fsds_simulator/control/fsae_control/fsae_control/mpc/nmpc_core.py`
(diff against the live copy is empty).

## Planner-side root cause, found in fsae_planning's own upstream history

The question left open above ("why does the planner emit a short path at
all") has an answer: this exact failure was already found and fixed
upstream (`origin/main` of `fsae_planning`, commit `0b4397e`, not yet
merged into the branch this repo runs, `feature/nmpc-and-controller-
improvements`, and not yet applied here at the time this was written).

`path_utils.blend_paths()` re-anchors the previous tick's published path
and the freshly-planned one onto a common forward grid
(`_resample_forward`) and averages them. `_resample_forward` clamps
samples past a path's real end to its last point, exactly the mechanism
that produces a frozen trailing tail: if the fresh path is genuinely short
this tick (nothing wrong with it, it is just short), its samples past its
own real end silently repeat its last point, and the OLD behaviour then
blended those repeated-point samples against the PREVIOUS path's genuine
further-out data, publishing a result that extends past where the fresh
path was actually validated to reach, using stale geometry to fill the
gap. That published, padded array is exactly the `t=23.62 s` snapshot
described above (51 points, only 20 distinct).

Fixed directly in `path_utils.py` (`fsae_planning`, applied here from the
upstream commit): `_resample_forward` now also returns the path's real arc
length (`total`), and `blend_paths` truncates its output to the FRESH
path's own validated length rather than the full blend horizon. A short
fresh path now produces a short, honest published path instead of a padded
one. Verified with a synthetic case (5 m fresh path blended against a 30 m
stale one): the published blend now stops at 5.0 m, where it previously
extended to 11.0 m of largely stale geometry.

This planner-side fix is the actual root cause fix. The NMPC-side
`PathReference` fix described above remains as defence in depth (any other
future path-shortening mechanism, or a planner change that reintroduces
padding, still can't feed NMPC a frozen tail as if it were real geometry),
but the padding itself should no longer be produced upstream of NMPC now
that `blend_paths` is fixed.

## Status

Root cause confirmed at TWO levels: the immediate mechanism (`PathReference`
trusting a frozen trailing tail as real geometry, fixed in `nmpc_core.py`)
and the actual source (`blend_paths` publishing a padded array when the
fresh path is shorter than the blend horizon, fixed in `path_utils.py`,
ported from `fsae_planning` upstream `main`). Both fixes are applied and
offline-verified (synthetic tests plus `tuner.nmpc_offline_check`/
`recorded_map_rollout`, unchanged). **Not yet live-tested together**: this
run cannot be replayed against either fix offline (the failure only occurs
against a live, per-tick-rebuilt planner path), so validate with a fresh
live planner-only run before considering this closed.
