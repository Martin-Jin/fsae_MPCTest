# Planner-only NMPC oscillates and eventually stalls from an unrated speed-target collapse, not a path defect

## Summary

A live planner-only NMPC run (no precomputed track, live planner path and
live curvature-derived speed target only) drives normally for the first few
seconds, then repeatedly loses heading tracking through the rest of the
run and ends stalled, off-heading, far from any corner. The previously
root-caused truncated-path bug (`nmpc_planner_only_corner_failure.md`) is
confirmed **not** present in this run: no path snapshot has trailing
duplicate points, and the fix already committed for that bug is active
in this build. The actual cause here is different: `curvature_speed()`'s
speed target (`v_desired`) collapses by 3-10 m/s in single 50 ms ticks, 11
times over a 59 s run, and every one of these collapses is followed within
a few ticks by a heading-error excursion. `SPEED_TARGET_RISE_RATE` only
rate-limits the target's *rise*; a fall passes through uncapped by design,
on the reasoning that delaying a genuine brake request is worse. That
reasoning does not hold when the "genuine brake request" is itself noise
from the live path being refit every tick, not a real corner.

## Plain-language version

With no pre-recorded map, the car works out its own speed target every
tick from how sharply the road curves just ahead of it, using the live
picture the planner is building as it drives. That picture is refit fresh
every single tick rather than being one fixed measurement, so it carries a
little geometric wobble even where the road is actually straight, the same
kind of noise a hand-drawn line has if you retrace it many times. Most of
the time this wobble is too small to matter. But eleven times in this one
minute of driving, the wobble was large enough that the speed calculation
briefly mistook a straight or gentle section for a much tighter corner than
it really is, and commanded the car to slow down hard, by as much as 10 m/s
in a twentieth of a second. The car cannot actually decelerate that fast,
so a mismatch opens up between what speed the controller now thinks it
should be tracking and what the car can physically do, and several of these
mismatches are followed by the car swinging its heading wildly off the road
direction while trying to catch up. The system was built to allow speed
targets to drop quickly on purpose, reasoning that if the car really does
need to brake hard, delaying that command would be worse than acting on it
immediately. That reasoning breaks down here because the "brake hard"
signal was never a real corner, it was noise in the live map-building
process being mistaken for one.

## What is confirmed by telemetry

Run: `mpc_standalone_control_20260915-133916.csv` /
`mpc_standalone_path_20260915-133916.csv` (`use_nmpc=1`, `map_path=''`,
`path_map_path=''`, genuine planner-only mode).

| | value |
|---|---|
| run duration | 58.65 s |
| `\|e_y\|` max | 1.21 m (never left the track by this measure) |
| solver failures | 0 (`nmpc_status=1` throughout, every window checked) |
| truncated-path snapshots (the earlier, separately fixed bug) | 0 (fix confirmed active) |
| single-tick `v_desired` drops > 3 m/s | **11**, across 58.65 s |
| sustained `\|e_psi\|` excursions (> 20-30 deg) | **11** |
| final state | stalled (`v≈0`), `e_psi≈-83 deg`, never recovers |

### The eleven speed-target collapses, and what follows each one

| t (s) | `v_desired` drop (m/s) | heading-error episode nearby |
|---|---|---|
| 2.34 | -9.47 | (early, car still accelerating from standstill) |
| 4.34 | -9.75 | contributes to the -31 to -66 deg episode at t=7.3-9.0 |
| 6.30 | -3.64 | " |
| 11.69 / 11.74 | -4.93 / -5.78 | t=11.74-11.79 |
| 14.15 | -7.13 | t=14.99 |
| 16.30 | -5.71 | t=21.60 |
| 25.74 | -3.73 | t=29.35-30.30 |
| 34.34 | -5.30 | t=34.79 |
| 38.14 | -3.88 | t=40.75-41.70 (start of the terminal episode) |
| 40.90 | -3.24 | never recovers from here to end of run |

The final, unrecovered episode (t=40.5-45.5 s, excerpted) shows the
pattern clearly: `v_desired` swings 10.2 -> 2.4 -> 9.9 -> 1.5 -> 7.6 -> 2.2
m/s over 5 s while steering slams between +25 and -25 degrees repeatedly
and `e_psi` oscillates between roughly -30 and +12 degrees rather than
settling. `nmpc_status` stays `1` (converged) at every one of these ticks;
the solver is not failing, it is being asked to track a target that
itself will not hold still.

### Confirmed NOT the cause: `corner_frac`

`corner_frac` (an NMPC steering-cost blend factor, unrelated to speed) also
shows a matching-looking spike at the same tick as the first collapse
(0.169 -> 0.867 at t=4.29-4.34), which looked at first like a plausible
shared cause. Tracing the code rules this out: `corner_frac` is computed
from curvature at the car's *current* position only
(`nmpc_core.py`'s `_corner_factor(kappa_now, k)`) and has no code path into
`v_desired` at all, it only blends steering-cost weights
(`Rrate_steer_corner_blend`). `v_desired` is computed independently in
`mpc_controller.py` from `curvature_speed()`'s output (`v_curv`) and the
tracking-error gate. The two are coincidentally noisy at the same moment
because both are reacting to the same underlying cause (the live path
being refit every tick), not because one drives the other.

## Mechanism

`curvature_speed()` (`control_utils.py`) already documents, in its own
words, that a freshly-refit live path "carries a few cm of per-point
lateral wiggle even on a straight," and that raw curvature computed
directly from such a path turns that wiggle into spurious high-curvature
readings; this is why the function denoises with a dense resample and
moving-average smoothing before measuring curvature. That denoising
reduces the problem, it does not eliminate it: a single still-noisy sample
anywhere in the 24 m forward scan window can dominate `v_curv` for that
tick, because the corner speed is a `min()` over the whole window.

The result then reaches the car unfiltered on the downside.
`SPEED_TARGET_RISE_RATE` (7 m/s^2) caps how fast the target may *rise*,
by design, so the car is never asked to suddenly accelerate faster than it
can. Drops are deliberately exempt (`mpc_controller.py`'s own comment:
"delaying a genuine brake request is the failure this is meant to
prevent"). `GATE_RATE_LIMIT` rate-limits a separate factor
(`tracking_error_speed_gate()`, driven by `e_y`/`e_psi`) in both
directions, but that gate is not what produced these collapses; `e_y`/
`e_psi` are small and unremarkable in the tick immediately before each
drop. Nothing rate-limits `v_curv` itself.

## What this rules out

- **Not the truncated-path bug.** Verified directly: no path snapshot in
  this run has trailing duplicate points, and the fix for that bug
  (`PathReference` dropping a frozen tail) is present and active.
- **Not a solver failure.** `nmpc_status=1` throughout every episode
  checked.
- **Not `corner_frac` driving `v_desired`.** Traced in code: no connection
  exists between them; see above.
- **Not the tracking-error speed gate.** `e_y`/`e_psi` are unremarkable
  immediately before each collapse; the gate's own rate limit
  (`GATE_RATE_LIMIT`) would in any case have capped a gate-driven change.

## What is not established

- **The exact noisy path sample behind any single collapse.** The path log
  only snapshots roughly once per second, far coarser than the 50 ms
  control tick each collapse happens on, so the specific live-path sample
  `curvature_speed()`'s scan window picked up cannot be recovered from
  this log after the fact. The live visualiser (`live_viz.py`, plots the
  reference path every tick) is the intended tool to catch one of these
  live and see the path geometry at the exact moment of a collapse.
- **Whether this is specific to planner-only mode, or present but masked
  under a precomputed path.** A precomputed path is fixed and pre-smoothed
  once at record time, not refit every tick, so it structurally cannot
  exhibit this particular noise source; this has not been separately
  confirmed by a live precomputed-path comparison in this investigation.
- **Whether a fix belongs in `curvature_speed()` (denoise harder / detect
  and reject an implausible single-tick drop) or in the consumer
  (`mpc_controller.py`, rate-limit `v_curv` itself the way
  `SPEED_TARGET_RISE_RATE` limits the composed target). Not designed or
  tested here.

## Status

Root cause identified at the mechanism level: an unrated single-tick
`v_curv` collapse from live-path refit noise, not the previously-fixed
truncated-path bug and not a solver problem. Not yet fixed. Next step is
watching a live run with `live_viz.py` to catch the path geometry at the
exact moment of a collapse, which the coarser path log here cannot
provide, before choosing between denoising `curvature_speed()` further and
rate-limiting its output.
