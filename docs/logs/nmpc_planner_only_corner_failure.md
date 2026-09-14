# NMPC in planner-only mode loses heading control at one tight corner

## Summary

A single live run of NMPC with no precomputed track (`map_path`/
`path_map_path` both unset, live planner path and live curvature-derived
speed target only) shows a severe transient heading-tracking failure
approaching one tight corner: heading error runs from -23 to -88 degrees in
0.3 s while steering is pinned at the 25 degree lock, then the sign flips.
The car does not go off-track by lateral error and does not stall; it
recovers and finishes the run normally. Root cause is not established, only
the immediate mechanism (steering saturation while decelerating hard for
the corner).

## Plain-language version

The car had no pre-recorded map for this run, so it built its own picture
of the track live from the planner as it drove, and worked out its own
speed target for each corner as it went, rather than reading both off a
file made in advance. Nearly all of a one-minute run this worked fine, but
at one corner the car's actual heading swung wildly away from where the
path pointed, close to being turned side-on to the track, over about a
third of a second. The steering was already turned as far as it can go
(hardware limit) while braking hard for the same corner, so it did not have
any spare steering authority left to correct with. The car did not run off
the track and drove normally again straight after, but this is exactly the
kind of moment that would be dangerous with less margin (a tighter track,
a faster approach, or slightly worse luck on timing).

## What is confirmed by telemetry

Run: `mpc_standalone_control_20260915-084943.csv` (`comp_test_map_3`,
`use_nmpc=1`, `map_path=''`, `path_map_path=''`, confirming genuine
planner-only mode, no precomputed path or speed profile in play).

| | value |
|---|---|
| run duration | 62.7 s |
| `\|e_y\|` max | 1.49 m (track half-width 3.5 m, never approached) |
| `v_actual` max | 17.9 m/s |
| steering at/near 25 deg lock | 3.9% of ticks |
| solver failures | 0 |
| run outcome | completes, no stall, no off-track by `e_y` |

Failure window, `t = 23.75-24.10 s`:

| t (s) | v_actual | v_desired | e_y | e_psi (deg) | steer (deg) |
|---|---|---|---|---|---|
| 23.75 | 4.10 | 4.47 | -0.62 | -23.3 | -3.4 |
| 23.95 | 3.64 | 3.09 | -0.92 | -50.5 | 16.6 |
| 24.00 | 3.84 | 2.59 | -1.06 | -59.1 | 21.8 |
| 24.05 | 3.95 | 2.10 | -1.06 | -87.9 | 25.0 |
| 24.10 | 4.13 | 1.65 | -0.94 | +22.6 | 18.3 |

`v_desired` collapses hard (4.47 to 1.65 m/s in 0.35 s) as the live
curvature-based speed target reacts to the approaching corner. `e_psi`
grows steadily and monotonically while steering saturates, then jumps from
-87.9 to +22.6 degrees in one 50 ms tick, consistent with the car's actual
heading crossing through the path tangent direction (a genuine physical
spin-toward-the-corner, not a telemetry glitch: `e_y` and `v_actual` both
stay continuous and physically reasonable across the same tick).

## What is not established

- **Whether the live-built planner path itself had a geometric kink at this
  corner is not confirmed.** The path log's nearest sample to this instant
  (t=23.62, path only logged at coarser intervals than the control loop) shows
  no sharp turn (max consecutive-segment angle checked, none exceeded 20
  degrees), but this does not rule out a kink appearing between logged
  samples, or noise finer than that check's resolution. The known,
  documented planner defect (centreline curvature spikes, see CLAUDE.md's
  "Before changing the planner" section) remains a plausible contributor,
  not a confirmed one here.
- **Whether the live speed target (`curvature_speed()`) reacted late rather
  than wrong is not established.** It did correctly command a much lower
  speed for the corner; whether it had enough lookahead distance to give
  the car time to actually decelerate to that speed before the corner
  arrived, versus a precomputed profile's longer effective lookahead, is
  not measured here.
- **Single occurrence.** One corner, one run. Not yet checked whether this
  reproduces at the same corner on a repeat run, or is specific to this lap.

## Status

Not root-caused. Recorded so a future "car spins out in planner-only mode"
report is not re-investigated from scratch, and so any future planner or
`curvature_speed()` lookahead change can be checked against this specific
corner/timestamp. Planner-only NMPC is otherwise unaffected by this open
question: the run completes normally and this is the only such event in
62.7 s of driving.
