# Planner-only NMPC survives lap 1 but spins out on lap 2 at the same corner

## Summary

With both the truncated-path fix (`nmpc_planner_only_corner_failure.md`) and
the speed-target fall-rate fix (`planner_only_speed_target_oscillation.md`)
applied, a live planner-only run completes a full first lap cleanly, then
fails during the second lap at one specific corner and never recovers. The
corner itself produces the same symptom on both laps: a cluster of rejected
NMPC solves under hard braking, heading error swinging past 80-90 degrees,
steering pinned at the 25 degree lock. Lap 1 survives this and recovers
within about a second; lap 2 does not. The difference is not a new bug, it
is less margin: lap 2 carries about 1 m/s more speed into the same corner,
which is enough to tip an already-marginal recovery into a permanent one.

## Plain-language version

The car now finishes a full lap on its own, driving entirely off the live
planner with no pre-recorded map, which it previously could not do. But on
the second lap, at one specific corner, the same struggle that happened on
lap 1 happens again, slightly worse, and this time the car does not pull
out of it. Both times, right as the car brakes hard for this corner, the
steering controller's internal solver briefly fails to find a better plan
several ticks in a row, so it just keeps the previous tick's plan instead of
updating it. On lap 1 that was enough of a stumble to swing the car's
heading badly for about a second, but it caught back up. On lap 2 the car
arrived at the same corner going slightly faster, so the same stumble left
it further behind, and it never caught up: the heading kept sliding past
where the road was pointing until the car effectively spun to a stop. This
is not a new failure introduced by anything changed today, it is the same
underlying weak point at this corner showing through more clearly now that
the earlier, more severe bugs that used to mask it are fixed.

## What is confirmed by telemetry

Run: `mpc_standalone_control_20260915-142634.csv` (`use_nmpc=1`,
`map_path=''`, `path_map_path=''`, genuine planner-only mode, both prior
fixes active).

| | value |
|---|---|
| run duration | 70.2 s |
| max `\|e_y\|` | 1.58 m (never left the track by this measure) |
| NMPC solves where the SQP step was rejected | 24 of 1406 ticks (1.7%) |
| lap 1 | completes (car passes back near the start line at t=59.2-59.7 s) |
| lap 2 | fails at the same corner it struggled at on lap 1, does not recover |

### The same corner, twice, with a different outcome

| | lap 1 (t≈32.5-34.3 s, car near (-21, 90)) | lap 2 (t≈61.5-63.9 s, car near (48, 9)) |
|---|---|---|
| peak approach speed | 16.7 m/s | 17.5 m/s |
| solver-rejected ticks in this window | 6 | 8 |
| peak `\|e_psi\|` reached | 98 deg | recovers only briefly, then reaches 95 deg and does not recover |
| outcome | recovers: `e_psi` snaps back from -93 to -32 to -20 deg over 3 ticks, speed climbs again | does not recover: `e_psi` keeps sliding to -79, -83, -90, -94 deg and the car stalls |

`nmpc_status` (whether the SQP's proposed step was accepted this tick)
drops to `0` in a tight cluster at both corners, never scattered randomly
elsewhere in the run: `t=32.789`, then six more between `33.900-34.200`;
`t=62.691` through `62.888` (5 ticks), then three more at `63.400-63.550`.
Every one of these ticks has `nmpc_iters=1` (one SQP iteration ran) and
`solve_ms` in the 20-38 ms range, several above the 25 ms solve budget.
Tracing the solve loop (`nmpc_core.py`) confirms this combination can only
mean the iteration ran and its step was rejected by the backtracking line
search (`status='rejected'`), not that the budget cut the solve off before
an iteration started (that path leaves `iters` at 0, not 1, before the
break). A rejected step is not unsafe by design: the controller falls back
to holding the previous tick's plan rather than accepting a direction that
made the true nonlinear cost worse. But several of these in a row, all in
the same hard-braking window, mean several ticks pass without the plan
actually updating to the car's real, fast-changing situation.

## Mechanism

Both corners show the identical pattern: the car is braking hard
(`v_desired` dropping several m/s per tick, a real corner, not a noise
artifact this time) while heading error is already growing. The SQP scheme
here takes exactly one Gauss-Newton step per tick and relies on next
tick's re-linearization to correct course rather than iterating to
convergence within a tick (`nmpc_sqp_iters=1`, documented in
`nmpc_params.py`). When the reference itself (both the car's own state
under hard braking and the path curvature ahead) is changing quickly
tick-to-tick, that single linearized step is more likely to end up not
actually improving the true nonlinear cost, and gets rejected. A single
rejected tick is harmless, the car simply keeps its last plan for 50 ms.
Several rejected ticks in a row, exactly when the car most needs an
updated plan (deep in a corner entry, decelerating hard), is where the
heading error is free to keep growing unchecked by a fresh correction.

Lap 1 and lap 2 hit this same weak spot. The difference in outcome tracks a
difference in entry speed (16.7 vs 17.5 m/s) and in how many ticks got
rejected in the cluster (6 vs 8) — both consistent with the same marginal
recovery margin, not a different failure mode. This reads as an existing,
narrow safety margin at this specific corner's SQP convergence, not
something introduced by either fix applied earlier the same day.

## What this rules out

- **Not the truncated-path bug or the speed-target-collapse bug.** Both
  fixes are active in this run; neither symptom (a duplicated-tail path
  snapshot, an unrated single-tick `v_desired` fall) appears anywhere near
  either corner event.
- **Not a solver crash or an unsafe fallback.** `nmpc_status=0` here means a
  step was correctly rejected as not-an-improvement, not that the solver
  produced a bad command. The fallback (hold last tick's plan) is the
  intended, safe behaviour of a rejected step.

## What is not established

- **Whether this corner is geometrically unusually sharp/fast**, or whether
  any corner taken at a similarly high entry speed under hard braking would
  show the same rejected-step clustering. Not compared against other
  corners in this run.
- **Whether raising `nmpc_backtrack_max` (more step-halvings before giving
  up) or lowering `nmpc_sqp_iters`'s reliance on a single step (multiple SQP
  iterations per tick, at extra solve-time cost) would close this specific
  margin.** Neither tried here.
- **Whether this reproduces at the same corner reliably**, or whether a
  repeat run at the same speed profile fails or passes both laps by chance
  (the "1e-9 m shift in start position flips the outcome" chaotic
  sensitivity already documented elsewhere for this NMPC path suggests this
  is plausible without any code change).

## Status

Not root-caused to a fixable bug; the fixes applied earlier today (truncated
path, speed-target fall-rate) are confirmed working and this is a distinct,
pre-existing narrow margin at one corner's SQP convergence under hard
braking, now visible because the car survives long enough to reach lap 2
and encounter it twice. Recorded so a future "car spins out on lap 2" report
starts here rather than re-investigating the already-fixed bugs.

## Two candidate fixes tried, one confirmed working, one confirmed not

**Solve-latency compensation (`nmpc_latency_compensation_enabled`): tried,
does not close the margin.** Rolls the car's own state forward through the
solve's own wall-clock time, the same way the existing pose-age delay
compensation already rolls it forward through stale pose measurement. Two
live runs with the mechanism confirmed engaged (`n_latency=1` throughout,
after fixing a `round()`-half-to-even bug that silently zeroed it at its own
25 ms default) show a small rejected-solve-rate improvement but the same
stall mechanism recurs. Consistent with the root cause being reference
volatility, not the fixed solve-time gap: this compensates a KNOWN, roughly
constant delay, but the thing actually swinging tick to tick is the live
planner's own curvature/speed estimate, which this mechanism never touches.
Reverted to its safe default (off); kept in the code as a real, working
mechanism for a different problem.

**Curvature-reference rate limiting (`nmpc_kappa_rate_max`): tried, helps.**
Same idea as the speed-target fall-rate fix, applied to the horizon's own
curvature profile instead: caps how fast `kappa(s)` at a given arc-length
point is allowed to change between ticks, live-planner mode only. At 2.0
1/m/s, the car survives the corner that previously stalled permanently on
every recorded run (rejected-tick rate roughly halves, 1.7% to 1.0%,
cluster size at the corner itself shrinks), though it still visibly stumbles
through it (speed collapse, `e_psi` swinging out to -103 degrees before
recovering). Tightening to 1.0 was tried and made it WORSE: the car stalled
at the same corner, and `nmpc_kappa_horizon_end` showed MORE sign-flipping
through the corner at 1.0 than at 2.0, not less. Reading: 1.0 is tight
enough to lag the corner's own genuinely fast-firming-up curvature estimate
as the car gets close and perception resolves it, and then overshoot when
the reference catches up, an oscillation, not a smoothing improvement. This
is a real corner-specific rate requirement, not obviously noise: **do not
retest values below 2.0 without new evidence**, and note that any future
retuning of this constant should check `nmpc_kappa_horizon_end` for
oscillation, not just the rejected-tick count, since the count alone
improved at 1.0 relative to no limiter at all while the actual outcome got
worse.

Neither test is a full root-cause fix; both are corner-symptom mitigations.
The corner still visibly stumbles even at the validated 2.0 setting.

## Separate precomputed-path corner failure: a genuine off-centre centreline

A later same-day precomputed-path run (`path_map_path`/`map_path` both set,
not the live planner) showed a different corner (`comp_test_map_3`, arc
length ~177-196 m, near map coordinates (35-42, 40-50)) producing a large
`e_y` excursion (up to -2.7 m) and the car visibly hugging the yellow cone
wall. This is NOT the live-planner reference-volatility mechanism above
(precomputed mode has no live-planner noise at all) and NOT a solver
failure (`nmpc_status=1` throughout, the solver's own
`nmpc_pred_ey_max_abs` tracked the growing error smoothly and accurately).

Root cause, confirmed by direct measurement against the cone map: the
precomputed centreline is genuinely off-centre through this corner,
consistently closer to the yellow boundary than the blue one (1.5-2.1 m to
yellow vs 2.0-2.4 m to blue at several sample points), through the
tightest part of the track (radius bottoms at ~4.65 m here vs 7-10 m
elsewhere nearby). This is a data problem in `centerline.csv`, not
something any controller-side tuning can fix. Not yet corrected; flagged
here so a future "car hugs the wall at this corner" report starts from
this finding rather than re-investigating the controller.

### Debugger fix, found and fixed along the way

Diagnosing this required trusting `live_viz.py`'s displayed reference
path, which was showing the LIVE PLANNER's rolling output even in
precomputed-path mode, not the actual precomputed reference the car was
driving against. Root cause: `centerline_planner.py` had no
`use_precomputed_path` awareness and kept running/publishing regardless
(no launch-time gating existed). Fixed two ways together: `sim.launch.py`
now gates `planning.launch.py`'s own inclusion on `use_precomputed_path`
(the planner process itself never starts in that mode), and
`mpc_controller.py` publishes the static path once, with `TRANSIENT_LOCAL`
QoS durability (not a fixed-delay timer, which was tried first and still
lost the race against `live_viz.py` starting before the controller node
exists), on its own topic (`/fsae/control/static_reference_path`, separate
from the live planner's topic to avoid a QoS durability conflict).

## Speed-target filter (`nmpc_v_des_filter_alpha`): wide sweep, landed on 0.09

Same day, a THIRD corner (a different hairpin, ~11 m to 4.65 m radius,
target speed collapsing ~4.7 m/s in ~1 s) showed the NMPC's actual
optimisation target (`v_ref`, filtered through this alpha) lagging the raw
`desired_speed` by up to 1.7 m/s for over a second, so commanded braking
stayed weak while the car carried too much speed into the tightest part of
the corner. This filter (originally 0.08, dt/alpha ~= 0.6 s time constant,
introduced 2026-06-29 to smooth ~1 Hz live-planner target jumps) has no
offline analogue at all — `nmpc_optimiser.py` never filters the speed
target — so it had never been offline-validated or tuned against this
specific lag.

**Raising alpha to fix the lag made OVERALL performance worse at every
large step tried**, confirmed across multiple full live runs: 0.25
(dt/alpha ~= 0.2 s) caused oscillating accel/brake commands and steering
hunting at several OTHER corners plus a genuine off-track excursion; 0.13
still showed 4 excursion clusters (one worse than the 0.08 baseline's
single corner); disabling the filter entirely (`v_ref = desired_speed`,
no smoothing) was worse still, 7 clusters, the worst result of the whole
session. This flips the naive read of the lag finding: the filter is
doing real, useful noise rejection, and blanket speedup is the wrong
shape of fix.

**A later same-evening run also showed a broader regression** (not
specific to any one corner): jerky/twitchy behaviour on GENTLE corners
that had never been a problem before, with measurably worse `jerk_rms`
(0.58 vs 0.15) and `control_smooth_rms` (0.45 vs 0.13) than the best run
of the day. Three things had changed by that point: a fast `alpha`, plus
two OTHER same-day additions, `nmpc_kappa_rate_max` and
`nmpc_latency_compensation_enabled`. Disabling all three at once
recovered most of the regression (score 0.476, jerk_rms 0.126, close to
the day's best). Re-enabling `nmpc_kappa_rate_max=2.0` alone (validated
value, `alpha` kept at the original 0.08) produced the BEST smoothness
numbers of the day (score 0.431, jerk_rms 0.115) — consistent with
`_path_reference()`'s own code confirming this limiter is structurally
inert whenever a static/precomputed path is set (returns the cached
static reference before ever reaching the rate-limiting code), so it was
never a real suspect for a precomputed-path regression. This leaves
`nmpc_latency_compensation_enabled` (which runs every tick regardless of
path source, unlike the kappa limiter) as the live, un-isolated suspect
for the smoothness regression, alongside the already-evidenced too-fast
`alpha` values — NOT separately isolated from each other before landing
on the settled config below.

**Landed configuration, live-tested as the best full-run result of the
day**: `nmpc_v_des_filter_alpha=0.09` (a small nudge above the original
0.08, not a large jump), `nmpc_kappa_rate_max=2.0` (validated, structurally
inert on this precomputed-path test), `nmpc_latency_compensation_enabled=
False` (reverted, unresolved suspect). Score 0.410 (vs the day's best-ever
0.369 at the unmodified original config, and the 0.476 clean-baseline
recovery run), zero rejected solves, `max|e_y|` 0.84 m, healthy jerk/
smoothness numbers. Not a proven optimum — `nmpc_latency_compensation_
enabled` was never tested in isolation to confirm or clear it as the
regression's actual cause, so treat it as guilty-by-association, not
guilty-by-evidence, if revisiting this later.
