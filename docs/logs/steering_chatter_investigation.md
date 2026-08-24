# Steering chatter investigation (NMPC, live + offline)

## In plain terms

The car's steering wheel was twitching instead of holding a smooth angle
through a corner, and it also seemed reluctant to turn in. Two separate causes,
both now fixed:

1. **The controller was not charged enough for moving the wheel.** Its cost
   function had a term meant to discourage rapid steering changes, but the
   price was set roughly 18× too low, so constant small corrections were
   nearly free. Raising it removed most of the twitching.
2. **The line the car was told to follow was one it could not drive.** The
   "racing line" cut corners in a way that demanded more grip than the
   simulator provides. Following the middle of the track instead removed the
   rest — with no change to any controller setting.

A third, separate problem (sudden jumps in steering at the tightest corners)
turned out not to be a steering problem at all, but the car arriving at those
corners too fast. See "Fault 2" at the end.

---

**Status: RESOLVED, in two parts.**

- **`r_rate_delta` was far too low.** Raising the steering-rate cost in the
  optimiser's objective from 2.8 to ~50 (live-tested) more than halved the
  chatter. *(`r_rate_delta` prices each unit of change in the steering command
  between one 50 ms control tick and the next.)*
- **The reference line was a confound for all of it.** Switching from
  `raceline.csv` to `centerline.csv`, with every weight held fixed, took
  steering reversals 13 → 1 and both saturation and slew-limited ticks to
  exactly 0.

**Before trusting any absolute number in the sections below**, check which
reference line it was measured against — see "The reference line was a
confound". A figure measured on the raceline includes the cost of recovering
from excursions the line itself caused, which is not attributable to the weight
under test.

Everything below is retained as ruled-out ground. Note that one of the "ruled
out" entries is **wrong**, and the reason is instructive.

## Resolution

Live A/B on `comp_test_map_3` (t>5s, NMPC, output_smoothing off unless noted):

| `r_rate_delta` | std_d | mean\|d\| | sign-flip | \|e_y\| | \|e_psi\| |
|---|---|---|---|---|---|
| 2.8 (shipped) | 3.145° | 2.538° | 66.1% | 0.177 | 5.37° |
| 20 | 2.685° | 1.913° | 65.0% | 0.204 | 4.99° |
| **50** | **1.855°** | **1.173°** | **55.5%** | 0.211 | 4.60° |

`mean|d_steer|` per tick more than halved (2.54 → 1.17°), std_d down 41%,
and the sign-flip rate finally moved (66% → 55.5%) after being pinned near
65% through every other intervention tried. Cost is a modest `|e_y|` rise
(0.177 → 0.211 m); `|e_psi|` actually improved.

Caveats on those numbers: `r_rate_a` also moved (2.25 → 5.0) in the same
runs, so the longitudinal weight is a confound for the `|e_psi|`/speed
columns (the steering result is attributable to `r_rate_delta`); and the
20/50 runs are shorter (42-62 s vs 90 s), so they may not cover identical
corners — the per-tick chatter metric is robust to that, `|e_y|` less so.

### Follow-up sweep and the shipped value (52.5)

Later live runs, on the faster speed profile (`CURVATURE_SPEED_A_LAT_MAX`
4.0 → 5.5) and with `output_smoothing` off:

| `r_rate_delta` | mean\|d\| | flip% | \|e_y\| | \|e_psi\| | v | sat% |
|---|---|---|---|---|---|---|
| 0.0 (control) | 4.724° | 70.9% | 0.465 | 7.49° | 9.40 | 1.09 |
| 52.5 | **1.919°** | 59.7% | 0.288 | 5.84° | 10.29 | 0.03 |
| 70.0 | 1.786° | 60.7% | 0.306 | 5.96° | 10.33 | 0.04 |

The `r_rate_delta=0.0` run is the cleanest confirmation of the diagnosis:
with the rate cost fully off, chatter is 4.72°/tick vs 1.92° at 52.5.
**52.5 is shipped** — it beats 70 on `|e_y|` and `|e_psi|` while the chatter
difference is small. Persisted in `mpc_params.py`, `fsae_params.yaml` and
`settings.py` (`R_rate_diag[0]`).

Versus the pre-investigation baseline (`r_rate_delta` 2.8, `a_lat_max` 4.0,
`output_smoothing` ON): chatter 2.538 → 1.919°, speed 9.74 → 10.29 m/s,
saturation 0.00 → 0.03%, but `|e_y|` 0.177 → 0.288 m (+62%). That `|e_y|`
rise is shared between the rate damping and the faster speed profile and has
NOT been attributed between them — one run at `a_lat_max=4.0` with
`r_rate_delta=52.5` would separate them.

### RESOLVED: the offline harness DNF was caused by `a_lat_max=5.5`

At `a_lat_max=5.5` + `r_rate_delta=52.5`, `python -m tuner.nmpc_offline_check`
**DNF'd** the NMPC closed-loop section (`|e_y|` mean ~0.50, offtrack, ~450 of
~1000 ticks) while the same config ran cleanly live.

**Fixed by `a_lat_max` 5.5 → 4.75** (the current shipped value). The harness
now passes every check, NMPC included: `|e_y|` mean 0.459, p90 0.990,
saturation 4.3%, `reached_end=True`, `dnf=False`.

The offline/live disagreement was therefore real but not a harness artefact —
the offline plant was simply less tolerant of a corner speed that the live car
survived, and lowering the planned cornering acceleration removed the failure
on both sides. Retained here because the same signature (offline DNF, live
clean) recurred repeatedly during this investigation and this is the one case
where the cause was found.

Isolated offline (via `tuner/steering_chatter_check.py --set`):

| a_lat_max | r_rate_delta | offline result |
|---|---|---|
| 4.0 | 2.5 | OK (flip 48.8%, \|e_y\| 0.295) |
| 4.0 | 50 | OK (flip 34.2%, \|e_y\| 0.376) |
| 5.5 | 2.5 | OK (flip 50.4%, \|e_y\| 0.424) |
| **5.5** | **50** | **DNF / offtrack** |

Each change is survivable alone; the combination is not, offline only. The
plausible mechanism is that a heavily rate-damped controller cannot turn the
wheel fast enough for corners entered ~14% quicker — which is the SAME
physical story as the open shallow-corner turn-in problem below, just
expressed as an outright failure because the offline plant is less forgiving.

**Consequence for future work: `nmpc_offline_check`'s closed-loop pass is
currently a FAILING gate and cannot be used as a regression check as-is.**
Do not read a fresh DNF there as "my change broke something" without first
confirming it also DNFs on an unmodified tree. Either fix the underlying
turn-in problem (which may clear it), or re-baseline the check — do not
simply relax the assertion.

## TRAP: enabling `nmpc_corner_rrate_blend_enabled` silently discards `r_rate_delta`

Live-tested and it broke the controller. The blend does NOT scale
`r_rate_delta` — it **overwrites** `R_rate[0,0]` outright:

```python
R_rate[0,0] = _blend(rrate_steer_straight, rrate_steer_corner, corner_frac)
```

With `nmpc_rrate_steer_straight`/`_corner` left at their `-1.0` ("inherit")
defaults they resolve to the **LTV-QP's** `rrate_steer_straight=2.0` /
`rrate_steer_corner=1.25` — values never tuned for the NMPC. Measured
result: the controller ran at `Rrate_steer` 1.53-2.00 (mean 1.78) instead of
52.5, a ~30x cut, giving **20.95% steering saturation**, `|e_y|` **1.18 m**,
`|e_psi|` **12.1°**, chatter 3.51°/tick — worse than the `r_rate_delta=0.0`
control run on several axes.

**If re-enabling, set BOTH endpoints explicitly**, scaled to the current
`r_rate_delta` (e.g. 52.5 straight / ~25 corner). Never leave them at `-1`.
The blend is still a promising shape for the open problem below — heavy
damping on straights, softer in corners — but only with explicit endpoints.

## OPEN: late/jerky turn-in on shallow corners

Reported after the `r_rate_delta=52.5` fix shipped. The high rate gain
suppresses hunting, but on **shallow** corners — where `e_y`/`e_psi` stay
small enough not to overcome the rate cost — the controller does not want to
turn. It holds smooth, then jerks once the NMPC's predicted errors grow
enough to overpower the damping, then resumes smooth tracking. So the fix
traded high-frequency hunting for a low-frequency turn-in discontinuity.

This is the direct consequence of a FLAT rate cost: one weight cannot be
simultaneously stiff enough to kill straight-line hunting and compliant
enough for a gentle corner's small, early steering input. See
`docs/steering_turn_in_upgrade_options.md` for the full option analysis.

### Why this was missed for a whole investigation

The `r_rate_delta` sweep below only went to **4x** (2.5 → 10) and showed a
small effect, which was then written up as "not the dominant lever." That
inference was wrong: it was evidence about the *magnitude tried*, not about
the *mechanism*. The fix needed ~18x. Compounding it, the sweep was run
OFFLINE, where chatter is a different regime (std_d ~5.7° vs live ~3.1°),
and the offline flatness was allowed to stand in for a live answer.

**Lesson for future work in this file: before concluding a knob is not the
lever, push it until something breaks, not just until the first sweep looks
flat — and confirm on the live car when the symptom was reported live.**

## Superseded: output_smoothing

`OUTPUT_SMOOTHING_ENABLED` is now **false** in `launch_all.sh`. Raising
`r_rate_delta` addresses the same jitter at its source — the solver stops
*choosing* chattery commands — so it adds no lag and needs none of the
output filter's curvature/error/lookahead fade machinery to avoid slowing
the response. The filter's whole design existed to buy smoothness back
without lag; a correctly-weighted rate cost gets that for free. Re-enable
only if a rate-cost-only setup turns out to need extra output damping for a
reason that is understood.

## Also reverted: the curvature-relative rate target

`NMPC_CURV_RATE_REF_ENABLED` / `_du_ref` / `PathReference.dkappa_at` were
built during this investigation, live-tested, found ineffective, and have
been **fully reverted** from both repos (offline commit reverted; live and
mirror files restored). Kept here as a finding, not as code:

- Mechanism: retarget the rate cost off zero onto the path-justified rate
  `du_ref ≈ gain·L·(dκ/ds)·ṡ·dt`, so a constant-radius corner still
  penalises wiggle but a corner entry/S-bend is not penalised for the
  steering change it needs.
- **Why it failed: the path-justified rate is ~5-10x too small to matter.**
  Measured `du_ref` = 0.48°/step mean (live), 0.44°/step (offline), against
  ~2.5°/step of actual chatter. Retargeting by that much shifts the rate
  cost only ~±20%.
- An S-bend falsification test PASSED (|e_y| 0.0496 → 0.0497), so the idea
  was sound and correctly permissive — just irrelevant at real magnitudes.
- **The durable finding:** the chatter is almost entirely steering motion
  the path never asked for, and the existing `r_rate` cost was already
  penalising it correctly — it was just weighted ~18x too weakly. This
  retires the "the cost wrongly punishes necessary steering" hypothesis
  that motivated the feature.
- Implementation note worth remembering if anything similar is attempted:
  `e_rate` in `_solve_step` feeds BOTH the rate-cost gradient AND the
  slew-rate CONSTRAINT bounds. Retargeting must use a separate variable;
  shifting `e_rate` itself lets the solver command a physically impossible
  slew.

---

*Everything below predates the resolution above.* This doc exists so a
future session can resume without re-deriving what was already ruled out.

## Symptom (as reported by the user, 2026-08-20)

While turning, the car does not hold a smoothly-varying steering angle.
Instead: steers slightly harder than needed, relaxes/drives straight a bit,
steers again, drives forward a bit, repeat — rather than one continuous,
gradually-changing angle through the corner.

## What has been confirmed

### It is a real, measurable tick-to-tick oscillation, not user perception

Live log `mpc_standalone_control_1787454493.csv` (NMPC, `use_nmpc=1`,
`output_smoothing_enabled=True`, `alpha=0.425`, `k_ey=0.8`, `k_epsi=1.115`),
window t=22.6-23.7s (clean cornering, no teleport/startup artifact in this
window — see below): `steer_deg` bounces by 5-10 deg tick-to-tick while
`e_y`/`e_psi` themselves change smoothly and monotonically. This is a
genuine magnitude-chatter pattern (same sign throughout this window, so NOT
the sign-flip "reversal" pattern `reversal_penalty_boost` targets) —
different symptom, different mechanism, do not conflate with the
reversal-penalty feature.

Quantified over the whole live run (t>5s, past the startup/teleport-affected
region): tick-to-tick steer std = 3.15 deg, sign-flip rate = 66.1%
(n=1692 ticks).

### NOT the standstill/e_v-coupling bug from late_turn_in_investigation.md §16.12

That bug is specific to near-zero speed with near-zero e_y/e_psi. This
chatter happens at v=6-11 m/s with real, substantial tracking error
(e_y up to ~0.35m in the clean window checked) — a different regime
entirely.

### NOT a solver convergence failure

`nmpc_status`=1 (solved) and `nmpc_iters`=1 on every tick in the checked
window — the SQP is converging cleanly every single tick, not hitting
max-iterations or getting rejected. Whatever is oscillating, it is not a
symptom of the solver failing to converge.

### NOT the live planner / centreline-curvature-spike defect

This run used a STATIC precomputed path (`path_map_path=.../raceline.csv`,
`path_age_s=0.0000` throughout — confirmed no live planner in the loop).
`nmpc_kappa_horizon_end` is smooth and monotonic in the checked window. The
known open planner defect (`planning_control_sync.md`'s "Known planner defect: centreline curvature spikes") cannot
be the cause here since there is no live planner path being consumed.

### NOT output_smoothing amplifying/causing it

Initially suspected (the retuned `alpha`/`k_ey`/`k_epsi` are recent), but
ruled out: `delta_cmd` and `steer_deg` in the CSV are BOTH logged from the
same `steer_rad` variable, which is captured in
`mpc_controller_standalone.py` AFTER output_smoothing has already been
applied (`cmd.steering, cmd.throttle, cmd.brake = steering, ...` happens
before `steer_rad = -float(cmd.steering) * MAX_STEER_RAD`). **There is no
pre-smoothing raw value in the CSV at all** — an earlier draft of this
investigation incorrectly concluded "shipped == raw" from this; that
was wrong, both columns are post-smoothing. This does NOT by itself rule
smoothing in or out — it just means the CSV cannot show the raw solve
directly. See "Not yet tried" below for how to actually get the raw value.

### An initial "refresh-rate mismatch" hypothesis was raised, then found unsupported

`yaw_rate` and `pose_age_s` were both observed alternating between two
bands tick-to-tick, which was initially (wrongly) asserted as evidence of a
pose/odometry sampling-rate aliasing artifact against the 20 Hz control
loop, WITHOUT actually checking correlation direction. When challenged and
actually checked:

- Tick period itself jitters (0.0365-0.0597s observed in one 1s window),
  inconsistent with a clean deterministic two-rate beat pattern, which
  needs both source rates to be fairly stable.
- Cross-correlating `|steer[i]|` vs `|yaw_rate[i]|` (same tick, n=22 sample
  window t=22.6-23.7s): **r = -0.49** (negative).
- Cross-correlating `|steer[i-1]|` (previous tick) vs `|yaw_rate[i]|`
  (current tick): **r = +0.32** (positive).

Negative same-tick + positive one-tick-lagged correlation is the signature
of a genuine closed-loop ringing (large yaw rate now -> controller backs
off steering -> smaller yaw rate next tick -> controller steers harder
again), not a sensor-timing artifact. **The refresh-rate-mismatch
explanation is not supported by this check and should not be repeated
without new evidence.** `sim_perception.py`'s `_car_yaw_rate` is a straight
passthrough of the upstream bridge odom's `angular.z` — no differentiation
or filtering happens in that file, so if there IS a sensor-side artifact it
would have to originate further upstream (`fsds_ros2_bridge` or FSDS
itself), not in this repo's perception code. Not yet checked.

### The oscillation reproduces OFFLINE too, at BOTH old and new q_r

This is the most important finding so far: the same character of
oscillation is NOT live-only. Reproduced via
`sim/rollout_core.py`'s `run_core_rollout` on `comp_test_map_3`
(`USE_NMPC=True`, static path, no SLAM noise, no pose-hold, no delay
jitter — the cleanest possible test), at `q_r` (offline: `Q_diag[3]`) =
0.70 (the OLD value, pre-retune) and 1.20 (the NEW, current value):

| q_r | steer d(tick) std (deg) | 2nd-diff std (deg) | sign-flip rate | \|e_y\| mean | score |
|---|---|---|---|---|---|
| 0.70 (old) | 5.88 | 7.65 | 47.2% | 0.287 | 0.6047 |
| 1.20 (new) | 5.69 | 7.69 | 48.8% | 0.295 | 0.6032 |

**Conclusion: the q_r retune (0.70->1.20, done 2026-08-20 as part of
mirroring a live-tuned value) is NOT the cause.** The two are statistically
indistinguishable. This also means the oscillation almost certainly
PRE-DATES that retune and was simply never measured this way before.

Live vs offline character differs (live: smaller per-tick magnitude
(3.15 deg) but MORE frequent sign flips (66.1%); offline: larger magnitude
(~5.7 deg) but somewhat fewer flips (~48%)). This suggests a live-only
contributor stacks on top of a baseline oscillation the NMPC already has
offline, but the live-only piece has not been identified.

### r_rate_delta (steering RATE cost) has only a small effect, even at 4x

Same offline rollout, sweeping `R_rate_diag[0]` (offline `r_rate_delta`)
multiplier at the CURRENT `q_r=1.20`:

| r_rate_delta | steer d(tick) std | sign-flip rate | \|e_y\| mean | score |
|---|---|---|---|---|
| 2.5 (x1, current) | 5.69 | 48.8% | 0.295 | 0.6032 |
| 5.0 (x2) | 5.69 | 49.1% | 0.299 | 0.6183 |
| 10.0 (x4) | 5.09 | 42.4% | 0.307 | 0.6065 |

Even a 4x increase only takes the sign-flip rate from 48.8% to 42.4%, with
a small `e_y` cost increase. Not the dominant lever. Not yet tried below
4x-to-10x range or above it (diminishing returns already visible, unlikely
a much larger multiplier is the answer, but not conclusively ruled out).

## Reproduction script

**Committed**: `tuner/steering_chatter_check.py` (run with
`python -m tuner.steering_chatter_check [--controller nmpc|ltv]
[--set NAME=VALUE ...]`) — supersedes the session-local, uncommitted
scratchpad scripts used to gather every number in this doc (those lived in
a temporary directory and were not retained). Use this script for any future
sweep in this investigation; extend it rather than writing a new one-off
script, so the harness boilerplate does not have to be re-derived. It already prints
a warning if any tick had a non-`'solved'` `nmpc_status`, specifically to
catch the horizon=35-style trap (a low chatter number that's actually a
frozen/failing solver, not a real fix) before it's misread again.

Note while using it: running it against the SHIPPED defaults (no `--set`
overrides) already shows 8/1044 ticks with non-`'solved'` status — 6 of
them (ticks 10-15) are clustered at the very start of the rollout with
`e_y` near-zero (consistent with the standstill/e_v-coupling regime from
`late_turn_in_investigation.md` §16.12, a KNOWN, separate, partially-fixed
issue — not this doc's cornering-chatter symptom), and 2 are isolated
mid-run failures (ticks 112, 156) not yet explained. This means the
"baseline" comparison point itself is not perfectly clean; keep this in
mind when comparing a swept value's non-'solved' count against the
baseline's 8, rather than assuming the baseline is entirely status='solved'.

## Session 2 (continued investigation, same day)

### NMPC-specific: LTV-QP does NOT show this chatter under matched conditions

Direct A/B, same track/weights/plant, `run_core_rollout(..., use_nmpc=False)`
vs `use_nmpc=True`:

| controller | steer d(tick) std (deg) | sign-flip rate | score |
|---|---|---|---|
| LTV-QP | 0.83 | 28.7% | 0.4362 |
| NMPC | 5.69 | 48.8% | 0.6032 |

**NMPC is ~7x noisier than the LTV-QP under otherwise-identical conditions.**
This strongly narrows the search to something in the NMPC's own SQP
mechanics (warm-start, trust region, horizon/terminal cost structure),
not the shared plant model or path geometry (both controllers see the
same plant, same path, same Q/R base values).

### SQP iteration count: ruled out (no meaningful effect)

`nmpc_sqp_iters` swept 1/2/3 (5 timed out — too slow for a 2-minute budget
at this horizon length, not attempted at a longer budget yet): std_d
5.69/5.58/5.66 deg, sign-flip 48.8%/47.0%/48.0%. No trend. Running more
Gauss-Newton iterations per tick does not converge the chatter away, which
argues against "one iteration just isn't enough" as the mechanism.

### Trust region size (`nmpc_trust_delta_rad`): weak effect, real accuracy cost

Swept 9/4/2 deg: std_d 5.69/5.44/4.95 deg, sign-flip 48.8%/44.3%/40.8%,
but `|e_y|` mean WORSENS 0.295/0.344/0.384 and score worsens too. Shrinking
the trust region trades tracking accuracy for less chatter, doesn't remove
it (even at 2 deg — a very tight cap — still ~6x the LTV-QP's std). Not a
clean fix; not fully ruled out as A contributing factor, but not the
dominant one and comes with a real cost.

### r_delta (steering effort, distinct from r_rate_delta): weak effect

Swept 1x/3x/8x (base 1.8): std_d 5.69/5.46/5.07 deg, sign-flip roughly flat
(48.8/48.2/48.3%), `|e_y|` mean creeps up at 8x (0.295->0.340). Same pattern
as every other weight tested so far: modest reduction, real accuracy cost,
does not get anywhere near the LTV-QP's baseline noise level.

### Finite-difference Jacobian substeps: NO effect at all

`nmpc_jac_substeps` swept 1/2/4: std_d 5.69/5.80/5.69 — pure noise, no
trend whatsoever. Rules out FD-Jacobian coarseness at the current epsilon
values as a contributor.

### Warm-start is ALREADY noisy before the SQP step runs

Instrumented `compute_step` to record `U[0]` (the shifted warm-start
value, before that tick's Gauss-Newton step) alongside the final shipped
`u_opt[0]`:

- Warm-start U[0] tick-to-tick std = **6.38 deg** (higher than the final
  output's 5.69 deg -- the single SQP step is net DAMPING the warm start's
  own noise slightly, not amplifying it).
- Per-tick correction (final - warm): mean|correction| = 3.08 deg, with a
  51.6% sign-flip rate on the CORRECTION ITSELF (i.e., whether the SQP step
  pushes steering further from or back toward the warm start is close to a
  coin flip tick to tick).
- Sample window showed corrections as large as +9.0 deg in the SAME
  direction as an already-large warm-start jump (idx 408-409: warm swings
  -16.66->-11.03, SQP correction ADDS another +7.58 then +9.00 deg on top,
  landing at -9.08 then -2.03) -- i.e. sometimes the SQP step amplifies an
  already-noisy warm start rather than correcting it.

This means "last tick's optimal trajectory, shifted by one step" is ALREADY
not a smooth, stable prediction of what next tick wants -- the noise
substantially predates the current tick's own solve.

### Reference construction: definitively ruled out

`NMPCController.path_reference()` caches the built `PathReference` (spline
fit, `kappa(s)`/`psi_ref(s)` arrays) keyed on a signature
`(len(path), path[0], path[-1])`. With `use_planner=False` in every A/B run
above, `path` is a literal fixed array for the whole rollout, so the
signature never changes and the SAME `PathReference` object -- same spline,
built ONCE -- is reused for all ~1044 ticks. There is categorically no
tick-to-tick reference-rebuild noise possible in any of the offline repros
above. (This does NOT rule out reference noise as a LIVE-only contributor
when `use_planner=True`/a real live planner path is in the loop -- only
rules it out for these specific offline A/Bs.)

### Frenet projection (`base_idx`/`s0`): clean, not the cause

Instrumented `PathReference.project()`: called exactly once per tick
(1:1 with rollout ticks, no hidden extra calls). `base_idx` NEVER goes
backward (0/1043 ticks), `s0` NEVER decreases (0/1043 ticks). `base_idx`
does sometimes jump by >1 waypoint per tick (147/1043, expected at higher
speed relative to the ~0.465m waypoint spacing on this track). No
pathological flip-flopping in the nearest-waypoint projection.

### Underlying yaw-rate response IS smooth offline (unlike live)

In a 40-tick clean window (idx 400-440, no sensor noise, no SLAM/pose-hold/
delay-jitter enabled -- the offline rollout used for all A/Bs above), the
plant's own yaw rate `r` changes smoothly tick to tick (e.g. -0.611,
-0.638, -0.653, -0.632, -0.598, -0.641, -0.871, -1.148, -1.335, -1.352,
-1.215, -0.874, -0.444 ...) -- NO alternating-band pattern like the live
log showed. But `steer` in that SAME window still swings wildly (-8.0,
-7.9, -7.2, -14.5, -18.6, -20.8, -20.5, -17.5, -9.1, -2.0 ...).

**This is an important distinction from Session 1's live-only
same-tick/lag-1 correlation finding.** Same-tick correlation here
(offline, this window) is near zero (-0.034, not -0.49 like live);
lag-1 correlation is positive (+0.338, same SIGN as live's +0.32, weaker
magnitude). The offline chatter is NOT primarily "the car's real yaw
dynamics ringing against the controller's reaction to it" the way Session
1 hypothesized for live -- offline, the plant response is smooth while the
CONTROLLER'S OWN CHOSEN STEERING is not. This points at the solver
choosing an unstable steering sequence for a smoothly-evolving state,
which is a genuinely different (though not necessarily exclusive)
mechanism from Session 1's live closed-loop-ringing hypothesis. Both may
be true simultaneously (a live-only contributor stacked on an
offline-reproducible baseline) -- this is consistent with Session 1's own
observation that live sign-flip rate (66%) exceeds offline's (~48%).

### Cost value itself swings non-monotonically alongside steer

`nmpc_cost` (the true nonlinear cost, from `history['nmpc_cost']`) in the
same idx 400-419 window does NOT move smoothly: 6.71, 7.11, 8.16, 7.52,
9.97, 10.78, 14.41, 17.85, 11.47, 13.25, 15.67, 18.02, 18.85, 18.32, 16.65,
15.01, 14.61, 14.58, 15.82, 16.56 -- rises for several ticks, then drops
sharply (17.85 -> 11.47 at idx 407->408) at EXACTLY the tick where steer
relaxes sharply (-17.49 -> -9.08 deg). The solver is doing its job
(finding a genuinely lower-cost point each tick), but the fact that a much
smaller steering angle is suddenly much cheaper suggests the cost
landscape itself may be poorly conditioned near the optimum for this
horizon/terminal-cost setup -- e.g. a shallow or multi-modal region where
several quite different steering choices score similarly, so the
warm-started SQP settles wherever the (noisy) warm start happens to nudge
it, rather than there being one clearly-best answer the solver reliably
converges toward tick after tick.

`history['pred_X']`/`pred_Y'` (the horizon's own predicted trajectory,
which would have been the most direct way to SEE whether the solver's
"plan" itself swings tick to tick) turned out to be empty/unpopulated for
NMPC runs (`want_horizon_pred=True` did not produce predictions -- this
looks like an LTV-QP-only feature in `rollout_core.py`, not implemented
for the NMPC branch). Confirming/adding NMPC horizon-prediction history
would be a useful follow-up tool, not yet done.

## Session 2, continued: terminal_scale and horizon length

### terminal_scale: NO effect

Swept 1.0/3.0/8.0/20.0 (base 1.0): std_d 5.69/5.70/5.78/5.93 deg -- flat to
very slightly worse. Ruled out.

### NMPC_HORIZON: no clean effect within the solve-time budget; a longer-
### horizon "improvement" at 35 steps is a BUDGET-TRUNCATION ARTIFACT, not
### a real fix

Swept 10/20(base)/22/25/28/35:

| NMPC_HORIZON | std_d (deg) | sign_flip | \|e_y\| mean | score | dnf |
|---|---|---|---|---|---|
| 10 | 5.61 | 50.3% | 0.448 | 0.813 | No |
| 20 (current) | 5.69 | 48.8% | 0.295 | 0.603 | No |
| 22 | 5.22 | 40.8% | 0.259 | 0.621 | No |
| 25 | 5.78 | 50.3% | 0.284 | 0.611 | No |
| 28 | 5.47 | 45.3% | 0.276 | 0.638 | No |
| 35 | **3.68** | **29.4%** | 0.414 | 15.10 | **YES** |

The horizon=35 result LOOKS like a strong fix (chatter approaching the
LTV-QP's own baseline) but the run DNFs off-track at step 279. **Initially
misdiagnosed as a solve-time-budget truncation artifact** (`nmpc_solve_ms`
mean=28.2ms at that horizon, vs `NMPC_SOLVE_BUDGET_MS=25.0`) -- this
hypothesis was then DIRECTLY TESTED and DISPROVEN: raising
`NMPC_SOLVE_BUDGET_MS` to 50.0 alongside `NMPC_HORIZON=35` (confirmed via a
standalone import check that the bare-name `from settings import
NMPC_SOLVE_BUDGET_MS` binding in `rollout_core.py` actually picked up the
new value, same gotcha as `USE_NMPC`) reproduced the IDENTICAL failure at
the IDENTICAL tick (279, `|e_y|`=2.38m), even though `solve_ms` max (46.1ms)
was now comfortably under the new 50ms budget.

**The real cause, confirmed by checking `nmpc_status` on the failing
ticks: the SQP itself starts failing to find an improving step.** The last
~15 ticks before DNF show `status` alternating between `'rejected'` and
`'maximum iterations reached'` on EVERY tick (not a budget abort) --
`_solve_step`'s backtracking line search cannot find a step that improves
cost, so `U` (and hence the shipped steering command) freezes at ~5.7 deg
while `|e_y|` climbs monotonically and unboundedly (0.848 -> 2.376m over
15 ticks) with steering barely moving (5.692 -> 5.751 deg, then finally
collapsing to 2.17 deg on the very last tick as the situation is fully
lost). This is a genuine SQP-infeasibility/numerical-conditioning failure
at horizon=35 -- NOT a time-budget issue, and NOT a real "less chatter"
result: the apparent low chatter at horizon=35 is because the solver is
STUCK, not because it found a smooth policy. A frozen, non-responding
controller trivially scores low on a tick-to-tick std metric while
actively failing.

**Do not read horizon=35's low chatter numbers as "increase the horizon to
fix this" under ANY circumstance, budget-adjusted or not -- confirmed
invalid twice now.** The 22-28 range (within budget, no rejected/max-iter
statuses observed) shows no consistent chatter trend either (5.22, 5.78,
5.47 bouncing around the baseline 5.69 with no monotonic pattern), which
argues against horizon length being the lever in the range where the
solver actually still works. Root-causing WHY the SQP starts rejecting
every step at horizon=35 (track-halfwidth slack constraint interaction?
OSQP numerical conditioning at that problem size? trust region too small
relative to the longer horizon's own dynamics range?) has not been done
and might be worth a separate, narrower investigation on its own merits
(it is a real robustness gap regardless of whether it relates to the
chatter symptom), but is NOT the chatter fix.

## Separate issue found while investigating (not the chatter cause, flag for its own investigation)

At `NMPC_HORIZON=35` (nearly double the shipped `NMPC_HORIZON=20`), the SQP
starts REJECTING every candidate step (`status` = `'rejected'` /
`'maximum iterations reached'` on essentially every tick once it starts)
partway through a lap on `comp_test_map_3`, freezing steering and letting
`|e_y|` grow unboundedly to a DNF. This reproduces regardless of
`NMPC_SOLVE_BUDGET_MS` (tested at both 25ms, the shipped default, and 50ms
-- identical failure tick and `|e_y|` trajectory both times), so it is a
genuine SQP-infeasibility/conditioning problem at that horizon length, not
a time-budget symptom. This is a real robustness gap worth its own
investigation independent of the chatter symptom (a horizon that becomes
infeasible partway through a normal lap is a latent DNF risk if anyone
retunes `NMPC_HORIZON` upward for other reasons) -- not investigated
further here since it is not the chatter cause and this doc's scope is the
chatter symptom.

## Not yet tried (updated after Session 2)

- **Get the actual raw (pre-output_smoothing) steering value from a live
  run.** The CSV cannot show it (see above). Either: (a) temporarily add a
  raw-value telemetry column before smoothing is applied, live-test, then
  revert; or (b) set `OUTPUT_SMOOTHING_ENABLED=false` in `launch_all.sh` for
  one test run and compare chatter with it on vs off, live. (b) is simpler
  and doesn't require a code change — do this first. Still not done.
- **Terminal cost / horizon length interaction.** Session 2's strongest
  remaining lead: `nmpc_cost` itself swings non-monotonically tick to tick
  (see "Cost value itself swings non-monotonically" above) even though the
  reference is provably static and the plant's own yaw-rate response is
  smooth in the same window. This suggests the cost landscape near the
  optimum may be shallow/poorly-conditioned for the current
  `terminal_scale`/horizon-length (`N_HORIZON`) combination, such that
  several distinct steering sequences score similarly and the (noisy)
  warm start decides which one the single-iteration SQP lands near each
  tick. NOT YET TESTED: sweep `NMPC_TERMINAL_SCALE`
  (`settings.TERMINAL_Q_SCALE`, mirrors `nmpc_terminal_scale`) and
  `N_HORIZON` itself (currently 20 steps = 1.0s) the same way the other
  weights were swept above, using the same std_d/sign-flip metric.
- **Add NMPC horizon-prediction history to `rollout_core.py`.** Attempted
  via `want_horizon_pred=True`, but `history['pred_X']`/`pred_Y'` came back
  empty for NMPC runs -- this flag appears to only populate history for the
  LTV-QP branch currently. Wiring the NMPC's own predicted `X[:, IDX_S]`
  trajectory (or global X/Y via `ref`) into the same history keys would let
  a future session directly VISUALIZE whether the solver's "plan" swings
  tick to tick (the most direct possible evidence for/against the
  cost-landscape hypothesis above), rather than inferring it indirectly
  from cost values and warm-start deltas the way Session 2 did. Not
  attempted -- would need a `nmpc_core.py`/`nmpc_optimiser.py` change plus
  a `rollout_core.py` wiring change; keep both sides in sync per `planning_control_sync.md`.
- **Whether the warm-start's own noise (std 6.38 deg, found in Session 2)
  reproduces even with a MUCH larger trust region + more SQP iterations
  together** (as opposed to each swept independently, as Session 2 did).
  E.g. `nmpc_sqp_iters=4` AND `nmpc_trust_delta_rad=20deg` at once, to see
  if the SQP can fully re-converge from scratch each tick (approximating a
  non-RTI, full-reoptimization solve) and whether THAT removes the warm-
  start noise. This is a more expensive/slower test (both parameters
  raised together multiply the per-tick cost) — budget accordingly, may
  need a longer per-process timeout than the 60-120s used in Session 2.
- The live-only "extra" chatter contributor (live 66.1% sign-flip vs
  offline ~48%, and live's negative same-tick / offline's near-zero
  same-tick correlation — see Session 2's "Underlying yaw-rate response"
  finding, these are NOT the same mechanism) is still completely
  unexplained. Candidates not yet checked: SLAM/pose noise model
  differences from anything offline uses, actual FSDS physics vs the
  offline Pacejka-model plant, or the `pose_age`/`n_delay`
  delay-compensation rollforward interacting badly with a genuinely-jittery
  live pose timestamp (the delay-compensation ROLLFORWARD MECHANISM
  itself, as opposed to a sensor refresh-rate mismatch, has not actually
  been ruled out and may be worth a second look with a more careful test).

## Ruled out, do not re-test without new evidence

- **q_r / Q_diag[3] value (0.70 vs 1.20)**: statistically indistinguishable,
  see table above.
- **Sensor/pose refresh-rate aliasing as a standalone explanation** (Session
  1): the same-tick vs lag-1 correlation signs on the LIVE log point at
  genuine closed-loop ringing instead. (The delay-compensation ROLLFORWARD
  mechanism is a distinct hypothesis from this and is NOT ruled out -- see
  "Not yet tried" above.)
- **output_smoothing as the SOLE cause**: `settings.OUTPUT_SMOOTHING_ENABLED`
  defaults to `False` and none of Session 2's A/B scripts touched it, so
  every offline rollout above was genuinely smoothing-free -- the chatter
  reproduces with output_smoothing completely out of the loop. Does NOT
  rule it out as a live-only AMPLIFIER of a baseline chatter that already
  exists without it.
- **Reference-path/spline construction noise**: definitively ruled out for
  `use_planner=False` offline runs -- the `PathReference` is built exactly
  ONCE per rollout (signature-cached, never rebuilt when the path array is
  unchanged) and reused for all ~1044 ticks. Categorically cannot be the
  source of tick-to-tick noise in any offline repro used above. (Not ruled
  out live, where `use_planner=True` WOULD rebuild it every tick from a
  possibly-noisy live planner path -- but the live log analyzed in Session
  1 also used a static precomputed path, `path_age_s=0.0000` throughout, so
  this is not yet implicated live either given the data gathered so far.)
- **Frenet projection instability (`base_idx` flip-flop)**: ruled out --
  `base_idx` never regresses, `s0` never decreases, across 1043 checked
  tick-to-tick transitions.
- **SQP iteration count (1/2/3)**: no meaningful effect on chatter metrics.
- **Finite-difference Jacobian substeps (1/2/4)**: no effect at all (pure
  noise across the sweep).
- ~~**Any single cost weight in isolation**~~ — **THIS ENTRY WAS WRONG. See
  the Resolution section at the top of this file.** It originally read: "each
  produces only a modest reduction... do not expect a single-weight retune to
  be a clean fix." `r_rate_delta` IS the fix; the sweep behind this claim only
  went to 4x (2.5 → 10) and offline, where the chatter is a different regime.
  At ~18x (2.8 → 50), live, chatter more than halves. The other weights in
  the original sweep (`q_r`, `r_delta` to 8x, `nmpc_trust_delta_rad` to 2deg,
  `nmpc_terminal_scale` to 20x) are still fairly regarded as non-levers at
  the ranges tried, but the blanket generalisation drawn from them was not
  supported.
- **`NMPC_HORIZON` in the budget-feasible range (10-28 steps)**: no
  consistent trend (values bounce 5.22-5.78 deg std_d with no monotonic
  relationship to horizon length). The apparent improvement at
  `NMPC_HORIZON=35` is INVALID -- confirmed (twice, at two different solve
  budgets) to be the SQP failing/freezing, not a real fix; see "Separate
  issue found while investigating" above. Do not retest horizon=35 (or
  presumably anything near/above it) as a chatter fix without first fixing
  the underlying SQP-rejection problem at that horizon length, which is
  unsolved and out of this doc's scope.

---

## The reference line was a confound for every chatter measurement

**A large part of what this document measures as "chatter" was the car
failing to track a reference it could not follow — not the cost weights.**
Switching `path_map_path` from `raceline.csv` to a newly-added
`centerline.csv` (`raceline_optimizer.py --mode centerline`), with **every
weight held fixed**, produced:

| | raceline | centreline |
|---|---|---|
| composite score | 0.752 | **0.488** |
| lap time | 54.50 s | **51.34 s** |
| RMSE | 0.506 | **0.366 m** |
| peak \|e_y\| | 1.804 | **1.004 m** |
| max \|e_psi\| | 37.6° | **22.0°** |
| steering saturation | 0.18% | **0.00%** |
| slew-limited ticks | 0.07% | **0.00%** |
| steering reversals (scoring) | 13 | **1** |
| \|e_y\| > 1.0 m | 4.00% | **0.14%** |

Both runs: NMPC, `r_rate_delta=52.5`, `nmpc_rjerk_delta=150.0`,
`q_e_y=6.35`, smoothing off, zone off, same `speed_profile.csv`, 3 laps.

Mechanism is documented in `docs/reference_path_and_speed.md`'s "Reference line:
raceline vs centreline": the raceline's offset is small (0.13 m mean, 0.48 m
max) but it perturbs the curvature of the track's tightest corner
(`|κ|`=0.209, ~70% of the full-lock kinematic floor) where there is no width
to gain time with. The car under-turns, washes 1.8 m wide, and recovers —
and the recovery is what the chatter metrics were partly counting.

### What this invalidates, and what it does not

- **Does not invalidate the `r_rate_delta` resolution above.** That A/B was
  run on the raceline for all three arms, so the comparison is internally
  consistent; `r_rate_delta` really is a strong lever.
- **Does cast doubt on the absolute magnitudes** in every table in this
  document. A chatter figure measured on the raceline includes an
  excursion-recovery component that has nothing to do with the weight under
  test. Do not compare a number measured before this date against one
  measured after without checking which reference each used.
- **Explains why `flip%` was so stubborn.** It sat near 55-66% through every
  intervention. On the centreline the scoring reversal count is 1 (from 13)
  and slew-limited ticks are exactly 0 — a floor that no weight retune
  reached on the raceline.

### Corrected reading of the `rjerk=150` live result

A live run of `nmpc_rjerk_delta=150.0` on the raceline shows 4.49% steering
saturation, which reads as the jerk penalty trading smoothness for saturation.
**That reading is wrong** — the same weight on
the centreline saturates 0.00% of ticks. The saturation belonged to the
reference. (That run also contains a separate clock anomaly:
timestamps went backwards 0.399 s twice, with a 6.29 → 2.71 m/s speed step —
see `periodic_pose_teleport_investigation.md`.)

### Ambiguous lateral error is why centreline mode exists

A lateral error measured against a raceline is ambiguous, because a large lateral error can be either a
tracking failure or the line intentionally apexing near a boundary. The
centreline mode exists specifically to remove that ambiguity, and it should
be the reference for any future chatter or turn-in measurement; switch back
to the raceline only for timed runs.

### Zone schedule, first live run: it never engaged

`NMPC_RRATE_ZONE_ENABLED=true` (2.0 / 0.35 / 0.15) on the centreline
baseline, `k` left at its inherited `-1` (= LTV-QP `8.0`):

| | zone off | zone on |
|---|---|---|
| composite score | **0.488** | 0.522 |
| lap time | **51.34** | 52.40 s |
| \|e_y\| p90 | 0.581 | **0.524** |
| \|e_y\| max | **1.004** | 1.170 |
| flip% | 42.1 | **39.0** |
| mean\|d\| | **0.628** | 0.674 |
| sat% / slew% | 0.00 / 0.00 | 0.00 / 0.00 |

A wash — **because the mechanism never actually ran.** `m_Rrate_zone` spanned
0.829–1.962 with **0% of ticks in either the ease or the floor band**. What
executed was a mild global rate *boost* on straights, not a three-zone
schedule.

Cause is a calibration coupling, not a design fault:
`_corner_factor(|κ|, k=8)` needs `|κ|`=1.125 to reach 0.9, but this track's
tightest corner is 0.209, so `corner_frac` tops out at 0.626 and the
multiplier bottoms at 0.84 instead of its 0.15 floor. Full derivation and the
`k ≈ target/((1−target)·κ_max)` rule are in `planning_control_sync.md`'s
"Three-zone rate schedule".

`nmpc_corner_factor_k=27.0` (0.209 → `corner_frac` 0.85) fixes the
saturation, so **the zone's endpoints have still not been tested**; do not
record the table above as evidence about 2.0/0.35/0.15.

**But `k=27` with `ease_approach`=0.35 DNFs offline**, off-track at the
track's tightest corner with full-lock steering and `|e_y|` growing to
2.53 m — a late-turn-in failure, the opposite of the intended effect. It is
not a monotonic tuning effect: `boost_straight`=0.8, which makes the
multiplier ≤1 everywhere (uniformly *weaker* than no zone), also DNFs.
Compounding is ruled out (`_Rr_flat` is rebuilt each tick — `rrate_zone_enabled`
is in the rebuild guard). Full elimination list in
`docs/tuning.md`'s "Three-zone rate schedule"; the cause is **open**,
with a not-yet-confirmed suspicion about the zone being applied after the
rollout (it needs horizon curvature) interacting with the warm start.

Shipped instead: `ease_approach`=**0.80**, the value at which the offline
rollout completes and modestly beats baseline (score 0.796 vs 0.822,
`|e_y|` 0.476 vs 0.496, p90 0.976 vs 1.044).

Ordering matters here: `k=27` was derived from the
saturation arithmetic and set as the live config *before* running
`tuner.steering_chatter_check`. The check then showed it DNFs. Run the
offline rollout before shipping a weight-shaping change, not after.

A null result in this area more often means "the mechanism did not engage"
than "the mechanism does not help". Twice now a null has turned out to be "the mechanism did not engage" rather than "the mechanism
does not help" (the first was `NMPC_HORIZON=35`, which was SQP failure).
Check the mechanism's own telemetry column before writing a null into the
ruled-out list.

### Next

Re-run the zone at `k=27` / `ease_approach`=0.80, against the centreline
baseline (score 0.488).
It composes multiplicatively with `nmpc_rjerk_delta=150.0`, which is already
on, so a regression could be either — disable the zone first.

Still untested live: the offline low-rate variant `r_rate_delta=5.0` +
`nmpc_rjerk_delta=250.0`, which beats the flat-52.5 baseline on every
offline metric.

---

## Late turn-in and "won't turn enough": six causes ruled out

**Symptom:** turns in slightly late; carries lateral error
through the corner before correcting back; sticks close to the boundary;
steering command occasionally jumps at a tight corner. Initially attributed to the high
steering-rate weight.

**Scope — planner mechanisms are excluded by construction.** The reference is
a precomputed path (`centerline.csv`), not the live planner, so
`sim_to_real_investigation.md` §12.8/§14/§26 do not apply. Verified, not
assumed: with `use_planner=False` the controller's reference and the scoring
spline are identical, so §26's planner-added heading-rate excess is exactly
**0.0** on every tick.

### What was measured (live, 3 runs, `centerline.csv`)

| quantity | value |
|---|---|
| commanded steering vs geometric requirement | **leads by 0.15 s**, corr 0.93 |
| controller gain (commanded / required, in corners) | **0.936** |
| `delta_cmd` vs applied `steer_deg` | identical (max diff 0.0005°) |
| `a_lat` above the ~7.5 ceiling | 4.69% of ticks, peak 10.21 |
| drift episodes, rate-normalised | ~31.5 /min, **invariant across every config tried** |

**Conclusion from the table:** the controller commands ~94% of the required
angle, does so *early*, and nothing between the solver and the actuator alters
it. Whatever causes the symptom is not the command's timing or magnitude.

### Falsified, with evidence — do not retry without new information

- **Steering-rate weight `r_rate_delta`.** Swept 52.5 / 30 / 20 / 10 / 5
  offline: 52.5 is the BEST on every lateral-error measure (`|e_y|` mean
  0.476, p90 0.976) and two lower values DNF. Lowering it makes the car go
  *wider*, not earlier. The high rate weight is what keeps it on the line.
- **`nmpc_corner_factor_k` beyond 27.** k=60 live: score 0.497 vs 0.454,
  flip% 39.0 vs 36.0. It does lower the corner weight as designed (effective
  r_rate 38.7 -> 29.2 in drift zones) and drift barely moves (10.86 ->
  10.45 m) — which is itself the evidence that the rate cost is not the limit.
- **`nmpc_q_e_y` as a drift fix.** 7.5 is kept (score-neutral, lower peak
  error) but it does NOT remove the drift: the rate-normalised episode count
  is unchanged. 8.5 and 9.0 DNF offline.
- **`NMPC_SQP_ITERS`.** Offline 1/2/3: the ticks actually pinned at the slew
  limit are FLAT (|d|>8.9° = 2.29 / 2.22 / 2.33 %) and max|d| sits at the
  9.00°/tick ceiling in all three. Extra iterations trim only mid-range
  activity while `|e_y|` and score degrade; `solve_ms` max reaches 52.8 ms
  (over the 50 ms tick) at iters=2 and iters=3 DNFs.
- **Speed-profile braking feasibility.** Both exported profiles are feasible:
  worst implied decel −5.03 (`speed_profile.csv`) / −5.95 (`centerline.csv`)
  against a −7.0 limit, **0.00%** of stations exceeding it. `u_accel` never
  reaches the brake limit in the rollout (min −5.88, 0.00% at limit). The
  backward brake pass is working.
- **Delay compensation under-estimating.** `n_delay` is 1 on every tick,
  derived from `pose_age_s` only, while `solve_ms` and `cmd_latency_ms` are
  sampled after it — a real accounting gap, but NOT the cause: splitting each
  run at its median total latency gives the **same** 0.15 s lag in both
  halves, and a run with nearly double the pose age (44.5 vs 26.1 ms median)
  shows the same lag.
- **`alat_ceiling` model mismatch.** Checked per speed bin: in the 6–14 m/s
  band the car ACHIEVES MORE `a_lat` than `max(7.5, 0.47v + 2.46)` predicts
  (by 0.5–1.4 m/s²). The model is conservative there, not optimistic; only
  3.88% of ticks exceed it. No mismatch to fix.

### Two measurement traps that each produced a wrong conclusion

1. **Aggregate ratios over a full lap are dominated by straights.** A "plant
   yaw gain" of 0.42 at p10 looked like severe understeer; binned by speed it
   is an artefact — the low-gain ticks are fast and nearly straight (median
   v 10.96, `|steer|` 6.83°, `a_lat` 3.52, `|e_y|` 0.151 — *better* than
   average), where `v/L·tan(δ)` divides by a near-zero denominator. Gain
   actually RISES with `a_lat` (0.131 at 0–2 → 1.102 at 6–8), the opposite of
   a grip limit. Bin before believing a tail statistic.
2. **`dv_target/dt` per tick is not the profile's gradient.** Measuring the
   speed target's time derivative along a rollout gave −26.9 m/s² and looked
   like an infeasible reference; the profile's own spatial gradient is
   −5.03 m/s². The car crossing stations faster inflates the time
   derivative. Check the exported CSV directly.

### Still open

No confirmed cause for the residual late turn-in. What is established is that
it is **not** the steering-rate cost, the error weight, the solver iteration
count, the speed profile's feasibility, the delay compensation, or the
ceiling model — and that the command itself leads and is ~94% of required.
The next measurement worth doing is per-corner-event, not aggregate: isolate
corner-entry windows and compare commanded steering, achieved yaw and lateral
error tick by tick inside them.

---

## Resolution summary: two distinct faults, similar symptoms

What presents as a single complaint — "jerky steering, won't turn enough,
turns in late" — is **two independent problems**. That is why sweeping any one
weight kept returning a null result: each sweep addressed at most one of them.

### Fault 1 — tick-to-tick chatter: a controller-cost problem

Fixed by, in order of measured impact:

1. **`r_rate_delta` 2.8 → 52.5.** The dominant lever. Live chatter 2.538 →
   1.919 °/tick, and the first thing to move the sign-flip rate off ~65%.
2. **Input-jerk term `nmpc_rjerk_delta=150`** (second difference of steering).
   Separates a sustained ramp from an oscillation in a way `|du|` cannot; on
   the centreline it holds 0 saturated ticks, 0 slew-limited ticks and 1
   reversal over three laps. See `docs/tuning.md`'s "Input-jerk
   cost".
3. **Post-solve output LPF disabled** (`output_smoothing_enabled=false`),
   made redundant by 1–2.
4. **Three-zone `R_rate` schedule** — the smallest contributor of the four,
   and only after `nmpc_corner_factor_k` 8 → 27; at 8 it never left its boost
   band. Running at `ease_approach=0.80`, not the intended 0.35, which DNFs
   offline.

Also load-bearing but not a weight: the **reference line** (raceline →
`centerline.csv`, score 0.752 → 0.488), which by itself took steering
reversals 13 → 1.

### Fault 2 — sudden steering jumps at tight corners: a SPEED problem

Not a controller fault at all. `CURVATURE_SPEED_A_LAT_MAX` **5.5 → 4.75** took
stutters 33.3 → 9.8 /min and `|d_steer|>5°` events 20 → 1. Mechanism and the
full table are in `docs/reference_path_and_speed.md`'s "Speed-profile aggressiveness".

**This is the diagnostic lesson of the whole investigation.** Six
controller-side hypotheses were tested against Fault 2 and all six failed,
because the steering command was already leading the geometric requirement by
0.15 s at ~94% of the required magnitude. The car was simply arriving at the
hardest curvature ramp needing ~15 m/s² of lateral acceleration against a
~7.5 ceiling. **Check `a_lat` demand and speed overshoot before touching a
steering weight.**

### What remains

~9.8 stutters/min, amplitude 1.5–3.2° (was 5.9–7.9°), clustered at corner
**exits** — the signature is large `|e_psi|` (13–16°) with small and shrinking
`|e_y|` (0.03–0.28 m), i.e. the car is back on the line but still rotating,
and the solver makes small alternating corrections while heading unwinds.
Distinct from Fault 2's entry-side jumps. Not worth chasing at this amplitude; the
next lever would be `q_e_psi`/`q_r` at corner exit.
