# Steering chatter investigation (NMPC, live + offline)

**Status: OPEN.** Root cause not yet found. This doc exists so a future
session can resume without re-deriving what's already been ruled out.

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
known open planner defect (CLAUDE.md's "centreline curvature spikes") cannot
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
scratchpad scripts used to gather every number in this doc (those lived
under `/tmp/claude-1000/.../scratchpad/` and do not persist between
sessions). Use this script for any future sweep in this investigation;
extend it rather than writing a new one-off script, so the next session
doesn't have to re-derive the harness boilerplate again. It already prints
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
  a `rollout_core.py` wiring change; keep both sides in sync per CLAUDE.md.
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
- **Any single cost weight in isolation** (`q_r`, `r_rate_delta` up to 4x,
  `r_delta` up to 8x, `nmpc_trust_delta_rad` down to 2deg,
  `nmpc_terminal_scale`/`TERMINAL_Q_SCALE` up to 20x): each produces only a
  modest reduction in chatter (or none at all, for terminal_scale), always
  at a real `|e_y|`/score cost, never approaching the LTV-QP's baseline
  noise level even at the most aggressive setting tried. Do not expect a
  single-weight retune to be a clean fix based on the pattern established
  here -- if a future session finds one weight that DOES cleanly fix it,
  treat that as a surprising result worth double-checking, not a
  confirmation of the existing pattern.
- **`NMPC_HORIZON` in the budget-feasible range (10-28 steps)**: no
  consistent trend (values bounce 5.22-5.78 deg std_d with no monotonic
  relationship to horizon length). The apparent improvement at
  `NMPC_HORIZON=35` is INVALID -- confirmed (twice, at two different solve
  budgets) to be the SQP failing/freezing, not a real fix; see "Separate
  issue found while investigating" above. Do not retest horizon=35 (or
  presumably anything near/above it) as a chatter fix without first fixing
  the underlying SQP-rejection problem at that horizon length, which is
  unsolved and out of this doc's scope.
