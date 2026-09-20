# Progress-term NMPC: implemented, but it does not reliably complete a lap

## The most useful result here is not the progress term

Investigating why the progress term could not raise commanded acceleration
found that **`SPEED_TARGET_DEFICIT_MAX = 2.5` was the binding constraint on
acceleration for 36.8% of a normal lap**, not the launch guard it was written
as. Raising it to 5.0 makes the car about 2.5 s faster per lap with lower
lateral error and lower steering saturation, and makes run to run scores 40x
more repeatable. That change is independent of the progress term, applies to
both controllers, and is already applied. See "Accel authority" below.

The progress term itself is implemented and wired end to end, but it does not
reliably complete a lap and is not worth enabling.

## Summary

The NMPC can be made to choose its own speed from an arc-length progress
reward instead of tracking a speed profile. It launches and drives, but on a
repeat measurement it does NOT reliably complete a lap at any weight tried,
and it never beats the tracking controller it would replace. The table below
is the first sweep, kept because the failure modes in it are real; the
repeatability check that supersedes its one passing row is the section after
it.

| configuration | score | progress | DNF | off-track | \|e_y\| mean/p90 | a_cmd max |
|---|---|---|---|---|---|---|
| **baseline (tracking, `q_e_v`)** | **0.757** | 0.994 | no | no | 0.418 / 0.944 | 4.45 |
| progress `q_progress=3` | 13.000 | 0.000 | yes | no | never launches | 1.67 |
| progress `q_progress=5` | 12.583 | 0.570 | yes | **yes** | 0.549 / 1.580 | 4.19 |
| progress `q_progress=5` + linear slack | **0.892** | 0.995 | no | no | 0.561 / 1.460 | 4.19 |
| progress `q_progress=7` | 15.417 | 0.097 | yes | **yes** | 0.187 / 0.343 | 4.76 |

Lower score is better. `comp_test_map_3`, oracle path, `USE_NMPC=True`,
otherwise settings.py defaults.

One configuration finished in the first sweep: `q_progress=5` with the linear
track-boundary slack enabled, scoring 0.892 against the baseline's 0.757.
**That result did not hold up** (see "The completing run was not repeatable"
below) and should not be quoted as the feature's standing result.

**The feature is implemented, defaults off, and is not recommended for live
testing in its current state.** The two measured obstacles below are specific
and probably fixable, but neither is fixed yet.

## Plain-language version

The controller normally drives forward because it is told a target speed and
punished for missing it. This change removes that instruction and replaces it
with a reward for covering distance along the track, plus a speed ceiling it
must not exceed. The idea is that the controller works out its own speed,
going quickly where the track allows and slowing where it does not, instead of
following a speed profile computed in advance.

It works in the sense that the car launches and drives. It does not work in
the sense that matters: on repeat measurement it leaves the track before
finishing a lap at every reward strength tried. Too weak and the car never
pulls away from a standstill, too strong and it runs wide at a corner, and the
gap between those two turned out not to contain a setting that reliably
finishes.

## The completing run was not repeatable, and the default stays OFF

Turning the progress term on by default was requested on 2026-09-21,
attempted, and **reverted the same day** after measuring the exact shipped
configuration end to end rather than trusting the earlier sweep.

At the current `SPEED_TARGET_DEFICIT_MAX = 5.0`, with the linear track slack
active, nothing completes a lap:

| `q_progress` | slack_linear 1000 | 10000 | 50000 |
|---|---|---|---|
| 4.0 | never launches (progress 0.000) | same | same |
| 5.0 | off track at 9.9% | same | same |
| 6.0 | off track at 9.8% | same | same |

Raising the boundary penalty 50x changes nothing at any weight, so the slack
term is not the missing piece. Tracking on the same map completes cleanly and
scores **0.714**.

**Two corrections to the earlier write-up:**

1. **The single "completed lap" at `q_progress=5` was not repeatable.** Re-run
   at `DEFICIT_MAX = 2.5` (the value it was originally measured at) it goes
   off track at 57% of a lap. The original pass was a lucky draw from this
   rollout's run to run variance, not a property of the configuration, and
   reporting it as a result was wrong.
2. **The usable band NARROWED when `SPEED_TARGET_DEFICIT_MAX` went 2.5 to
   5.0.** The higher speed cap hands the progress reward more headroom, and it
   spends it going into corners. The two changes interact, so any future
   progress-term measurement has to be taken against the deficit value
   actually in force.

## The weight band is narrow and both edges are hard failures

`q_progress` is the strength of the progress reward. The usable band at
`r_a_accel = 1.0` is roughly 5 to 6.

**Below the band the car never moves.** At `q_progress=3` the commanded
acceleration settles at 1.67 m/s² and the car stays at exactly zero speed
indefinitely. The plant needs about **2.3 m/s²** to break static friction
(`F_stiction = 600 N` over `m = 255 kg` in `model/vehicle_physics.py`). The
reward is not strong enough to pay for the acceleration effort that would get
there, so the car sits still and the rollout ends in a stall DNF.

**Above the band the car leaves the track.** At `q_progress=7` the car goes
off-track after 97% of a lap remained. Note its tracking error while running
is *better* than the baseline (`|e_y|` mean 0.187 against 0.418): it is not
wandering, it is carrying too much speed into a corner and running out of
road. This is the documented corner-cutting incentive, below.

## The corner-cutting incentive is real, and the linear slack term helps but does not fix it

In Frenet coordinates the rate of progress is

```
s_dot = (v_x*cos(e_psi) - v_y*sin(e_psi)) / (1 - kappa*e_y)
```

The denominator means that for a given speed, sitting toward the inside of a
bend (`e_y` and `kappa` the same sign) produces *more* progress than sitting on
the line. A reward on progress therefore pays the car to hug the inside of
corners, entirely separately from any racing-line argument. Nothing in the
original soft track boundary opposes this strongly, because that penalty is
purely quadratic and a quadratic has **zero gradient at zero violation**: the
first small amount of boundary violation is nearly free.

Adding a linear term to the boundary slack (Liniger's MPCC reference
implementation carries both a quadratic `sc_quad_track` and a linear
`sc_lin_track` for this reason) measurably delays the excursion, and in one
run it produced a completed lap:

| | score | progress | off-track |
|---|---|---|---|
| `q_progress=5`, quadratic slack only | 12.583 | 0.570 | yes |
| `q_progress=5`, plus `slack_linear_weight=1000` | 0.892 | 0.995 | no |

**That completed lap did not reproduce**, and raising the linear weight from
1000 to 50000 does not rescue any weight (see the repeatability section
above). So the mechanism is real and worth keeping in any future attempt, but
it is not sufficient on its own, and it was a mistake to describe it as a
fix.

## The unreachable target must be floored, or the car cannot launch

The progress reward is written as a least-squares residual against a target
arc length that is deliberately never reachable, `s_target_N = s0 + gap`.
Writing it this way rather than as a bare linear reward is what keeps it
compatible with the Gauss-Newton solver (see "Why not a linear reward" below).

Setting `gap = v_cap * N * dt * reach` alone is wrong at launch.
`SPEED_TARGET_DEFICIT_MAX` deliberately holds the speed cap near 2.5 m/s while
the car is still stationary, so the gap becomes a few metres, the residual
saturates almost immediately, and the reward stops pushing before the car ever
breaks static friction. Measured: `a_cmd` plateaus between 0.6 and 1.3 m/s²
and `v_x` stays at exactly 0 for the whole run.

The fix is to floor the gap at the horizon's own kinematic reach,
`0.5 * a_max * (N*dt)^2 * reach`, which is a property of the controller's own
commanded authority rather than of the plant's stiction constant. At `N=20`,
`dt=0.05`, `a_max=12`, `reach=2.0` this is 12.0 m against a measured minimum of
about 11 m at this weight set.

The floor is inert once the car is moving properly:

| `v_cap` | `v_cap*N*dt*reach` | floor active |
|---|---|---|
| 2.5 | 5.0 | yes |
| 5 | 10.0 | yes |
| 8 | 16.0 | no |
| 15 | 30.0 | no |

So it shapes launch only, not cornering or cruising.

## Why not a bare linear reward

The textbook MPCC progress term is linear, `-q_s * s_N`. That cannot be used
directly here. This solver builds its Hessian as `G'G` from a least-squares
residual vector, so a linear term contributes to the gradient and **nothing at
all to the Hessian**. The QP is then pushed in the progress direction with no
curvature opposing it, and a single SQP step goes straight to whichever bound
it hits first. This is the standard economic-NMPC difficulty (Zanon, "A
Gauss-Newton-Like Hessian Approximation for Economic NMPC", IEEE TAC,
arXiv:2007.13519); acados documents the same zero-Hessian-block case and
regularises it explicitly.

The unreachable-target form sidesteps it: `(s_target_N - s_N)^2` is monotone
equivalent to maximising `s_N` while contributing `q_progress` to the Hessian
diagonal like every other cost row.

## What this does NOT reproduce

Two previously-rejected mechanisms are structurally excluded by design, not by
tuning:

- **The summed-horizon loophole** that killed
  `nmpc_horizon_speed_profile_enabled` (a high `v_ref` on the straight after a
  corner paying for a low one at the corner, live result 16.7 m/s into a 3 to
  5 m/s target). The progress reward scores the **terminal stage only**, so no
  later stage can offset an earlier one.
- **The inert-constraint failure** that killed `nmpc_speed_limit_enabled` (its
  own diagnostic read exactly 0.0 violation while the real car was several m/s
  over target, because the model's prediction was wrong). The speed cap here is
  a soft penalty with gradient everywhere, not a hard inequality that can
  report itself satisfied.

## Accel authority: the real ceiling is `SPEED_TARGET_DEFICIT_MAX`, and raising it is a free win

Standing question from the plan: `a_cmd` peaks at about 2.3 m/s² under
tracking against a plant capable of roughly 12, and it was unclear whether
`NMPC_TRUST_A = 0.6` with one SQP iteration was the real ceiling.

**Two candidates were eliminated by direct measurement**, each swept over a
10x range with every other setting held fixed:

| swept | values | `a_cmd` max | score |
|---|---|---|---|
| `trust_a` | 0.6, 2.0, 6.0 | **4.45 at every value** | 0.757 at every value |
| `r_a_accel` | 1.0, 0.5, 0.25, 0.1 | **4.43-4.45** | 0.757-0.759 |

Neither moves a single digit. The trust region bounds how fast `a_cmd` may
*change* per tick, but the warm start carries the value across ticks, so it
never caps the steady state.

**The actual constraint is `SPEED_TARGET_DEFICIT_MAX`.** It holds the speed
target at `car_speed + 2.5`, and the measured speed error sits at exactly
-2.500 at p05 with **36.8% of the lap pinned there**. The controller is not
choosing 4.45 m/s², it is being handed a target only 2.5 m/s away and
correctly declining to accelerate harder than that requires.

Raising it to 5.0 improves everything at once, with no trade:

| `DEFICIT_MAX` | score (3 runs) | mean | lap steps | `a_cmd` max | pinned | \|e_y\| mean | steer sat |
|---|---|---|---|---|---|---|---|
| **2.5** | 0.757 / 0.804 / 0.757 | 0.772 | 1081-1117 | 4.45 | 36.8% | 0.418 | 4.71% |
| **5.0** | 0.693 / 0.692 / 0.693 | **0.693** | 1033-1034 | **8.32** | 2.3% | 0.402 | 3.77% |

10% better score, about 2.5 s faster, nearly double the peak acceleration,
and *lower* lateral error and steering saturation. Run to run spread also
collapses from 0.047 to 0.001, because the clamp is no longer arbitrating
most of the lap.

Launch, the behaviour the clamp exists to protect, is unchanged: launch at
step 9 at both values, launch-phase `|e_y|` 0.27 m against a 3.5 m boundary.
Values above 5 buy nothing (10.0 and 100.0 both plateau at `a_cmd` 8.87), so
5.0 is the knee.

**This is not "the clamp was wrong".** It is a real guard and still needed
(removing it entirely re-introduces the launch excursion it was written for).
It was simply set tight enough to bind far outside the regime it was designed
for, and nobody had measured how often it was active during normal driving.

Changed to 5.0 in all three copies (`sim/rollout_core.py`, the live
`mpc_controller.py`, and the `fsds_simulator` mirror). **Offline only, not yet
live-validated.** Faster corner entry is the specific risk to watch on the
car, given the documented sim-to-real gap.

## `R_A_ACCEL` was stale during part of this work

The weight calibration above assumes `R_A_ACCEL = 1.0`. An earlier pass used
2.25, which was the value on `origin/main` at the time, and reached the
opposite-looking conclusion that `q_progress` needed to be 20 or more. With
the corrected 1.0 the launch threshold drops to about 5. Any weight quoted
here is only meaningful against `r_a_accel = 1.0`.

## What to try next

In rough order of expected value:

1. **Sweep `q_progress` finely between 5 and 7 with the linear slack on**, to
   find whether any point in that band beats 0.757. The band is narrow enough
   that the current three-point sweep may have stepped over a good value.
2. **Tune `NMPC_SLACK_LINEAR_WEIGHT`.** It was tried at one value (1000) and
   the difference between 1000 and 10000 was negligible, so its useful range
   has not been located at all.
3. **Re-check the horizon.** N=20 was chosen for tracking, and the measured
   sweep shows N=35 gives the fastest lap at worse tracking. Lap time is what
   a progress term optimises, so N is a genuine co-variable here, not a
   constant. Solve time at N=35 was 14.1/20.1 ms against a 25 ms budget.
4. **Only then consider a live test.** Offline alone is not confirmation here:
   both previous speed-related features looked acceptable offline and failed
   live, and the documented sim-to-real gap has live saturating about four
   times more often than the sim.

## Status

`NMPC_PROGRESS_ENABLED = False` / `nmpc_progress_enabled: false` everywhere,
and `nmpc_slack_linear_weight` back to `0.0` alongside it (inert without the
progress reward). Enabling by default was tried on 2026-09-21 and reverted
the same day, see "The completing run was not repeatable" above.

Flags-off is bit-identical to the pre-change controller on **both** sides:
offline (same `u_opt`, same solved cost) and live (all 6 test ticks and the
cost match to full float64 precision against `git HEAD`).
`tuner.nmpc_offline_check` passes all four sections.

Ported across the full parity boundary, all defaults off:

| side | files |
|---|---|
| offline | `controller/nmpc_optimiser.py`, `settings.py`, `sim/rollout_core.py` |
| live | `mpc/nmpc_core.py`, `mpc/nmpc_params.py`, `mpc/mpc_params.py`, `mpc/mpc_controller.py`, `live_viz.py`, `telemetry_logger.py`, `fsae_params.yaml`, `launch_all.sh` |
| mirror | the same eight files under `fsds_simulator/` |

Three telemetry columns (`nmpc_v_cap`, `nmpc_speed_cap_over`,
`nmpc_s_target_gap_end`) are declared in `telemetry_logger.py` **before** the
feature is used live, deliberately: `nmpc_friction_circle_enabled` shipped
without its two columns and its own diagnostics silently never reached a CSV,
which is how a conflicting `F_max` went undiagnosed.

### How to actually run it

All five settings are wired end to end, verified by walking the chain:
`launch_all.sh` variable -> launch arg -> `DeclareLaunchArgument` -> node
parameter -> dataclass -> controller. Uncomment in `ros2/launch_all.sh`:

```
NMPC_PROGRESS_ENABLED=true
NMPC_Q_PROGRESS=5.0            # the band is narrow, see above
NMPC_SLACK_LINEAR_WEIGHT=1000.0  # required, or it goes off-track
```

`NMPC_PROGRESS_REACH` and `NMPC_PROGRESS_V_MIN` can stay at their defaults.
`USE_NMPC=true` is also required, since every one of these is NMPC-only.

The launcher GUI's Settings tab carries the same five: the flag under
**Feature flags - NMPC only**, and the four numeric values under **NMPC
progress term (experimental)**. Saving writes both `settings.py` and the
matching live dataclass field, and the GUI now falls back to
`nmpc_params.py` for fields that are not in `mpc_params.py` (the progress
flag is structural, not a weight, so it lives in the former).

### Debug surfaces

- **Live cost bars** (`live_viz.py`): a `progress` bar appears in the
  tracking panel when the flag is on, and is skipped otherwise since the
  term tables are filtered allow-lists. Row 4 is relabelled `v_cap_hinge`
  in progress mode (normally exactly 0, not a two-sided speed error) rather
  than staying `e_v`, which would otherwise make a working controller look
  like it had zero speed error. `horizon_terms` was found hardcoded to the
  5-name tracking-mode tuple, which mislabelled the hinge AND silently
  dropped the progress row from the horizon panel entirely; both are now
  mode-aware.
- **Live stats box** (`live_viz.py`): a `progress term:` block prints
  `v_cap`, `cap_over` (tagged `CAP BINDING` or `under cap`, the one signal
  that separates "held back by the cap" from "chose to go slower") and
  `s_gap_end`. Fed by a new `progress` key on the `debug_weights` topic.
- **Offline playback** (`tuner.tools.plot_playback`): the three CSV columns
  plot by name via `--signals`, plus a new derived `v_cap` pair that
  overlays `v_actual` against `nmpc_v_cap` on one axis.

One pre-existing mirror divergence was found and deliberately left alone per
the "do not fix unrelated drift" rule: `mpc_controller.py`'s
`DISABLE_LIVE_CURVATURE_SPEED` block exists live but not in
`fsds_simulator/`. Only the specific change made here was propagated.

## `speed_target_deficit_max` promoted to a real parameter (2026-09-21)

Was a bare module constant (`SPEED_TARGET_DEFICIT_MAX`) hand-synced across
three files with no ROS param, launch arg, or GUI entry, the only way to
change it was editing code in all three at once. Promoted to
`MPCParams.speed_target_deficit_max`, following the exact path
`nmpc_q_progress` already uses: dataclass field (`controller: both`, since
it shapes the target before either controller's own speed-error/cap row
sees it) -> `fsae_params.yaml` -> `launch_all.sh` shortlist
(`MPC_SPEED_TARGET_DEFICIT_MAX`) -> GUI Settings-tab scalar ->
`settings.py` on the offline side. Default unchanged at 5.0.
`tuner.nmpc_offline_check` and `tuner.recorded_map_rollout` both reproduce
their pre-change numbers exactly (score 0.442), confirming the promotion
is wiring only, not a behaviour change.

## `nmpc_track_halfwidth` narrowed 3.5 -> 3.0 m, and a GUI launch checkbox added (2026-09-21)

Two changes made together while extending the progress-term GUI surface:

- **`nmpc_track_halfwidth` (both the quadratic and linear slack boundary)
  narrowed from 3.5 m to 3.0 m.** Requested to give the slack headroom to
  catch a mistake before the car reaches the true track edge, not right at
  it, given the progress reward's analytic incentive to hug the boundary
  (kappa(s)'s `1/(1-kappa*e_y)` metric factor rises toward the inside, see
  the design doc's landmine #2). Already a fully-plumbed `nmpc_only`
  dataclass field; only the default changed, plus adding it to
  `launch_all.sh`'s shortlist (missing until now) and the GUI Settings
  tab's progress-term card, since both were absent despite the field
  itself being wired. `tuner.nmpc_offline_check`'s tracking-mode numbers
  move slightly (|e_y| p90 0.922 -> 0.811 deg, |e_psi| mean 5.65 -> 4.76
  deg), improving, since the tighter boundary pulls the car back sooner;
  the LTV-QP path (which does not read this field) is bit-identical.
  NOT yet re-measured with the progress term itself on.
- **GUI checkbox to launch with the progress term on**, on the Launch tab,
  visible only when NMPC is selected. `NMPC_PROGRESS_ENABLED` ships as a
  commented-out `launch_all.sh` shortlist line, which the existing
  `_read_var`/`_rewrite_var` pair cannot see or edit (their regex requires
  the line to already be uncommented) -- new `_read_shortlist_var`/
  `_rewrite_shortlist_var` helpers handle the commented case, matched
  against real file content, not just synthetic test strings. Checking the
  box also force-sets `NMPC_SLACK_LINEAR_WEIGHT=1000.0`, measured
  necessary (not just helpful) once the progress term is on. The
  confirmation dialog before launch lists both so neither is a silent
  side effect.

**Found in passing: `ros2/launch_all.sh` had `NMPC_PROGRESS_ENABLED=true`
live-enabled**, contradicting its own comment ("ON was attempted
2026-09-21 and REVERTED the same day") and the mirror copy in
`fsds_simulator/launch_all.sh`, which correctly had it commented out.
Corrected to match the documented, measured-off-track-at-every-weight
verdict. Root cause not established (manual edit while testing, most
likely); flagged rather than silently fixed given more than one session
works in this checkout.
