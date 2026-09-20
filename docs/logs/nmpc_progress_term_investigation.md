# Progress-term NMPC: implemented, and it drives, but it does not beat tracking yet

## Summary

The NMPC can be made to choose its own speed from an arc-length progress
reward instead of tracking a speed profile, and it completes a lap. It does
not currently beat the tracking controller it would replace, and it only
completes a lap at all inside a narrow weight band.

| configuration | score | progress | DNF | off-track | \|e_y\| mean/p90 | a_cmd max |
|---|---|---|---|---|---|---|
| **baseline (tracking, `q_e_v`)** | **0.757** | 0.994 | no | no | 0.418 / 0.944 | 4.45 |
| progress `q_progress=3` | 13.000 | 0.000 | yes | no | never launches | 1.67 |
| progress `q_progress=5` | 12.583 | 0.570 | yes | **yes** | 0.549 / 1.580 | 4.19 |
| progress `q_progress=5` + linear slack | **0.892** | 0.995 | no | no | 0.561 / 1.460 | 4.19 |
| progress `q_progress=7` | 15.417 | 0.097 | yes | **yes** | 0.187 / 0.343 | 4.76 |

Lower score is better. `comp_test_map_3`, oracle path, `USE_NMPC=True`,
otherwise settings.py defaults.

Only one configuration finishes: `q_progress=5` with the linear track-boundary
slack enabled. It scores **0.892 against the baseline's 0.757**, i.e. about
18% worse, and carries a 55% higher `|e_y|` p90.

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

It works, in the sense that the car launches, drives, and completes a lap. But
it is worse at staying on the intended line than the version it replaces, and
the range of reward strengths that produce a finished lap is narrow: too weak
and the car never pulls away from a standstill, too strong and it leaves the
circuit.

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

## The corner-cutting incentive is real and the linear slack term fixes it

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
`sc_lin_track` for this reason) converts a `q_progress=5` off-track DNF into a
completed lap:

| | score | progress | off-track |
|---|---|---|---|
| `q_progress=5`, quadratic slack only | 12.583 | 0.570 | yes |
| `q_progress=5`, plus `slack_linear_weight=1000` | 0.892 | 0.995 | no |

It does not rescue `q_progress=7`, which fails for a different reason (too much
speed, not too little boundary pressure).

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

## Accel authority: the cost shape was the binding constraint, not the trust region

Standing question from the plan: `a_cmd` peaks at about 2.3 m/s² under
tracking against a plant capable of roughly 12, and it was unclear whether
`NMPC_TRUST_A = 0.6` with one SQP iteration was the real ceiling.

It is not. Under the progress term `a_cmd` scales cleanly with the reward
weight, well past the old plateau:

| `q_progress` | a_cmd max |
|---|---|
| 2 | 1.30 (never launches) |
| 5 | 2.48 |
| 10 | 5.09 |
| 20 | 8.55 |

The trust region limits how fast `a_cmd` may *change* per tick, but the warm
start carries it across ticks, so it is not what caps the steady value. The
old ceiling was the shape of the speed-error cost, not solver machinery. This
answers the question the plan flagged, independently of whether the progress
term itself is adopted.

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

`NMPC_PROGRESS_ENABLED = False`. Flags-off is bit-identical to the
pre-change controller (same `u_opt`, same solved cost, to full float64
precision) and `tuner.nmpc_offline_check` passes unchanged. The feature is
present, wired through `settings.py` and `sim/rollout_core.py`'s
`nmpc_overrides`, and not enabled anywhere.

Live-side (`nmpc_core.py`) port not yet done, deliberately: there is no point
mirroring a mechanism across the parity boundary until it beats the controller
it would replace.
