# `nmpc_speed_limit_enabled`: rejected twice, same failure both times

## Summary

`nmpc_speed_limit_enabled` (a soft per-stage speed ceiling for the NMPC
horizon) looks good offline but causes off-track excursions live, on both
attempts (2026-08-19, 2026-09-15). The flag stays `false`. The mechanism
that was supposed to make this safe, a hard-per-stage constraint instead of
a summed cost, does not actually engage during the failure: the constraint's
own diagnostic (`nmpc_speed_limit_over_max`) reads exactly 0.0 throughout,
even while the car is measurably several m/s over its speed target. The
constraint never sees a problem to constrain.

## Plain-language version

The NMPC plans a whole future path in one shot (a "horizon" of predicted
steps), and one of the things it tries to hit at every step is a target
speed. Normally that target speed is one flat number for the whole plan,
which means the controller can't see a corner coming early enough to slow
down in good time.

This feature tried to fix that by giving the controller a target speed that
changes across its plan, lower near an upcoming corner, higher after it, so
it could start braking earlier. An earlier version of this idea (a
different flag, already rejected before this investigation) failed because
it let the controller "trade off" a bad result at the corner against a good
result afterward, in the same calculation, so it never actually had to slow
down. This version was built to close that loophole with a hard rule
("you may not exceed this speed at this point in your plan") instead of a
soft preference.

It still failed, live, twice. Not because the loophole reopened, but because
the hard rule only checks the controller's own *predicted* speed against its
own *predicted* position on the corner. If the controller predicts it will
have already braked in time (even if that prediction is wrong), the rule
is satisfied and does nothing, while the real car is still going too fast.
This project's simulator is known to not perfectly predict the real car's
behaviour under braking and cornering (see CLAUDE.md's "offline sim does
not yet fully predict the car" section); this appears to be another symptom
of that same gap, not a new bug in this feature specifically.

## Result table

| | run 1 (08:38:52) | run 2 (08:39:32) |
|---|---|---|
| `\|e_y\|` max | 4.40 m | 2.19 m |
| track half-width | 3.5 m | 3.5 m |
| went off-track | yes | yes |
| `v_actual` max | 16.5 m/s | 16.3 m/s |
| `nmpc_speed_limit_over_max` max/mean | 1.86 / 0.17 | 2.71 / 0.18 |
| ended in permanent stall (`v_actual=0`) | yes | no (recovering at log end) |

Both runs used `comp_test_map_3` (default `TRACK`), `USE_PRECOMPUTED_SPEED=true`,
`USE_PRECOMPUTED_PATH=true` (so a real speed-profile array was supplied,
the flag was not a no-op), `NMPC_SPEED_LIMIT_ENABLED=true`, all other NMPC
flags at their `launch_all.sh` defaults.

## Failure sequence (run 1, `t=14.86-15.36s`)

| t (s) | v_actual | v_desired | e_y | speed_limit_over_max | steer |
|---|---|---|---|---|---|
| 14.86 | 9.10 | 2.66 | -2.60 | 0.0 | 25.0° |
| 15.02 | 7.78 | 2.73 | -3.06 | 0.0 | 17.4° |
| 15.27 | 5.53 | 2.83 | -3.65 | 0.0 | 25.0° |
| 15.36 | 4.04 | 2.85 | -3.96 | 0.0 | 25.0° |

`a_cmd` is strongly negative throughout (-5 to -6 m/s², braking hard), so
the controller is not failing to brake, it starts braking too late relative
to how fast it is actually travelling. Steering saturates at the 25° lock
while this happens. `e_y` crosses the 3.5 m track half-width at `t=15.22s`
and the car does not recover in run 1; `v_actual` reaches 0 by the end of
the log and stays there (permanent stall, not a transient dip).

## Why the constraint doesn't catch this

`nmpc_speed_limit_over_max` reports the worst predicted overspeed the
solver's own horizon needed slack for, at the final accepted solve each
tick. It reads 0.0 across the entire overspeed event in both runs. That
means the solver's own rollout, at every tick during this approach, predicted
a trajectory that already satisfies `v_x_k <= v_ref_at(s_k) + margin` at
every stage, using its own model of how the car brakes and how quickly arc
length `s` advances. The real car did not brake the way the model predicted
(9.1 m/s actual vs. a corner target of 2.7-2.8 m/s, closing far too slowly),
so the constraint had already been satisfied on paper before reality caught
up to it.

This is consistent with the documented, still-open sim-to-real gap: live
steering saturates roughly four times as often as the offline sim predicts,
and heading error runs about twice as high (see CLAUDE.md's "offline sim
does not yet fully predict the car"). A constraint keyed to the model's own
predicted trajectory is exactly as exposed to that gap as the plain
tracking cost is; wrapping the same lookup in a hard inequality does not
by itself make the lookup accurate.

## What this rules out for a future attempt

- **Loosening `nmpc_speed_limit_margin` or raising `nmpc_speed_limit_slack_weight`
  is unlikely to help.** The constraint is not engaging weakly, it is not
  engaging at all (`over_max` reads exactly 0, not a small positive residual).
  Retuning either knob only changes how the constraint behaves once it
  detects a violation; it does nothing for a case where the model does not
  perceive a violation in the first place.
- **The mechanism itself (hard per-stage inequality vs. summed cost) is not
  the problem this time.** That was the fix for the *first* rejected
  attempt (`nmpc_horizon_speed_profile_enabled`, cost-term version, see
  `late_turn_in_investigation.md` Part 16 §16.7) and it worked as designed:
  it is a real per-stage constraint, correctly wired, correctly gated. It
  simply constrains a prediction that is itself wrong in this regime.
- **A future attempt should address the underlying model-prediction gap
  first** (or measure whether it is smaller in some other speed/curvature
  regime before assuming this flag is unusable everywhere), rather than
  retuning this feature's own parameters in isolation.

## Status

`NMPC_SPEED_LIMIT_ENABLED=false` in both `ros2/launch_all.sh` and
`fsae_MPCTest/fsds_simulator/launch_all.sh`. Do not re-enable without new
evidence bearing on the sim-to-real gap specifically, not just on this
flag's own margin/weight parameters.
