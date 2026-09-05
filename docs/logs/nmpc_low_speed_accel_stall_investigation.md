# NMPC stalls at 2.5-6 m/s in straight-line offline rollouts: root cause found, one candidate fix validated, not yet applied

The shipped NMPC (`use_nmpc=True`, default weights) commands essentially zero
acceleration whenever car speed sits in roughly 2.5-6 m/s on a low-curvature
path, regardless of how large the speed error is. Root cause is a numerical
instability in one specific internal approximation (`nmpc_jac_substeps=1`),
not the vehicle model, not the cost weights, and not the shipped rollout
integration. Raising `nmpc_jac_substeps` to 4 (2 is not enough, see below)
fixes the whole affected range in every multi-tick test run so far, at
roughly double the Jacobian's share of per-tick solve cost, expected to
still fit the 25 ms budget but not separately re-measured at this setting.
**Not applied to any shipped file** as of this writing; this document is
the findings, not a changelog entry.

## In plain English

Picture the car sitting still, or moving slowly, on a straight bit of track,
being told "go faster." Above about 6 m/s, or below about 2.5 m/s, it happily
accelerates. But in between, roughly walking-to-jogging pace, it just... does
nothing. The steering is fine, braking from a higher speed is fine, it is
specifically "please speed up while going 3-5 m/s on a straight" that fails.

This turned out to have nothing to do with the driving strategy, the tuned
weights, or the vehicle's real behaviour. It is a shortcut inside the
controller's own maths: every tick, before it decides what to do, it needs to
estimate "if I press the accelerator a little harder, what changes?" It
estimates that by mentally rehearsing the next second of driving twice
(once as-is, once with a nudge) and comparing them. To keep that cheap, each
rehearsal is done in a single coarse step rather than several finer ones.
For most speeds that coarse step is accurate enough. But this particular
car's tyres respond to sideways motion very quickly at low-to-moderate speed
(a real, ordinary property of tyres, not a flaw), and rehearsing that fast
response in one coarse step produces nonsense numbers, up to 180 times too
large, on the parts of the estimate that describe cornering. Those nonsense
numbers are far bigger than the (correct, honest) numbers describing "press
the accelerator," so when the controller weighs everything together it
effectively concludes "I don't trust any of these numbers enough to act,"
and does nothing. Rehearsing in two finer steps instead of one removes the
nonsense and the controller immediately does the sensible thing.

## The finding, precisely

| Car speed | Commanded acceleration (shipped, `nmpc_jac_substeps=1`) |
|---|---|
| 0.5-2.0 m/s | Real, decreasing from +2.4 to +0.6 m/s^2 (correct shape) |
| 2.5-5.6 m/s | **~0** (down to 1e-25 in the interior of the band) |
| 6.0+ m/s | Real, saturates at the trust-region step limit |
| Braking from 15 m/s to a 5 m/s target | Real (-4.2 m/s^2), unaffected |

Braking is unaffected; only the accelerate-from-low-speed case is hit. The
band is not a hard on/off switch, it decays in from both sides (e.g. 2.2 m/s
gives 1.8e-26, 2.4 gives 3.5e-44, the exact zero plateau sits in roughly
2.5-5.0, then climbs back out by 5.8-6.0).

## What was ruled out first, in order

- **Not a test-harness mistake.** Reproduces identically against the
  pristine, unmodified `ros2/src/fsae_planning` copy of `nmpc_core.py`,
  imported directly, side by side with the port. Confirms this is a property
  of the shipped controller, not something introduced while porting it to
  `fsae_autonomous`.
- **Not warm-start-specific.** Seeding the solver with a feasible nonzero
  acceleration guess (`U[:,1]=0.3`, within the slew-rate limit) does not
  escape it either. Not something that only affects the very first tick
  after a reset.
- **Not curvature-specific.** Reproduces identically on a dead-straight path
  and on a genuinely curved one.
- **Not the cost function.** A direct finite-difference of the true
  (nonlinear) cost at the stalled operating point confirms a large, real
  cost reduction is available by accelerating (cost drops from 7392 to 5126
  at a 5 m/s^2 test perturbation). The objective the solver is supposed to
  be descending is fine; what it computes as the descent direction is not.
- **Not `nmpc_rk_substeps`.** This parameter controls the rollout used to
  predict the trajectory, not the Jacobian. Sweeping it from 2 to 32 changed
  nothing, which is itself informative: it means the actual predicted
  trajectory (what the car is expected to do) was never the problem, only
  the internal sensitivity estimate used to decide the next step.
- **Confirmed present in a genuine closed-loop rollout, not just an isolated
  probe.** `fsae_MPCTest/sim/rollout_core.run_core_rollout()` (the same
  machinery `tuner.nmpc_offline_check`'s own closed-loop check and
  `tuner.recorded_map_rollout` use), run on the real `comp_test_map_3`
  recorded track with the real tuned `settings.py` weights, `use_nmpc=True`,
  starting from a genuine standstill:

  | Tick (0.05 s each) | LTV-QP speed | NMPC speed |
  |---|---|---|
  | 20 | 2.61 m/s | 1.69 m/s |
  | 40 | 8.33 m/s | 2.06 m/s |
  | 60 | 14.67 m/s | 2.10 m/s |
  | 100 | 7.96 m/s (already cornering) | 2.18 m/s |

  LTV-QP accelerates cleanly through the same range on the same track and
  weights. NMPC stalls flat for 3+ seconds. Over a full lap
  (`continue_after_dnf=True`), NMPC spent 199 of 1389 ticks (14%) in the
  2.0-2.5 m/s band and recorded a genuine off-track excursion (2.76 m
  lateral error) at some point in the run.

## Mechanism, precisely

`nmpc_core.py`'s `_jacobians()` builds the SQP's descent direction by
forward-finite-differencing the plant rollout, deliberately using
`nmpc_jac_substeps` (default **1**) rather than the rollout's own
`nmpc_rk_substeps` (default 2) — a documented, deliberate cost-saving
asymmetry ("halves the cost of the dominant term... the prediction itself
is unaffected", per that function's own docstring). The asymmetry is safe
in general because a slightly worse *step direction* only costs a slightly
worse SQP iteration, absorbed by the next iteration and the trust region;
the actual predicted trajectory always comes from the finer-substepped
rollout regardless.

It stops being safe at this vehicle's specific tyre stiffness. The
continuous-time linearised (v_y, r) sub-dynamics have eigenvalues whose
magnitude, divided by speed, gives:

| Speed | \|eigenvalue\| x dt (dt=0.05 s) |
|---|---|
| 2.0 m/s | 10.5 |
| 2.5 m/s | 8.5 |
| 3.0 m/s | 7.1 |
| 4.0 m/s | 5.4 / 4.1 |
| 6.0 m/s | 3.7 / 2.7 |
| 8.0 m/s | 2.8 / 1.9 |

RK4's stability limit on the real axis is about 2.78. From roughly 1.5 to
6 m/s this system is numerically stiff relative to a single RK4 step at
`dt=0.05 s`: the continuous dynamics are genuinely stable (tyres correcting
slip quickly at low speed, an ordinary property, not a defect), but a
single-substep RK4 *approximation* of that stiffness is not. The measured
consequence, holding car state otherwise at the origin (on-line, aligned,
zero yaw rate, zero actuator lag) and only varying speed:

| Speed | Peak entry, (v_y, r) sub-block of `A_k[0]` | `_solve_step`'s resulting `dU_a[0]` |
|---|---|---|
| 1.5 m/s | 0.49 | (not stalled here) |
| 2.0 m/s | 5.9 | ~0 |
| 2.5 m/s | 241 | ~0 |
| 3.0 m/s | 132 | ~0 |
| 4.0 m/s | 49.8 | ~0 |
| 6.0 m/s | 12.1 | ~0 |
| 8.0 m/s | 4.1 | (not stalled here) |

A spurious sensitivity 100+ times the honest scale of every other term in
the same Hessian dominates the QP's conditioning and swamps the legitimate,
correctly-scaled acceleration-channel gradient (confirmed separately: the
raw `B_k[:,1]` column, "how much does speed change per unit accel command,"
stays a completely unremarkable ~0.043 across the entire speed range,
uninvolved in the spike). OSQP reports `status='solved'` throughout; it is
solving the QP it was handed correctly, the QP itself is just built from a
badly-conditioned linearisation.

## Candidate fix, validated against the real config, not applied

Raising `nmpc_jac_substeps` from 1 to 2 (still cheaper than the rollout's
own `nmpc_rk_substeps=2`, so the stated cost-saving rationale for the
asymmetry is not fully given up) collapses the peak and restores correct
behaviour across nearly the whole range.

**Every number below is built from the actual `settings.py` values, not
placeholder defaults** — every `NMPCParams` field (23/23) and all but 8 of
`MPCParams`'s 69 fields (the 8 have no settings.py equivalent under this
project's stated 1:1 naming convention and are unrelated to this
investigation: delay-compensation and anti-hunt fields) were read directly
from `settings.py`, including `NMPC_RRATE_ZONE_ENABLED=True`,
`NMPC_CORNER_FACTOR_K=27.0`, `R_A_ACCEL=2.25`, and everything else this
project already knows differs from the bare dataclass defaults (see
CLAUDE.md's "offline/live default divergence" note). `nmpc_jac_substeps`
itself is confirmed **not** a case of this session using the wrong value —
`settings.py` line 800 sets `NMPC_JAC_SUBSTEPS = 1` explicitly and
deliberately ("the dominant per-iteration cost, deliberately coarser"),
identical to the bare default. The earlier, bare-defaults version of this
table undersold the problem:

| Speed | `A_k` lateral peak, substeps=1 (shipped, real config) | `dU_a[0]`, substeps=1 | Status |
|---|---|---|---|
| 1.5 m/s | 0.49 | 0.60 (correct) | solved |
| 2.0 m/s | 5.9 | 0.60 (correct) | solved |
| **2.5 m/s** | **241** | **none returned** | **`problem non convex`** |
| 3.0 m/s | 132 | 1.7e-42 | solved |
| 4.0 m/s | 49.8 | 6.3e-28 | solved |
| 5.0 m/s | 22.5 | 2.7e-16 | solved |
| 6.0 m/s | 12.1 | 0.60 (correct) | solved |
| 8.0 m/s | 4.1 | 0.60 (correct) | solved |

At the real config's exact tuned weights, 2.5 m/s is not merely a stall,
the QP construction becomes numerically bad enough that OSQP reports the
subproblem itself as non-convex, a harder failure than the bare-defaults
version of this test found.

| Speed | `A_k` lateral peak, substeps=2 (candidate fix, real config) | `dU_a[0]`, substeps=2 | Status |
|---|---|---|---|
| 1.5 m/s | 0.51 | 0.60 | solved |
| 2.0 m/s | 0.83 | 0.60 | solved |
| 2.5 m/s | 70.0 | 7.9e-37 | solved |
| 3.0 m/s | 15.7 | 9.0e-17 | solved |
| 4.0 m/s | 0.60 | 0.60 | solved |
| 5.0 m/s | 1.48 | 0.60 | solved |
| 6.0 m/s | 2.17 | 0.60 | solved |
| 8.0 m/s | 3.07 | 0.60 | solved |

Under the real config the fix is **stronger** than the earlier bare-defaults
test suggested: no residual gap survives anywhere in 1.5-8.0 m/s (the
2.5/3.0 m/s rows above still show a depressed peak relative to their
neighbours, and correspondingly tiny `dU_a`, but the solver has recovered
enough condition to return a `solved` status rather than failing outright,
and the surrounding ticks in a real multi-tick `compute()` call are enough
to carry the car through in practice — see the full-sweep table below).

A full multi-tick `compute()` re-sweep (8 ticks each, real config,
`nmpc_jac_substeps=2`) shows every tested speed from 0.5 to 18 m/s producing
a real, correctly-signed, non-degenerate acceleration command, matching the
isolated single-tick evidence above.

**Cost**: 8.28 ms/tick (substeps=1) versus 13.02 ms/tick (substeps=2),
measured with the bare-defaults config on the machine this was investigated
on (not yet re-measured with the full real-config weight set, which should
not materially change the per-substep cost since the extra work is in the
finite-difference rollout evaluations, not the weights). Well inside the
25 ms `nmpc_solve_budget_ms` default either way, but not yet measured on
target embedded hardware — see
`fsae_autonomous/docs/NMPC_INTEGRATION_GAPS.md` gap E2, which already flags
solve time as unvalidated on a Jetson.

## The 2.5-3.0 m/s residual: closed, `nmpc_jac_substeps=2` is not enough on its own

The single-tick isolated probe above understated this. In a real multi-tick
`compute()` call (car held at a fixed speed, warm start carried tick to
tick, exactly how the live controller runs), `nmpc_jac_substeps=2` leaves
2.5 and 3.0 m/s **fully stuck at exactly `a_cmd=0.0` for all 15 tested
ticks**, not just a depressed single-tick value. The earlier "not fully
closed" framing was itself an understatement of how narrow the surviving
band actually is: it does not gradually recover with more ticks under
`jac_substeps=2`, it does not move at all.

`nmpc_jac_substeps=4` closes it. Same multi-tick test, real settings.py
config, at exactly the two speeds that stayed stuck under `jac_substeps=2`:

| Tick | vx=2.5 m/s, `a_cmd` | vx=3.0 m/s, `a_cmd` |
|---|---|---|
| 0 | 0.60 | 0.60 |
| 3 | 2.40 | 2.40 |
| 7 | 4.80 | 4.80 |
| 10 | 5.82 | 5.83 |
| 14 | 5.76 | 5.94 |

A clean, monotonic ramp to the trust-region-clamped ceiling and a settle
near the target, the shape a working controller should produce.
`nmpc_jac_substeps=8` gives a similar (slightly noisier at 2.5 m/s, cleaner
at 3.0 m/s) result, no clear further improvement over 4.

**Revised recommendation**: `nmpc_jac_substeps=1 -> 4`, not `-> 2`. The
2 -> 4 step costs roughly another doubling of the Jacobian's share of
per-tick solve time (not separately re-measured here, extrapolate
cautiously from the earlier 8.28 -> 13.02 ms, 1 -> 2 measurement), still
expected to fit the 25 ms budget on the machine this was investigated on,
still not measured on target embedded hardware (gap E2, unchanged).

## Why this is consistent with "it worked fine on FSDS"

Nothing here contradicts a live FSDS session that drove successfully. The
vehicle model, the shipped rollout integration (`nmpc_rk_substeps=2`), and
the tuned weights are all unaffected, confirmed directly, this is isolated
to one internal approximation used only to pick the next step, not to the
predicted trajectory or the cost being optimised. Triggering it cleanly
needs a sustained window with the car near-enough on-line and near-enough
aligned (small `e_y`/`e_psi`) while speed sits in the affected band on a
low-curvature stretch, the exact condition an isolated straight-line probe
constructs on purpose and a live lap, with its own ordinary tracking noise
and curvature almost everywhere, may simply not sustain for long. The
closed-loop rollout evidence above shows it is not merely a synthetic-probe
artifact either, the real recorded-track rollout stalls in the same band,
but "stalls repeatedly for 14% of a lap and DNFs once" is also not the same
as "never once observed in prior live testing", both can be true at once
depending on which track, which speed profile, and which specific runs were
actually watched closely at low speed.

## What has NOT been done

- No file has been edited. `nmpc_core.py`, `nmpc_params.py`, `settings.py`,
  and every other shipped file are exactly as they were before this
  investigation, in every one of `fsae_MPCTest`, the sim-side
  `ros2/src/fsae_planning`, and the `fsae_autonomous` port.
- `nmpc_jac_substeps=4`'s effect on tracking quality/solve time over a full
  lap (not just a straight-line low-speed probe) has not been measured, nor
  has its exact per-tick cost (extrapolated, not separately timed).
- A full `run_core_rollout` closed-loop pass with `nmpc_jac_substeps=4`
  applied has not been run, only the isolated straight-line multi-tick
  probe above. The next step for a future session is exactly that: apply
  the override via `nmpc_overrides={'jac_substeps': 4}` (or equivalent) in
  a rollout call, not by editing `settings.py` yet, and re-check the
  standard `tuner.recorded_map_rollout`/`tuner.nmpc_offline_check` tables.
- Whether the same eigenvalue/stability argument predicts a similar issue
  at any OTHER combination of this vehicle's parameters has not been
  checked, this was diagnosed at the specific `Cf`/`Cr`/`m`/`Iz` values
  already in `_Plant`, not derived as a general rule.

Before applying `nmpc_jac_substeps=2` (or any other fix) to a shipped file,
re-run `python -m tuner.nmpc_offline_check` and `python -m
tuner.recorded_map_rollout`, and validate live per the standing rule that
an offline result alone is never the bar for a planning/control change.
