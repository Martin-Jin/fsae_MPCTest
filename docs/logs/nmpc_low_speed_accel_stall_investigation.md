# NMPC stalls at 2.5-6 m/s, confirmed in real FSDS telemetry: root cause found, one candidate fix validated, not yet applied

The shipped NMPC (`use_nmpc=True`, default weights) commands essentially zero
acceleration whenever car speed sits in roughly 2.5-6 m/s on a low-curvature
path, regardless of how large the speed error is. Root cause is a numerical
instability in one specific internal approximation (`nmpc_jac_substeps=1`),
not the vehicle model, not the cost weights, and not the shipped rollout
integration. Raising `nmpc_jac_substeps` to 4 (2 is not enough, see below)
fixes the whole affected range and closes it in a full closed-loop lap too
(DNF eliminated, tracking error improves, the stalled-tick fraction drops
from 14% to 3%), but roughly doubles mean per-tick solve time (9.56 -> 18.63 ms)
with a measured p95 close to, and a max past, the 25 ms `nmpc_solve_budget_ms`
default, and more than doubles steering saturation incidence (5.0% -> 11.0%,
though at the same mean speed in both cases, see below for why that reads as
a consequence of driving through the affected band far more often rather
than a new defect). **Not applied to any shipped file** as of this writing;
this document is the findings, not a changelog entry.

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

## Confirmed present in real FSDS telemetry, not just the offline plant

`fsae_logs/mpc_standalone_control_20260905-213630.csv`, a genuine live FSDS
run (`use_nmpc=1`, `nmpc_jac_substeps=1`, the unfixed shipped default,
`path_map_path=centerline.csv`, the same track and the same path source
as every offline number above) shows the identical pattern:

| | Offline rollout (`comp_test_map_3`, `jac_substeps=1`) | Live FSDS run, same track and path source |
|---|---|---|
| correlation(v, \|e_y\|) over the whole run | -0.655 | **-0.601** |
| mean \|e_y\| inside the 2.5-6.0 m/s band | (not separately isolated) | **0.779 m** |
| mean \|e_y\| outside that band | (not separately isolated) | **0.237 m** (3.3x smaller) |
| worst \|e_y\| excursions, speed at each | 1.0-5.4 m/s (see table above) | **4.6-5.6 m/s** |

This directly answers the "is it cutting corners" observation: the worst
tracking-error excursions on a real FSDS lap, run against the exact
`centerline.csv` this whole investigation used offline, sit at 4.6-5.6 m/s,
inside the same band this document is about, and the car's mean lateral
error more than triples the moment it enters that band versus outside it.
Not a synthetic-plant artifact, and not explained by a different path
source (`centerline.csv` and this investigation's offline reconstruction
are numerically identical, confirmed by nearest-point comparison,
sub-millimetre agreement). This run used the unfixed shipped config, so it
shows the residual at full severity, not the reduced version measured
under `jac_substeps=4` above.

## All six available live logs show the same pattern, but not the same severity

Every `mpc_standalone_control_*.csv` in `fsae_logs/` (six total, spanning
2026-08-24 to 2026-09-05, all `use_nmpc=1`, all `nmpc_jac_substeps=1`, all
on `comp_test_map_3` against `centerline.csv`) shows worse tracking inside
the 2.5-6.0 m/s band than outside it:

| Run | \|e_y\| ratio, in-band vs out | `nmpc_status` mean, in-band | `nmpc_status` mean, out-of-band |
|---|---|---|---|
| 2026-08-24 17:41 | 1.72x | 1.000 | 1.000 |
| 2026-08-24 17:45 | 2.88x | 1.000 | 1.000 |
| 2026-09-01 08:18 | 2.73x | 0.651 | 0.998 |
| 2026-09-05 08:13 | 1.94x | 0.952 | 0.993 |
| 2026-09-05 15:41 | 1.40x | 0.812 | 0.994 |
| 2026-09-05 21:36 | 3.29x | **0.504** | 0.997 |

`nmpc_status` is `1.0` only when the SQP's own OSQP call reports a status
string starting with "solved", `0.0` for anything else (a budget bailout,
a rejected backtracking step, or an outright solver failure like the
"problem non convex" this investigation measured offline at exactly
2.5 m/s). A mean of 0.504 means the solver genuinely fails to solve
cleanly on roughly HALF of every tick spent in this band, on a real car.
This is the direct, mechanism-level link between the live symptom and the
offline diagnosis, not merely a correlation: the car isn't just tracking
worse in slower corners in some generic sense, its own solver is
reporting trouble at exactly the band this investigation's `A_k` Jacobian
measurements identified.

**Not every run shows this.** The two oldest runs (2026-08-24) have worse
`|e_y|` ratios than several of the newer ones, yet their solver status is
a clean 1.000 in-band, identical to outside it. Diffing each run's own
recorded config (every log header carries its full `mpc_params`/
`nmpc_params` dump) explains why: the two old runs carry an explicit
`nmpc_r_rate_delta=2.8` override, while every run from 2026-09-01 onward
carries `nmpc_r_rate_delta=-1.0` ("inherit the base `r_rate_delta`", which
is `52.5` in both eras, unchanged), an 18.75x higher effective
steering-rate weight for the NMPC specifically. The newer runs also carry
`nmpc_rrate_zone_enabled=True` (was `False`), `nmpc_corner_factor_k=27.0`
(was inherited at a much lower base value), and `nmpc_rjerk_delta=150.0`
(was `0.0`, off). **All four of these match `settings.py` exactly as it
stands today** (confirmed directly), so every run from 2026-09-01 onward
is the current, actually-shipped configuration, not a stale one, and it is
specifically this configuration where the solver genuinely fails in the
band rather than merely tracking worse.

**Tested directly and NOT confirmed, correcting the paragraph that stood
here previously.** The paragraph originally here claimed the heavier
steering-rate weight and the corner-dependent gain schedule likely explain
the split by worsening the Hessian's conditioning. Reconstructing both the
old and the new `MPCParams`/`NMPCParams` exactly (the same field-by-field
`settings.py` derivation used throughout this document) and comparing them
directly refutes that claim as stated:

- `A_k` itself is byte-identical between the two configs at every speed
  tested (2.0-6.0 m/s, straight and mid-corner states alike). This is
  expected on reflection, not a surprise once checked: `A_k`/`B_k` are pure
  plant-dynamics sensitivities from `_jacobians()`, they do not depend on
  any cost weight at all, only `Hess`/`grad` do.
- `_solve_step`'s returned **status is also identical** between configs at
  every single-tick state tested, including the exact `vx=2.5` state that
  produces `problem non convex` under the shipped `jac_substeps=1`, under
  both the old and the new weights.
- A 40-tick warm-started sequence approaching a corner at a constant low
  speed (crudely forward-integrated, not the real plant, but enough to
  exercise the SQP's own warm start across many ticks) shows **zero**
  solver failures under either config.

So the specific causal mechanism proposed (heavier weights directly
worsening this Hessian's conditioning) is not supported by direct testing.

**A second candidate explanation was checked and also ruled out.** Every
in-band failure across the four "newer" runs clusters at one of two
arc-lengths: the launch (`nmpc_s0` near 0, expected, this is the same
low-speed-from-a-stop condition this whole document is about) and a
specific corner around `nmpc_s0` = 185-195 m, recurring in two independent
runs. This looked like it might simply be "the two clean old runs never
drove far enough to reach that corner", since they are also the two
shortest runs (489 and 1247 ticks). Checked directly: **both old runs DO
pass through that exact arc-length range**, at broadly similar speeds to
the runs that fail there (mostly 5-9 m/s, versus 3-6 m/s in the failing
runs, an overlapping range), and both show a clean `nmpc_status=1.0` the
whole way through.

**The cause of the config-era split is genuinely unresolved.** Both
proposed explanations (heavier weights, never having reached the hard
corner) are now checked and rejected. What is confirmed: the split is real
(four of six runs show it, reliably, at the same track location across
independent runs), it is not an artifact of which weights were loaded, and
it is not an artifact of run length. What remains unknown: whether it's a
path-dependent/chaotic sensitivity (small differences in SLAM noise, pose
history, or warm-start state accumulated earlier in each specific run,
tipping an already-marginal solve one way or the other at that corner) or
a genuine, not-yet-diffed change to something outside `mpc_params.py`/
`nmpc_params.py` between 2026-08-24 and 2026-09-01 (the plant model,
delay/pose-age handling, perception/SLAM noise settings). Not investigated
further here.

## The recurring failure corner (s=165-196 m) is the single hardest corner on the lap

Reported independently, before this section was checked: a driver watching
the car described the problem as happening "just like one place basically,
where there is a sudden turn." That matches this document's own arc-length
finding directly, the `nmpc_s0`=185-195 m failure cluster recorded in two
independent live logs falls inside a single corner segment (`s`=165.2 to
196.4 m).

Measuring every corner on the lap the same way (`|kappa|` from finite-
differenced heading, and each corner's own minimum speed target):

| Corner (arc length) | Radius | Speed target minimum |
|---|---|---|
| 41-51 m | 5.6 m | 6.55 m/s |
| 113-128 m | 7.5 m | 7.07 m/s |
| 132-154 m | 6.2 m | 6.59 m/s |
| **165-196 m (the failure corner)** | **4.7 m** | **6.00 m/s** |
| 259-270 m | 6.8 m | 6.82 m/s |
| 276-303 m | 9.3 m | 7.60 m/s |
| 338-363 m | 7.6 m | 6.97 m/s |
| 399-414 m | 9.5 m | 7.76 m/s |
| 430-444 m | 10.9 m | 8.13 m/s |

The failure corner is both the **tightest radius on the entire lap** (4.7 m,
the next-tightest is 5.6 m) and carries the **lowest target speed of any
corner** (6.00 m/s). No other corner combines both. That is the plain-
English reason it alone triggers the failure repeatedly while none of the
other eight do: it is the one place on the track that forces the car to be
at low speed and turning hard at the same time, exactly the combined
condition (small `v_x`, genuinely nonzero curvature and yaw rate together)
that stresses the `A_k` Jacobian spike this document's root cause section
describes. A corner with a higher speed target stays above the fragile
band even while cornering hard; a gentler corner doesn't demand enough
yaw rate to matter even at low speed.

Checked and NOT confirmed: whether this corner's curvature ramps up more
*suddenly* than the others (a steeper `d(kappa)/ds` at corner entry). By
that specific measure it is not the steepest (0.0123 rad/m/m, versus up to
0.033 for two gentler corners), though this finite-difference measure is
inherently noisy (see this document's own reliance on the project's
already-documented centreline-curvature-spike caveat) and should not be
read as ruling suddenness out, only as not confirming it as the
distinguishing factor. Tightness combined with a low speed target is the
distinguishing factor that was actually measured.

## Why some runs fail there and others don't: dwell time, not a hidden config or code cause

Three candidate explanations for the config-era split (identified above)
were tested directly and rejected in turn: heavier cost weights worsening
the Hessian conditioning (`A_k` and `_solve_step`'s status are byte-
identical between old and new weights at every state tested), the clean
runs simply never reaching the corner (both do, at similar speed), and a
genuine code difference found while diffing the two eras' full recorded
config, a rise-limiter seeding fix
(`if self._v_des_prev is None: self._v_des_prev = self._car_speed`, absent
in the old code, present in the new) landing in the same commit as the
weight change. Simulating both the old (unseeded, skips the clamp
entirely on first use after a reset) and the new (seeded from actual
speed) behaviour side by side, at this exact corner's geometry, in both a
braking-in and an accelerating-out scenario, produced byte-identical
`nmpc_status` sequences in every configuration tried. None of these three
explain the split.

**Looking directly at the raw telemetry instead of guessing a mechanism
finds the actual difference.** In the one clean old run and the one
failing new run, `v_actual` and `v_desired` are nearly identical averaged
over the WHOLE lap (mean 10.07 vs 10.06 m/s, fraction of ticks under
5.0 m/s 5.9% vs 5.4%), ruling out "the newer run just drives more
conservatively overall". But at THIS corner specifically, the two runs'
braking behaviour differs concretely: the old run's passes through
`nmpc_s0`=180-200 m mostly stay at 6-14 m/s, with one brief dip to
5.02 m/s that recovers within a tick or two; the new run's single pass
shows a continuous, monotonic deceleration from 6.4 down to 4.2 m/s
sustained over 20+ consecutive ticks (roughly a second of real time), and
`nmpc_status` flips to `0.0` right as that sustained dwell crosses into
the low-4s.

The pre-existing `jac_substeps=1` fragility (root cause section above) is
a property of a single tick's linearisation at a given speed, it does not
itself require dwelling there, but a solve that is merely stalled or
slightly degraded for one or two ticks can still look "solved" by luck
(the warm start carrying over is close enough, or the backtracking line
search finds *something* to accept) in a way that a MANY-tick sustained
dwell in the same fragile band cannot: more consecutive fragile ticks
means more chances for the SQP's own internal state (warm start, trust
region) to wander into a genuinely infeasible or non-convex corner of the
subproblem rather than being carried past it by momentum from a
still-healthy adjacent tick. Braking hard enough into this specific corner
to linger at 4-5 m/s for a second or more, rather than clipping through it
at 6+ m/s, is what turns the underlying fragility from a near-miss into an
observed failure. Why some approaches to this corner brake harder than
others (perception noise, a slightly different line taken lap to lap, or
genuine closed-loop sensitivity to small differences) is not investigated
further here, ordinary run-to-run variation in a chaotic closed loop is a
sufficient explanation and no further hidden cause needs to be assumed.

## Revised: this is not merely consistent with a successful FSDS session, it is directly visible in one

An earlier version of this section argued the effect might simply never
surface in live testing (a live lap's ordinary tracking noise and curvature
breaking the exact-zero conditions an isolated probe constructs on
purpose). The real FSDS telemetry above rules that argument out: the same
3x jump in mean `|e_y|` on entering the 2.5-6.0 m/s band is directly
present in an actual, very recent live run.

What is still true, and still explains "a live session can look fine
overall" without contradicting the finding: the vehicle model, the shipped
rollout integration (`nmpc_rk_substeps=2`), and the tuned weights are all
unaffected, confirmed directly, this is isolated to one internal
approximation used only to pick the next step, not to the predicted
trajectory or the cost being optimised, so most of the lap (everywhere
outside the affected band) is genuinely unaffected and can look completely
normal. A lap that spends most of its time above 6 m/s, or that only
briefly touches the band while already carrying speed through it rather
than trying to accelerate from a stop inside it, would show this as a
handful of slightly-wide corners rather than an obvious failure, exactly
matching "cutting corners slightly sometimes" rather than a dramatic,
unmissable one.

## What has NOT been done

- No file has been edited. `nmpc_core.py`, `nmpc_params.py`, `settings.py`,
  and every other shipped file are exactly as they were before this
  investigation, in every one of `fsae_MPCTest`, the sim-side
  `ros2/src/fsae_planning`, and the `fsae_autonomous` port.
- `nmpc_jac_substeps=4`'s effect on tracking quality/solve time over a full
  lap (not just a straight-line low-speed probe) has not been measured, nor
  has its exact per-tick cost (extrapolated, not separately timed).
## Full closed-loop result on the recorded track, `nmpc_jac_substeps=1` vs `4`

`jac_substeps` is not actually wired through `run_core_rollout`'s
`nmpc_overrides` dict (unlike `q_e_y`, `r_delta`, and the other weight
fields), it is read as a plain module-level name
(`jac_substeps=NMPC_JAC_SUBSTEPS`, `sim/rollout_core.py`), bound from
`settings.py` at import time. Testing an override without editing
`settings.py` means reassigning that name directly on the already-imported
module (`rollout_core.NMPC_JAC_SUBSTEPS = 4`), not passing it through
`nmpc_overrides`, which this field does not support.

Full lap, `comp_test_map_3`, real settings.py weights, `continue_after_dnf=True`:

| | `jac_substeps=1` (shipped) | `jac_substeps=4` (candidate) |
|---|---|---|
| DNF | **True** (real off-track excursion) | **False** |
| Reached end | True | True |
| Lap length | 1389 ticks | 1146 ticks (faster) |
| \|e_y\| mean / p90 / max | 0.592 / 1.390 / 2.760 | 0.488 / 1.126 / 2.029 (all better) |
| \|e_psi\| mean | 7.68 deg | 7.63 deg (about the same) |
| Ticks stuck in the 2.0-2.5 m/s band | 199 / 1389 (14.3%) | 31 / 1146 (2.7%) |
| Solve time mean / p95 / max | 9.56 / 14.05 / 32.99 ms | 18.63 / 23.61 / 38.86 ms |
| Steering saturation | 5.0% (70 ticks) | 11.0% (126 ticks) |
| Mean speed during saturated ticks | 3.21 m/s | 3.23 m/s (same) |
| Mean speed over the whole lap | 6.52 m/s | 7.89 m/s |

The DNF is gone, tracking error improves across the board, and the dead
zone shrinks from 14% of the lap to 3%. This is a real, full-lap
confirmation, not just the isolated straight-line probe above.

**Two real costs, not glossed over:**

- **Solve time roughly doubles** (9.56 -> 18.63 ms mean), and its p95
  (23.61 ms) sits close to the 25 ms `nmpc_solve_budget_ms` default, with a
  measured max (38.86 ms) that exceeds it. `nmpc_solve_budget_ms` is a soft
  per-SQP-iteration check that ships the best feasible iterate rather than
  hard-failing, so an occasional overrun does not crash anything, but this
  is a real, not-yet-explained cost. Neither figure has been measured on
  target embedded hardware (gap E2 in `fsae_autonomous/docs/NMPC_INTEGRATION_GAPS.md`,
  unchanged).
- **Steering saturation more than doubles** (5.0% to 11.0% of ticks). Not
  an unexplained regression: saturated ticks happen at essentially the
  same mean speed in both runs (3.21 vs 3.23 m/s), squarely inside the
  residual band this section is about. With the fix, the car actually
  accelerates through that band on every corner exit instead of getting
  stuck there once and staying stuck, so it passes through the same
  saturation-prone speed range far more often over the course of a full
  lap (mean lap speed rises from 6.52 to 7.89 m/s, consistent with driving
  a genuinely faster, more active lap rather than a differently-broken
  one). Not independently confirmed beyond this correlation.

**Still not done**: no per-corner or per-speed-bin breakdown of exactly
which saturated ticks are "new" versus "the same corner, encountered
again because the car didn't stall out of the lap the way it used to."
Not measured on target embedded hardware. Not applied to any shipped file.
- Whether the same eigenvalue/stability argument predicts a similar issue
  at any OTHER combination of this vehicle's parameters has not been
  checked, this was diagnosed at the specific `Cf`/`Cr`/`m`/`Iz` values
  already in `_Plant`, not derived as a general rule.

Before applying `nmpc_jac_substeps=2` (or any other fix) to a shipped file,
re-run `python -m tuner.nmpc_offline_check` and `python -m
tuner.recorded_map_rollout`, and validate live per the standing rule that
an offline result alone is never the bar for a planning/control change.
