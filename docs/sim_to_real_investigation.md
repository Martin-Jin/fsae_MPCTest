# The sim-to-real gap: a full investigation log (2026-08-06)

**What this document is.** A chronological account of how the gap between
`fsae_MPCTest` and the car was tracked down — including every hypothesis that
turned out to be wrong, and *why* each one looked right at the time.

**Why it exists.** The commit history records what changed; it does not record
the reasoning, the dead ends, or the measurements that eliminated them. Several
of those dead ends were individually convincing (one produced an exact
numerical match and was still wrong). Anyone revisiting this — or tempted to
re-try a candidate that looks unexplored — needs the reasoning, not the diffs.

For the current state and what to do about it, read
[`planning_control_sync.md`](planning_control_sync.md) → "MECHANISM: a
dynamically-enforced lateral-acceleration ceiling". This file is the *history*.

---

## The bottom line, first

**FSDS enforces a sustained LATERAL-ACCELERATION ceiling of ~7.5 m/s².**
Below ~6 m/s the car never reaches it and delivers the commanded steering
angle exactly; above that the yaw response collapses. It is a *sustained*
ceiling, not a wall — the live car exceeds it on 9.8% of ticks (peak 12.34) in
short excursions. The offline plant modelled no such cap.

> Earlier revisions of this file described it as a **yaw-rate** cap at
> ~0.7 rad/s. That was the sweep's reading from a single capped speed; the
> step test showed lateral acceleration is what is held constant (spread 1.07×
> across speeds, vs 1.56× for yaw rate). Yaw rate and lateral acceleration are
> indistinguishable at one speed and diverge across a sweep.

It is **not** tyre grip, not the planner, not latency, and not the tuner —
every one measured and eliminated (§1–§7).

**The cap is necessary but NOT sufficient.** Modelling it moves every metric
toward the car, yet live still saturates 3× more often (21.1% vs 6.3%) and
carries 2× the heading error. Something else remains — see "Open / deferred".

---

## 0. The starting symptom

Tuned weights that scored well offline produced a visibly worse car: more
wobble, more steering reversals. Measured on the same recorded map
(`comp test map 3`) with identical gains:

| | offline sim | live car |
|---|---|---|
| steering saturation | 3.4% | **21.1%** |
| \|e_psi\| mean / p90 | 6.0° / 13.8° | **15.9° / 42.0°** |
| reversals/s | 0.83 | 1.62 |

The user's framing was the correct one and set the direction for everything
after:

> "Until this big gap is fully addressed, tuning will always be off as it tunes
> for a perfectly behaving MPC that can track the centre line with little to no
> wobble."

**Key early observation.** When the live car saturated steering it was pulling
only **4.14 m/s²** lateral at **5.74 m/s** — *slower* than its 8.03 m/s
average. It was not cornering hard and running out of grip. It was asking for
full lock while barely cornering. That single fact should have pointed at an
imposed limit much earlier than it did; instead it was read as evidence of a
bad reference.

---

## 1. Hypothesis: the objective function was mis-weighted

**Why it looked right.** The retuned car had *fewer* reversals (1.62/s vs
3.49/s) but far worse tracking — exactly what an objective over-weighted on
smoothness and lap time would produce. The step-5 restructure had set
`TIME_OBJECTIVE_WEIGHT = 1.0` against `QUALITY_WEIGHT = 0.35`, and anchored
time to a bound assuming 12 m/s² lateral grip.

**What was done.** Traced the incentive: saturating the steering cost the tuner
`steering_sat_ratio × 0.045 × 0.35` ≈ nothing, against a large time reward. The
tuner did exactly what it was told.

**Result.** Real, but **not the cause**. Rebalancing was deliberately deferred:
tuning against an unrepresentative simulator would just produce a differently
wrong answer. Still open — see "Deferred work" below.

**Lesson.** A plausible cause that is genuinely broken can still be the wrong
explanation for the symptom in front of you.

---

## 2. Hypothesis: the simulator lacks noise/latency the car has

**Why it looked right.** The offline sim is deterministic: uniform 50 ms ticks,
no pose noise, no dropped frames. The car has none of those luxuries.

**What was done.** Measured live loop timing (median 49.8 ms, p99 74 ms, 1.0%
of ticks >1.5× nominal — tight), then swept the candidates on the same map:

| configuration | sat % | reversals/s |
|---|---|---|
| baseline | 3.4 | 0.83 |
| + SLAM noise | 3.3 | 3.76 |
| + SLAM noise, 2× jitter | 4.5 | 4.88 |
| + SLAM noise + 2-step delay | 2.3 | 3.67 |
| **live** | **21.1** | **1.62** |

**Result. Eliminated, and informatively so.** Pose noise *overshoots* reversals
(3.76 vs 1.62 live) while barely moving saturation. It produces the wrong
*kind* of error. Extra delay made saturation *better*.

**Lesson.** Saturation and reversals are independent symptoms. Anything that
moves one without the other is not the explanation.

Latency instrumentation was added anyway (`pose_age_s`, `path_age_s`,
`n_delay`, `solve_ms`, `cmd_latency_ms` in the telemetry CSV) and remains
useful.

---

## 3. Hypothesis: the live planner's centreline is worse

**Why it looked right.** There is a *known, documented* planner defect
producing curvature spikes, and the live planner accumulates a cluttered cone
map while the offline `SimPlanner` rebuilds from a clean oracle. Sustained
heading-error episodes (median 0.47 s, up to 2.44 s, 96% of energy below 1 Hz)
looked exactly like a reference wrong for seconds at a time.

**What was done.** Reconstructed published-path curvature from the live path
log and compared against the offline planner on the same map.

| | live planner | offline planner |
|---|---|---|
| median peak curvature | R = 8.7 m | R = 10.4 m |
| p90 | R = 3.03 m | R = 3.08 m |
| **worst** | R = 1.26 m | **R = 0.16 m** |

**Result. Eliminated — and it inverted the expectation.** The *offline*
centreline is worse in the tail. Both stacks get a bad reference; the sim
survives it and the car does not.

**Lesson.** This reframed the question from "why is the car's reference worse?"
to "why does the same bad reference break only the car?" — which is what
eventually led to the plant.

A separate finding: the live planner republishes at **0.97 Hz** while the
controller runs at 20 Hz, so the car reuses one path for ~21 ticks. Throttling
the sim to match *improved* it, so this too was eliminated.

---

## 4. Hypothesis: the plant model has too much grip

**Why it looked right.** If the offline car corners better than FSDS, it makes
corners the real one cannot, and never saturates.

**What was done — and a mistake worth recording.** The first check compared
*mean* lateral acceleration (3.57 sim vs 3.76 live) and dismissed grip. **That
was the wrong test**: mean a_lat reflects how hard the controller chose to
corner, not what the plant can do. Corrected with a steady-state full-lock
test, which *did* show a real difference (×1.16 sim vs ×2.03 live understeer).

**Result.** Grip eliminated as the cause, but only after the corrected test.
Peak a_lat is 14.5 sim vs 12.3 live — roughly right, and if anything the sim is
slightly *more* capable.

**Lesson.** Choose statistics the controller cannot influence. Any closed-loop
average is contaminated by the controller's choices.

---

## 5. Hypothesis: tyre understeer — the near-miss

**Why it looked right.** Fitting the understeer gradient
`δ = L/R + K·a_lat` over 261 quasi-steady live points gave
`K = 0.00869 rad/(m/s²)`. Bisection found **μ = 1.455** reproduces it
**exactly**. An exact match on real data is about as convincing as a fit gets.

**Why it was wrong.** It failed two independent checks:

| check | fitted μ=1.455 | live |
|---|---|---|
| understeer gradient K | 0.00869 ✓ | 0.00869 |
| full-lock understeer @6 m/s | ×1.16 ✗ | **×2.03** |
| closed-loop saturation | 4.9% ✗ | **21.1%** |

Pushing μ lower caused DNFs with heading error *decreasing* — a different
failure mode from the car's.

**Result. Eliminated.** Two measurements of the same physical property
disagreeing means the model *structure* is wrong, not a parameter.

**Lesson — the most important one here.** The fit was parameterised on
**lateral acceleration**, but the real effect is organised by **speed**. On a
lap those are confounded, so a fit can nail one basis and be badly wrong on the
other. The tell was there and initially missed: a later joint fit returned
`K = −4.75 deg/g`, a *physically backwards negative* understeer gradient — the
signature of a model contorting to absorb an effect it has no term for.

> **This is why `CLAUDE.md` says do not "fix" the plant by scaling `mu`.**

---

## 6. Hypothesis: `MAX_STEER_RAD` command scaling

**Why it looked right.** `cmd.steering = -delta / MAX_STEER_RAD` with
`MAX_STEER_RAD = 25°`. If FSDS's real rack limit differs, every command is
scaled by a constant and the car under-turns at all speeds. It would explain
the ×2.03 full-lock deficit and the modest gradient with one mechanism — and it
is exactly the class of bug a previous telemetry-units error had hidden.

**What was done.** Inverted the kinematic bicycle on logged `yaw_rate` and
`delta_cmd` across two independent lap logs.

**Result. Refuted.** A constant scale predicts a *flat* `s = δ_ach/δ_cmd`
across speed. Measured `s` collapses with speed: **0.85 → 0.42 → 0.31 → 0.28**
across speed quartiles. Model comparison put constant-scale (R² 0.655) well
behind an understeer law (0.759).

Also eliminated in the same pass:

| candidate | verdict |
|---|---|
| actuator lag | No — `s` does not degrade as command rate rises |
| telemetry error | No — `yaw_rate` matches `d(car_yaw)/dt` at slope 0.95, corr 0.95 |
| tyre front/rear balance | No — reaching live `K_us` needs `C_f` at **10%** of physical (43 N/deg) |

**Lesson.** Two competing models can only be separated by the *shape* of the
relationship, never by a single aggregate number.

---

## 7. Hypothesis: longitudinal scaling (user-prompted)

**Why it looked right.** The user asked whether max throttle / accel / braking
in the offline model actually match FSDS.

**What was done.** Regressed achieved against commanded acceleration.

| | value |
|---|---|
| `a_actual / a_cmd` slope | **1.14–1.18** (slightly *over*-delivers) |
| peak accel | 11.2–12.4 m/s² (model 12.0) |
| peak braking | −12.2 to −13.0 m/s² (model −9.0) |
| throttle saturation | **0.00%** of the run |

**Result. Eliminated as the cause — but it found a real defect.**
`fsds_bridge` **discards the MPC's `a_cmd`** entirely and re-derives throttle
with its own P-controller (`KP_THROTTLE = 0.06`). Offline, `a_cmd` drives the
plant directly, so the live longitudinal loop has a proportional lag stage the
sim does not model. Cannot cause a yaw deficit; still a genuine divergence.
**Not yet acted on.**

**Lesson.** Asking "is *this* scaled correctly?" about each interface is cheap
and found a real bug the yaw investigation would never have touched.

---

## 8. The measurement that worked: open-loop system-ID

**Why the previous approach was stuck.** Every candidate had been tested
against *lap logs*, where speed, steering and lateral load are all confounded,
and the MPC is always reacting. A yaw cap and genuine understeer are
indistinguishable in that data.

**What was built.**

- `steering_sysid.py` (in `fsae_planning`) — bypasses the MPC, publishes fixed
  steering at fixed speeds directly to `/fsds/control_command`.
- `run_steering_sysid.sh` (FSDS repo `ros2/`) — one-command harness.
- `tuner/steering_sysid_analysis.py` — fits five candidate models to achieved
  yaw rate (a *common target*, so R² is comparable) and reports the margin to
  the runner-up.

**Validation before trusting it.** Synthetic logs with each mechanism injected;
all five identified correctly and parameters recovered exactly (scale 0.500,
c 0.0600, K_us 0.0400, a_lat_max 12.0). Then end-to-end through the real node
against a stub plant with known `K_us = 0.04` — recovered `0.0400`.

**Three bugs caught before the real run:**

1. **First verdict logic was wrong.** It keyed on "does `s` fall with speed",
   which returned "speed-scaled rack" for *three different* injected
   mechanisms. It would have confidently reported the wrong answer. Falling `s`
   is not diagnostic; the discriminator is functional form.
2. **Default speed range too narrow.** Over 4–10 m/s, speed-scaled rack and
   understeer sat within **0.004 R²** — indistinguishable. Widened to 3–14 m/s
   (margin 0.018–0.054) and made the analyser *warn* rather than guess.
3. **Throttle gain copied from `fsds_bridge` could not launch the car.**
   `KP_THROTTLE = 0.06` gives 0.18 throttle at 3 m/s — not enough to break
   static friction from rest (the sim sat at 0 m/s; the user reported
   `Accel: 0.180000`). Replaced with PI + launch floor. A pure-P law also
   cannot hold speed without steady-state error, which would bias every
   measurement.

**Then the harness drove into a wall**, exposing three geometry faults:

1. Straight-line settling carries the car ~120 m downrange per point at
   14 m/s (measured: 103 m of drift before impact). Now settles speed *while
   already turning*, so it orbits.
2. A geofence alone is insufficient — fired ~126×/sweep, 16/20 points in
   40 min.
3. Some (speed, steering) pairs cannot be driven in a bounded area at all:
   14 m/s at 0.5 steering is an **~86 m** orbit. The estimate that let those
   through assumed a near-neutral car (K=0.005) and predicted 23 m; now uses a
   deliberately pessimistic `K_US_ESTIMATE = 0.05`.

Fixed: **16/16 points, 2.6 min, zero geofence triggers.**

---

## 9. The result

Full clean sweep — `fsae_logs/steering_sysid_1786014330.csv`.

| speed | commanded | achieved | `s` |
|---|---|---|---|
| 3 m/s | 25° | 25.3° | **1.01** |
| 5 m/s | 25° | 24.9° | **1.00** |
| **8 m/s** | 25° | **8.5°** | **0.34** |
| 11 m/s | 25° | 5.8° | 0.23 |
| 14 m/s | 25° | 4.1° | 0.17 |

A **cliff between 5 and 8 m/s**, not a gradual curve.

| model | R² |
|---|---|
| **grip saturation** (a_lat ceiling) | **0.987** |
| understeer (v²) | 0.898 |
| speed-scaled rack | 0.893 |
| constant scale | 0.617 |
| neutral | −1.226 |

First decisive separation in the whole investigation (margin 0.089).

**The cap is not tyres.** Fitted ceiling **7.0 m/s²** vs **12.3 m/s²** the same
car reaches on a lap and 14.5 offline. A tyre limit is speed-independent. And
lateral acceleration vs steering goes *flat* once it engages:

| v | 0.50 | 0.65 | 0.80 | 1.00 |
|---|---|---|---|---|
| 5 m/s | 4.37 | 5.28 | 6.30 | 7.48 |
| **8 m/s** | **6.27** | **5.95** | **5.72** | **6.09** |

At 8 m/s **more steering produces less cornering**.

This closes the loop on the original symptom: MPC plans a corner → FSDS clamps
the yaw → heading error builds → controller demands more steer → hits the 25°
stop → 21% saturation.

*(A verdict-text bug was found here too: the analyser compared "near the
ceiling" against a hardcoded `GRIP_CEILING = 12.0`, so a fitted ceiling of 7.0
printed "TYRE SATURATION … 0/16 points near the ceiling". Fit was right,
explanation was checking the wrong number. Fixed.)*

---

## 10. Modelling the cap (and two false starts)

Implemented in `model/vehicle_physics.py` as a restoring yaw moment with a
first-order lag — new state `IDX_ALAT_LIM`, `N_STATES` 24 → 25. A clip was
rejected: it reproduces steady state but removes the turn-in transient, which
is exactly what the MPC reacts to.

    alat_ceiling         7.5 m/s2    measured settled a_lat
    alat_ceiling_gain    700         fitted to measured PEAK a_lat
    alat_ceiling_tau     0.25 s      fast enough to cap a real corner entry
    alat_ceiling_enabled True        models FSDS, NOT the physical car

**False start 1 — fitting `tau` to the overshoot.** The step test measured ~30%
overshoot, so `tau = 1.0 s` was chosen to reproduce it. It did, beautifully —
and then **DNF'd the car at 6.3 s** on a real lap. Corners arrive in ~0.4 s, so
a term taking ~1 s to build does nothing during turn-in, lets the car run wide,
and engages only once it is already off line. *A ceiling that acts too late is
worse than no ceiling.* `tau = 0.25` matches peak a_lat instead and acts in
time.

**False start 2 — fitting `gain` to the settled value.** `gain = 3000` matched
the settled 7.80/7.29 almost exactly, and thereby enforced 7.5 as a
near-absolute limit. But 7.5 is a *sustained* ceiling: the live car exceeds it
on 9.8% of ticks, peaking at 12.34, in bursts of median 0.05 s. The stiff gain
was **stricter than the real car** and pushed `e_y` off track. Refitting to the
measured *peak* (700) reproduces the excursions and completes the lap.

**And a wrong diagnosis in between.** When the stiff-gain version DNF'd, I
blamed the recorded track's speed profile — claiming it was ~50% faster than
the car. It is not. That compared the stored oracle profile `V` (mean 12.08,
used only to size the step budget, never the runtime target) against the live
car's *achieved* speed (8.03). Like for like, achieved speeds differ by 2%
(8.20 vs 8.03), and over the first 6 s the live car is *faster*. Code parity
was verified too: both stacks run `curvature_speed` with the same constants and
both apply `tracking_error_speed_gate` and the rise-rate limit at runtime.

Same failure mode as the μ=1.455 fit: **check that the two numbers being
compared are the same quantity** before drawing a conclusion from their ratio.

**Result** — same map, same tuned gains:

| | before | after | live |
|---|---|---|---|
| steering saturation | 4.4% | **6.3%** | **21.1%** |
| reversals/s | 0.84 | 0.82 | 1.62 |
| \|e_psi\| mean / p90 | 6.3 / 14.2 | **7.3 / 19.2** | **15.9 / 42.0** |
| a_lat max | 14.06 | **10.53** | 12.34 |
| a_lat > 7.5 | 14.2% | 14.2% | 9.8% |

Every metric moves the right way and peak lateral is now realistic — but the
gap is only partly closed.

---

## 11. Also verified: cone geometry is accurate

Checked because a track-geometry mismatch would corrupt every planner
comparison. Track width is **exactly 3.50 m** with zero variance, matching
`TRACK_HALF_WIDTH = 1.75`; cone spacing is median ~4.0 m with 98–99% inside the
5 m FS limit. Not a source of the gap.

> Measure spacing **along the path**, not down the array. Cones are stored in
> recording order, so differencing consecutive entries reports phantom gaps up
> to 43.7 m. Project onto the nearest centreline index and sort first.

---

## What generalises

1. **Closed-loop data cannot separate plant faults from controller faults.**
   Six hypotheses died slowly against lap logs; one open-loop test settled it
   in three minutes. Build the open-loop rig sooner.
2. **An exact numerical match is not validation.** μ=1.455 matched the
   understeer gradient perfectly and was wrong. Always validate a fit against a
   measurement it was *not* fitted to.
3. **Check the parameterisation basis.** `a_lat` vs `v²` are confounded on a
   lap. A negative fitted coefficient for a physically positive quantity means
   the model is absorbing an effect it cannot represent.
4. **Compare models on a common target.** Scoring against `δ_achieved` inflates
   R² for everything, because both sides scale with steering angle. Score
   against achieved yaw rate.
5. **Report the margin, not just the winner.** Near-degenerate models must
   warn, not guess.
6. **Validate the diagnostic before trusting it.** Synthetic injection caught a
   verdict rule that was wrong for 3 of 5 mechanisms.
7. **Statistics the controller can influence are not plant measurements.**
   Mean a_lat measures the controller; steady-state full lock measures the
   plant.

---

## Open / deferred

| item | status |
|---|---|
| **Model the yaw cap offline** | **Done** — `alat_ceiling*` in `model/vehicle_physics.py`. Moves every metric toward the car (saturation 4.4→6.3%, a_lat max 14.06→10.53) but closes only part of the gap; live is still 21.1% saturation |
| Remaining saturation gap | **Open** — the cap was necessary but not sufficient. Needs its own investigation |
| Identify the exact FSDS mechanism | **Done** — step test run (§9–10): a *dynamically enforced lateral-acceleration* ceiling. Both signatures present: different steering angles settle to the same response (a cap), yet yaw overshoots ~30% first (not a static clip) |
| Ceiling decay time constant | **Not reliably measured.** Fitting peak→settled gave median 0.08 s over a 0.04–1.06 s range (rise a cleaner 0.40 s). `alat_ceiling_tau = 0.25` was chosen to match peak a_lat and act in time, not from this fit. Needs a longer `step_s` and more repeats |
| Speed profile | **Closed, not a discrepancy** — see the correction below. Achieved speeds differ by 2% |
| Objective rebalancing | Deferred (§1) — `QUALITY_WEIGHT` 0.35 → ~0.8, saturation as near-constraint |
| Step 4: held-out tracks | 5 of 10 tracks unused |
| Live scorer reports `13.0` | Every live run scores `CONSTRAINT_FLOOR + DNF_PENALTY` — the car has no known path end |
| `fsds_bridge` discards `a_cmd` | §7 — real divergence, unmodelled |
| Planner centreline defect | Pre-existing, documented; controller carries workarounds |

**Do not retune yet.** The cap is now modelled, which makes the sim materially
better, but it closes only part of the gap: live still saturates 3x more often
(21.1% vs 6.3%) and carries 2x the heading error. A good offline score has
already produced a car saturating steering 21% of the time.

### A correction worth keeping (2026-08-06)

While chasing the DNF that appeared after modelling the cap, I claimed the
recorded track's speed profile was ~50% faster than the car and that this was a
second discrepancy. **Both claims were wrong.** The comparison put the stored
oracle profile (12.08 m/s, used only to size the step budget) against the live
car's achieved speed (8.03 m/s). Compared correctly, achieved speeds differ by
2% (8.20 vs 8.03), and the live car is actually *faster* over the first 6 s.

The real cause was my own ceiling gain being too stiff — enforcing 7.5 m/s2 as
an absolute limit when the live car exceeds it on 9.8% of ticks. The lesson is
the same one as the mu=1.455 fit: **check that the two numbers you are
comparing are the same quantity** before concluding anything from their ratio.
