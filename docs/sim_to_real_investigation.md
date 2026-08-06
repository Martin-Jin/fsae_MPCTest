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

> **Update 2026-08-07 (§12).** The cap's *law* was structurally wrong — a
> proportional term, whose equilibrium must sit above its setpoint — and is now
> an integral term that matches settled, peak and sustained cornering at once.
> That fixed a real 13–35% surplus in sustained cornering and brought
> time-above-ceiling from ×1.45 to ×0.91 of live. It moved saturation by
> **0.4 points** (6.3 → 6.7% against live's 21.1%).
>
> §12 then eliminated, by measurement, both cornering capability and the
> `fsds_bridge` `a_cmd` divergence as causes of the residual gap, and narrowed
> it to a single statistic: the car enters the high-heading-error state
> **2.6× more often per second**, while behaving near-identically once in it.

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

## 12. The ceiling's LAW was wrong, and what that did (and did not) fix

*(2026-08-07 — chasing the residual saturation gap left by §10.)*

### 12.1 The tell was already in the published table

| | before ceiling | after ceiling | live |
|---|---|---|---|
| a_lat max | 14.06 | 10.53 | 12.34 |
| **a_lat > 7.5** | **14.2%** | **14.2%** | **9.8%** |

Adding a "sustained 7.5 m/s² ceiling" left the fraction of time spent above
7.5 m/s² **completely unchanged**, and higher than the live car's. Live also has
a *higher* peak (12.34 vs 10.53) but spends *less* time above the ceiling. That
pairing — lower peak, more time above — is the signature of a model that
**settles above** the ceiling while the car only **visits** above it.

### 12.2 Why no gain could work: a proportional law has a steady-state offset

The term was `M_z -= sign(r) · alat_lim · gain` where `alat_lim` lags toward
`excess = |a_lat| − ceiling`. At steady state `alat_lim = excess`, so it is a
pure **proportional** controller on the excess — and a P controller needs a
finite error to produce any output. Its equilibrium therefore *must* sit above
the setpoint. Measuring the trade-off makes it explicit (capped step trials,
measured settled 7.68, peak 10.42):

| gain | settled | err | peak | err |
|---|---|---|---|---|
| 300 | 9.51 | +1.83 | 10.84 | +0.42 |
| **700** (shipped) | **8.65** | **+0.97** | **10.11** | −0.31 |
| 3000 | 7.83 | +0.15 | 8.74 | −1.68 |
| 6000 | 7.67 | −0.01 | 7.87 | −2.55 |

**No gain fits both.** The two documented "false starts" in §10 were not two bad
guesses — they were the two ends of one structural flaw. Fitting the peak left
sustained cornering 13% high; fitting the settled value flattened the excursions
and DNF'd the lap.

### 12.3 The fix: integrate the excess

    err      = |a_lat| - ceiling            # SIGNED
    alat_lim = max(0, alat_lim + err·h/tau) # leaky integral, clamped >= 0
    M_z     -= sign(r) · alat_lim · gain

An integral can only stop growing when the error is **zero**, so the settled
value is pinned AT the ceiling *by structure, for any gain* — leaving exactly
one free parameter for the transient. Clamping at zero makes it unwind when the
car drops back under, so it never adds yaw.

| law | settled (meas 7.68) | peak (meas 10.42) |
|---|---|---|
| proportional | can fit one | or the other |
| **integral, gain 450** | **7.50** (not fitted) | **10.37** (fitted) |

Reproduce with `python3 -m tuner.plant_openloop_validation --ab`.

### 12.4 Validated on data no fit had seen

`gain` was fitted to the step test's **peak** only. The **sweep** — sustained
cornering over a long orbit, the regime that builds heading error on a lap —
was never used to fit anything:

| | proportional (700) | integral (450) |
|---|---|---|
| capped-point mean err | +1.41 m/s² | **+0.29** |
| capped-point MAE | 1.60 | **0.87** |
| all-point MAE | 1.15 | **0.79** |

The sim had been sustaining **8.3–8.9 m/s² where FSDS sustains 6.1–8.1** — a
20–35% surplus centred on 8 m/s, which is the live car's mean speed (8.03).

### 12.5 What it fixed, and what it didn't

| | P (700) | **PI (450)** | live |
|---|---|---|---|
| a_lat > 7.5 % | 14.25 (×1.45) | **8.89 (×0.91)** | 9.80 |
| a_lat max | 10.53 | 10.80 | 12.34 |
| reversals/s | 0.82 | 0.93 | 1.62 |
| score | 0.675 | 0.700 | — |
| **steering sat %** | **6.32** | **6.74** | **21.10** |

A genuine fidelity fix — and **worth 0.4 points of saturation**. Report it as
such: the plant now reproduces the car's *lateral-acceleration distribution*,
not its steering behaviour.

### 12.6 Eliminated: cornering capability is not the residual gap

Lowering the ceiling to 6.5 or 5.5 m/s² does **not** raise saturation toward
21% — it **DNFs offtrack at ~10% progress**. Same discriminator that killed
μ=1.455 in §5: a *different failure mode* from the car's. The live car completes
laps while saturating 21% of the time; the sim either completes with ~6–7% or
crashes. No capability level reproduces live behaviour.

### 12.7 Eliminated: `fsds_bridge` discarding `a_cmd` is not the differentiator

§7 flagged this as a real unmodelled divergence, and it is: measured live,
`a_cmd → a_achieved` has **corr 0.56–0.58** (the sim's is ~1 by construction),
and live over-speeds its target by **+2.49 m/s at p90 against the sim's +1.05**,
peaking at 13.9 vs 11.1 m/s. Note the earlier speed-profile check compared
*mean* speed (2% apart) and MAE is also nearly identical (1.92 vs 1.98) — the
divergence lives entirely in the over-speed tail, which a mean cannot see.

**But it is not the cause.** Conditioning on saturation:

| signal | live inside sat | sim inside sat |
|---|---|---|
| \|e_psi\| | 41.4° | **40.4°** |
| v_target | 4.72 | 4.95 |
| **speed err** | **+1.13** | **+2.05** |
| \|e_y\| | 0.66 m | 0.27 m |

The sim already arrives **hotter** than live inside saturation. Modelling the
bridge's P-loop would have moved the sim the wrong way. Both stacks saturate
under near-identical conditions.

> This is why the conditional table matters: an aggregate divergence can be
> real, large, and still not causal. Condition on the symptom before modelling.

### 12.8 The car is on the line, pointing the wrong way

Inside saturation the live car is at full lock while pulling only **4.22 m/s²**
and sitting just **0.66 m** off the centreline. A car genuinely unable to turn
through a corner runs wide — `e_y` grows. It does not. This generalises §0's
"key early observation" from an anecdote to the *typical* saturation condition.

Decomposing heading-error growth via
`e_psi = wrap(car_yaw − ref_psi) ⇒ d(e_psi)/dt = yaw_rate − d(ref_psi)/dt`:

| | live | sim |
|---|---|---|
| \|d(ref_psi)/dt\| mean / p99 / max | 28.8 / 163 / 350 °/s | 25.9 / 142 / 261 °/s |
| \|yaw_rate\| mean / p99 / max | 28.2 / 73 / 88 °/s | 22.3 / 77 / 85 °/s |
| growth reference-driven | **77.6%** | **100%** |

**In both stacks the reference heading swings far faster than the car can ever
yaw** (p99 142–163 °/s against a physical maximum near 85–112 °/s), and
reference motion — not the car failing to yaw — dominates heading-error growth.
This is the known planner centreline defect, quantified in *heading* terms for
the first time; §3 eliminated only the *curvature* comparison, which is a
different statistic.

Live's reference is worse, but by 15% (p99) to 34% (max) — **not 3×**. So this
is a strong lead, not yet the answer.

### 12.9 Where the gap actually lives

| | live | sim | ratio |
|---|---|---|---|
| saturation episode **rate** | 0.32/s | 0.12/s | **2.6×** |
| mean episode **duration** | 0.71 s | 0.55 s | 1.3× |
| \|e_psi\| inside saturation | 41.4° | 40.4° | 1.0× |

Both saturate at the same heading error, for comparable durations. The gap is
almost entirely **how often the car enters that state** — a *turn-in transient*
property, not a steady-state capability one. That points squarely at
`alat_ceiling_tau`, the one ceiling parameter still never measured (§10 chose it
behaviourally; the measured decay ranged 0.04–1.06 s). Under the integral law
`tau` no longer affects the settled value at all, so it now controls *only* the
transient — which makes it both more identifiable and more clearly the next
thing to measure, with a longer `step_s`.

> **Measured 2026-08-07 (§12.12a).** `tau` is now **0.40 s** (was 0.25,
> behavioural). Fixing `tau` did **not** close the saturation gap — see below.

### 12.10 Tooling added (the loop that was missing)

`steering_sysid_analysis.py` and `steering_step_analysis.py` answer "what does
FSDS do?". Nothing answered **"does our plant reproduce it?"** — which is how a
13% sustained-cornering surplus survived a refit. Now:

- **`tuner/plant_openloop_validation.py`** — replays both measured open-loop
  experiments through the offline plant at matched (speed, steering).
  `--ab` reproduces the law comparison; `--robustness` runs the confound checks.
- **`tuner/recorded_map_rollout.py`** — headless closed-loop run on the recorded
  map. The `comp test map 3` baselines were previously unreproducible from the
  repo (the track loaded only via a GUI button), so the single most important
  number in the investigation could not be re-checked after a plant change. It
  reproduces the published table exactly (6.32%, 10.53, 14.25%, 0.82, 7.26/19.16).
- **`tuner/live_vs_sim_diagnostics.py`** — the conditional and
  reference-heading decompositions above.

**Rig validated before use, per lesson 6.** The capped-regime result is robust
to a 6× range of speed-hold gains (8.24–8.53, ±3.5%) and exactly
timestep-independent; the ceiling is provably inactive at 3/4/5 m/s. **The
low-speed comparison is NOT trustworthy** — at 4 m/s full lock the plant cannot
hold speed and a_lat swings 2.50→4.03 with the same gains, so the apparent
low-speed under-cornering is a rig confound and is *not* reported as a finding.

### 12.11 Also fixed: two harness bugs that blocked the measurement

- `${EXTRA_ARGS[*]}` **unquoted** in both `run_steering_step.sh` and
  `run_steering_sysid.sh` — joins then word-splits, so any parameter containing
  a space breaks argument parsing. This also silently broke the harnesses' own
  `--quick` flag (`-p 'speeds:=[4.0, 10.0]'`).
- The **shebang sat on line 3**, below two header comments, so the kernel never
  saw it and the script inherited the caller's shell — dying immediately under
  dash on `set -o pipefail`. Fixed in both; `ros2/launch_all.sh` and its
  `fsds_simulator/` mirror have the same defect, left alone as out of scope.

### 12.12 `alat_ceiling_tau` measured — tight, but does not close the gap

*(2026-08-07, continued.)* The longer step test could not be run from this
environment (launching FSDS requires WSL→Windows process spawning that it
terminates — even `--no-sim` against an already-running FSDS still needs
`ros2 launch`/`ros2 run`, which was killed at dispatch every time it was tried,
sandbox-disabled or not). The user ran it directly:

    ros2/run_steering_step.sh --no-sim \
      -p 'speeds:=[5.0,8.0,12.0]' -p 'steer_cmds:=[0.6,1.0]' \
      -p 'step_s:=8.0' -p 'repeats:=2' -p 'require_go:=false'

That surfaced a **second, more fundamental** harness bug behind the one fixed in
§12.11: `"${EXTRA_ARGS[@]}"` was being expanded *outside* the quoted `bash -c
"..."` string that contained the actual script, so the array elements never
became part of the inner command at all — they landed as extra positional
arguments to the outer `bash -c`, and `ros2` saw a bare trailing `-p` with
nothing after it ("Couldn't parse trailing -p flag"). Quoting the array
correctly (§12.11) fixed word-splitting but could not fix this, because the
array was never inside the string to begin with. Fixed in both harnesses by
passing `EXTRA_ARGS` as real positional parameters to `bash -c '...' _ "$@"`
instead of string-interpolating them, and verified with a dry run before asking
for another live attempt.

**Result — `fsae_logs/steering_step_1786047535.csv`, 12 trials, 8 s hold:**

| v | steer | tau (fit) |
|---|---|---|
| 5.0 | 0.60 / 1.00 | 0.28, 0.88, 0.44, 0.46 |
| 8.0 | 0.60 / 1.00 | 0.32, 0.40, 0.38, 0.36 |
| 12.0 | 0.60 / 1.00 | 0.30, 0.34, 0.30, 0.28 |

Median **0.35 s**, 11/12 trials within 0.28–0.46 s (the one 0.88 s outlier is at
5.1 m/s, right at the cap's speed threshold). Compare the old 3 s test's
0.04–1.06 s range — the longer hold resolved it decisively, exactly as
predicted.

**Refit:** under the integral law the settled value is pinned regardless of
`tau` (confirmed: 7.50 across a 0.25→0.45 sweep), so `tau` was fit to this
run's peak alone. `0.40` takes peak error from −0.45 to −0.04 m/s²
(measured 10.82, old model 10.37, new model 10.78). The SWEEP validation is
unchanged (it measures only steady state, which `tau` cannot affect).

**Also checked: does a_lat keep decaying past 3 s?** No — flat within noise at
the 3 s/5 s/8 s marks across all 12 trials. The step test's short-hold settle
(~7.5–7.9) and the sweep's long-orbit sustained value (6.1–8.1, lower at every
matched speed) are **not** reconciled by slow decay inside a single hold. That
disagreement stays open — see the table in `planning_control_sync.md`.

**Closed-loop check (recorded map, `tau=0.40`):** no DNF, saturation actually
**dropped** slightly (6.74% → 4.80%) rather than rising toward live's 21.1%.
Ship it anyway — this is a plant-fidelity fit to a direct FSDS measurement, not
a saturation-tuning knob, and §12.9 already established the residual gap is a
planner/reference problem, not this parameter. Re-verify after any planner
change that shortens corner-entry time, per the standing DNF risk noted in §10.

**So the top-priority parameter from §12.9 is now measured, and it was not the
answer.** The residual saturation gap is still open. The reference-heading lead
(§12.8 — both stacks chase a reference swinging faster than either car can yaw,
78–100% of heading-error growth is reference-driven) is the next thing to
pursue, and it is testable **offline**: planner republish rate, centreline
smoothing parameters, no FSDS session required.

## 13. How much does each tested factor actually explain? A quantified ledger

*(2026-08-07.)* Every factor above was reported as a single before/after number
on the one recorded map — a point estimate with no error bar. "6.3% → 6.7%"
could be signal or map-specific noise, and nothing so far could tell the
difference. This section fixes that with `tuner/gap_attribution_ledger.py`.

**Why the recorded map alone cannot supply variance.** Every rollout on it is
fully deterministic by construction: SLAM noise, delay jitter and pose-hold all
use FIXED seeds (`settings.SLAM_NOISE_SEED`, `DELAY_JITTER_SEED`,
`POSE_HOLD_SEED`) so CMA-ES sees a repeatable score per tuning candidate.
Re-running the same map twice reports exactly zero variance — a false
confidence signal, not a real one. The ledger instead runs every factor across
`settings.VALIDATION_SUITE` (5 synthetic paths: spiral, sudden turn, hairpin,
FS corner, micro-slalom) as the repeat axis. This measures whether an effect is
consistent across track geometry or an artefact of one map's specific corners.
These paths have no live-log counterpart, so the recorded map stays the only
**live-comparable** number; the suite is purely for attributing how much each
factor moves the *sim*, with a real spread attached.

**A baseline mismatch, caught before it corrupted the ledger.** The obvious
first step — "gap = live 21.1% − historical no-ceiling baseline 4.4%" — is
wrong. Measuring "no ceiling" with today's code (`python3 -m
tuner.recorded_map_rollout --no-ceiling`) gives **3.67%**, not 4.4%. The 4.4%
figure predates `recorded_map_rollout.py` and several since-fixed bugs
(including this session's `curvature_speed` parity fix, itself a real change
to planner/controller interaction) — it is not the same quantity and mixing it
with today's numbers would repeat the exact mistake this document exists to
catalogue (§8, lesson 2 in "What generalises"). The gap used below is
**live 21.1% − today's no-ceiling baseline 3.67% = 17.43 pp**, measured with
one consistent code state.

**Recorded map (live-comparable) vs `VALIDATION_SUITE` (variance-comparable):**

| factor | rec. map sat % | suite mean | suite std |
|---|---|---|---|
| A. no ceiling (today's baseline) | 3.67 | 7.71 | 4.10 |
| B. ceiling, proportional law (700, 0.25) | 5.51 | 9.85 | 6.06 |
| C. ceiling, integral law, old tau=0.25 (450) | 5.84 | 10.45 | 5.94 |
| D. ceiling, integral law, **shipped** tau=0.40 (450) | 4.80 | 9.42 | 5.11 |
| E. ceiling lowered to 6.5 (capability probe) | 6.90 | 10.86 | 6.23 |

**The step-by-step "closes X% of the gap" table, and why it should not be
trusted at face value:**

| step | closes (pp) | closes (%) | suite std (pp) |
|---|---|---|---|
| B. proportional law | 1.84 | 10.5 | 6.06 |
| C. integral law, old tau | 0.33 | 1.9 | 5.94 |
| D. integral law, shipped tau | −1.04 | −6.0 | 5.11 |
| E. lower ceiling to 6.5 | 2.10 | 12.0 | 6.23 |
| **REMAINING UNEXPLAINED** | **14.20** | **81.5** | — |

Every one of those per-step deltas (0.33 to 2.10 pp) is smaller than the
suite's own standard deviation (5.1–6.2 pp) at that step. **None of them is
distinguishable from noise on this evidence.** Do not read "closes 12.0%" as a
real, confident effect size — it is a point estimate with an error bar several
times larger than itself. This is true even of D showing *negative* attribution
(the measured, validated `tau` fix moved the recorded-map number slightly
*away* from live) — that looked like a real regression in §12.12, and this
ledger shows it is not distinguishable from a null result either.

**What the ledger changes about the conclusion: nothing about which candidates
are viable, but it upgrades the confidence in ruling them out.** Even reading
every point estimate at face value and summing the largest ones (B + E, the two
biggest, non-overlapping single-parameter changes tried): 1.84 + 2.10 = 3.94 pp,
23% of the 17.43 pp gap — and E is already known from §12.6 to DNF the recorded
map before reaching a level that would close more. So even the most generous
possible reading of noisy, unvalidated point estimates cannot get the tested
plant/ceiling parameters past roughly a quarter of the gap. **At least three
quarters of the saturation gap is not explained by anything about the ceiling's
law, level, or time constant.**

**A genuinely new finding, from the paired (per-path) breakdown the raw
suite mean/std hides:**

| step | SPIRAL | SUDDEN_TURN | HAIRPIN | FS_CORNER | MICRO_SLALOM | mean | std |
|---|---|---|---|---|---|---|---|
| B | −1.0 | +3.1 | +0.0 | −0.0 | +8.7 | +2.15 | 3.55 |
| C | −0.4 | +5.4 | +0.0 | −0.0 | +8.7 | +2.75 | 3.68 |
| D (shipped) | −1.5 | +3.8 | +0.0 | +0.0 | +6.2 | +1.72 | 2.85 |
| E | −1.7 | +7.8 | +0.0 | −0.1 | +9.7 | +3.15 | 4.66 |

**The ceiling has ZERO measurable effect on HAIRPIN and FS_CORNER in every
configuration tried**, and its entire effect is concentrated on SUDDEN_TURN and
MICRO_SLALOM — paths with sustained moderate-radius bends at speed, not tight
low-speed corners. That is consistent with the ceiling's own mechanism (it only
engages above ~6 m/s, §9) and is a genuinely new, structural finding: the
ceiling's contribution — whatever it is — is not a general saturation
mechanism, it is specific to a particular corner *type*. Any future attempt to
explain saturation via plant/ceiling parameters should be checked against this
same per-path breakdown before trusting an aggregate number.

**Reproduce with:** `python3 -m tuner.gap_attribution_ledger` (needs
`MPLBACKEND=Agg` set — see the note at the top of that file; `cma`'s optional
matplotlib import was observed to crash the whole process, not just skip
`cma.plot()`, depending on backend state, and cost two failed runs before being
tracked down).

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
8. **When no parameter value fits, suspect the functional form.** Two failed
   fits at opposite ends of a range are one structural error, not two mistakes.
   A proportional law cannot hold a setpoint; an integral one cannot miss it.
   Sweep the parameter and *look at the trade-off curve* — that a monotone
   trade-off exists at all is the proof.
9. **Prefer structure over fitting.** Under the integral law the settled value
   is pinned by the form of the equation, not by a fitted number, so it cannot
   drift when something else is refitted. One free parameter, one target.
10. **Close the loop on your own model, not just on the system.** Analysers that
    only answer "what does the real thing do?" let a model error survive
    indefinitely. Every measurement of FSDS should have a matching replay of the
    plant at the same operating point.
11. **An aggregate divergence can be real, large, and not causal.** The
    discarded `a_cmd` is all three. **Condition on the symptom** — inside
    saturation the sim was already *hotter* than live, so modelling it would
    have moved the sim the wrong way.
12. **Aggregates hide tails, and means hide both.** The speed divergence is
    invisible in the mean (2%) and in the MAE (1.92 vs 1.98), and obvious in the
    p90 (+2.49 vs +1.05). Pick the statistic that matches the mechanism.
13. **Decompose the error before attributing it.** `e_psi` growth splits exactly
    into car-lag and reference-motion. Measuring the split (78–100%
    reference-driven) beat years of arguing about whether the planner was
    "good enough".
14. **Separate rate from duration.** "21% of ticks" hid the actual finding: the
    durations match (1.3×) and only the *entry rate* differs (2.6×). That
    reframes the search from steady-state capability to turn-in transients.
15. **A parameter matching the mechanism is not a guarantee it's the whole
    story.** `tau` was the single most implicated unmeasured parameter — it
    directly controls the transient, and the residual gap is specifically a
    transient. It was measured, refit, and validated (§12.12) and the
    saturation gap did not move. Localising *what kind* of thing is wrong
    (transient vs steady-state) narrows the search; it does not name the cause.
16. **A quoting fix can be incomplete.** `[*]` → `[@]` (§12.11) fixed
    word-splitting but the array was still expanding *outside* the string that
    contained the command — a different bug with the same symptom (broken
    arguments), only caught by an actual failed run, not by reasoning about the
    fix in isolation.

---

## Open / deferred

| item | status |
|---|---|
| **Model the yaw cap offline** | **Done** — `alat_ceiling*` in `model/vehicle_physics.py`. Moves every metric toward the car (saturation 4.4→6.3%, a_lat max 14.06→10.53) but closes only part of the gap; live is still 21.1% saturation |
| Remaining saturation gap | **Open, quantified (§13).** ≥75% of the 17.43 pp gap (live 21.1% vs today's 3.67% no-ceiling baseline) is not explained by the ceiling's law, level, or tau — every tested plant/ceiling factor's effect is within noise (suite std 5–6 pp vs effect sizes 0.3–2.1 pp). Localised to the *entry rate* into the high-heading-error state (2.6×) with matching in-state behaviour — a turn-in transient property |
| Ceiling's effect is corner-type-specific | **New (§13).** Zero measurable effect on HAIRPIN/FS_CORNER in every configuration tried; entire effect concentrated on SUDDEN_TURN/MICRO_SLALOM (sustained moderate-radius bends at speed). Check any future plant explanation against this per-path breakdown before trusting an aggregate |
| **`alat_ceiling_tau`** | **Done (§12.12).** Measured 0.35 s median (0.28–0.46, 11/12 trials) with `step_s=8.0`; model set to 0.40. Fixing it did **not** close the saturation gap — the residual is elsewhere |
| Ceiling is speed-dependent | **New, unmodelled (§12.4).** Measured sustained a_lat rises with speed — 6.45 @ 8 m/s, 7.54 @ 11, 9.26 @ 14 — while the model pins it flat at 7.5. Residuals +1.0 / ~0 / −1.76. Deliberately not fitted: 16 points, one run |
| Step vs sweep disagree on level | **Open.** Step's 3 s settle says 7.5 @ 8 m/s; the sweep's long orbit says 6.45. A longer `step_s` resolves whether sustained a_lat keeps decaying past 3 s — same experiment as the `tau` re-measurement |
| Planner reference heading | **New lead (§12.8).** In *both* stacks the reference heading swings faster than the car can yaw, and drives 78–100% of heading-error growth. Live's is 15–34% worse in the tail. Distinct from the *curvature* comparison §3 eliminated |
| Identify the exact FSDS mechanism | **Done** — step test run (§9–10): a *dynamically enforced lateral-acceleration* ceiling. Both signatures present: different steering angles settle to the same response (a cap), yet yaw overshoots ~30% first (not a static clip) |
| Ceiling decay time constant | **Done — see `alat_ceiling_tau` above (§12.12).** Superseded the original 0.04–1.06 s scattered fit from the 3 s hold |
| Speed profile | **Closed, not a discrepancy** — see the correction below. Achieved speeds differ by 2% |
| Objective rebalancing | Deferred (§1) — `QUALITY_WEIGHT` 0.35 → ~0.8, saturation as near-constraint |
| Step 4: held-out tracks | 5 of 10 tracks unused |
| Live scorer reports `13.0` | Every live run scores `CONSTRAINT_FLOOR + DNF_PENALTY` — the car has no known path end |
| `fsds_bridge` discards `a_cmd` | §7 — real divergence, quantified in §12.7 (corr 0.56, over-speed p90 +2.49 vs +1.05). **Not the cause of the gap.** Still worth fixing on the car: the MPC plans a braking profile the bridge throws away, so live decelerates reactively and arrives hot |
| Planner centreline defect | Pre-existing, documented; controller carries workarounds |

**Do not retune yet.** The cap is now modelled *and its law corrected*, which
makes the sim materially better — it reproduces the car's lateral-acceleration
distribution to within 9% (§12.5). But it does not reproduce the car's steering
behaviour: live still saturates 3x more often (21.1% vs 6.7%) and carries 2x the
heading error. A good offline score has already produced a car saturating
steering 21% of the time.

The §12 work makes the *plant* trustworthy in the lateral-acceleration sense and
rules out the two candidates that looked most promising. The residual is a
turn-in/reference problem, and `alat_ceiling_tau` is the cheapest untested lever.

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
