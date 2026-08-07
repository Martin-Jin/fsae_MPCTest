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
  `fsds_simulator/` mirror had the same defect — see §16, where both were
  fixed too (not left out of scope after all — it caused a real launch
  failure, not just a latent risk).

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

## 14. Testing the reference-heading lead: does `blend_paths`' reset bypass fire?

*(2026-08-07.)* §12.8/12.9 found the residual gap is a *turn-in transient*
property (2.6× more saturation-episode entries, same duration/severity once
entered) and that both stacks chase a reference heading swinging faster than
either car can yaw. §13's ledger then ruled out every *plant/ceiling*
parameter as the explanation. This section tests the next candidate — the
*planner* — without touching any planner file, per the standing caution in
`planning_control_sync.md` ("Known planner defect: centreline curvature
spikes").

**The mechanism.** `planning/path_utils.py`'s `blend_paths()` exists
specifically because the planner rebuilds the centreline from scratch every
pose tick (`centerline_planner.py::_planning_loop`, confirmed by reading the
code — `build_path_walls()` is called fresh each cycle from the accumulated
`ConeMap`, with no previous-path seed into the geometry fit itself). Without
blending, successive rebuilds would jump and the controller would track that
jump as a heading swing — exactly the symptom in §12.8. `blend_paths` eases
between rebuilds via an EMA (`alpha=0.4`, both `centerline_planner.py` on the
live side and `sim/sim_track.py::SimPlanner.update()` on the sim side — same
function, same default, genuine parity, confirmed by reading both). But it
has an escape hatch: if the mean resampled distance between the fresh rebuild
and the previous *published* path exceeds `reset_dist=2.0` m, the blend is
skipped and the raw new path is published unblended — "the paths have
genuinely diverged... snap to the new path instead of lagging toward the old
one." That bypass fires exactly when the rebuild has moved the most, which is
plausibly correlated with the documented curvature-spike defect (cone-map
clutter worsening lap-over-lap). If it fires often and produces large
near-field heading jumps, it is a second, independent mechanism for exactly
the symptom in §12.8 — distinct from curvature magnitude, because even a
smooth centreline could still jump if consecutive publishes are not
temporally consistent with each other.

**Measured with `tuner/blend_reset_diagnostics.py`** (new, this session) — a
non-invasive wrapper around `blend_paths` that counts reset events and the
resulting near-field heading jump, without changing the function's behaviour
(the rollout is byte-for-byte identical to an uninstrumented run):

| track | calls | resets | reset dist (mean/max) | heading jump at reset (mean/p90/max) |
|---|---|---|---|---|
| **recorded map** (live-comparable) | 1038 | **0 (0.0%)** | — | — |
| PATH_SPIRAL | 220 | 6 (2.7%) | 7.35 / 13.08 m | 88.9° / 166.2° / 166.4° |
| PATH_SUDDEN_TURN | 225 | 0 (0.0%) | — | — |
| PATH_HAIRPIN | 125 | 1 (0.8%) | 2.63 / 2.63 m | 8.4° / 8.4° / 8.4° |
| PATH_FS_CORNER | 75 | 0 (0.0%) | — | — |
| PATH_MICRO_SLALOM | 155 | 0 (0.0%) | — | — |

Confirmed not an artifact of the 2.0 m threshold being unreachable: the
recorded map's own reset-trigger distance distribution (measured directly,
not just pass/fail) is mean 0.14 m, p99 1.00 m, **max 1.98 m** — a real
distribution that sits just under the threshold, not a mechanism that never
engages.

**Result: the mechanism is real, parity-correct across both stacks, and can
produce large discontinuities (up to 166° on a spiral) — but on the one
live-comparable track it is measured against, it never fires.** This does not
support "reset-bypass discontinuity" as the explanation for the recorded
map's saturation gap: 0 events cannot produce 21.1% saturation. It does mean
the mechanism is a live hazard on other geometries (tightening-radius spirals
being the worst case measured) and should be re-checked if a future planner
fix changes rebuild volatility enough to bring the recorded map's max (1.98
m) over the 2.0 m line — it is close enough that a small increase in
cone-map clutter would cross it.

**What this does and does not rule out.** It rules out this *specific*
mechanism (blend-defeat-by-divergence) as the explanation for the
recorded-map gap. It does not rule out the planner more broadly — the
reference-heading lead itself (§12.8) is still open and unexplained; this
only closes off one candidate mechanism within it.

**Reproduce with:** `python3 -m tuner.blend_reset_diagnostics` (needs
`MPLBACKEND=Agg`, same `cma`/matplotlib import issue as §13's ledger).

### 14.1 Does rebuild magnitude explain the blended path's heading rate? Mostly no.

The remaining part of §12.8's lead is whether the *blended* path — what
actually gets published on all 1037/1038 recorded-map ticks that don't hit
the reset — nonetheless carries a heading rate exceeding what the car can
yaw, i.e. whether `alpha=0.4` is itself insufficient rather than bypassed.
Measured with `tuner/reference_heading_vs_rebuild.py` (new): per-tick rebuild
distance (same statistic as §14, but recorded for every tick, not just
above-threshold ones) against the blended path's own reference-heading rate,
reconstructed the same way as §12.8 (`ref_psi = car_yaw − e_psi`).

| | value |
|---|---|
| \|d(ref_psi)/dt\| mean / p90 / p99 / max | 26.4 / 63.1 / 140.0 / 265.0 °/s |
| rebuild distance mean / p90 / p99 / max | 0.14 / 0.25 / 1.00 / 1.98 m |
| raw correlation(rebuild distance, \|d(ref_psi)/dt\|) | **0.149** |
| partial correlation, controlling for \|yaw_rate\| and speed | **0.101** |
| top-10%-rebuild-distance ticks (>0.25 m), mean \|d(ref_psi)/dt\| | 40.1 °/s |
| bottom-90% ticks, mean \|d(ref_psi)/dt\| | 24.8 °/s |

A real but weak effect: the top-decile-rebuild ticks run 62% hotter than the
rest, and controlling for corner severity (`|yaw_rate|`, speed — a proxy for
"this is just a tight corner, not planner noise") barely moves the
correlation (0.149 → 0.101), so it is not purely a severity confound. But
r≈0.10–0.15 means rebuild magnitude explains at most ~1–2% of the variance
in reference-heading rate (r²) — nowhere near enough to be the dominant
driver of the 78–100% reference-driven heading-error growth in §12.8. **Most
of the reference-heading swing on this map is not explained by
rebuild-to-rebuild planner instability**, blended or bypassed. It is more
likely intrinsic to the geometry the centreline is fitting (a genuinely
tight corner has a fast-changing tangent on its own) — which loops back to
the *curvature-spike* defect this section set out to distinguish from: not
as a temporal-consistency problem, but as the original spatial one after
all, just measured in heading rather than curvature terms for the first
time. That reopens the planner's actual smoothing/fitting logic
(`centerline_planner.py`, `boundary.py`) as the place to look — not
`blend_paths`, which is now checked on both fronts (reset-bypass in §14,
blended-magnitude correlation here) and cleared each time.

**Reproduce with:** `python3 -m tuner.reference_heading_vs_rebuild` (same
`MPLBACKEND=Agg` requirement).

## 15. Cone-detection noise: closing a real blind spot, testing a real gap in `_absorb()`

*(2026-08-07, continued.)* §14.1 pointed the reference-heading lead back at
the planner's spatial fit, specifically at *why* the documented
curvature-spike defect gets worse lap-over-lap ("consistent with cone-map
clutter accumulating," `planning_control_sync.md`'s "Known planner defect"
section). Reading `planning/cone_map.py::ConeMap._absorb()` (identical in
both `ros2/src/fsae_planning` and `fsae_MPCTest`) found a candidate mechanism:

```python
for pt in obs:
    dists = np.linalg.norm(store - pt, axis=1)
    best  = int(np.argmin(dists))
    if dists[best] < MERGE_DIST:          # MERGE_DIST = 0.8 m
        store[best] = (store[best] + pt) * 0.5
    else:
        new_pts.append(pt)
```

Each detection in an incoming batch is only ever compared against `store`
(the existing map) — never against the other detections already queued into
`new_pts` from the *same* call. Two noisy detections of one physical cone,
both more than `MERGE_DIST` from anything already in the map (e.g. a cone
newly entering the FOV) but within `MERGE_DIST` of *each other*, would both
be appended as separate, permanent map entries — a phantom duplicate that
never merges later, since every future detection still only checks against
`store`. Combined with `cone_sorting.py`'s greedy nearest-neighbour walk and
greedy nearest-unpaired-cone pairing, a single such duplicate is exactly the
kind of thing that could pull a boundary wall into a spurious sub-3.7 m-radius
kink.

**This could not be tested as-is.** `sim/sim_track.py::SimPerception` (and
the identical ROS2-side oracle) returns *exact* ground-truth cone positions,
only cropped by range/FOV — confirmed by reading `visible_cones()`: the
returned array is `cones[mask]`, a slice of the stored ground truth, never
perturbed. `SlamNoise` corrupts the *pose* used to compute the FOV mask, not
cone coordinates, so even with `SLAM_NOISE_ENABLED=True` a visible cone always
reports at its exact true position. Per `planning_control_sync.md`'s own
"Simulator fidelity limits" table: **"Cone map... Not modelled anywhere."**
With zero detection noise, two detections of one cone can never land on
opposite sides of `MERGE_DIST` from each other — the `_absorb()` gap cannot
fire, regardless of whether `MERGE_DIST=0.8` is itself well- or
mis-calibrated. Neither question was answerable before this.

**Added `ConeNoise`** (`sim/rollout_core.py`, alongside the existing
`SlamNoise`/`PoseFeedHold`) — independent per-cone, per-frame position
jitter, applied at the `SimPerception.visible_cones()` boundary, both at
rollout initialisation and in the main step loop. New settings, following the
exact `SLAM_NOISE_*` convention (default off, seeded, magnitude documented as
a placeholder pending a real measurement):

```python
CONE_NOISE_ENABLED = False        # default off — FSDS itself has no such error
CONE_POS_JITTER_STD = 0.05        # m, per-cone-per-frame white noise on x/y
CONE_NOISE_SEED = 97531
```

Unlike `SlamNoise`, there is deliberately no drift/bias term: a vision cone
detector re-estimates each cone's position from scratch every frame rather
than tracking a belief over time, so nothing should carry over between
frames the way SLAM's OU drift does — if a future measurement shows real
detections DO carry correlated frame-to-frame error, add a drift term the
same way `SlamNoise` does rather than repurposing jitter. Also deliberately
per-cone rather than a single shared frame offset (`SlamNoise`'s model):
detection error is per-object (each cone has its own range/angle/occlusion
to the sensor), not a single pose error shared by the whole visible set.

**Scope, deliberately narrow.** This models position jitter only. False
positives, false negatives, and range-dependent noise growth are all real
and still unmodelled — the fidelity-limits table's "Cone map... Not modelled
anywhere" row is only partly closed, not fully. This was scoped to the
minimum needed to test the `_absorb()` hypothesis, not to build a complete
perception noise model.

**Full-rollout duplicate-cluster counting was the wrong tool: a stress test at
0.5 m jitter never finished in 400 s** (the growing, unmerged cone map makes
every subsequent `_absorb()` call's linear scan more expensive, compounding
badly — a real, if secondary, finding about the cost of unbounded jitter, not
about correctness). **A direct unit test of `_absorb()` in isolation was
decisive instead** — no rollout needed at all:

```python
>>> _absorb(np.empty((0, 2)), np.array([[10.0, 2.0], [10.01, 2.0]]))
array([[10.  ,  2.  ],
       [10.01,  2.  ]])   # two PERMANENT entries, 1 cm apart, never merged
```

**This is a real, deterministic correctness bug, not a noise-magnitude
question.** Two detections of one physical cone **1 cm apart** — far below
any plausible detection noise — both get appended as separate, permanent map
entries whenever the map has nothing within `MERGE_DIST` yet (a cone's first
sighting, or `store` starting empty). It fires independent of jitter
magnitude, so tuning `MERGE_DIST` cannot fix it: the loop only ever compares
each candidate against `store`, never against the other candidates already
accepted into `new_pts` from the same call. Confirmed the scope precisely:
once a cone has ONE entry in the map, later multi-detection frames merge
correctly (`_absorb()` on an existing store, two nearby new detections,
correctly averages to one point) — the gap is specific to a cone's entry
into the map, not persistent per-cone jitter.

**Fixed** in `planning/cone_map.py::_absorb()` (both `fsae_MPCTest` and the
identical `ros2/src/fsae_planning` copy — see parity note below): new-point
candidates are now also checked against each other (and against
new-points already accepted earlier in the same call) before being appended,
with the same running-mean merge used against `store`. Also fixed the
`len(store) == 0` early-return, which bypassed all merge logic (including the
new same-batch check) whenever the map started empty.

**Verified:**
- Unit tests: the 1 cm-apart case now merges to one entry; two genuinely
  distinct new cones (>`MERGE_DIST` apart) still both get added; an
  already-known cone plus a same-frame duplicate-of-it plus a same-frame
  genuinely-new cone all resolve correctly (2 entries, not 3); a 3-way
  same-frame duplicate merges to 1; result is order-independent.
- No regression at the default (noise disabled): recorded map still 4.80%
  saturation exactly, and the validation-suite mean/std/per-path breakdown
  for the shipped params is **byte-identical to §13's original table**
  (9.42% mean, 5.11% std, SPIRAL 11.3/SUDDEN_TURN 7.6/HAIRPIN 7.7/FS_CORNER
  2.6/MICRO_SLALOM 18.0) — expected, since `_absorb()`'s new-vs-new check is
  a no-op when there's at most one genuinely new cone per frame, which is
  what a noise-free oracle always produces.
- With noise on (default 0.05 m jitter): runs cleanly, no crash, no DNF;
  236 permanent cone entries accumulated over the recorded-map rollout, a
  sane count for the track length (no explosion). Saturation unchanged from
  the pre-fix noise-on measurement (4.86%) — expected, since 0.05 m jitter
  was already too small to make two same-cone detections land on opposite
  sides of `MERGE_DIST=0.8` m from EACH OTHER either; the fix closes the
  logic gap regardless, since it doesn't depend on that being the case.

**What this does and does not establish.** The bug is real and fixed, and
the fix is verified safe (byte-identical outputs when it cannot fire, correct
merges in every case tested when it can). It does **not** establish that this
bug explains any part of the live saturation gap: at the realistic default
noise level, the aggregate saturation number does not move, because 0.05 m
jitter is far too small for the specific first-sighting scenario to
meaningfully compound over a lap on this map. Confirming or ruling out a
real-world effect needs either a measured real-detector noise figure (not
guessed) or a live log showing actual duplicate-cone clustering in the
accumulated map — neither exists in this environment. This section fixes a
genuine bug and makes the hypothesis testable; it does not close the
investigation.

**Parity applied to all three copies:** identical fix in
`ros2/src/fsae_planning/planning/fsae_planning/fsae_planning/cone_map.py`
(the live copy), `fsae_MPCTest/planning/cone_map.py` (offline), and
`fsae_MPCTest/fsds_simulator/planning/fsae_planning/fsae_planning/cone_map.py`
(the PR-staging mirror, which already had this file — per CLAUDE.md, an
existing shared file gets the same change, not just the two "parity" copies).
The `ConeNoise` half of this change (`settings.py`, `sim/rollout_core.py`) is
`fsae_MPCTest`-only by design — it is an offline testing-harness fidelity
feature with no live-node counterpart to keep in sync (the real car already
has real sensor noise for free; FSDS does not), the same reasoning that
already applies to `SLAM_NOISE_*`/`POSE_HOLD_*`, neither of which appears
anywhere in the live ROS2 code either.

**Reproduce with:** unit-test `_absorb()` directly (see above) for the
correctness check; set `CONE_NOISE_ENABLED = True` in `settings.py` (must be
edited before import — like `SLAM_NOISE_ENABLED`, the rollout reads it via
`from settings import CONE_NOISE_ENABLED`, so patching the module attribute
after import has no effect) and re-run `python3 -m tuner.recorded_map_rollout`
for the closed-loop check.

## 16. Getting a live run at all: two real bugs in `launch_all.sh`

*(2026-08-07, continued.)* Every section above measured the offline sim only —
no live control log existed anywhere in this environment for the whole
investigation. Getting one required `ros2/launch_all.sh` (the host-side
orchestrator that starts FSDS, the ROS 2 bridge, and the autonomous stack) to
actually work, and it didn't. Two distinct bugs, found in sequence because
fixing the first one exposed the second:

**Bug 1 — the shebang sat on line 3, so the script never ran as bash.**
Same defect already documented in §12.11 for `run_steering_step.sh`/
`run_steering_sysid.sh`, and *this* file was noted there as "left alone as
out of scope" — it wasn't, it wired straight into the failure being debugged
here. `/bin/sh` on this machine is `dash`, and a script invoked without its
shebang recognised falls back to the caller's shell. The pre-existing
`sleep 2` after launching FSDS had no shell-specific requirements, so this
was invisible for as long as the script only ever slept. The first fix
attempted here — replacing that blind sleep with a poll loop — was the first
thing in the file to need a real bash feature (`/dev/tcp`), and it exposed
the shebang bug immediately: under dash, `/dev/tcp/...` isn't a special path
at all, so the connection attempt fails unconditionally regardless of
whether the target port is open, producing an infinite poll with no way to
tell "genuinely down" from "wrong shell" apart from reading the shell itself.
Fixed by moving `#!/bin/bash` to line 1.

**Bug 2 — the real cause: WSL2 cannot reach the Windows host via
`127.0.0.1`.** With bash actually running, the poll loop correctly reported
the AirSim RPC port as unreachable — because it is, from WSL's network
namespace, via loopback. WSL2's NAT networking gives WSL and Windows separate
loopback interfaces; the Windows host is reachable via WSL's default gateway
instead (`ip route show default`, third field). Confirmed directly: FSDS's
RPC port was observed open via `netstat.exe` from the Windows side while the
WSL-side poll loop still reported it closed, and connecting via the gateway
IP succeeded immediately where `127.0.0.1` failed with `Connection refused`.

This also explains why the ORIGINAL bug report (`fsds_ros2_bridge` crashing
with `Failed connecting to RPC server (airsim)`) was never really about
timing at all: `fsds_ros2_bridge.launch.py` already anticipated this exact
problem — it accepts an `FSDS_HOST_IP` environment variable specifically for
WSL, documented in its own `DeclareLaunchArgument` comment, defaulting to
`localhost` otherwise. `launch_all.sh` never set it, so the bridge was always
going to fail to connect the same way regardless of how long the wait before
launching it — the 2-second sleep and the later 120-second poll were both
solving a timing problem that didn't exist; the actual problem was
address-only, on a machine where `localhost` had apparently worked before
(user-confirmed), meaning this may be newly broken by a Windows/WSL
networking change rather than always-broken. Not investigated further; the
fix does not depend on knowing why it changed.

**Fixed** in `ros2/launch_all.sh` by computing
`FSDS_HOST_IP="$(ip route show default | awk '{print $3; exit}')"` once (only
for the native, non-Docker path — the Docker container has its own
networking) and exporting it before both the readiness poll and the bridge
launch, so both consumers agree on the same address without a manual
`export` step. **Applied to both copies** — `ros2/launch_all.sh` and
`fsds_simulator/launch_all.sh` — despite the standing rule that the mirror
copy is "intentionally adapted, not a sync target" (different Windows
username, resolution, and cone-map output path): that rule is about
*configuration* divergence, not about carrying a bug fix. The
username/resolution/cone-path differences were preserved exactly; only the
shebang position and the RPC-wait/networking logic were ported, matching how
§12.11's quoting fixes were applied to both `run_steering_*.sh` copies
without collapsing their other differences.

**Consequence for this investigation:** a live control log now exists
(`mpc_standalone_control_1786065783.csv`) for the first time since this
document began. Every finding in §12–§15 was measured against the offline
sim only, with the recorded map as the sole live-comparable reference. The
next step is comparing this fresh log against the sim baseline via
`tuner/live_vs_sim_diagnostics.py` — not yet done as of this section.

## 17. First live-vs-sim comparison since §0 — improved, but confounded, and one metric got worse

*(2026-08-07, continued.)* A second, longer live run completed
(`mpc_standalone_control_1786066237.csv`, 207.9 s, 4160 ticks, no solver
failures) on the same map as §0's original baseline — confirmed by comparing
recorded (x, y) extent against `comp test map 3` (x: −35.3..52.7,
y: −0.8..90.0, matching to within the path-recorder's own resolution).
`tuner/live_vs_sim_diagnostics.py` runs it against the current sim baseline
directly:

| | §0 baseline (live) | **this run (live)** | current sim |
|---|---|---|---|
| steering saturation | 21.1% | **15.2%** | 4.80% |
| saturation episode rate | 0.32/s | **0.20/s** | 0.12/s |
| mean episode duration | 0.71 s | **0.77 s** | 0.55 s |
| \|e_psi\| inside saturation | 41.4° | **43.3°** | 40.4° |
| \|e_psi\| mean / p90 | 15.9° / 42.0° | **12.1° / 33.6°** | 6.9° / 18.5° |
| steering reversals/s | 1.62 | **3.15** | — |

**Read this as a mixed result, not a fix.** Saturation and mean heading error
both improved materially, and the entry-rate gap against sim narrowed from
2.6× to about 1.7× — consistent with §12.9's framing that the gap is about
*how often* the car enters the high-error state, not its severity once
there (duration and in-state \|e_psi\| are essentially unchanged, as before).
But reversals/s very nearly doubled — a real signal, not zero-crossing
noise (`delta_cmd` sits within ±1° of centre on only 9.0% of ticks). A
controller that saturates less but reverses direction more is not an
unambiguous improvement; it may be trading one wobble symptom for another.

**This cannot be attributed to any single fix from §12–§16.** The live
`fsae_planning` repo's working tree currently carries a full uncommitted
rewrite on top of the commit that produced the 21.1% baseline (`dfd1a08`,
2026-08-04) — retuned `Q_diag`/`R_diag`/`R_rate_diag` (including a deliberate
manual correction to `Q_diag[3]`, the yaw-rate/e_psi_dot damping term, per its
own inline comment), a rewritten `mpc_core.py`/`control_utils.py`, the
`ConeMap._absorb()` fix (§15), and changes to `sim_perception.py`,
`boundary.py`, and `centerline_planner.py` — fifteen files, ~1060 insertions,
none of it committed, all of it present in this one run. There is no way to
isolate which change moved which number from a single before/after pair.
Getting attribution would need re-running with only one change at a time
(starting with reverting the MPC weights back to `dfd1a08`'s values, since
that is the largest and most likely candidate for both the saturation
improvement and the reversal regression — a stiffer yaw-rate penalty
plausibly cuts saturation by damping the turn-in transient while also
inducing more correction oscillation).

**Reproduce with:** `MPLBACKEND=Agg python3 -m tuner.live_vs_sim_diagnostics`
(picks up every `*.csv` in `fsae_logs/` automatically; the `cma`/matplotlib
import warning printed first is the same benign non-fatal issue noted in
§13/§14, not a real error).

## 18. Researched whether FSDS's physics engine itself is publicly documented to explain the ceiling

*(2026-08-07, continued.)* Everything in §8–§13 characterised the ceiling by
measurement, from outside FSDS as a black box, without ever checking what is
publicly known about the engine producing it. FSDS is a fork of Microsoft
AirSim, whose vehicle physics run inside Unreal Engine's PhysX-based wheeled-
vehicle system — not something either project re-implements. This section
checked whether that upstream engine is documented to do anything that would
produce our exact measured signature (full steering response below ~6 m/s,
collapsing to a 0.17 ratio by 14 m/s, ~0.35–0.40 s first-order lag, ~30% yaw
overshoot before settling).

**No smoking gun, but one concrete, previously-untested, and directly
checkable candidate: `SteeringCurve`.**

- **AirSim's own C++ layer does nothing relevant.** `CarPawnApi.cpp` passes
  steering straight through to UE4's `WheeledVehicleMovementComponent4W`
  (`SetSteeringInput(controls.steering)`) with no scaling, clamping, or
  traction/stability-control logic anywhere in AirSim's source, and AirSim's
  public docs (`settings.md`, `using_car.md`) expose no traction-control or
  steering-curve fields. This rules out AirSim's own wrapper code as the
  mechanism — whatever is happening is either inside UE4's vehicle component
  itself or inside FSDS's specific vehicle asset configuration.
- **NVIDIA's PhysX Vehicle SDK documentation describes exactly this shape of
  behaviour — as an optional, developer-wired pattern, not a built-in
  default.** The SDK's own tutorial (nvidia-omniverse.github.io/PhysX)
  recommends *"filter[ing] the steer angles passed to the car at run-time to
  generate smaller steer angles at larger speeds,"* with a sample table (0–5
  m/s → full steer, 30 m/s → 0.125× steer). That is qualitatively our
  signature. Critically, PhysX's docs are explicit this only exists if a
  vehicle's setup wires a **`SteeringCurve`** — a speed-keyed
  `FRuntimeFloatCurve` that is a first-class field on every
  `WheeledVehicleMovementComponent4W` in UE4's stock wheeled-vehicle
  template. If FSDS's car Blueprint carries a nonzero curve here (inherited
  from Epic's template content, not something FSDS would have had to add
  deliberately), it would produce a genuine, designed-in speed-dependent
  steering reduction — a fundamentally different mechanism from tyre grip,
  and one that would explain why scaling tyre friction/cornering-stiffness
  parameters could never reproduce it (already established as failing,
  §"Consequences" in the top-level summary).
- **FSDS explicitly never built a custom dynamics model.** Issue #2 and PR
  #63 in FS-Driverless/Formula-Student-Driverless-Simulator confirm the
  maintainers only retuned engine inertia, 0-throttle damping, gear-switch
  time, and differential type on top of the stock UE4 PhysX vehicle
  template, quoting: *"most of the math is inside the UnrealEngine repo and
  I did not want to mess with that."* `docs/vehicle_model.md` states the
  physics engine was deliberately chosen for fidelity high enough that
  *"even the developers of the simulation... would [not] have an edge"* —
  i.e. FSDS was intentionally built to resist exactly the kind of system-ID
  reverse-engineering this investigation has been doing. This is consistent
  with (not proof of) an unexamined, inherited nonlinearity like
  `SteeringCurve` surviving untouched from Epic's template.
- **Precedent for exactly this failure mode already exists in FSDS's own
  issue tracker**, just on the longitudinal axis: issue #342 documents a
  hidden throttle deadzone (torque curve inactive below throttle ≈0.278)
  baked into the UE4 vehicle template that surprised users. No issue,
  discussion, or doc anywhere in the repo names a lateral/steering
  equivalent — this is a structurally similar but *not directly confirming*
  precedent, not independent evidence the same thing happens for steering.
- **Issue #270 independently confirms max steering is hardcoded to exactly
  25°** in `FormulaFrontWheel.uasset` — validates the reference point our
  system-ID ratios are measured against, though it says nothing about
  speed-dependent scaling.

**Attempted to check directly against this repo's own asset — blocked, not
inconclusive.** `FormulaFrontWheel.uasset` and the car Blueprint assets exist
locally, but this repo's Git LFS is not functional here (`git lfs` is not a
runnable subcommand) and every `.uasset` under
`UE4Project/Plugins/AirSim/Content/` is a 129-byte LFS pointer stub, not the
real binary. Even with LFS fixed, a `SteeringCurve`'s keyframes are stored in
serialized binary form inside the asset, not as greppable text — confirming
or ruling this out requires opening the vehicle Blueprint's movement
component in the Unreal Editor and reading the curve directly. This has
**not been done**; report this as untested, not as negative evidence.

**What this changes:** raises `SteeringCurve` (or an equivalent inherited
UE4 template nonlinearity) alongside the already-open reference-heading lead
(§12.8, §14) as a candidate for the residual gap — specifically because it
is a mechanism-level explanation for a *speed-dependent* ceiling that no
plant/tyre parameter tried in §4–§7 could imitate (already measured to fail
cross-validation), whereas a `SteeringCurve` would be speed-dependent by
construction and could, in principle, be measured directly rather than
inferred from closed-loop symptoms. **It does not replace or supersede
anything already found** — the ceiling itself, however it arises internally,
is unchanged by knowing (or not knowing) its implementation, and §13's
ledger result (≥75% of the gap unexplained by any *ceiling parameter*) holds
regardless, since a `SteeringCurve` would be a different, additional/
alternative mechanism to model, not a re-parameterisation of the one already
built. Opening the project in the UE4 Editor to read the curve directly is
the next concrete, low-cost action this section identifies — cheaper than
another sim experiment, and something no amount of further offline
measurement can substitute for.

**Sources:**
[FS-Driverless#270](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/270) (25° max steer confirmed) ·
[FS-Driverless#2](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/2) and
[PR #63](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/pull/63) (no custom dynamic model) ·
[`docs/vehicle_model.md`](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/master/docs/vehicle_model.md) (design philosophy) ·
[NVIDIA PhysX Vehicles guide](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/Vehicles.html) (steer-vs-speed curve pattern) ·
[AirSim `CarPawnApi.cpp`](https://github.com/microsoft/AirSim/blob/main/Unreal/Plugins/AirSim/Source/Vehicles/Car/CarPawnApi.cpp) (no scaling at the AirSim layer) ·
[FS-Driverless#342](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/issues/342) (analogous hidden template nonlinearity, longitudinal axis).

## 19. Root cause of the curvature-spike defect: a hard `min_ahead` cutoff, not cone-map clutter

*(2026-08-07, continued.)* `planning_control_sync.md`'s "Known planner defect"
section named cone-map clutter accumulating over a lap as the likely cause,
and suggested investigating "why lap 2 is worse than lap 1" first. This
section does that, directly on today's actual recorded cone map
(`cone_map.json`, 118 blue + 118 yellow cones) and the new live log
(`mpc_standalone_control_1786066237.csv`), by replaying `build_path_walls()`
at every real car pose from the log — a non-invasive read, no planner file
changed.

**Reproduced immediately.** A tick-by-tick scan (every 0.05 s) around the
tightest corner on the map found the same physical location visited on all
four laps (car returns to within 1.5 m of a fixed point four times, each
time hitting full steering lock). Two of those four visits (laps 2 and 4)
show the fitted centreline's radius jump discontinuously from R≈2.2–2.8 m to
R≈13–14.7 m in a single 0.15–0.2 s step — a >5× radius change while the car
moved <0.15 m — while the other two visits (laps 1 and 3) show radius
tightening smoothly through the same region (R: 10.5 → 0.45 m over 2.6 s, no
jump). Same physical corner, same cone map, two visibly different planner
behaviours depending on exactly where the car's pose lands relative to the
window.

**Root cause, confirmed by comparing the two builds directly.** At the two
consecutive ticks straddling a jump (t=74.83 and t=74.98 in the new log),
`_gen_midpoints()` returns the **exact same 13 midpoints, byte-identical**,
at both ticks — ruling out cone-map duplication, `_absorb()`, or exclusive
nearest-neighbour reassignment as the cause (all three were already
suspected candidates from §15 and §"Known planner defect", and are now
eliminated for this specific mechanism). What changes is which midpoint
survives `filter_cones_window`'s `min_ahead=0.5` forward cutoff as the car's
own position crosses it: at t=74.83 the nearest surviving midpoint sits 0.71
m ahead of the car and anchors a short, gently-curving first segment; 0.15 s
later the car has passed that same point, it drops out of the window
entirely (now behind the 0.5 m line), and the spline's car-pinned first
segment (`pin_start`/`_ANCHOR_WEIGHT=100.0` in `smooth_centreline`) must now
reach the *next* midpoint instead — 4.0 m away, at a different bearing, while
the car itself moved only 0.87 m. The spline bends hard to make that
connection, producing exactly the transient tight-radius artifact measured
here and in the original `planning_control_sync.md` table (R down to 0.45–1.0
m). This is a **near-field discretisation/windowing artifact of the
anchor-to-nearest-midpoint distance**, not degrading cone-map quality — it
is reproducible from a single, static, already-fully-built cone map with no
lap-to-lap accumulation involved at all, which narrows "why lap 2 is worse
than lap 1" to *how often the car's pose happens to straddle a midpoint's
window edge on that lap*, not the map getting dirtier.

**`blend_paths` only partially absorbs it, and never trips its own reset
bypass.** Replaying `blend_paths` sequentially across the same window shows
the blended radius reaches only R=3.19 m at the worst tick (vs. the raw
build's instantaneous 14.66 m) and takes ~0.3–0.4 s to recover — real
damping, not nothing, and it explains why §14 measured 0 reset-bypass events
on this map (the mean resample shift here, 1.3 m, stays under the 2.0 m
`reset_dist` threshold). But it is not sufficient to erase the effect: a
full-run replay of every tick found the blended path's near-field point
(second waypoint) shifts >0.5 m in a single 50 ms step on **22.4% of all
4160 ticks** — well above what car motion alone explains (mean per-tick
displacement 0.43 m, max 0.70 m at the run's top speed, and that upper bound
is for the *anchor point itself*, not the next point down the path). Both
§14's finding (reset bypass never fires) and §14.1's finding (blended-path
heading rate only weakly correlates with rebuild distance, r≈0.10–0.15) are
consistent with this: this is a distinct, narrower, higher-frequency effect
that a coarse per-tick rebuild-distance correlation would not isolate,
because the artifact is highly local (one path point, one brief window) and
largely damped by the time it reaches the heading-rate statistic §14.1
measured a few points further down the path.

**What this does and does not establish.** This identifies a concrete,
reproducible mechanism for a portion of the documented curvature-spike
defect, on real recorded data, without touching any planner file. It does
**not** establish how much of the ≥75%-unexplained saturation gap (§13) this
accounts for — that needs a rollout comparison with the windowing fixed
(e.g. hysteresis on the `min_ahead` cutoff, or excluding the single nearest
midpoint from the anchor-distance jump) against one without, which has not
been done. Per the standing caution in `planning_control_sync.md`, no
planner file has been edited to test this — this section is read-only
instrumentation (see reproduction script below) against the existing
committed/uncommitted code exactly as it stands.

**Reproduce with** (ad hoc script, not yet checked in — the analysis is
straightforward to redo against any `(cone_map.json, control_log.csv)` pair):
call `planning.boundary.build_path_walls()` directly at each logged
`(car_x, car_y, car_yaw)`, track the arg-min forward-distance midpoint's
identity frame-to-frame, and flag ticks where it changes while the car moved
less than the previous midpoint's forward margin.

> **Correction (2026-08-07, same day) — the mechanism is real; the 22.4%
> figure above is not, and the effect is smaller than this section implied.**
> Attempting a fix exposed that the "near-field point shifts >0.5 m per 50 ms
> tick" metric above is dominated by ordinary path resampling, not the
> artifact: `blend_paths`'s `_resample_forward()` re-anchors to the car's
> *current* position and resamples at fixed 0.5 m spacing on every call, so
> "the point 0.5 m ahead along the path" is a different physical point almost
> every tick on any curving section — large apparent jumps in that specific
> metric on both a genuine hairpin (t≈147.5, traced directly, path continuous
> and correct) and nowhere near either flagged anchor-jump instant. Two
> targeted fix attempts against this metric (relaxing `filter_cones_window`'s
> `min_ahead` from 0.5 to −1.0/−2.0 m; relaxing `_build_wall_path`'s seed
> threshold `fwd > 0.3` to −1.0/−2.0/−3.0) both left the metric **unchanged or
> slightly worse**, which is the tell that it was never measuring this
> mechanism. A second, cleaner metric — tracking the blended path's own
> near-field tangent direction (`blended[2]-blended[0]`) tick-to-tick, which
> is what §12.8's reference-heading-lead framework already uses — found only
> **3 single-tick jumps >20° across all 4160 ticks (0.07%)**, and tracing all
> three directly showed they were genuine corner-to-corner transitions (`e_psi`
> and `steer_deg` both reverse sign in step with the tangent, exactly as a real
> S-turn/chicane should look), not artifacts. Re-tracing the *original*
> t=74.83→74.98 jump with this correct metric found a real single-tick tangent
> reversal of **−17.3°** — smaller than the 20° threshold, and an order of
> magnitude smaller than what the flawed 22.4%/"up to 5×radius" framing
> suggested was pervasive. **Net finding: the anchor-to-nearest-midpoint
> mechanism is confirmed real (the t=74.83/74.98 pair is not in dispute — the
> raw builds are byte-identical-midpoints with a genuinely different chain
> output), but on this run it is a modest, occasional effect, not the dominant
> driver of any global path-quality statistic tried so far.** No fix was
> shipped — two parameter-based attempts were tested and rejected because
> they didn't move a metric that, on reflection, wasn't the right one, and
> chasing a third attempt for an effect this size was not judged worthwhile
> before checking with the effect's actual measured scope in hand (see lesson
> 23 below). This does not reopen "cone-map clutter" as the explanation —
> that is still ruled out by the byte-identical-midpoints evidence — it only
> revises how much this specific mechanism matters.

## 20. `SteeringCurve` is a real, compiled-in field — but its value is still unread

*(2026-08-07, continued.)* §18 identified `SteeringCurve` (a speed-keyed
steering-scaling curve, first-class on UE4's `WheeledVehicleMovementComponent4W`)
as a documented PhysX/UE4 pattern matching our measured ceiling signature,
but could not confirm FSDS's vehicle actually carries a non-trivial one — the
source `.uasset` files in this checkout are unpulled Git LFS pointer stubs.

**Checked the shipped Windows binary instead of the source tree.** The
downloaded release (`/mnt/c/Users/marti/Downloads/fsds-v2.2.0-windows`) is
the actual cooked build FSDS runs — a stronger source than source assets,
since it's what executes. Its packed content (`FSOnline-WindowsNoEditor.pak`,
452 MB) is a compressed/serialized archive that generic tools (`strings`,
`file`) cannot parse into asset data — no `UnrealPak`/extraction tool is
available in this environment and installing one is out of scope here. But
the **compiled game binary itself** (`Blocks.exe`) still carries its
reflection metadata as plain strings, and a direct search confirms:

- `SteeringCurve` **is present** as a string in `Blocks.exe` — the property
  name is real and compiled into this exact build, not a hypothetical UE4
  feature that might not be linked in.
- `SteeringInputRate`, `PxVehicleAntiRollBarData`/`mAntiRollBars` (the PhysX
  anti-roll-bar system — a different vehicle-dynamics feature, weight-transfer
  related, not investigated further here) are also present, confirming the
  PhysX Vehicles plugin (already known enabled, §18) is a non-trivial,
  multi-feature integration, not a minimal stub.

**This is not confirmation.** A property name being compiled into the
executable's reflection metadata says the *field exists on the class* — it
says nothing about the *value* baked into FSDS's specific vehicle instance
(a curve can exist and still be flat/disabled, which is the default when a
Blueprint never touches it). That value lives in per-instance binary data
inside the `.pak`, not in the executable's string table, and no tool
available here can read it. **The check §18 called for — opening the
vehicle Blueprint's movement component in the UE4 Editor and reading the
curve directly — is still the only way to resolve this, and remains
undone.**

## 21. The §17 live improvement is NOT the MPC reweight — isolated by reverting weights offline

*(2026-08-07, continued.)* §17 found live steering saturation improved
(21.1% → 15.2%) between the `dfd1a08` baseline and the newest live run, but
could not attribute it: fifteen files of uncommitted changes sit on top of
`dfd1a08`, including a full `Q_diag`/`R_diag`/`R_rate_diag` reweight — the
most likely single suspect by inspection, since a stiffer yaw-rate penalty
plausibly reduces saturation while inducing more correction oscillation
(matching the reversals/s increase, 1.62 → 3.15).

**Directly tested offline — the opposite of the hypothesis.** The offline
sim's weights are already in parity with the *new* live weights (confirmed
identical in `settings.py`). Swapping `settings.py`'s `Q_diag`/`R_diag`/
`R_rate_diag` to `dfd1a08`'s original values and re-running
`tuner.recorded_map_rollout` on today's map, with nothing else changed:

| | new weights (current) | old weights (`dfd1a08`) |
|---|---|---|
| steering sat % | 4.80 | **2.53** |
| reversals/s | 0.80 | 0.79 |

The *old* weights score **better** on saturation offline and are
statistically indistinguishable on reversals. If the reweight explained
live's saturation improvement, the new weights should look at least as good
offline too — they do not, on this map. **The MPC reweight is not the
explanation for either live change.** (Settings reverted to the working
values immediately after this test; no working-tree changes left behind.)

**A better-timed candidate, found while ruling this out: the pose-rate
fix.** `sim_perception.py`'s diff (still uncommitted) documents, with its
own measured evidence, that `sim_perception` used to publish pose and cones
on one shared 10 Hz timer while the MPC ran at 20 Hz — a real 2:1 mismatch
that left the controller re-solving against a stale pose on 50.5% of control
steps, causing measured steering oscillation (catch-up jumps exceeding
`v*dt` on 24 logged steps). That measurement's own source log
(`mpc_standalone_control_1785976976.csv`, epoch 1785976976 = **2026-08-06**)
dates the discovery to *after* `dfd1a08` (2026-08-04, the run that produced
the 21.1% baseline) and *before* today's runs (2026-08-07) — meaning the bug
was almost certainly still active when 21.1% was measured, and fixed by the
time the new 15.2% run happened. This has no offline counterpart to
isolate it against: the offline rollout has no concept of a separate pose
timer at all (it always gives the controller a fresh pose every step, per
the fidelity table in `planning_control_sync.md`), so unlike the MPC
weights this cannot be tested by a settings toggle.

**Status: plausible and well-timed, not proven.** This is inference from
commit/log timestamps and the fix's own documented mechanism, not a
controlled A/B — there is no live rollout with the pose-rate bug
deliberately re-introduced on top of everything else to test in isolation.
The `plan_horizon`/`look_radius` 15→25 m change (also in this file pile, but
already in parity with the offline sim's `_WALL_PLAN_HORIZON=25.0`, so it is
not new relative to what has already been validated) is not a live-only
suspect and is set aside here. **Next step, if this needs a real answer**: a
controlled live run with `pose_rate` deliberately dropped back to 10 Hz
(one-line parameter change, no code revert needed) against the current
code otherwise unchanged, replicating this section's offline A/B but for
the one variable that cannot be isolated offline.

## 22. Pose-rate A/B test: run, and the result confirms the hypothesis — with a wrinkle

*(2026-08-07, continued.)* §21 identified the pose-rate fix as the
best-timed candidate for the §17 confound but could not isolate it offline —
the offline rollout has no separate pose timer to disable. Staged the test
it called for, the user ran it, and the result is in
(`mpc_standalone_control_1786070296.csv`).

**Setup.** `sim_perception`'s `pose_rate`/`cone_rate` are ROS 2 node
parameters with no dynamic-reconfigure callback — the timer period is fixed
once at `__init__`, so a live `ros2 param set` would not have worked; the
value had to be changed at startup. Confirmed the install is
`--symlink-install`ed all the way through, so editing the source YAML
(`fsae_params.yaml`, `sim_perception.pose_rate` 20.0 → 10.0) took effect
immediately with no `colcon build`. File-only edit, uncommitted, no git
command run against the live repo, reverted back to 20.0 immediately after
the run below (confirmed via `git diff` — no leftover trace beyond the
pre-existing session change).

**Result — confirmed real, but the 10 Hz run is not a return to the 21.1%
baseline, it's worse than both:**

| | 21.1% (`dfd1a08`, 10 Hz bug) | 15.2% (today, 20 Hz fixed) | **10 Hz reintroduced (this test)** |
|---|---|---|---|
| steering sat % | 21.1 | 15.2 | **31.9** |
| episode rate | 0.32/s | 0.20/s | **0.67/s** |
| reversals/s | 1.62 | 3.15 | **2.03** |
| \|e_psi\| mean/p90 | 15.9°/42.0° | 12.1°/33.6° | **19.4°/47.4°** |
| a_lat > 7.5 | 9.8% | — | **13.0%** |

Saturation and heading error both move strongly in the predicted direction
(worse at 10 Hz) and by more than the 21.1%→15.2% gap alone — reintroducing
just this one bug overshoots the original baseline rather than merely
recovering it, which is itself informative: whatever else changed between
`dfd1a08` and today (the MPC reweight, ruled out in §21; the `_absorb()` fix;
`plan_horizon`/`look_radius`) was net-positive enough to more than offset a
fully-reintroduced pose-rate bug on top of it. Reversals/s, by contrast, is
*not* monotonic — 10 Hz (2.03) sits between the 20 Hz run (3.15) and the old
baseline (1.62), rather than tracking either cleanly. Saturation and
reversals are not simply two faces of the same mechanism; the pose-rate bug
maps more cleanly onto the saturation/heading-error axis than the
reversal-rate one.

**Directly confirms the user's live observation of the car going off-track
and slowly recovering.** `|e_y|` exceeds a 1.5 m half-track-width threshold
on 1.2% of ticks (max 2.18 m), in episodes rather than isolated spikes — the
worst, t=7.0–8.0 s, is a sustained ~1s excursion with `steer_deg` pinned at
the 25° stop the entire time, `e_y` growing from −1.3 m to −2.17 m while
`e_psi` swings out to −91.7° before slowly recovering back under 1.5 m by
t≈8.1 s. This is the catch-up-jump mechanism from
`planning_control_sync.md`'s "Measurement rate" section operating exactly as
documented: the car saturates steering trying to correct against a partly
stale reference, cannot turn any harder once at the 25° stop, and drifts
wide until the controller's belief catches up.

**Status: the pose-rate mechanism is now measured, not just timed —
promoted from §21's "plausible, not proven."** It is a real, reproducible
contributor to steering saturation and heading error. It is very unlikely to
be the *only* remaining factor, though: even the 15.2% (fixed) run is still
well above the sim's 4.8% baseline, and this single-run A/B (90 s, ~1.3
laps, no repeat) has no error bar — treat the exact magnitudes as directional,
not final, the same caution §13's ledger raised about single-map point
estimates generally.

## 23. A longer 20 Hz run (~5 laps) shows the "improvement" does not hold up — this needs to be said plainly

*(2026-08-07, continued.)* §22's 20 Hz reference point (15.2% saturation)
was a single ~208 s run. The user then ran a longer session
(`mpc_standalone_control_1786076797.csv`, 297 s, same map, `pose_rate`
confirmed at 20.0, no solver failures) — five real laps, not one. The
result revises the picture downward, and should not be smoothed over
because it contradicts the more optimistic framing in §17/§22.

**Headline numbers, same map, same 20 Hz config as the "improved" run:**

| | 21.1% baseline (`dfd1a08`, 10 Hz bug) | 15.2% run (single lap, §17) | **this run (5 laps, 20 Hz)** | sim |
|---|---|---|---|---|
| steering sat % | 21.1 | 15.2 | **26.4** | 4.8 |
| reversals/s | 1.62 | 3.15 | **1.43** | 0.80 |
| \|e_psi\| mean/p90 | 15.9°/42.0° | 12.1°/33.6° | **18.6°/51.8°** | 6.9°/18.5° |

**Saturation on this longer run (26.4%) is worse than both the single 20 Hz
run it's supposed to corroborate AND the original 21.1% baseline it was
meant to have improved on** — despite the same config that produced 15.2%
minutes earlier. Reversals/s, by contrast, is the best of any run measured
so far (1.43, below even the baseline's 1.62).

**A lap-boundary artifact was caught and corrected before trusting the
per-lap breakdown.** An automatic "car returns within 3 m of start" lap
splitter initially reported a 25.2 s "lap 2" at 48.1% saturation sandwiched
between two much longer laps — this is not a real short lap: arc length
travelled in that window is 124 m against ~490 m for an adjacent full lap,
so the car re-entered the 3 m capture radius without completing a loop (the
track likely passes close to its own start twice per lap, e.g. at a
figure-eight-like crossing or a start/finish chute near another straight).
Discarding that split and measuring the three genuine laps directly:
saturation trends downward across the session (30.0% → 19.1% → 24.0%
across three merged segments) but never drops near either the single-run
15.2% figure or the sim's 4.8% — and the last segment ticks back up rather
than continuing to improve. **The lesson generalises beyond this one
metric: any per-lap or per-segment breakdown needs its segment boundaries
sanity-checked against arc length or duration before being trusted, the
same way §13 required checking that two saturation percentages were the
same underlying quantity before subtracting them.**

**What this means for the pose-rate finding (§22) and for §17's "improved"
framing.** §22's A/B result — reintroducing the 10 Hz bug made things worse
than 20 Hz, live, with a directly-observed matching symptom — still stands;
that comparison was against this same run's config, not against a cherry-
picked good run, and the mechanism (stale pose → catch-up jump → saturate →
drift wide) is independently visible in the raw log regardless of what the
aggregate percentage does elsewhere. What does **not** stand is treating
15.2% as "the new normal" for the fixed config: it was one lap out of what
is evidently a noisy, run-to-run-variable process, and a 5-lap sample at the
identical config landed at 26.4% — closer to the original problem than to
an improvement. **Read §17's framing ("saturation improved 21.1%→15.2%") as
superseded by this section**: the correct statement is that saturation
*varies* across roughly 15–32% run-to-run at the current (fixed) live
config, still well above the sim's 4.8%, and no single run so far
establishes a stable new number.

**What generalises from getting this wrong the first time.** §17 drew a
conclusion from n=1. That was flagged as confounded at the time (too many
simultaneous code changes to attribute it to one cause), but the confound
analysis in §21/§22 implicitly treated the 15.2% *value itself* as solid
ground to explain, rather than treating single-run noise as a live
possibility to rule out first. With n=3 real laps in one run plus one
comparison run, the honest range is wide enough that further single-run
A/B tests (like §22's) risk the same trap: a single 10 Hz run reading worse
than a single 20 Hz run is suggestive, matches a real, independently-visible
mechanism, and should not be discarded — but neither number should be
quoted as *the* saturation figure for its condition without more repeats.

**Next, if this needs a firmer answer:** several repeat runs at the current
(20 Hz) config, on the same map, to establish a real mean/spread the way
`VALIDATION_SUITE` does for the offline sim (§13) — a single live run is not
sufficient evidence for or against any fix, this session's own result
included.

## 24. Getting to a point where `SteeringCurve` can actually be read: installing UE 4.27 and building AirLib on Windows

*(2026-08-07, continued.)* §18/§20 identified `SteeringCurve` — a UE4/PhysX
speed-dependent steering-scaling field — as the most concrete remaining
candidate for the measured ceiling, confirmed compiled into the shipped
`Blocks.exe`, but unreadable without the Unreal Editor open on the vehicle
Blueprint. This section is the process of getting there: installing UE 4.27
and building the project from source on the user's Windows machine, so its
own gotchas are documented rather than re-discovered next time. **As of this
section, the build is in progress and not yet confirmed successful** — this
is a process log, not a completed result.

**Which engine version, and why it matters.** `UE4Project/FSOnline.uproject`
pins `"EngineAssociation": "4.27"` explicitly — not a guess. UE5 was
considered and rejected: opening a UE4 project in UE5 forces an asset
upgrade/conversion step that can silently rewrite `.uasset` binary data,
which is exactly the data this whole exercise exists to read unmodified.
Installed via the Epic Games Launcher, matching `docs/software-install-
instructions.md` (root repo) exactly.

**Bug 1 — the initial project folder was never a real git clone.**
The user's first attempt used a folder obtained via GitHub's "Download ZIP"
(named `Formula-Student-Driverless-Simulator-master`, the exact naming
pattern GitHub's zip download produces). `git submodule update --init
--recursive` appeared to succeed (no errors) but changed nothing, because
`git rev-parse --show-toplevel` from inside that folder resolved to
`C:/Users/marti` — an unrelated, pre-existing git repo somewhere up the
directory tree — not the FSDS folder itself. A submodule command run
against the wrong repo silently no-ops rather than erroring, which is what
made this look like a submodule problem at first. **Diagnosed by checking
`git rev-parse --show-toplevel` and `.gitmodules` directly** rather than
trusting the submodule command's silence. **Fixed** by cloning fresh with
`git clone --recurse-submodules
https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator.git`
into a new folder — confirmed correct afterward via the same two checks
(`rev-parse` now resolves to the new folder; `.gitmodules` is present and
lists three submodules: `AirSim/external/rpclib`, `ros/src/fs_msgs`,
`ros2/src/fs_msgs` — note `AirLib` itself is **not** one of these three; it
is tracked as regular files under `AirSim/AirLib/`, which is a distinct fact
from the next bug).

**Bug 2 — the actual missing step was `AirSim/build.cmd`, not a submodule.**
Even with a correct clone, `UE4Project/Plugins/AirSim/Source/AirLib` did not
exist, and the plugin build failed on a missing header
(`common/AirSimSettings.hpp`) after getting partway through compilation —
`UnrealBuildTool` only warns (does not fail immediately) when a referenced
source directory is absent, so the build proceeds until it actually needs a
file from that directory. `docs/getting-started.md` (root repo) documents
this exactly: `AirLib` (the code) and the UE4 plugin (`Source/AirLib`, the
*destination*) are separate, and `build.cmd` is what compiles the former and
`robocopy /MIR`s it into the latter (see `AirSim/build.cmd` lines 113–114).
This step is easy to miss because nothing in the plain `git clone` /
`.uproject`-open flow prompts for it — it is a manual, documented
prerequisite, not something git or UnrealBuildTool does automatically.

**Bug 3 — `build.cmd` hardcodes a CMake generator for the wrong Visual
Studio version.** `build.cmd` line 53: `cmake -G"Visual Studio 16 2019" ..`
(building `rpclib`, `AirLib`'s one true external dependency built via
CMake rather than MSBuild directly). The user has VS2022 installed, not
VS2019 — CMake correctly reported *"could not find any instance of Visual
Studio"* for the 2019 generator name specifically, which does not mean no
Visual Studio is installed, only that the exact generator string doesn't
match anything present. **Fixed** by editing the one live occurrence (line
52's identical string is inside a `REM` comment and doesn't need changing)
from `"Visual Studio 16 2019"` to `"Visual Studio 17 2022"`, after deleting
the stale `external\rpclib\build` directory left behind by the failed
configure attempt (a CMake cache pointing at a generator that was never
found does not self-heal on retry — it must be cleared). rpclib then built
successfully.

**Bug 4 — `AirLib.vcxproj` itself pins the VS2019 (`v142`) platform toolset.**
A distinct issue from Bug 3 (a different project file, a different build
system — MSBuild via `AirSim.sln`, not CMake): `AirLib\AirLib.vcxproj`
hardcodes `<PlatformToolset>v142</PlatformToolset>` (4 occurrences, one per
build configuration). MSBuild's own error names the fix directly
("Retarget solution" / install v142 build tools). Confirmed via `grep -rl
"v142"` that only this one `.vcxproj` in the whole `AirSim/` tree references
it (`AirLib.vcxproj` at the top level and `UnrealPluginFiles.vcxproj` do
not) — so a single find/replace was sufficient, not a per-project retarget.
**Fixed** via a one-line PowerShell replace (`(Get-Content ...) -replace
'v142','v143' | Set-Content ...`) rather than the GUI "Retarget solution"
flow, since only one file needed it. Note this had to be run from a
*separate* plain PowerShell window — the Developer Command Prompt for VS
2022 (needed for `build.cmd` itself, to get `%VisualStudioVersion%` and the
MSBuild environment set up) is `cmd.exe`, not PowerShell, and does not
understand `-replace`/pipeline syntax.

**Status: `build.cmd` completed successfully after Bug 4's fix** — reached
its final `REM //---------- done building ----------` marker, after the
`robocopy /MIR` of `AirLib` into `UE4Project/Plugins/AirSim/Source/AirLib`
and the `copy /y AirSim.props ...` step both completed with no `:buildfailed`
jump. `AirLib` should now exist inside the UE4 plugin folder, which was the
condition the original plugin build (top of this section) was missing.
**Not yet verified:** that the UE4 Editor itself opens and compiles the
`BlocksEditor`/`AirSim` plugin modules clean against this newly-populated
`AirLib` — `build.cmd` only builds the standalone `AirSim.sln` (`AirLib`
plus its CMake dependency, rpclib), not the UE4 plugin/game modules
themselves; those compile separately, inside/via the Editor, the first time
`FSOnline.uproject` is opened (per `docs/getting-started.md`'s step 3,
"When asked to rebuild the 'Blocks' and 'AirSim' modules, choose 'Yes'").
**Next actions:** open `FSOnline.uproject`, allow the prompted module
rebuild, then locate the vehicle Blueprint (likely under
`Content/VehicleAdv/Cars/`, per the folder names already seen in the
LFS-pointer listing in §20), open its `WheeledVehicleMovementComponent4W`,
and read the `SteeringCurve` field directly — the entire point of this
section's work. **A known, separate, not-yet-encountered blocker remains
queued behind this one:** the Git LFS problem from §20 (`.uasset` files as
unpulled pointer stubs) was diagnosed on the WSL/Linux checkout
specifically; whether the user's Windows git client has working LFS support
is untested as of this section — if the vehicle Blueprint or its mesh/curve
assets come up broken/missing in the Editor once it opens, that is the next
thing to diagnose, not a regression in anything fixed here.

**What generalises, provisionally:** every one of the four bugs above
produced an error message that either directly named its own fix (Bug 4) or
was one diagnostic command away from being unambiguous (Bugs 1–3) — none
required guessing. The recurring trap was trusting a command's *silence* as
success (Bug 1's submodule update) or a partial build's *progress* as
evidence a directory really existed (Bug 2, where compilation got 6/11
actions deep before the missing-directory warning became a fatal error) —
consistent with lesson 18 earlier in this document (a timing/progress signal
can mask an address/existence problem) recurring in a completely different
domain (a Windows C++ build, not a WSL network path).

## 25. `SteeringCurve` read directly — the hypothesis is RULED OUT

*(2026-08-07, continued.)* With the build from §24 complete, the user opened
`FSOnline.uproject`, located the default vehicle (`TechnionCarPawn`, under
`AirSim Content/VehicleAdv/Cars/TechnionCar` — not the main project's
`Content/`, since it lives in the AirSim plugin's own content root, which
the Content Browser does not show by default; "Show Plugin Content" has to
be enabled explicitly and the tree/search is scoped per-folder, which cost
some back-and-forth before landing on it), opened its `VehicleMovement`
component (a `WheeledVehicleMovementComponent4W`, confirmed via the
component's own "Vehicle Setup"/"Mechanical Setup"/"Steering Setup"
categories — `Mass 255.0`, `Chassis Width 144.0`, `Chassis Height 112.0`,
4 wheel setups), and read the `Steering Curve` field directly in the
expanded Curve Editor view.

**The curve is FLAT at 1.0 across its entire defined domain.** Three
keyframes, all at Y=1.00: X=0, X=64, X=144 (X-axis units not separately
confirmed, but irrelevant to the shape — a flat line is a flat line
regardless of what the X-axis is measured in). UE4 curves hold their last
keyframe's value for any input beyond the final defined point, so this
curve returns 1.0 for every speed, not just the 0–144 range shown.

**This rules out `SteeringCurve` as the mechanism behind the measured
ceiling.** §18/§20's hypothesis required a curve that tapers *down* with
speed to reproduce the measured signature (ratio 1.00 below 6 m/s, 0.34 at
8 m/s, 0.17 at 14 m/s) — a flat curve at 1.0 applies no speed-dependent
scaling at all, so whatever raw steering value AirSim's `SetSteeringInput`
passes in reaches the wheels unscaled by this particular mechanism,
regardless of speed. The `PxVehicleAntiRollBarData`/anti-roll-bar system
noted in §20 as a lower-priority, not-investigated lead remains exactly
that — untouched by this finding, since it's a physically distinct
mechanism (weight transfer, not steering input scaling).

**What this changes.** The measured lateral-acceleration ceiling (§8–§10)
is real and still unexplained at the engine level — this was always true,
since §18/§20 treated `SteeringCurve` as a *candidate*, never a confirmed
cause, precisely so a negative result here wouldn't need walking back
anything already stated as fact. The remaining candidates from §18 (PhysX
Vehicle SDK sticky-tire-friction thresholds at low speed — already
deprioritised there as implausible at 6–14 m/s; the anti-roll-bar system;
or some other UE4/PhysX internal not yet identified) are now the only
engine-level leads left, and none of them has the same direct, checkable
path this one did — reading a Blueprint curve was uniquely easy compared to,
say, tracing PhysX's own tyre-friction code, which is compiled into the
engine binary, not exposed as an editable Blueprint asset.

**What generalises:** this is the cleanest example in the whole
investigation of a well-specified, falsifiable hypothesis (§18: "if
`SteeringCurve` tapers with speed, that's the mechanism") checked directly
against the one piece of ground truth that could settle it, rather than
inferred from closed-loop symptoms. It took real effort to get to that one
check (§24's four build bugs), and the result was a clean elimination, not
a confirmation — exactly the outcome the effort was worth taking regardless
of which way it went, per lesson 20 earlier in this document ("a strong
circumstantial case is still zero measurements").

## 26. The reference-heading lead, resumed: most of the swing is geometry, but the planner adds a tail excess that predicts saturation directly

*(2026-08-07, continued.)* §14/§14.1/§19 each tested one specific mechanism
that could make the planner's online reference swing faster than pure track
geometry demands (blend-reset bypass, blended-magnitude correlation,
anchor-window jump) and found each too small individually. None of them
asked the more basic question directly: **is the swing measured in §12.8
geometry at all, or is the online planner adding heading-rate on top of
it?** This section answers that directly rather than by elimination, using
a comparison the rollout already computes for an unrelated reason.

**The mechanism.** `sim/rollout_core.py` tracks error against two
independent references on every tick: `e_psi` (the controller's view — the
planner's cone-derived, FOV-limited, EMA-blended centreline) and
`e_psi_true` (the score's view — the fixed, offline-optimal global spline
`path_X/path_Y/path_Psi` that `sim/track_io.py::load_recorded_track` fits
once from the full recorded cone map, never touched by the online planner's
per-tick rebuild). `ref_psi = car_yaw − e_psi` reconstructs what the
planner's online reference was doing; `ref_psi_true = car_yaw − e_psi_true`
reconstructs what a fixed, geometrically-ideal centreline would have
required at the exact same car positions. Comparing `d(ref_psi)/dt` against
`d(ref_psi_true)/dt`, tick for tick, directly separates "the track demands
this heading rate" from "the planner's rebuild adds this heading rate."

**Measured with `tuner/reference_heading_geometry_check.py`** (new, this
session) on the recorded map, same rollout config as §12.8/§14.1:

| \|d(ref_psi)/dt\|, °/s | mean | p90 | p99 | max |
|---|---|---|---|---|
| planner's online reference | 26.1 | 62.5 | 130.6 | 265.0 |
| fixed geometric reference | 21.4 | 49.3 | 69.8 | 75.4 |
| ratio (planner / geometric) | 1.22 | 1.27 | **1.87** | **3.51** |

Correlation across ticks: **0.797**.

**Reading this precisely: the bulk is geometry, the tail is not.** Mean and
p90 sit close to 1.2×, with strong (0.80) correlation — most of the
reference-heading swing §12.8 measured is simply what this track's real
curvature requires, exactly as §14.1's own stated (but previously untested)
interpretation predicted ("more likely intrinsic to the geometry the
centreline is fitting"). But the ratio grows sharply in the tail: 1.87× at
p99, 3.51× at the max. The planner's online reference is not just
inheriting geometry in the tail — it is adding a real, growing excess on
top of it, concentrated in a small number of ticks.

**That tail excess predicts steering saturation directly.** Defining
`excess = |d(ref_psi)/dt| − |d(ref_psi_true)/dt|` per tick: ticks with
`excess > 30°/s` are only **5.8%** of the run (64/1104), but:

| | saturation rate now | P(saturation within next 1 s) |
|---|---|---|
| high-excess ticks (excess > 30°/s) | **42.2%** | **68.8%** |
| all other ticks | 2.3% | 8.0% |

An 18× jump in immediate saturation rate and an 8.6× jump in near-term
saturation risk, from a signal that is directly computable and has nothing
to do with the plant, the ceiling, or `alat_ceiling*` — it is purely a
property of the planner's online reference relative to true geometry.
Tracing the individual high-excess ticks shows them clustered into a
handful of distinct corner-entry/exit events (e.g. t≈5.5–6.55 s, 17.6–18.5
s, 33.45–34.2 s on this map), not a pervasive tick-by-tick artifact — one of
them (t≈6.45–6.55 s) is a sign reversal of the planner's reference rate
(−106°/s → +244°/s across 2 ticks) while the geometric reference itself
stays small and single-signed, i.e. the planner's path briefly reverses
direction relative to itself where the true centreline does not.

**What this does and does not establish.** This directly confirms, for the
first time with a clean measurement rather than by elimination, that (a)
§12.8's "reference swings faster than the car can yaw" is mostly a
geometry fact and NOT primarily a planner defect, but (b) a real,
tail-concentrated planner-added excess exists on top of that geometry, and
(c) that excess is strongly associated with saturation. It does **not**
establish that fixing this excess would close the ≥75%-unexplained gap
from §13 — 5.8% of ticks is a small fraction of the run, and correlation
(even this strong) is not yet a demonstrated closed-loop fix: no planner
change has been attempted, per the standing caution in
`planning_control_sync.md`. It also does not yet identify *which* part of
`centerline_planner.py`/`boundary.py` produces these specific
sign-reversal/excess events — §19's `min_ahead` mechanism is a candidate
(it produces exactly this kind of discontinuous tangent change) but has not
been directly checked against these specific tick ranges.

**Next step, not yet done:** cross-reference the high-excess tick ranges
found here against §19's anchor-jump instrumentation to check whether they
are the same mechanism recurring, or a distinct one — if the same, §19's
"modest, occasional" framing needs revisiting specifically for its
saturation-relevance (not its raw frequency, which is genuinely low, but
its concentration at exactly the moments that matter).

**Reproduce with:** `python3 -m tuner.reference_heading_geometry_check`
(same `MPLBACKEND=Agg` requirement as the other diagnostic scripts in this
document).

## 27. The tail excess is NOT §19's seed-jump — it's a sustained hairpin turn-in lag

*(2026-08-07, continued.)* §26 left the cross-check against §19's
`min_ahead` mechanism as the explicit next step. This section does it,
using `tuner/reference_excess_mechanism_check.py` (new) — a wrapper around
`build_path_walls()` (same non-invasive pattern as §14/§19) that logs, per
tick, the seed midpoint `_build_wall_path` chains from (the exact quantity
§19's mechanism perturbs) alongside the raw centreline's near-field
tangent.

**Result: only 6/64 (9%) of §26's high-excess ticks coincide with a
`>1m` seed-midpoint jump — the great majority show `seed_jump = 0.00`, no
discontinuity in the seed at all.** §19's mechanism is a real but separate,
minor contributor; it is not what produces most of the tail. This also
matches §19's own corrected frequency (0.07% of all ticks, vs. §26's 5.8%
high-excess ticks) — the two were never the same size of effect to begin
with, this section just confirms they are not even the same *ticks*.

**What is actually happening, traced directly on the largest episode
(t=5.40–6.65 s, a hairpin — car decelerates 10.0→3.2 m/s over 1.4 s):**
`e_psi_true` (against the fixed geometric reference) stays smooth and
modest throughout (−6.3° → −20.1°, a real but unremarkable corner-entry
error). `e_psi` (against the planner's online reference) blows out from
−2.5° to −63.4° over the same interval, then snaps back sharply
(−63.4°→−44.5°→−31.4° across the next two ticks). Tracing the planner's
own published near-field tangent directly shows why: it swings from 21.3°
at t=5.25 to 139.9° at t=6.45 — a full 119° rotation — while the car's own
heading only rotates from 17.4° to 80.8° (63°) over the same ticks. **The
online planner's reference is not glitching — it is correctly anticipating
a sharp hairpin earlier and more aggressively than the car has physically
had time to yaw into**, so the reference-minus-car gap grows continuously
for over a second before the car catches up and the gap closes. This is a
sustained lead/lag dynamic, not a discrete artifact — it would not show up
as a "jump" in any single-tick diff, which is exactly why §19's
discontinuity-based instrumentation (built to catch a different failure
mode) missed it.

**What this does and does not establish.** This identifies, for the first
time, the actual shape of the mechanism behind §26's saturation-predictive
tail excess: sustained over-anticipation at sharp corner entries, not a
discrete windowing bug. It does **not** yet show this is wrong or fixable
without cost — a planner that anticipates a hairpin early is not
obviously a defect (arguably desirable preview behaviour), and the
question of whether the *controller* should track that early reference as
tightly as it does (versus some form of curvature-aware reference-rate
limiting, conceptually similar to the existing `SPEED_TARGET_RISE_RATE`
limiter but for heading rather than speed) has not been investigated. No
change has been made to any planner or controller file.

**Confirmed on the other two flagged episodes, same shape.** Checked
t≈17.25–18.55 s (v: 8.05→5.11 m/s) and t≈33.20–34.35 s (v: 9.27→5.41 m/s) —
both braking corner entries. In both, `e_psi_true` stays modest and
bounded (±12–14°) while `e_psi` swings roughly 2–3× wider (−37.1° to
−4.5°, and −33.9° to −1.9° respectively). Same pattern as the hairpin: the
online planner's reference outpaces the car's actual yaw during braking
corner entry, not a one-off. Not yet checked: whether this is universal to
every braking corner entry on the map (these three were selected because
§26 already flagged them as high-excess, not sampled independently), or
whether some corner entries produce no such gap.

**Reproduce with:** `python3 -m tuner.reference_excess_mechanism_check`
(same `MPLBACKEND=Agg` requirement).

## 28. A reference-heading rate limiter: real improvement on the recorded map, a real DNF hazard on the suite

*(2026-08-07, continued.)* §27 identified the mechanism (sustained turn-in
lag) but did not test a fix. This section implements and measures one,
following the same shape as the existing `SPEED_TARGET_RISE_RATE` limiter
(§ references throughout this doc) — cap how fast the reference *rate* is
allowed to change, never the final direction, so the controller is never
asked to snap onto a heading the car has had no time to physically reach.

**Implementation** (`sim/rollout_core.py::_rate_limit_ref_psi`, gated by new
`settings.REF_HEADING_RATE_LIMIT_ENABLED`/`REF_HEADING_RISE_RATE`, default
OFF): applied only in the planner-in-the-loop branch — the fallback/oracle
branches reference the fixed geometric path, which §26 showed does not
carry this excess, so there is nothing to limit there. Unlike
`SPEED_TARGET_RISE_RATE`, this limiter is symmetric (limits swings in either
direction) — there is no "always safe" direction for a heading reference the
way slowing down always is for speed.

**Recorded-map sweep** (`tuner/ref_heading_limiter_ab.py`), saturation vs.
rate limit:

| rate (°/s) | sat % | \|e_psi\| mean | \|e_psi\| p90 | steps | DNF |
|---|---|---|---|---|---|
| OFF (baseline) | 4.62 | 6.92° | 18.46° | 1104 | No |
| 120 | 4.66 | 6.72° | 18.21° | 1117 | No |
| 100 | 4.12 | 6.55° | 18.70° | 1093 | No |
| **90** | **3.07** | **6.43°** | **18.38°** | 1075 | **No** |
| 80 | 2.27 | 6.42° | 18.25° | 1058 | No |
| 70 | 0.10 | 6.16° | 16.48° | 1047 | No |
| 65 | **0.00** | 6.02° | 15.24° | 1046 | No |
| 60 | 0.00 | 2.35° | 11.58° | 125 | **Yes** — off-track (`\|e_y\|`=2.28 m) at step 124 |

On the recorded map alone this looks like an unambiguous win — saturation
falls monotonically to zero at 65°/s with lower heading error too (not just
less saturation, genuinely better tracking), a clean DNF cliff only at
60°/s, and progress ≈0.994 (a complete lap) at every surviving rate.

**Checked against `VALIDATION_SUITE` before trusting it, per the standing
rig-validation lesson (§12.10/§13 — "the absence of this check is exactly
how the 13% sustained-cornering surplus survived a refit").** This is where
the recorded-map picture breaks down:

| | OFF | 90°/s | 70°/s | 65°/s |
|---|---|---|---|---|
| SPIRAL | 11.26% | 0.52% | 0.00% | 0.00% |
| SUDDEN_TURN | 7.59% | 7.51% | 0.00% | 0.00% |
| HAIRPIN | 6.15% | 6.20% | 4.69% | 2.34% |
| FS_CORNER | 2.56% | 0.00% | 0.00% | 0.00% |
| MICRO_SLALOM | 17.39% | 15.89% | **DNF** (`\|e_y\|`=2.41m, 56% progress) | **DNF** (`\|e_y\|`=2.45m, 56% progress) |
| suite mean | 8.99% | 6.02% | — | — |

**Both 65°/s and 70°/s — the rates that looked best on the recorded map —
DNF `PATH_MICRO_SLALOM` off-track, at only ~56% progress.** The recorded map
never surfaces this because it does not contain a slalom-like sequence of
fast direction reversals; the suite does. Narrowing the sweep (75/80/85°/s)
found the DNF persists through 80°/s and only clears at 85°/s — but 85°/s
also *regresses* MICRO_SLALOM sharply (17.39%→27.27% saturation, worse than
doing nothing) while still surviving. **90°/s is the recommended value**: no
DNF anywhere in the suite, a real improvement on the recorded map (4.62%→
3.07%) and the suite mean (8.99%→6.02%), and no per-path regression (every
path is flat-to-better, including MICRO_SLALOM 17.39%→15.89%).

**Why the failure happens.** The limiter holds the reference back during
exactly the sustained turn-in transient §27 identified — correct on a single
corner, where the car eventually catches up. A slalom asks for the opposite
of that: fast, sequential direction reversals with little time between them.
Holding the reference back on entry to reversal N means the controller is
still lagging when reversal N+1 arrives, compounding rather than resolving —
the exact mechanism the "modest, occasional... but disproportionate at the
tail" framing in `planning_control_sync.md` warned would need this kind of
check before trusting a fix.

**What this does and does not establish.** A validated, real, suite-safe
improvement exists at 90°/s: lower saturation and no per-path regression,
confirmed on both the live-comparable recorded map and all 5 synthetic
paths. It does **not** establish this closes any part of the ≥75%-unexplained
gap from §13 at scale — 90°/s's effect (recorded map −1.55 pp, suite mean
−2.97 pp) is real but modest next to live's 21.1% vs. sim's ~5% gap, and
per the standing caution throughout this document, **no live run has been
attempted**. Left `REF_HEADING_RATE_LIMIT_ENABLED = False` by default in
`settings.py` pending that. If enabling for a live test, use 90°/s, not a
tighter value chasing the recorded map's more dramatic (but suite-unsafe)
numbers.

**Reproduce with:** `python3 -m tuner.ref_heading_limiter_ab` (recorded-map
sweep) and `python3 -m tuner.ref_heading_limiter_suite_check`
(`VALIDATION_SUITE` cross-check) — both `MPLBACKEND=Agg`.

## 29. First live test of the rate limiter: it did NOT help — saturation went up

*(2026-08-07, continued.)* `REF_HEADING_RATE_LIMIT_ENABLED` was flipped
`True` at 90°/s in the live `mpc_core.py` (both `ros2/src/fsae_planning` and
the `fsds_simulator` mirror) and the user drove one lap
(`mpc_standalone_control_1786095789.csv`, 84.5 s, 1686 ticks).

**The limiter is confirmed active and doing exactly what it was designed to
do.** `|d(ref_psi)/dt|` max fell from 1508.0°/s (this same day's no-limiter
5-lap run, `...1786076797.csv`) to 220.1°/s — the cap is genuinely
constraining the reference on the real car, not a no-op.

**Saturation went up, not down: 28.0%, vs. 21.1% (§0 original baseline) and
26.4% (this same day's no-limiter 5-lap run).** This is the opposite of the
offline prediction (recorded map: 4.62%→3.07% at the same 90°/s). Per lesson
24 (n=1 is not a finding), this single lap does not on its own prove the
limiter makes things *worse* — but it clearly did not deliver the predicted
improvement, and the failure mode is specific enough to be informative
regardless of sample size.

**Tracing the worst episode (t=6.45–10.23 s, 3.77 s continuously
saturated) shows the exact offline MICRO_SLALOM failure mode, live, just
short of a full DNF.** `e_psi` grows smoothly from near zero to −85.0° while
the car decelerates hard (10.7→2.4 m/s) and steering sits pinned at the 25°
stop the entire time; `e_y` stays modest throughout (≤1.3 m — the car
never leaves the track). This is precisely §12.8's original signature ("the
car is on the line, pointing the wrong way") and precisely the mechanism
§28 measured causing `PATH_MICRO_SLALOM` to run off-track offline: the
limiter holds the reference back during turn-in, so less heading correction
gets applied while there was time for it, and the car arrives at the
corner with a much larger heading deficit to claw back at full lock —
longer, not shorter, saturation episodes (this run's episodes ran up to
3.77 s; the un-limited 5-lap run same day topped out lower per the §23
table). The suite already warned this trade existed; this is the first
evidence it also applies on the recorded map's own track/car, not just
`PATH_MICRO_SLALOM`'s synthetic geometry.

**What this does and does not establish.** It does not disprove the §26/§27
mechanism — the reference genuinely does swing faster than the car can yaw,
that finding stands on its own measurement. It does show that *this
specific fix* (holding the reference back uniformly) trades one failure
mode for a worse one on the live car, consistent with — not contradicting —
the suite-level warning already on record in §28. It also does not yet
distinguish "90°/s is the wrong rate for the live car" from "this class of
fix is wrong regardless of rate" — both remain open. Given the offline
suite already flagged this exact risk before the live test confirmed it,
the appropriate response is not to search for a better rate by further live
trial and error (expensive, and each attempt burns a lap); if this is
revisited, it should be offline first, on a synthetic path that reproduces
this specific "long, smooth heading deficit building through a decelerating
corner" shape, rather than tuning the rate further against the recorded map
alone — the same mistake §28 already corrected once.

**Reverted.** `REF_HEADING_RATE_LIMIT_ENABLED` set back to `False` in
`ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_core.py`
(the live file; the temporary-flip comment is removed). `settings.py` and
`sim/rollout_core.py` were already `False` by default and untouched by this
section.

**Reproduce with:** `MPLBACKEND=Agg python3 -m tuner.live_vs_sim_diagnostics`
against `mpc_standalone_control_1786095789.csv`.

## 30. Combining several small-effect factors at once: still doesn't close the gap

*(2026-08-07, continued.)* §13's ledger tested every plant/ceiling factor
ONE AT A TIME and found each individual effect (0.3–2.1 pp) indistinguishable
from the suite's own 5–6 pp std. That says nothing about whether several
small, independent effects compound when stacked — untested until now.

**Three factors, chosen to target different subsystems (not redundant with
each other, and not re-deriving the ceiling via tyre grip — the mechanism
CLAUDE.md's standing warning is specifically about):**
1. Ceiling level lowered to 6.5 (§13's factor E; already individually
   measured at 4.80%→6.90% recorded-map, no DNF there).
2. `SLAM_NOISE_ENABLED=True` at documented defaults — models real
   localisation error (off by default specifically because FSDS's own pose
   is exact; the real car's is not, which is exactly the asymmetry this
   whole investigation is trying to explain).
3. `CONE_NOISE_ENABLED=True` at documented defaults — models real
   cone-detector position jitter (same off-by-default rationale).

**All 8 combinations, on both the recorded map and `VALIDATION_SUITE`**
(`tuner/combined_factors_sweep.py`, new):

| ceiling=6.5 | SLAM noise | cone noise | rec. map sat % | suite mean | suite std | suite DNF |
|---|---|---|---|---|---|---|
| — | — | — | 4.62 | 8.99 | 5.04 | 0/5 |
| — | — | ✓ | 4.89 | 8.95 | 4.84 | 0/5 |
| — | ✓ | — | 5.41 | 8.60 | 4.62 | 0/5 |
| — | ✓ | ✓ | 5.04 | 9.56 | 4.41 | 0/5 |
| ✓ | — | — | 6.47 | 10.35 | 6.20 | 0/5 |
| ✓ | — | ✓ | 6.03 | 10.08 | 5.63 | 0/5 |
| ✓ | ✓ | — | 6.07 | 10.56 | 5.56 | 0/5 |
| ✓ | ✓ | ✓ | 5.82 | 11.54 | 5.95 | 0/5 |

**No DNF anywhere, and no combination gets close.** The full range across
all 8 cells is 4.62–6.47% on the recorded map — a 1.85 pp spread, smaller
than the suite's own std at every single cell (4.41–6.20 pp). Stacking all
three does not even reliably beat the best single factor alone (ceiling=6.5
alone: 6.47%; all three together: 5.82%, *lower* — the factors are not
additive, and in this combination partially cancel). SLAM noise and cone
noise individually move the number by ±0.5 pp in either direction depending
on what else is enabled — consistent with noise around a symmetric
baseline, not a real directional effect from either.

**What this does and does not establish.** This rules out the specific
combination tested — plant-level ceiling + the two currently-modelled
perception/localisation noise sources — as jointly sufficient to explain any
meaningful fraction of the ~17 pp gap, at each factor's currently-measured
or documented-default magnitude. It does not rule out combining factors at
*more aggressive* magnitudes (e.g. a ceiling lower than 6.5, or noise
levels beyond the documented defaults) — but §12.6 already showed lowering
the ceiling much further trades saturation for a DNF-prone failure mode the
live car doesn't exhibit, so that direction is not free to push. It also
does not test combining a plant factor with a controller/planner-side
factor (e.g. ceiling=6.5 + the §28 rate limiter) — not attempted here since
§29 already showed the rate limiter fails on its own; stacking it with
something else would need to first establish it doesn't independently make
things worse in combination, which was not checked.

**Reproduce with:** `python3 -m tuner.combined_factors_sweep`
(`MPLBACKEND=Agg`).

## 31. A real parity bug in the offline planner harness — the sim has been running an over-smoothed, under-blended centreline this whole time

*(2026-08-08.)* Prompted by a direct question — "did you ever check the
planner itself matches between the live node and the offline sim harness?"
— rather than another closed-loop symptom. Every planner LOGIC file
(`boundary.py`, `cone_sorting.py`, `path_utils.py`, `cone_map.py`) was
confirmed byte-identical (module-path differences aside) across all three
copies, which is not where the bug was. The bug was in **how the offline
harness calls that logic**.

**The bug.** `centerline_planner.py`'s `_compute_path()` calls
`build_path_walls(..., smooth_per_pt=self._smooth_per_pt,
look_radius=self._look_radius, plan_horizon=self._plan_horizon)` and
`blend_paths(..., alpha=self._path_blend, horizon=self._plan_horizon)`,
with all four values sourced from `fsae_params.yaml`'s tuned
`centerline_planner` block (`smooth=0.015`, `look_radius=25.0`,
`plan_horizon=25.0`, `path_blend=0.4`). `sim/sim_track.py::SimPlanner.update()`
called both functions with **no keyword arguments at all**, silently
falling back to each function's own hardcoded defaults:

| parameter | live (tuned) | offline (silent default) | matched? |
|---|---|---|---|
| `smooth_per_pt` | 0.015 | 0.05 (`DEFAULT_SMOOTH_PER_PT`) | **no — 3.3× smoother offline** |
| `look_radius` | 25.0 | 25.0 (hardcoded) | yes, by coincidence |
| `plan_horizon` | 25.0 | 25.0 (hardcoded) | yes, by coincidence |
| `blend_paths` `alpha` | 0.4 | 0.4 (hardcoded) | yes, by coincidence |
| `blend_paths` `horizon` | 25.0 (`plan_horizon`) | 15.0 (hardcoded) | **no** |

Two of five happened to match only because a function's hardcoded default
equalled the live-tuned value — fragile agreement, not real parity. `git
log` shows no sign this was ever intentional; `SimPlanner` was simply never
updated to pass through the same tunables the ROS node exposes as
parameters. **This bug was live for the entire investigation** — every
rollout behind §12–§30, including the reference-heading lead (§26/§27) and
the rate limiter (§28), was measured against an over-smoothed, differently-
blended centreline than what `fsae_params.yaml` actually configures.

**Fixed** in `sim/sim_track.py::SimPlanner.update()`: both calls now pass
the same four values explicitly, sourced from four new constants in
`settings.py` (`PLANNER_SMOOTH_PER_PT`, `PLANNER_LOOK_RADIUS`,
`PLANNER_PLAN_HORIZON`, `PLANNER_PATH_BLEND`) set to match
`fsae_params.yaml` exactly, per the standing rule that copied FSDS/live
settings must be adjustable and documented in one place. (The `settings`
import is deferred inside `update()`, not module-level, because
`settings.py` itself imports `TRACK_HALF_WIDTH` from `sim_track.py` at
module scope — a top-level `from settings import ...` here would be
circular.)

**Effect: large, and not in the reassuring direction.** Recorded-map
baseline (shipped ceiling, factor D): saturation **4.80%→10.42%**, and the
run now **DNFs off-track at step 95 (`|e_y|`=2.37 m, ~4.75 s into the
lap, 10.5% progress)** — a run that previously completed the full lap
cleanly now fails early. The "remaining unexplained gap" from §13
(live 21.1% vs. no-ceiling baseline) narrows from **17.43 pp → 10.68 pp**
— a bigger single-step reduction than every previously-tested plant/ceiling
factor achieved, combined (§13's best-case reading topped out around 3.94
pp). Re-running §13's ledger with the fix: **B/C/D (the three ceiling laws)
are now within 0–1.04 pp of each other** on the recorded map — indistinguishable,
where the ceiling's law used to visibly matter — and **E (ceiling=6.5) now
DNFs 1/5 on `VALIDATION_SUITE`**, where it previously ran clean everywhere.
The harder, correctly-configured planner geometry appears to dominate over
ceiling-law differences that mattered when the centreline was artificially
smoother.

**What this does and does not establish.** This is a real, mechanical bug
fix, not a new hypothesis — the before/after numbers are a direct
consequence of finally calling the shared planner code with the same
arguments on both sides, nothing else changed. It does **not** mean
§13/§26–§30's qualitative conclusions are wrong (the ceiling's law/level/tau
were still not the dominant explanation; the reference-heading lead's
mechanism was still real) — but it does mean **every precise number in
§12–§30 was measured against the wrong planner configuration** and should
be treated as superseded pending re-measurement, not trusted at face value
for anything beyond direction-of-effect. In particular: §26/§27's exact
ratios (1.22/1.87/3.51 etc.), §28's exact sweep values (90°/s, 65-70°/s DNF
threshold), and §30's exact combined-sweep numbers were all measured
pre-fix and need re-running before being relied on again. This also
does **not** establish that 10.68 pp is now "the real gap" — it is the
gap under one arbitrary (if now correctly-configured) weight/ceiling
setting, and the DNF at step 95 means the recorded-map score itself is no
longer a clean completed-lap comparison until either the corner that fails
is understood or the weights are re-tuned against the corrected planner.

**Immediate consequence: the offline tuner's weights may now be stale.**
`Q_diag`/`R_diag`/`R_rate_diag` were tuned (via CMA-ES and manual correction)
against the buggy, over-smoothed planner. Whether they are still a good fit
for the correctly-configured planner — or need re-tuning — is now an open
question this bug fix creates, not one it answers.

**What generalises, immediately:** the exact question that found this
("did you ever check X matches between the two copies?") is precisely
CLAUDE.md's standing parity rule, applied to a piece of code
(`SimPlanner`'s call sites) that is not itself one of the two "kept in
sync" file copies — it's a *third*, independently-written harness that
*calls* the shared logic, and nothing had ever verified it called that
logic the same way. Byte-identical shared files are necessary but not
sufficient for parity; the call sites also have to agree.

**Reproduce with:** `python3 -m tuner.recorded_map_rollout` and
`python3 -m tuner.gap_attribution_ledger` (`MPLBACKEND=Agg`) — both now
reflect the fix.

## 32. A parallel live-side investigation reached the same conclusion independently — and is mid-edit on the exact next lead

A separate, concurrent investigation was running on the live (`fsae_planning`)
side while §1–§31 above happened on the offline side, using real recorded car
logs rather than the offline plant. It independently found and fixed several
of the same mechanisms this document did — steering-rate limit missing
offline, position-update-rate split (10 Hz shared timer → 20 Hz pose / 10 Hz
cones), a units bug inflating logged steering by 2.3×, the scoring-scale bug
where 10 of 12 quality terms were numerically inert, the tiered
safety/laptime/quality scoring rebuild, the `ConeMap` same-frame duplicate-cone
bug, and — independently, via the FSDS `SteeringCurve` engine-level check —
converged on the same "≥75% of the gap is unexplained by the ceiling" number
this document reached in §16 (their numbering)/§13 (this doc's ledger). Their
write-ups are `fsae_planning_changes.md` (repo root) and, mirrored into this
repo, `docs/change_story.md` / `docs/changelog_2026-08-04_to_2026-08-07.md`.

Two things worth recording before they're lost across the two documents:

- **Their investigation ends exactly where this one's §31 begins.** Both
  independently landed on "the leading remaining suspect is a defect in
  exactly how the path-planning software shapes the car's intended route" as
  the open lead, without either side having found the specific `SimPlanner`
  call-site parity bug §31 describes. That bug is new information neither
  write-up has yet — worth surfacing to that investigation rather than only
  recording it here.
- **`curvature_speed`'s braking-distance propagation is being corrected on
  the live side right now, concurrently with this note being written.** As of
  2026-08-08, `fsds_simulator/control/fsae_control/fsae_control/control_utils.py`
  and this repo's own `sim/speed_profile.py` have matching, uncommitted,
  in-progress edits fixing an index-offset bug in how a curvature sample's
  distance-ahead is computed for the braking-distance propagation added in
  their Part 7 / this doc's braking-distance fix (the old code assumed a fixed
  `+2` sample-index offset that was wrong by a few metres in both directions
  depending on which sampling branch ran; the fix tracks the true arc-length
  offset via a new `pts_s0`/`kappa_at` bookkeeping pair, and makes a
  corner-entry braking margin explicit instead of accidental). **This means
  every rollout in this document that calls `sim.rollout_core.run_core_rollout`
  with `use_planner=True` — i.e. essentially every recorded-map and
  `VALIDATION_SUITE` measurement in §12–§31 — runs through a `curvature_speed`
  that is mid-edit as of this writing.** Do not run a re-measurement pass
  until that edit lands and is confirmed syntactically and behaviourally
  complete; a measurement taken mid-edit would be stale before it's even
  written down. `fsae_MPCTest/sim/speed_profile.py` and its `fsds_simulator`
  mirror were the only two files with uncommitted changes at the time this
  note was written — no other file in this repo was touched by that
  concurrent session.

**What this means for the "re-measure §13/§26-30 against the §31 fix" plan**
that was the standing next step: hold it until the concurrent
`curvature_speed` edit is committed (check `git status` /
`git diff --stat` in `fsae_MPCTest` for a clean tree on those two files), to
avoid measuring — and writing into this document as if settled — numbers from
a half-landed function.

**Update, same day: the concurrent edit landed** (committed `51deba0`,
verified AST-identical between `sim/speed_profile.py` and the
`fsds_simulator` mirror, ignoring docstrings). Re-measurement resumed — see
§33.

## 33. Re-measurement against §31 + the braking-distance fix: the recorded map still DNFs, and a new, distinct short-path-cap mechanism is why

With both fixes landed, `recorded_map_rollout` and `gap_attribution_ledger`
were re-run.

**Headline numbers (recorded map, shipped ceiling law):** steering
saturation 10.42% (was 4.80% pre-§31), `|e_psi|` mean/p90 6.09/20.05° (was
6.9/18.5), `a_lat` max 10.42, reversals/s 1.25 — all essentially unchanged
from the immediate post-§31 numbers in the table at the top of this
document. **The recorded map still DNFs at step 96 (10.5% progress)**, one
step later than pre-braking-fix (was 95) — the braking-distance index-offset
fix did not move this failure at all.

**Ledger re-run, factors B/C/D (the three ceiling laws) are now
statistically indistinguishable from each other on the recorded map** — all
three read 10.42%, where they previously showed measurable spread. Against
the freshly-measured no-ceiling baseline (9.38%, itself different from any
historical "4.4%"/"9.38%" figure quoted before this fix — see the ledger's
own printed warning not to mix them), the ceiling now explains at most
1.04pp (factor B only; C and D explain 0.00pp) of an 11.73pp live gap —
**10.68pp (91.1%) remains unexplained**, essentially unchanged from the
pre-braking-fix state. The braking-distance fix is a genuine correctness fix
(confirmed AST-identical to the live copy) but does not move this
particular metric.

### Root cause of the new DNF: a short-path speed cap masking a real corner until it's too late

Traced directly (car state history, not aggregate stats): the rollout drives
cleanly to 14.8 m/s by t=3.0s, then between t=3.25–4.75s `e_psi` explodes
from -3.9° to -65.6°, steering pins at the 25° stop, and the car goes
off-track shortly after — the same "heading runaway while `e_y` stays
modest" shape as every earlier saturation episode in this document.

The mechanism is **not** a repeat of the braking-distance index-offset bug
just fixed, and **not** exactly the same as the `min_ahead` seed-jump
(§19) or the turn-in-lag tail (§27) already documented. It's a third,
previously-undocumented sub-mechanism in the same family:

- The published centreline (`planner_X`/`planner_Y` per step) shrinks
  steadily as the car approaches the corner — 18.9 m at t=3.5s down to
  8.9 m by t=4.4s — well short of `PLANNER_PLAN_HORIZON=25.0`. This is
  `build_path_walls()` genuinely running out of walkable boundary that far
  ahead through a tight bend, not a bug in the §31 kwarg-passing fix (the
  four `PLANNER_*` params are confirmed correctly threaded through both
  call sites).
- `curvature_speed()`'s short-path cap, `v_max_eff = max(v_min, v_max *
  min(1.0, total_len/scan_end))`, then dominates `v_target` while the path
  is short: at `v_max=20.0` (the offline planner branch's `PLANNER_V_MAX`,
  confirmed at `sim/rollout_core.py:63` — not the module's own
  `v_max=15.0` default, which a naive direct repro of this call gets
  wrong) and `scan_end=24.0`, a ~10 m path caps `v_target` at only
  ~8.3 m/s. Verified directly: calling `curvature_speed()` on the exact
  per-step sliced waypoints (`cl[cl_idx:]`, not the full published
  centreline stored in history) reproduces `v_target` within rounding at
  every checked step.
- The near-field curvature at these same steps is genuinely tight (R≈4.5m,
  triples 1-3 of the published path, consistent step to step — not a
  spurious spike) and implies `v_corner=√(4×4.5)≈4.2 m/s`, but the
  braking-distance propagation never gets to bind on it, because the
  short-path cap is already the more restrictive term while the path is
  short. Only once the path shortens further (step 88, 8.88 m) does the
  genuine curvature-aware term finally take over — and by then the car is
  still at 10.0 m/s with only ~9 m of remaining path to shed 6+ m/s in,
  which `A_BRAKE_PLAN=5.0` cannot do in time.

In short: **the short-path cap is a coarse, distance-blind stand-in for "I
can't see far enough to trust a speed target," and it silently masks a
genuine, correctly-detected tight corner for exactly as long as the path
stays short — which is precisely the regime where the corner is closest and
the masking is most dangerous.** This is a distinct failure mode from a
curvature *value* artifact (§19) or a reference-heading *lag* (§27): here
the curvature measurement is correct and stable, but a different, unrelated
term (the length-based cap) overrides it until too late.

**Not attempted:** a fix. Per CLAUDE.md's standing instruction on this
family of defects, this is exactly the kind of planner-adjacent change that
needs to be applied to both `fsae_MPCTest` and the live
`fsae_planning`/`fsds_simulator` copies together, and the existing
workarounds (curvature smoothing, tracking-error gate, speed-rise limiter)
are defence in depth that would need re-checking against any change here,
not assumed to still be correctly tuned. Candidate directions, not
evaluated: (a) blend the short-path cap with the curvature-based term
(e.g. take the min only when the path's near-field curvature is itself
unremarkable, rather than always deferring to path length first), (b)
investigate why `build_path_walls()` can't find a walkable boundary out to
`plan_horizon` through this specific corner — a cone-density or
`_WALL_MID_DIST` issue would be a different, and more fixable, root cause
than "tight corners inherently truncate the boundary walk."

**Reproduce with:** `python3 -m tuner.recorded_map_rollout` and
`python3 -m tuner.gap_attribution_ledger` (`MPLBACKEND=Agg`) for the
headline numbers; the per-step trace above was produced with a one-off
script (not committed) that calls `run_core_rollout(..., want_history=True)`
and inspects `history["v"]`, `history["v_target"]`, `history["e_psi"]`, and
`history["planner_X"/"planner_Y"]` around the DNF step.

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
17. **"Out of scope" is a claim about priority, not about risk.** §12.11 noted
    `launch_all.sh` had the same shebang defect as the harnesses being fixed
    and left it alone. It went on to cause a real launch failure (§16), not a
    hypothetical one — the defect doesn't care which script it's in. A noted-
    but-deferred bug is still a live bug; it surfaces on its own schedule, not
    the investigation's.
18. **A timing fix can mask an address fix, and look like it worked.** The
    original symptom (`fsds_ros2_bridge` crashing with "Failed connecting to
    RPC server") was diagnosed as a race and fixed with a longer wait — a
    reasonable read of the evidence at the time, and it never got the chance
    to be tested cleanly, because the *shebang* bug (§16) made the wait loop
    fail unconditionally before the real, unrelated address bug (WSL cannot
    reach the Windows host via `127.0.0.1`) could even be observed. Two
    independent bugs stacked behind one symptom; fixing the outer one was
    necessary to see the inner one at all, and every fix short of the last
    one produced a plausible-looking but still-broken result.
19. **An improved live number is not evidence for whichever fix you were
    thinking about.** §17's saturation improvement (21.1% → 15.2%) landed
    on top of fifteen uncommitted, simultaneous changes — a full MPC
    reweight, a real cone-map bug fix, and edits to three other files.
    Without isolating variables, a better number cannot be attributed to a
    specific cause, no matter how mechanistically plausible that cause
    sounds — and the same run's reversals/s got worse, which a
    single-metric read would have missed entirely.
20. **A mechanism can be plausible, documented, and still unverified —
    say so.** §18 found a specific, named UE4/PhysX feature
    (`SteeringCurve`) that would produce the measured signature by
    construction, with supporting circumstantial evidence (FSDS never
    built a custom dynamics model; a structurally similar hidden
    nonlinearity is already confirmed on the throttle axis). None of that
    is a substitute for opening the asset and reading the curve. A strong
    circumstantial case is still zero measurements.
21. **A documented "likely cause" is a hypothesis, not a finding — re-derive
    it from data before propagating it further.** `planning_control_sync.md`
    named cone-map clutter as the probable driver of the curvature-spike
    defect, reasonably, from a lap-1-vs-lap-2 correlation. §19 found the
    actual mechanism (a `min_ahead` window-edge cutoff) reproduces on a
    single static cone map with no accumulation at all — a different cause
    that happened to correlate with the same lap-number variable, because
    laps differ in exactly where the car's pose lands relative to a
    midpoint's boundary, not in map quality.
22. **The most likely-looking suspect in a pile of simultaneous changes can
    be wrong — test it, don't just name it.** §17 flagged the MPC reweight
    as the probable cause of a live improvement because the story fit
    (stiffer yaw-rate penalty → less saturation, more oscillation). §21's
    offline isolation showed the *old* weights score better on the exact
    metric the new ones were credited for. The correct suspect (pose-rate
    mismatch) surfaced only from checking commit/log timestamps against every
    candidate, not from picking the most mechanistically plausible one.
23. **A metric that doesn't move when you fix the thing it's supposed to
    measure is not confirming the fix failed — check whether it was ever
    measuring that thing.** §19's "22.4% of ticks show a >0.5 m near-field
    jump" survived two independent, targeted fix attempts completely
    unchanged, which should have been the first clue rather than a reason to
    try a third fix. The metric was dominated by ordinary arc-length
    resampling on a curving path, not the artifact it was built to catch. A
    second, better-chosen metric (near-field tangent direction, checked
    against `e_psi`/`steer_deg` to confirm each flagged event was real and
    not a legitimate corner) found the true effect was real but roughly two
    orders of magnitude smaller in incidence than the first metric implied
    (3 events / 4160 ticks, not ~933). Build the measurement, then sanity
    check it against ground truth *before* trusting its silence or its
    alarm — in either direction.
24. **n=1 is not a finding, even when the story fits.** §17 reported
    saturation "improved" 21.1%→15.2% from a single run and flagged the
    confound; §21/§22 then spent real effort explaining that one number
    (correctly ruling out the MPC reweight, correctly confirming the
    pose-rate mechanism) without first asking whether 15.2% itself would
    replicate. A 5-lap run at the identical config landed at 26.4% — worse
    than the number being explained. The pose-rate mechanism is still real
    (independently visible in the raw log, not just in an aggregate
    percentage), but the specific value it was measured against turned out
    to be one noisy sample, not a stable baseline. Attribute causes to
    mechanisms visible in the data, not to whichever single aggregate
    number happened to be measured first.
25. **An error message that names its own fix should be trusted over a
    plausible-sounding guess.** Every bug in §24 (a wrong git-repo root, a
    missing manual build step, a wrong CMake generator name, a wrong
    MSBuild platform toolset) either stated the fix directly in its own
    output (`"could not find any instance of Visual Studio"`, `"Retarget
    solution"`) or was resolved by one targeted diagnostic command
    (`git rev-parse --show-toplevel`, `findstr /n`, `grep -rl`) rather than
    by trying candidate fixes and seeing what stuck. Read the exact error
    text and act on it before reaching for a broader, slower remedy (e.g.
    installing a second Visual Studio version, which was available as an
    option but not needed here).

26. **Test the question directly before hunting for it by elimination.**
    §14/§14.1/§19 each ruled out one specific mechanism that could make the
    planner's reference swing faster than geometry, without ever computing
    "how fast does geometry alone actually demand" as its own number — the
    rollout had carried that exact fixed reference (`e_psi_true`) the whole
    time, for a different purpose (scoring). §26 answered in one script what
    three sections of elimination could not: most of the swing is geometry
    (ratio ≈1.2, r=0.80), and the real planner-added defect is a
    tail-concentrated excess, not a broad property — which also explains why
    every prior *aggregate* correlation (§14.1's r≈0.10–0.15) looked weak
    despite a real, saturation-predictive effect being present in 5.8% of
    ticks.

27. **A discontinuity-detector cannot find a mechanism that has no
    discontinuity.** §19's instrumentation was built to catch tick-to-tick
    jumps, and correctly found few (0.07%). §26 found a different, larger
    effect (5.8% of ticks) using a completely different measurement (excess
    over a fixed geometric reference) — and §27 confirmed by direct
    cross-check that the two are mostly disjoint sets of ticks (9% overlap),
    not the same mechanism measured two ways. A sustained lead/lag (the
    reference smoothly outpacing the car for over a second, then
    correcting) produces no large single-tick delta anywhere in the middle
    of the episode; only an integral-style measurement (excess accumulated
    against a stable reference) surfaces it. Matching the instrument to the
    suspected failure *shape*, not just re-running an existing instrument
    on new data, is what found this.

28. **The one live-comparable track is not a substitute for the synthetic
    suite, even when it is the only ground truth you have.** §28's rate
    limiter looked like a clean win on the recorded map — saturation to
    zero, heading error also improved, no DNF until a sharp cliff at 60°/s.
    `VALIDATION_SUITE` told a different story: the two best-looking rates
    (65, 70°/s) both DNF `PATH_MICRO_SLALOM` off-track, because the recorded
    map contains no fast sequential-reversal geometry to expose it. This is
    the same lesson §12.10/§13 already learned about the plant/ceiling
    parameters ("the absence of this check is exactly how the 13%
    sustained-cornering surplus survived a refit"), now confirmed to apply
    just as much to a planner/controller-side fix — a good idea with a
    correct mechanism behind it still needs the full suite before being
    trusted, not just the one map with live data to compare against.

29. **A correct mechanism does not guarantee a correct fix, and the suite
    already told us so before the live test confirmed it.** §26/§27's
    measurement (the reference genuinely outpaces the car's yaw) was never
    in question — §29's live test doesn't touch it. What failed was the
    specific intervention (§28's uniform rate limiter), and its failure mode
    on the real car (a 3.77 s saturation episode from a growing heading
    deficit) is the *same* mechanism §28 already caught offline on
    `PATH_MICRO_SLALOM`, just short of a full DNF. The suite-level warning
    ("do not tighten this further") was correct, and the live result adds
    "this rate, on this track, already crosses the line the suite warned
    about" rather than being a surprise. Cheap offline signal (a DNF on a
    synthetic path) generalised correctly to an expensive live signal
    (worse saturation on the real car) — exactly the payoff §12.10's
    validation tooling was built for, and a reason to trust suite warnings
    even when the recorded map itself looks fine.

30. **Small non-redundant effects do not automatically compound — check
    before assuming they do.** It is intuitive that several individually-
    marginal factors might add up to something real when combined, but
    §30 measured this directly rather than assuming it: three factors
    targeting different subsystems (plant ceiling level, localisation
    noise, cone-detection noise) summed to a smaller combined range
    (1.85 pp) than any one factor's own uncertainty band (4.4–6.2 pp
    suite std), and the largest combination (all three) scored *below*
    the single strongest factor alone — evidence of partial cancellation,
    not reinforcement. Whether small effects compound, cancel, or do
    neither is an empirical question specific to the mechanisms involved,
    not something to assume in either direction without measuring the
    actual combination.

31. **"The shared files are byte-identical" is not the same claim as "the
    two systems behave the same."** Every parity check in this document
    before §31 compared FILES (are `boundary.py`/`cone_map.py`/etc.
    identical across copies?) and every one passed. Nobody had checked
    whether the two independent CALL SITES — `centerline_planner.py`'s ROS
    node and `sim_track.py`'s plain-Python harness — invoked that identical
    shared code with the same arguments, and they didn't (offline silently
    used two hardcoded defaults instead of the live-tuned values). This
    survived the entire investigation because the question that surfaces it
    ("does the offline harness call the planner the same way the live node
    does?") is different in kind from "is this file the same as that file?"
    — a fair question to ask about any shared-logic-but-separate-harness
    boundary, not just this one, and worth asking explicitly rather than
    trusting that byte-identical dependencies imply identical behaviour.

---

## Open / deferred

> **§12–§30 predate §31; §33 is the current re-measurement.** A real parity
> bug in `sim/sim_track.py::SimPlanner` (offline never passed the live-tuned
> planner smoothing/blend parameters) was fixed 2026-08-08 (§31), and a
> concurrent live-side fix to `curvature_speed`'s braking-distance index
> offset landed the same day. §33 re-measured against both: recorded-map
> saturation is 10.42% (steady, not further moved by the braking fix), the
> DNF at ~step 95-96/10.5% progress persists, and the unexplained-gap
> fraction is still ~91% (10.68 of 11.73 pp). §33 also identifies the DNF's
> specific new mechanism (a short-path speed cap masking a correctly-detected
> tight corner). Treat every precise percentage/ratio in §12–§30 as
> directionally informative, not numerically current.

| item | status |
|---|---|
| **Model the yaw cap offline** | **Done** — `alat_ceiling*` in `model/vehicle_physics.py`. Moves every metric toward the car (saturation 4.4→6.3%, a_lat max 14.06→10.53) but closes only part of the gap; live is still 21.1% saturation |
| **Planner parity bug (`SimPlanner` call sites)** | **Fixed (§31), 2026-08-08.** Offline never passed `smooth_per_pt`/`look_radius`/`plan_horizon`/blend `alpha`/`horizon` to `build_path_walls()`/`blend_paths()`, silently using hardcoded defaults instead of `fsae_params.yaml`'s tuned values (2 of 5 coincidentally matched). Fix narrows the "unexplained gap" 17.43→10.68 pp in one step — bigger than every previously-tested plant/ceiling factor combined — and introduces a new recorded-map DNF at step 95. Weights may need re-tuning against the corrected planner |
| **Recorded-map DNF (post-§31/braking-fix)** | **Root-caused, not fixed (§33), 2026-08-08.** A short-path `v_max_eff` cap in `curvature_speed()` masks a correctly-detected, stable-radius tight corner (R≈4.5m) for as long as the published centreline stays short (`build_path_walls()` returns ~9-19m, well under `plan_horizon=25`), then the genuine curvature term takes over too late to brake in time. Distinct from §19's `min_ahead` seed-jump and §27's turn-in lag — the curvature measurement itself is correct and stable here; an unrelated length-based term overrides it. No fix attempted; two untried candidate directions noted in §33 |
| **Braking-distance index-offset fix (`curvature_speed`)** | **Landed (concurrent live-side session), 2026-08-08.** Fixed a `+2`-sample-index assumption that mis-attributed each curvature sample's distance-ahead by a few metres either direction. Verified AST-identical to the `fsds_simulator` mirror. Confirmed via §33 re-measurement: does not move recorded-map saturation (10.42% before and after) or the DNF step (95→96, unchanged in substance) |
| Remaining saturation gap | **Open, re-measured against both the planner fix and the braking-distance fix (§13, §31, §33).** Pre-§31: ≥75% of a 17.43 pp gap unexplained. Post-§31+braking-fix (§33): 91.1% (10.68 of 11.73 pp) unexplained, against a run that still DNFs at ~10.5% progress — neither fix moved this fraction. §30's combined-factor sweep still predates both fixes and needs re-running if pursued further |
| Ceiling's effect is corner-type-specific | **New (§13).** Zero measurable effect on HAIRPIN/FS_CORNER in every configuration tried; entire effect concentrated on SUDDEN_TURN/MICRO_SLALOM (sustained moderate-radius bends at speed). Check any future plant explanation against this per-path breakdown before trusting an aggregate |
| **`alat_ceiling_tau`** | **Done (§12.12).** Measured 0.35 s median (0.28–0.46, 11/12 trials) with `step_s=8.0`; model set to 0.40. Fixing it did **not** close the saturation gap — the residual is elsewhere |
| Ceiling is speed-dependent | **New, unmodelled (§12.4).** Measured sustained a_lat rises with speed — 6.45 @ 8 m/s, 7.54 @ 11, 9.26 @ 14 — while the model pins it flat at 7.5. Residuals +1.0 / ~0 / −1.76. Deliberately not fitted: 16 points, one run |
| Step vs sweep disagree on level | **Open.** Step's 3 s settle says 7.5 @ 8 m/s; the sweep's long orbit says 6.45. A longer `step_s` resolves whether sustained a_lat keeps decaying past 3 s — same experiment as the `tau` re-measurement |
| Planner reference heading | **Open — mechanism confirmed real, first candidate fix tried live and FAILED (§12.8, §14, §26, §27, §28, §29).** In *both* stacks the reference heading swings faster than the car can yaw, driving 78–100% of heading-error growth. §26: most of that swing is geometry (ratio ≈1.2 mean/p90 vs. a fixed geometric reference, r=0.80) — but a tail excess (ratio 1.87 p99, 3.51 max, 5.8% of ticks) is planner-added and predicts saturation directly (42.2% vs 2.3% immediate rate). §27: NOT §19's seed-jump — instead a sustained turn-in lag at braking corner entries. §28: a symmetric reference-rate limiter at 90°/s cut recorded-map saturation 4.62%→3.07% and suite-mean 8.99%→6.02% with no DNF in `VALIDATION_SUITE`, but tighter rates (65–70°/s) DNF `PATH_MICRO_SLALOM`. **§29: tried live (one lap, 90°/s) — saturation ROSE to 28.0% (vs. 21.1%/26.4% no-limiter baselines).** Confirmed active (max ref-heading rate capped 1508→220°/s) but traced to the same failure mode §28 found on `PATH_MICRO_SLALOM`: holding the reference back during turn-in produces a larger heading deficit to claw back later (one 3.77s saturation episode observed). Reverted live. The §26/§27 measurement itself is not in question — only this specific fix — so the mechanism remains open and unfixed |
| `blend_paths` reset-bypass discontinuity | **Eliminated for the recorded map (§14).** Real, parity-correct mechanism, can jump the reference up to 166° on other geometries (PATH_SPIRAL) — but fires 0/1038 times on the recorded map (max trigger-distance 1.98 m, just under the 2.0 m threshold). Cannot explain 21.1% saturation with 0 events |
| `blend_paths` blended-magnitude vs heading rate | **Mostly eliminated (§14.1).** Rebuild distance vs. blended path's own reference-heading rate: r=0.15 raw, r=0.10 controlling for corner severity — explains ~1-2% of variance, not the 78–100% reference-driven growth in §12.8. Points back at the planner's spatial fit itself (curvature-spike defect), not `blend_paths`, as the likely source of the heading-rate symptom |
| `ConeMap._absorb()` same-frame duplicate bug | **Fixed (§15), unmeasured effect.** Real, deterministic bug (two same-frame detections of a newly-sighted cone both became permanent, unmerged entries — confirmed at 1 cm apart, independent of `MERGE_DIST`). Fixed in all 3 copies. Does not move the recorded map's saturation at default noise (4.80%→4.86%, matches pre-fix), because FSDS's noise-free oracle perception never triggers it and the added `CONE_NOISE_ENABLED` jitter is too small to separately trigger it either. Whether this matters on the real car is still open — needs measured detector noise or a live log |
| Cone-detection noise model | **New capability (§15), not yet a finding.** `CONE_NOISE_ENABLED`/`CONE_POS_JITTER_STD`/`CONE_NOISE_SEED` added to `settings.py` + `sim/rollout_core.py::ConeNoise` — closes part of the "cone map... not modelled anywhere" fidelity gap (position jitter only; false positives/negatives/range-dependence remain unmodelled). Default off. `fsae_MPCTest`-only, no live-side counterpart needed (same as `SLAM_NOISE_*`) |
| `launch_all.sh` couldn't get FSDS running at all | **Fixed (§16).** Two stacked bugs: shebang on line 3 (script never ran as bash), then WSL2 unable to reach the Windows host via `127.0.0.1` (needs the WSL default-gateway IP, or `FSDS_HOST_IP` — `fsds_ros2_bridge.launch.py` already supported this, just was never set). Fixed in both `ros2/launch_all.sh` and the `fsds_simulator` mirror, preserving the mirror's intentional config differences |
| First live-vs-sim comparison since §0 | **Superseded (§23) — the "improvement" does not replicate.** §17's single-run 15.2% figure does not hold: a 5-lap run at the identical 20 Hz config landed at 26.4%, worse than the number it was meant to have improved on. §21 still correctly ruled out the MPC reweight as an explanation (offline A/B, old weights score *better* on saturation — that finding does not depend on which live run is "the" baseline). Correct current statement: live saturation varies roughly 15–32% run-to-run at the fixed config, well above the sim's 4.8%, no single run yet establishes a stable number. Needs several repeats to get a real mean/spread, the same way §13's `VALIDATION_SUITE` does offline |
| `SteeringCurve` (UE4/PhysX speed-dependent steering scaling) | **RULED OUT (§25).** Read directly in the Editor on `TechnionCarPawn`'s `WheeledVehicleMovementComponent4W`: the curve is flat at Y=1.00 across all 3 defined keyframes (X=0, 64, 144) and holds 1.0 beyond that by UE4's default curve extrapolation. No speed-dependent steering scaling from this mechanism. The ceiling's engine-level cause is still unidentified; remaining untested candidates (PhysX anti-roll-bar system, sticky-tire-friction thresholds) are lower-priority and less directly checkable than this one was |
| Curvature-spike defect | **Mechanism confirmed real (§19), but confirmed MODEST, not pervasive — corrected same day.** Root cause: the car-anchored spline's nearest midpoint drops out discontinuously as the car's pose crosses it, confirmed via byte-identical midpoint sets across a reproduced jump (rules out cone-map duplication/`_absorb()`/NN-reassignment). The initial "22.4% of ticks" estimate was an artifact of the measurement (ordinary arc-length resampling, not the defect) — two fix attempts against it correctly showed no effect, which is what exposed the flawed metric. A corrected metric (near-field tangent direction, cross-checked against `e_psi`/`steer_deg`) found only 3 large single-tick jumps in the whole run (0.07% of ticks), all genuine corner transitions, and re-measured the original instance at a real but modest −17.3° tangent reversal. Still supersedes "cone-map clutter" as the cause of *this* mechanism. No fix shipped — not judged worthwhile at this measured size; needs revisiting if the pose-rate test (§22) or another lead reopens the question |
| Pose-rate mechanism | **Confirmed as a real, independently-visible mechanism (§22) — still true after §23.** Reintroducing the 10 Hz pose bug reproduced the user's directly-observed symptom (car runs wide off-track, slowly corrects back — confirmed in the log as a sustained ~1s excursion, `|e_y|` to 2.18 m, steering pinned at the 25° stop throughout). This holds regardless of §23's correction, since it's visible mechanistically in the raw log, not just via the aggregate percentage it was originally compared against. What does NOT hold: quoting "31.9% vs 15.2%" as the effect size — 15.2% was one noisy sample (§23). `pose_rate` reverted to 20.0 after the test. Needs repeat runs at both 10 Hz and 20 Hz to size the real effect |
| Run-to-run variance at the live 20 Hz config | **New, open (§23).** Saturation measured at 15.2% (n=1 lap), 26.4% (n=5 laps, same config, same day), with the 5-lap run's own three real laps ranging 19.1–30.0%. No repeat-count yet establishes a trustworthy mean; treat any single live saturation number quoted elsewhere in this document as one sample from this spread, not a fixed value |
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
