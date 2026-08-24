# fsae_planning sim-to-real investigation, part 1: 2026-08-04 to 2026-08-07

Covers 61 commits (`41309fc` → `3b1bedd`) in `fsae_MPCTest`, the offline
driving simulator/tuner. Preceded by a five-day gap (last commit before this
run: 2026-07-30), which is why the log starts here.

Goal of this stretch: get the offline simulator into a state where it can be
trusted to predict how the real car drives. It ends on an open question, not
a resolution — see [Stage 6](#stage-6-2026-08-06-the-real-vs-offline-driving-gap).

## Index

| Stage | Date(s) | What happened |
|---|---|---|
| [1](#stage-1-2026-08-04-tooling-and-setup) | 08-04 | Re-synced offline code against the live copy; fixed setup bugs; got a real recorded track loading and replaying correctly (5 bugs) |
| [2](#stage-2-2026-08-04-steering-wobble-first-pass) | 08-04 | First pass at steering wobble — rebalanced cost weights, improved the wobble metric. **Not the real fix** (see Stage 3) |
| [3](#stage-3-2026-08-05-steering-wobble-root-cause) | 08-05 | Root cause of the wobble: no steering-rate limit offline, a units bug hiding it, and a 10 Hz/20 Hz pose-update mismatch |
| [4](#stage-4-2026-08-05-speeding-into-corners) | 08-05 | Car sped into corners it couldn't hold — added tracking-error speed gate + rise-rate limiter (a workaround, not a fix — see caveat) |
| [5](#stage-5-2026-08-05-scoring-couldnt-tell-good-driving-from-bad) | 08-05 | Scoring math was nearly blind to wobble; rescaled metrics, then rebuilt scoring into three tiers (safety / lap-time / quality) |
| [6](#stage-6-2026-08-06-the-real-vs-offline-driving-gap) | 08-06 | Main investigation: ruled out sensor noise, tyre grip, steering miscalibration; found FSDS enforces a hidden speed-dependent cornering-force ceiling; modelled it (two iterations); confirmed it explains **at most ~25%** of the gap |
| [7](#stage-7-2026-08-07-back-to-live-testing) | 08-07 | Unblocked live testing; first live comparison (confounded); isolated the pose-rate fix as a real contributor; found live results vary run-to-run (15–32%) — a single test isn't proof |

**Bottom line at the end of this stretch:** one real cause (the FSDS cornering
ceiling) confirmed and modelled, several plausible causes ruled out by direct
measurement, several unrelated bugs fixed — but at least 75% of the
real-vs-offline gap was still unexplained. Continued in
[`fsae_planning_sim_to_real_investigation_pt2.md`](fsae_planning_sim_to_real_investigation_pt2.md).

---

## Stage 1 — 2026-08-04: tooling and setup

**Plain-language summary:** Before any tuning could be trusted, the team had
to re-sync the offline copy of the driving code against the live version,
reorganise the project into clear folders, and get a real recorded lap
loading correctly — which took five separate bug fixes.

**What changed:**

- Re-synced the offline planning/control code against the current live
  version (topic names, message structures had drifted).
- Reorganised the project into clear folders (plant model, controller,
  simulator, GUI, tuning tools) and rebuilt the `fsds_simulator` staging
  mirror to match the live repo file-for-file.
- Fixed a broken example command, a missing startup dependency, and unclear
  launch docs.
- Added the ability to load a real recorded lap into the offline simulator
  (previously only synthetic test tracks).
- Fixed five bugs, found one after another, in turning raw recorded cone
  positions into a smooth centreline:
  1. Track self-crossings (e.g. figure-eight) mixed cones from unrelated
     sections — fixed by driving a virtual lap step-by-step using the same
     left/right logic as the live car.
  2. A wrong turn at the same crossings caused an extra spurious loop — fixed
     by detecting "already visited" and stopping cleanly.
  3. Lap direction was guessed by "which reconstruction looks longer" (unreliable) —
     fixed by using cone colour (left/right) relative to travel direction instead.
  4. The finish-line check mistook lap *start* for lap *end* — fixed by
     distinguishing "near the finish" from "actually completed the lap."
  5. Tight hairpins falsely triggered "lap complete" — fixed by checking
     distance travelled, not just points passed.
- Added a small pose-delay simulation (matching the real control loop's
  latency). This alone caused wild swerving; fixed by having the controller
  estimate its own delay and predict the car's current state forward
  (`predict_ahead()`). Confirmed a no-op at zero delay. Later ported to the
  live code too.
- Added a coarse-then-fine two-stage search to the tuner, and made its run
  history record which scoring rules were active per run (so results can't
  be compared under mismatched rulebooks by accident).

---

## Stage 2 — 2026-08-04: steering wobble, first pass

**Plain-language summary:** The steering was visibly wobbling, worst in
corners. The wobble-cost in the scoring function turned out to be 40–60×
smaller than the tracking-line cost, so the tuner had almost no reason to
avoid it. This stage's fix helped a little but **was not the real cause** —
see Stage 3.

**What changed:**

- Raised the steering-rate cost significantly; stopped a separate setting
  from relaxing that cost specifically in hard corners (where the wobble was
  worst).
- Replaced the wobble metric: it used to just count direction reversals
  (a tiny wiggle = a violent swing). Now sums the magnitude of each reversal,
  so large swings are weighted far above small ones.
- Re-tuned against the corrected scoring and got an improved result.
- Expanded the `fsds_simulator` staging mirror from control-only to the full
  software stack (perception + planning + control).

> **This was not the real fix.** The wobble returned and was traced to a
> different, deeper cause in Stage 3.

---

## Stage 3 — 2026-08-05: steering wobble, root cause

**Plain-language summary:** Three stacked bugs, found one after another:
the offline simulator let the car steer faster than physically possible; a
units bug was hiding how often steering was maxed out; and the position
sensor updated at half the rate the controller ran at, causing "freeze,
drift, catch-up jolt, overcorrect" — the actual wobble shape.

**What changed:**

- **Steering-rate limit missing offline.** A fresh log showed steering
  pinned at max rate-of-change 41% of the time, flipping ~8×/second, while
  track position stayed close to correct — a speed-limiter problem, not a
  weights problem. The offline plant had no steering-rate limit at all, so
  every setting tuned against it had been tuned against an unrealistically
  fast-steering car. Estimated the real limit from how fast real cornering
  behaviour changes in logs, and added it to both the offline plant and its
  scoring.
- **Steering-angle logging bug.** Logged steering angle for one controller
  type was fed the wrong input scale, inflating values ~2.3×. This had
  quietly hidden the rate-limiter problem for an entire round of testing.
  Fixed the conversion; made logging units-labelled.
- **Camera noise — ruled out.** Even with the rate limit fixed, offline
  wobbled far less than the real car. Guessed camera-position noise; added
  an optional noise model to test it. **Wrong** — the log being compared
  against was itself from the noise-free simulator, so noise couldn't have
  caused what was in that specific recording.
- **Root cause found: pose-update rate.** Cone/position reporting updated at
  10 Hz; steering decisions ran at 20 Hz. Over half of all steering ticks
  were working from stale, unchanged position data — freeze, drift,
  catch-up jolt, overcorrect, repeat. Fixed by splitting the update schedule:
  position now updates at ≥20 Hz; cone-tracking stays at its slower rate.

---

## Stage 4 — 2026-08-05: speeding into corners

**Plain-language summary:** After the wobble fix, a new failure appeared —
the car drove fine, then failed hard on lap 2, running 3 m off-line with
steering pinned at full lock. The speed planner only looked at road shape
ahead, with zero awareness of whether the car was even still on that road.

**What changed** (four fixes together):

- Smoothed the curvature-ahead reading (averaged nearby measurements) so a
  single noisy reading can't set the speed target for a whole stretch.
- Added `tracking_error_speed_gate()`: reduces target speed in proportion to
  how far off-line / off-heading the car is, with a floor so it's never told
  to fully stop.
- Capped how fast the *target* speed is allowed to rise (braking is never
  delayed — only speeding up is capped).
- Fixed a mismatch where the two live controllers measured the road ahead
  from a different starting point than the offline simulator did.
- Result on 10 synthetic test roads: worst-case lateral error dropped from
  1.76 m to 0.79 m; one previously-failing road now passes.

> **This is a workaround, not a repair.** The real root cause — the
> centreline builder occasionally producing physically-impossible kinks
> tighter than the track's tightest real corner — is still open (see
> `sim_to_real_investigation.md`). If that's ever fixed, re-check these
> safety nets rather than assuming they're still needed as-is.

---

## Stage 5 — 2026-08-05: scoring couldn't tell good driving from bad driving

**Plain-language summary:** A deliberately wobbly driving style scored
*better* than a calm one. The composite score summed twelve metrics of very
different natural magnitudes, so two of them dominated and the other ten
(including wobble) barely mattered regardless of weight.

**What changed:**

- **First pass — rescaling.** Divided each of the 12 metrics by a typical
  reference magnitude before weighting, putting them on comparable footing.
  Made wobble-related metrics ~10× more influential. **Helped, but summing
  competing priorities into one number has a hard ceiling** — a style that's
  merely good at one or two of twelve things can still "buy back" a real
  problem. That needed a structural fix, not a dial adjustment.
- **Second pass — tiered scoring.** Restructured into three tiers instead of
  one flat weighted sum:
  1. Hard safety tier (crash / off-track) — its own scale, not offsettable.
  2. Lap-time tier — measured against a genuinely re-derived physics-based
     best-possible lap time (the old target had been a rough placeholder,
     several times too generous).
  3. Quality tier — the twelve detailed metrics, now a smaller supporting
     factor.
  - Also fixed a braking-distance bug found while rebuilding this: the old
    corner-speed calc looked only at the sharpest visible curve with no
    sense of distance, sometimes demanding braking forces beyond what any
    real car can produce.
  - Re-tested the wobbly-vs-sensible comparison: the sensible style now
    correctly scores better.
  - A follow-up cleanup caught three gaps a first pass missed: the GUI was
    still using the old lap-time number; docs still described the old
    scoring scale; the staging mirror hadn't received the braking-distance
    fix. All three closed.

---

## Stage 6 — 2026-08-06: the real-vs-offline driving gap

**Plain-language summary:** Settings tuned offline drove smoothly in the
simulator but made the real car wobble and struggle on the identical track.
This stage is the largest because it worked through several plausible causes
in turn before finding the real one — which still only explained part of the
gap.

**What was tested and ruled out:**

- **Random pose-freezes (beyond the steady 10/20 Hz split from Stage 3).**
  Modelled a matching freeze pattern from real logs. Didn't move
  steering-lock-up meaningfully (3%→4% vs. the real car's 21%+).
- **Tyre grip.** Fit grip to match one real cornering-vs-steering
  relationship exactly — but it then failed two other independent checks
  (full-lock low-speed behaviour, lock-up frequency). Two measurements of the
  same physical property disagreeing is the signature of the wrong model,
  not a mistuned parameter. Ruled out.
- **Uniform steering miscalibration.** The commanded-vs-achieved steering
  shortfall grew with speed rather than staying constant, ruling out a flat
  scaling error, steering delay, a logging mistake, and unrealistic
  front/rear tyre balance.

**What was found — the real cause:**

- Built a dedicated open-loop test: fixed steering command, fixed speed,
  controller disconnected entirely. Below ~5 m/s the car delivers the
  commanded angle almost exactly; above it, actual turning collapses sharply
  (not gradually) — to roughly ⅓ of commanded at moderate speed, ⅙ at
  higher speed.
- **Conclusion: FSDS enforces an invisible ceiling on sustained cornering
  force above a speed threshold.** Not a tyre-grip limit — the ceiling sits
  well below what the same car reaches elsewhere, and grip limits don't
  depend on speed this way.
- This explains the original wobble end-to-end: planner assumes full
  commanded turning → FSDS clamps it → heading drifts → controller demands
  more steering → pinned at the mechanical limit — the exact Stage 3 pattern.
- A faster step-input test (sudden steering step, sampled quickly) found the
  car **overshoots the ceiling by ~30% before settling** — rules out a hard
  stop (which cannot overshoot itself), and points to a gradually-building
  restoring force. Held quantity is sideways force, not turning rate, at
  ~7.5 units.

**Modelling the ceiling — two rounds of correction:**

- First fit (to the *settled* force) behaved almost like a wall and spun the
  car off-track — real corners arrive faster than an overly-strong version
  can react to.
- Second fit (to the *overshoot peak*) fixed the spin-off but settled
  noticeably above the measured value. **Recognised as one structural flaw,
  not two separate mistakes**: a proportional-style law can only produce
  output once error already exists, so its resting state must sit past the
  true ceiling by construction — no amount of tuning fixes that.
- Switched to an integral-style law (keeps building pushback for as long as
  excess remains). Settles exactly on the ceiling by construction, leaving
  only the gain to fit. ~5× more accurate on a held-out data slice.
- Measured the timing constant (how fast the pushback builds) properly with
  a longer step test — **moved the offline lock-up rate the wrong way**,
  reported honestly as a small step backward, kept anyway as a genuinely
  measured physical property.
- Built a proper repeatability framework (multiple synthetic test roads, not
  just the one recorded lap). Result: **every one of these adjustments
  (ceiling strength, level, timing) individually produced an effect smaller
  than the test's own run-to-run variation.** Even combining the two
  largest non-overlapping effects closed only ~25% of the gap. **At least
  75% remained unexplained.**

**Other findings this stage:**

- Path-planner jump checks: one specific reset mechanism was confirmed real
  but measured to fire zero times on the recorded lap — not the explanation
  here. General smoothing noise explained only a small sliver of heading-error
  growth.
- Traced the Stage 4 "impossible kink" issue toward a narrower cause (how the
  nearest-point path-stitch works), but the first measurement of how often
  it occurs was itself flawed — mostly picking up harmless resampling.
  Re-measured: real, but too rare on its own to explain the gap.
- Fixed a genuine duplicate-cone bug: a new cone detected twice in the same
  instant became two permanent entries (only compared against
  already-stored cones, never against same-frame detections). Fixed
  everywhere. Confirmed it doesn't move the lock-up gap at realistic noise
  levels — a correctness fix, not the missing piece.
- Fixed an over-broad `.gitignore` rule that was silently hiding an entire
  mirrored folder from `git status` — present and correct on disk the whole
  time, just invisible to the usual check.

---

## Stage 7 — 2026-08-07: back to live testing

**Plain-language summary:** Two small bugs had blocked every attempt at a
live test this session. Once fixed, the first live comparison looked
promising but was confounded by too many simultaneous changes. Untangling it
confirmed the pose-rate fix matters — but repeating the "good" result five
times gave a worse number than the original baseline, proving a single test
run isn't trustworthy evidence.

**What changed:**

- Fixed a startup-script bug (a formatting mistake breaking a readiness
  check) and a WSL-to-Windows networking quirk that had blocked every live
  test attempt. Ported both fixes to the staging mirror. This unblocked the
  first live test of the whole investigation.
- First live test (many changes bundled at once): lock-up improved 21%→15%.
  Flagged immediately as **confounded** — no way to attribute the
  improvement to a specific change.
- Researched FSDS's underlying engine docs for a known off-the-shelf feature
  matching the measured ceiling shape. Found a plausible candidate feature,
  unconfirmed — the relevant internal file wasn't readable this session.
- Reverting the MPC-weight retune made lock-up *worse* — ruled that change
  out as the explanation for the 15% result.
- **Directly tested the pose-rate fix by deliberately reintroducing the old
  bug** — made things measurably worse (32% lock-up, worse than both
  baselines). This is the first live, direct reproduction (not just
  inference from old logs) of the exact "frozen position → drift →
  catch-up jolt" mechanism from Stage 3.
- Repeated the "promising" 15% test five times at identical settings:
  **result was 26%**, worse than both the 21% baseline and the original 15%
  single-run result. Caught and fixed a bug in the analysis script itself
  (miscounting a lap-counter re-trigger at the track's self-crossing as an
  extra lap) before trusting this number.

> **Conclusion:** the 15% figure does not hold up and is superseded. Live
> lock-up rate varies ~15–32% run to run under identical settings; offline
> sits consistently around 5%. The gap is real but needs several repeated
> live runs to size accurately, not one. Two things do survive this
> correction: the MPC-weight retune is still not the explanation, and the
> pose-rate mechanism is still directly confirmed.

---

**Continued in** [`fsae_planning_sim_to_real_investigation_pt2.md`](fsae_planning_sim_to_real_investigation_pt2.md).
