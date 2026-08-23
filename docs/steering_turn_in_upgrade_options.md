# Late / jerky turn-in on shallow corners — upgrade options

**Status: Option 2 is CONFIGURED and awaiting a live test** (config only,
no code change -- see that section). Options 1, 3 and 4 remain analysis
only, nothing implemented.

## The problem

`r_rate_delta = 52.5` (up from 2.8) fixed the steering chatter — see
`docs/logs/steering_chatter_investigation.md`. But it introduced a new
symptom on **shallow** corners: the car holds a smooth line, refuses to
begin turning, then jerks once the NMPC's predicted errors finally grow
enough to overpower the rate cost, then resumes smooth tracking.

## Root cause, quantified

The rate cost is `r_rate · Σ(du)²`, summed over the horizon. The tracking
cost is `Σ(q·e²)`. Both are quadratic, but **the tracking term scales with
error² while the rate term scales with the size of the steering step** — so
their ratio swings enormously with corner severity:

| corner | e_y | e_psi | tracking cost | rate cost of a 1° step | ratio |
|---|---|---|---|---|---|
| shallow | 0.05 m | 1.0° | 0.328 | 0.016 | **20×** |
| moderate | 0.20 m | 4.0° | 5.241 | 0.016 | 328× |
| sharp | 0.50 m | 12.0° | 33.198 | 0.016 | 2076× |

A **100× swing** in how strongly the rate cost resists the first steering
input. On a sharp corner the tracking term overwhelms it instantly; on a
shallow one they are the same order of magnitude, so the optimal plan is
genuinely to *wait* — the QP is behaving correctly, the weighting is wrong.

**Confirmed in the live log** (`…_1787483327.csv`, `r_rate=52.5`):

- Every jerk event lands at **exactly 9.00°/tick** — that is `du_max`
  (180°/s × 0.05 s). The controller is hitting the **actuator slew limit**,
  i.e. it deferred so long it then had to catch up faster than physically
  possible.
- Ticks at the slew limit: **1.75%** at `r_rate=52.5` versus **0.00%** at
  `r_rate=2.8`. The high gain created this.
- `max|e_psi|` rose 21.7° → **29.4°**.
- Chatter by corner severity: `mean|d_steer|` is 0.833°/tick at
  `corner_frac<0.1` but 2.482° at 0.25–0.50 — the damping bites hardest
  exactly where it is least wanted.

**One flat weight cannot be both stiff enough to kill straight-line hunting
and compliant enough for a gentle corner's small, early input.** Every
option below is a way of making the rate cost *conditional*.

Relevant structural fact: `nmpc_core.py:1110` builds the rate weight as
`np.tile(self.r_rate, N)` — **uniform across all horizon stages**. Nothing
in the current formulation varies it per stage, though the machinery to do
so already exists.

---

## Option 1 — Per-stage rate weight: soft now, stiff later ★ recommended

**Idea.** Keep the rate cost uniform in *magnitude* but ramp it across the
horizon: cheap to move the wheel at stage 0–2, expensive by stage 10+.
The controller can commit to a small input *immediately* without being
allowed to oscillate, because sustained wiggling costs full price.

**Why it should work.** The chatter being suppressed is a *tick-to-tick*
oscillation — it shows up as alternating `du` across many consecutive
stages. Turn-in is a *single sustained* input. A stage-ramped weight
distinguishes exactly these two: the flat weight cannot.

**Implementation.** Small and low-risk — the hook already exists:

```python
# nmpc_core.py, replacing np.tile(self.r_rate, N)
ramp = np.linspace(rate_w_near, 1.0, N)          # e.g. 0.15 -> 1.0
Rr_flat = (np.tile(self.r_rate, N).reshape(N, NU)
           * ramp[:, None]).reshape(-1)
self._ErE = self._E.T @ (Rr_flat[:, None] * self._E)
```

`_ErE`/`_Rr_flat` are already recomputed per tick when any of the existing
experimental flags is on (`nmpc_core.py:1627`), so the plumbing is proven.
Two new params: `nmpc_rrate_stage_ramp_enabled`, `nmpc_rrate_stage_near`.

- **Effort:** low (~40 lines + params + mirror).
- **Risk:** low. Hessian sparsity unchanged; flag-off is byte-identical.
- **Verify:** offline chatter metric must hold near 1.9°/tick equivalent
  while slew-limit ticks drop from 1.75% toward 0. If chatter returns,
  `rate_w_near` is too low.

## Option 2 — Curvature-scheduled rate weight, done properly

**Idea.** What `nmpc_corner_rrate_blend_enabled` already does — stiff on
straights, soft in corners — but with endpoints scaled to the real
`r_rate_delta` instead of the LTV-QP's stale 2.0/1.25.

**Status.** The code **already exists and is wired**. It was live-tested and
broke the car *only because its endpoints defaulted to `-1` (inherit)*,
substituting 2.0/1.25 for 52.5 — a 30× cut giving 21% saturation. See the
TRAP section in the chatter log.

**To test:**
```bash
NMPC_CORNER_RRATE_BLEND_ENABLED=true
NMPC_RRATE_STEER_STRAIGHT=52.5
NMPC_RRATE_STEER_CORNER=20.0     # try 15-25
```
- **Effort:** none — config only. **Try this first, before writing code.**
- **Risk:** low, and already understood.
- **Weakness:** keyed on **current** curvature, so on a shallow corner
  `corner_frac` is small and the softening barely engages — precisely the
  case that is broken. Likely insufficient alone, but nearly free to check.

### Measured before the first run: `corner_factor_k` must be raised too

`corner_frac = 1 - 1/(1 + k*|kappa|)`. Measured on the good `r_rate=52.5`
live run, `corner_frac` **never exceeded 0.63** (p50 0.31, p90 0.49) at the
default `k=8.0`. Because `_blend` is a plain lerp, the blend can therefore
only travel ~2/3 of the way to its corner endpoint - even a corner endpoint
of 2.0 leaves **~32** where the jerks actually occur (mean `corner_frac`
0.40 on slew-limited ticks), i.e. only a 25% cut from 52.5. Far too weak.

Raising `k` fixes the reach. Curvature seen on this track maps as
`corner_frac` 0.40 -> R~12 m, 0.63 -> R~4.7 m, 0.03 -> R~250 m. At `k=20`
the 12 m jerk zone reaches 0.63 while a 250 m near-straight stays at 0.07.

### LIVE-TESTED AND REJECTED

Ran with `k=20`, straight 52.5, corner 8.0. The schedule delivered exactly
as designed (`Rrate_steer` applied: min 16.5, p10 21.1, p50 29.2, p90 49.7,
max 52.5), so this is a fair test of the idea, not a misconfiguration.

**It was worse on every single metric:**

| | flat 52.5 | blend |
|---|---|---|
| slew-limited % (the target) | 1.75% | **2.54%** |
| mean\|d_steer\| | 1.919 deg | 2.269 deg |
| \|e_y\| | 0.288 | **0.467** |
| max \|e_psi\| | 29.4 deg | **36.8 deg** |
| saturation | 0.03% | **1.52%** |

Softening in corners did not buy earlier turn-in; it simply gave back the
chatter suppression, and the resulting worse tracking then produced MORE
slew-limited catch-up, not less. **Option 2 is falsified. Do not retry it,
and by extension be very sceptical of Option 3** (same signal, shifted
earlier) -- see the next section for why.

### Why curvature scheduling cannot work here (the measurement that matters)

Checked what the signals looked like **1 second before** each of the 51
slew-limited jerks in the flat-52.5 run:

- `corner_frac` median **0.360**, `|e_psi|` median **3.42 deg** -- and
- **14 of 51 events (27%)** had BOTH `corner_frac` < 0.15 AND
  `|e_psi|` < 3 deg one second earlier. At t=10.61 and t=28.45 the
  one-second-prior state was `corner_frac` 0.029/0.049 with `|e_psi|`
  0.10/-1.60 deg -- indistinguishable from a straight.

**Roughly a quarter of the jerks are invisible to any current-state
curvature or error signal until the corner is already on top of the car.**
No amount of retuning a schedule keyed on those signals can reach them.

But the information DOES exist: `nmpc_kappa_horizon_end` one second before a
jerk has median **0.0908** versus **0.0569** overall. **The horizon already
knows.** The cost function just is not using what the prediction can see.
That points away from state-keyed scheduling entirely and toward Option 1
(per-stage, i.e. keyed on horizon position) or Option 4.

## Option 3 — Lookahead-curvature scheduling

**Idea.** As Option 2 but keyed on **peak curvature ahead** rather than
current, so the weight softens *before* the corner arrives.

**Prior art in-repo.** `peak_kappa_ahead()` already exists in both
`control_utils.py` and `sim/speed_profile.py`, built for the (now-disabled)
output-smoothing lookahead fade. It is tested and can be reused directly.

- **Effort:** low-medium. Reuses an existing, working helper.
- **Risk:** medium. `dκ/ds`-style lookahead signals amplify planner
  curvature noise — the documented open centreline-spike defect. Fine
  against the static raceline in use now; re-validate before any live
  planner path.
- **Note:** composes naturally with Option 1 (when *and* where).

## Option 4 — Penalise steering *acceleration* instead of rate

**Idea.** Replace (or augment) the `Σ(du)²` term with a second-difference
penalty `Σ(du_k − du_{k−1})²`. A steady ramp into a corner has near-zero
second difference and is nearly free; an alternating wiggle has a huge one.

**Why this is the most principled option.** It targets the actual defect
directly. Chatter *is* high second-difference; turn-in *is* low. This
separates them structurally rather than by scheduling a compromise.

**MEASURED CONFIRMATION** (flat-52.5 live run, 2910 ticks classified by
whether steering reversed direction or continued the same way):

| behaviour | n | mean\|d1\| | mean\|d2\| |
|---|---|---|---|
| reversals (chatter) | 1739 | 2.375 deg | **4.669** |
| same-direction (ramp) | 1171 | 1.247 deg | **1.083** |

`|d2|` separates the two by **4.31x**, versus only ~1.9x for `|d1|` -- so
penalising steering ACCELERATION discriminates chatter from legitimate
turn-in roughly twice as sharply as penalising rate does. Additionally, 96%
of slew-limited ticks are direction REVERSALS rather than continuations,
i.e. even the big catch-up events are mostly the tail of an oscillation, not
clean ramps. This is the strongest quantitative support any option in this
document has.

**Implementation.** Needs a second-difference operator `E2` alongside `E`,
and `Hess += E2' diag(R_jerk) E2`. `E` is built once in `_build_qp`
(`nmpc_core.py:848-855`); `E2 = E @ E` structurally, so construction is
trivial. But it **changes the QP's Hessian sparsity pattern**, so
`_csc_pattern`/the OSQP `P` matrix setup must be rebuilt — the one option
here that touches solver plumbing rather than just weights.

- **Effort:** medium-high (~150 lines, new operator, P-matrix pattern,
  both repos, new test).
- **Risk:** medium. Sparsity/`P`-update changes are where subtle solver bugs
  live. Needs a dedicated `nmpc_offline_check` case.
- **Payoff:** highest. Likely the *correct* long-term formulation, and would
  let `r_rate_delta` drop back toward its original value.

## Option 5 — Raise `du_max` (actuator slew limit)

**Idea.** Jerks saturate at exactly 9.00°/tick, so raise the cap.

**Assessment: reject as a fix.** This treats the symptom — the controller
would still turn in late, just catch up more violently. Worse, 180°/s is a
*measured lower-bound estimate* of the real FSDS actuator (see
`planning_control_sync.md`'s slew-rate section); raising it past the real
limit means commanding motion the plant cannot deliver, reintroducing the
saturation-driven instability the limit exists to prevent. Only revisit
alongside a fresh system-ID of the true rate.

## Option 6 — Reduce `r_rate_delta` and re-attack chatter differently

**Idea.** Accept 52.5 is too blunt; drop toward ~15-20 and suppress the
residual chatter another way (Option 4, or output smoothing re-enabled but
*only* on straights).

**Assessment:** a reasonable fallback, but strictly worse than Options 1/4 —
it re-opens a problem already solved. Keep as a retreat if the others fail.

---

## Recommended sequence (REVISED after Option 2's live rejection)

1. ~~Option 2~~ — **done, rejected.** Worse on every metric; see above.
2. ~~Option 3~~ — **deprioritised to near-dead.** It keys on the same
   curvature signal as Option 2, merely earlier. Since 27% of jerks show no
   curvature or error signal at all one second out, and Option 2's actual
   failure was giving back chatter rather than being too late, shifting the
   same signal forward is unlikely to help. Not worth the noise risk.
3. **Option 1** (per-stage rate ramp) — **now the first thing to try.** It
   is keyed on horizon POSITION rather than measured state, so it is immune
   to the "no signal one second out" problem that killed Option 2. Low
   effort, low risk, and the `_Rr_flat`/`_ErE` per-tick rebuild hook already
   exists.
4. **Option 4** (steering-acceleration penalty) — **now has the strongest
   evidence of any option** (4.31x separation, 96% of slew events are
   reversals). If Option 1 leaves residual jerk, do this; it is likely the
   correct long-term formulation and would let `r_rate_delta` fall back
   toward its original value.

Options 1 and 4 are complementary: 1 says *when in the horizon* damping
applies, 4 changes *what is being damped*. Neither depends on a
measured-state schedule, which is the property that matters given the
measurements above.

Options 1, 2 and 3 are mutually composable — 1 schedules *when* in the
horizon, 2/3 schedule *where* on the track. Option 4 could eventually
replace the need for any of them.

## Measurement protocol (any option)

Verifying this needs a metric the chatter work did not have — chatter and
turn-in trade against each other, so one number is not enough:

- `mean|d_steer|` and sign-flip% — must not regress from ~1.92°/59.7%.
- **Slew-limit ticks** (`|d_steer| > 8.9°`) — the turn-in metric.
  **1.75% now; target ~0%.** This is the number that matters.
- `max|e_psi|` — 29.4° now, was 21.7° at `r_rate=2.8`.
- `|e_y|` mean/p90 and saturation% as guardrails.
- Reproduce with `python -m tuner.steering_chatter_check --set ...`, but
  **note the offline closed-loop harness currently DNFs on shipped
  defaults** (see the chatter log) — live A/B is authoritative until that is
  resolved. Fixing this turn-in problem may itself clear that DNF, since it
  is plausibly the same mechanism in a less forgiving plant.
