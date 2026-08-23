# Late / jerky turn-in on shallow corners — upgrade options

**Status: analysis only. Nothing here is implemented.**

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

## Recommended sequence

1. **Option 2 as a config-only probe** (zero effort). Establishes whether
   curvature scheduling helps at all, and how much softening is tolerable
   before chatter returns. Expect partial help at best.
2. **Option 1** (per-stage ramp). Best effort-to-payoff; directly targets
   the sustained-vs-oscillating distinction; low risk.
3. **Option 4** (steering-jerk penalty) if 1+2 leave residual jerk. The
   principled fix, and worth the refactor if it lets `r_rate_delta` come
   back down.
4. **Option 3** only once a live-planner path matters, given the noise risk.

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
