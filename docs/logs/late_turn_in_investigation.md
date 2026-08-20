# Late turn-in investigation — 2026-08-12

Working notes: an append-only research log, in the same spirit as
`sim_to_real_investigation.md` in this same directory. Cited by name from
`architecture.md`, `tuning.md`, `planning_control_sync.md`,
`junior_project_mpc_docs.md`, and the live `fsae_planning/CHANGES.md` as the
detailed source for specific Parts — do not delete. Conclusions that matter
for day-to-day tuning are summarized in `planning_control_sync.md` and
`tuning.md`; read this file when a citation points here for the full
derivation, the live-test data behind a conclusion, or the reasoning behind
an idea that was tried and rejected.

## Part 1 — Every adaptive gain mechanism, walked through

All of these live in `ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_core.py`
(mirrored in `fsae_MPCTest/controller/model_utils.py`, offline). Values below
are current live defaults (`mpc_params.py`).

### Base (non-adaptive) weights

**STALE SNAPSHOT as of 2026-08-12 (Part 1's original writeup) — base
weights have been retuned multiple times since (most recently the same
day, see Part 11's investigation). Do not treat this table as current;
read `mpc_params.py`'s actual field defaults for the live values.**

| Weight | Value (as of original Part 1 writeup) | Meaning |
|---|---|---|
| `q_e_y` | 6.0 | lateral error cost |
| `q_e_yd` | 0.2 | rate of lateral error |
| `q_e_psi` | 1.52 | heading error cost |
| `q_r` | 0.50 | yaw-rate cost |
| `q_e_v` | 3.55 | speed-tracking cost |
| `r_delta` | 1.6 | steering effort |
| `r_a_accel` / `r_a_brake` | 1.0 / 0.4 | accel/brake effort, split |
| `r_rate_delta` | 2.1 | steering rate-of-change |
| `r_rate_a` | 2.6 | accel rate-of-change |

Every multiplier below scales one of these six matrix entries
(`Q[0,0]`=e_y, `Q[2,2]`=e_psi, `Q[3,3]`=r, `R[0,0]`=delta effort,
`R_rate[0,0]`=delta rate) at runtime, multiplicatively, in the order listed.

### Signal: `kappa` vs `kappa_max_abs`

Two different curvature signals feed all of this, and mixing them up is the
single easiest way to misread the log:

- **`kappa`** — current-position curvature, from `_error_state`. Reactive:
  nonzero only once the car is already at a curving point on the path.
- **`kappa_max_abs`** — peak |curvature| anywhere within a forward lookahead
  window. This is the anticipatory signal. Window length:
  `lookahead_dist = clip(car_speed * adaptive_q_lookahead_time_s, dist_min, dist_max)`
  = `clip(car_speed * 1.13, 3.0, 25.0)` m. At 17 m/s that's 19.2 m (~1.13 s
  of travel); it saturates at the 25 m ceiling above ~22 m/s.

Every mechanism tagged "lookahead" below uses `kappa_max_abs` (anticipatory).
Every mechanism tagged "current" uses `kappa` (reactive) — these cannot turn
the car in before real curvature exists under the car.

### R[0,0] — steering EFFORT (how far the wheel is turned)

Applied in this order, all multiplying the same `R[0,0]`:

1. **`_adaptive_R_scaling`** (current speed, always on, not a flag) —
   `1 + 1.5*vx/(6+vx)`: 1.27x @ 5 m/s → 2.0x @ 17 m/s → 2.25x @ 30 m/s.
   Steering gets MORE expensive as speed rises. No lookahead relief built in.
2. **`_steer_effort_straight_boost`** (`steer_effort_straight_boost_enabled=True`) —
   lookahead-gated, `1 + (1.5-1)/(1+20*kappa_max_abs)`: 1.5x on a clear
   straight, collapsing to ~1.0x by kappa_max_abs≈0.1 (k=20 is sharp —
   collapses almost as soon as any corner is detected).
3. **`_lookahead_steer_effort_relax`** (`lookahead_steer_effort_relax_enabled=True`) —
   lookahead-gated, demand-normalised: `1 - (1-0.5)*demand_frac`. 1.0x with
   no corner ahead, floors at 0.5x as corner DEMAND (speed-aware, not raw
   kappa) approaches/exceeds what the car can hold. This is the ONLY
   mechanism that pushes R[0,0] BELOW baseline approaching a corner —
   everything else in this list only ever raises it or returns to baseline.

Net effect at a hot corner entry: (2) has already collapsed to ~1.0x, so
effective R[0,0] ≈ `_adaptive_R_scaling(vx) * 0.5` at full demand — steering
effort cost roughly HALVES from its speed-scaled baseline right as the
corner is detected. This part is already working as designed.

### R_rate[0,0] — steering RATE (how fast the wheel angle changes)

Applied in this order, all multiplying the same `R_rate[0,0]`:

1. **`_adaptive_R_rate`** (current+lookahead blend, `adaptive_r_rate_enable_in_corners=True`) —
   `min(during_scale, entering_scale)` where
   `during_scale = max(0.625, 1/(1+3*|kappa|))` (CURRENT curvature, floor 0.625)
   `entering_scale = max(0.85, 1/(1+4*kappa_max_abs))` (LOOKAHEAD, shallower floor 0.85)
   min() picks whichever is more aggressive. On approach (kappa≈0,
   kappa_max_abs>0) only `entering_scale` can differ from 1.0, floored at
   0.85x — a small, early relief. Full 0.625x floor only kicks in once
   `kappa` (current) is large, i.e. already mid-corner.
2. **`_steer_rate_anti_hunt`** (`steer_rate_anti_hunt_enabled=True`) —
   4-factor product, all saturating toward 1.0 as their input grows:
   `boost_kappa=1/(1+60|kappa|)`, `boost_ey=1/(1+30|e_y|)`,
   `boost_epsi=1/(1+23|e_psi|)`, `boost_lookahead=1/(1+15*kappa_max_abs)`
   (k=15 as of today, was 60 — see Part 3). `scale = 1 + (6-1)*product`.
   Ceiling 6x when ALL FOUR read "nothing happening" (straight, centred,
   aligned, no corner ahead); fades as any one moves. This is a PENALTY on
   fast steering-rate changes specifically when nothing seems to be
   happening — it actively fights any small early corrective steering the
   controller tries to make on a well-tracked approach.
3. **`_low_speed_steer_rate_boost`** — DISABLED (`low_speed_steer_rate_boost_enabled=False`).
   No-op currently.

Net effect on approach: `entering_scale` gives ~0.85x floor (mild help), but
`_steer_rate_anti_hunt` can be pushing UP TO 6x in the opposite direction
if `kappa_max_abs` isn't yet large enough to relax it. This is the exact
mechanism already found and partially fixed today (k_lookahead 60→15) but
NOT eliminated — 15 still leaves the gate mostly closed until kappa_max_abs
climbs past ~0.05 (see Part 3 numbers). **This is probably still fighting
early turn-in on gentle/gradual corners** where kappa_max_abs stays under
0.05 for a while before the sharp part arrives.

### Q[0,0] — lateral error (e_y)

Applied in this order:

1. **`_lookahead_approach_boost`** (lookahead, demand-normalised) —
   `1 + (2.0-1)*demand_frac`. 1.0x no corner ahead, up to 2.0x at full
   demand. (Tried 3.0x today, live-tested WORSE, reverted.)
2. **`_lookahead_straight_lateral_reduce`** (lookahead) —
   `0.7 + (1-0.7)/(1+8*kappa_max_abs)`. RAISES the floor from 0.7 back to
   1.0 as kappa_max_abs grows — i.e. relaxes lateral cost on a genuine
   straight, fades away (not further boosts) as a corner approaches.
3. **`_adaptive_Q_scaling`** (current e_y, `adaptive_q_scaling_enabled=True`) —
   softens toward a floor when |e_y| is small (well-centred), now
   itself gated by kappa_max_abs (fixed earlier today — floor relaxes from
   0.5 toward 1.0 as kappa_max_abs rises, k=30).
4. U-turn boost (only if accumulated heading change > 60°, N/A for ordinary
   corners).

Net: boost (1) and softening (3) directly oppose each other on a
well-tracked approach — (3)'s gate (k=30) means at kappa_max_abs=0.02 the
floor is only relaxed to ~0.69 (still nearly half-cancelling). See Part 3.

### Q[2,2] — heading error (e_psi)

Applied in this order:

1. **`_lookahead_epsi_approach_boost`** (lookahead, demand-normalised) —
   `1 + (1.5-1)*demand_frac`, up to 1.5x. (Tried 2.5x then 2.0x today, both
   live-tested WORSE, reverted to 1.5x.)
2. **`_lookahead_exit_boost`** (current, keyed on distance-since-last-peak-kappa) —
   up to 1.5x, decaying over a speed-scaled distance (`clip(vx*2.5, 5, 25)` m)
   after a corner's peak curvature has already been passed. Irrelevant on
   approach, only fires AFTER a peak has been recorded.
3. **`_lookahead_straight_boost`** (lookahead) — up to 1.1x on a clear
   straight (deliberately small — a bigger heading-error weight on
   straights amplifies noise-driven hunting), fading to 1.0x as a corner
   approaches (k=8, gentler fade than R[0,0]'s straight boost).

Net: heading error gets comparatively little anticipatory help (max 1.5x
before turn-in, already tried and reverted higher) vs lateral error's 2.0x.

### Q[3,3] — yaw rate (r)

1. **`_lookahead_yaw_rate_relax`** (lookahead, demand-normalised) —
   `1 - (1-0.5)*demand_frac`, floors at 0.5x as demand rises. Lets the QP
   command faster rotation without being penalised for it, as a corner
   approaches — this is arguably the most direct "let it turn in" lever
   and is already fairly aggressive (0.5x floor).
2. **`_lookahead_straight_boost`** (lookahead) — up to 1.5x on a clear
   straight, fading to 1.0x approaching a corner.

### Speed profile / accel-brake (R[1,1] split)

`r_a_accel=1.0`, `r_a_brake=0.4` — static, no adaptive scaling at all. Not
part of the turn-in mechanism directly, but see Part 2 for why this
matters for corner ENTRY speed indirectly.

### Full application order in `compute()` (for reference)

```
R[0,0]:      base -> *_adaptive_R_scaling -> *_steer_effort_straight_boost -> *_lookahead_steer_effort_relax
R_rate[0,0]: base -> *_adaptive_R_rate -> *_steer_rate_anti_hunt -> *_low_speed_steer_rate_boost (off)
Q[0,0]:      base -> *_lookahead_approach_boost -> *_lookahead_straight_lateral_reduce -> *_adaptive_Q_scaling
Q[2,2]:      base -> *_lookahead_epsi_approach_boost -> *_lookahead_exit_boost -> *_lookahead_straight_boost
Q[3,3]:      base -> *_lookahead_yaw_rate_relax -> *_lookahead_straight_boost
```

---

## Part 2 — Changes made today, pending live test (lag-blocked)

All live-only (`mpc_params.py` + `fsae_params.yaml`), nothing touched in
`fsae_MPCTest`/`fsds_simulator` per explicit instruction this session.

| # | Change | Status |
|---|---|---|
| 1 | `curvature_forcing_enabled` True→**False** | Live-tested WORSE (wrong-direction flicks before left corners) + found structurally unsound via offline QP isolation (gain=1 too weak, gain~6-15 pushes steering the WRONG way first, only gain~20 fixes sign but past saturation). Reverted, kept disabled, code kept for a future redesign. **Confirmed bad, stays off.** |
| 2 | `anti_hunt_k_lookahead` 60→**15** | Live-tested: fixed the specific regression from (1)'s interaction, but user reports "didn't fix early turn-in" — i.e. neutral-to-good, not a full fix. **Keep, but see Part 3 — 15 may still be too low (gate too closed) for gentle corners.** |
| 3 | `adaptive_q_scaling_k_lookahead` — NEW field, added | Not yet live-tested (added same session as the "still doesn't turn early" report, before laptop lag blocked further testing). Should reduce the Q[0,0] softening-vs-boost fight identified in Part 1. **Untested live — first thing to test once lag is resolved.** |
| 4 | `adaptive_q_lookahead_q_boost_max` 2.0→3.0→**back to 2.0** | Live-tested WORSE. Reverted. **Confirmed bad alone, stays at 2.0.** |
| 5 | `adaptive_q_lookahead_epsi_approach_boost_max` 1.5→2.5, then →2.0→**back to 1.5** | Both live-tested WORSE. Reverted. **Confirmed bad alone, stays at 1.5.** |

**Important pattern**: three independent "just raise the weight/boost"
attempts (#1 gain, #4, #5) all made things WORSE, not better, when tested
in isolation. This is a real, repeated signal — simply pushing existing
Q/R multipliers higher is not the fix. The mechanism that IS still
untested (#3, the anti-hunt-style gate on `adaptive_Q_scaling`) is
different in kind: it doesn't raise a ceiling, it removes a fight between
two mechanisms that were cancelling each other. That's the one lead this
investigation should prioritize confirming/denying before trying more
weight increases.

---

## Part 3 — Further offline investigation (this session, no live test possible)

### 3a. Is `anti_hunt_k_lookahead=15` still too conservative for GENTLE corners?

The gate: `boost_lookahead = 1/(1+15*kappa_max_abs)`.

| kappa_max_abs | boost_lookahead | anti-hunt scale (all other factors =1, boost_max=6) |
|---|---|---|
| 0.00 | 1.00 | 6.00x (full penalty) |
| 0.01 | 0.87 | 5.35x |
| 0.02 | 0.77 | 4.85x |
| 0.05 | 0.57 | 3.85x |
| 0.10 | 0.40 | 3.00x |
| 0.17 (typical sharp-corner peak) | 0.28 | 2.40x |

A GENTLE corner (kappa_max_abs 0.01-0.03, e.g. the S-curve's first gentle
left bend from the lap1-vs-lap2 investigation) only relaxes anti-hunt to
~4.85-5.35x — still nearly the full 6x penalty on any early corrective
steering-RATE change, even though a real corner is unambiguously ahead
within the lookahead window. **This is consistent with "still doesn't turn
early" being reported even after the 60→15 fix**: the fix helped SHARP
corners (kappa_max_abs 0.1+) much more than gentle ones, and gentle-corner
entries are exactly where early, small corrective steering matters most
(a sharp corner will eventually force a big correction regardless; a
gentle one is where "should have started turning 1s ago" is invisible
until it's too late).

Candidate next test (not yet applied, pending user decision + live access):
lower `anti_hunt_k_lookahead` further (e.g. 15→6) so even a mild
kappa_max_abs meaningfully relaxes the gate. Cross-check against
`adaptive_r_rate_entering_floor` (0.85, k=4.0) — that mechanism already
provides SOME lookahead relief on R_rate[0,0] independently, so anti-hunt
going to a very low k might be redundant/compounding — need the two
side-by-side on a real log, not just formulas, before recommending a value.

### 3b. `adaptive_Q_scaling` gate (`adaptive_q_scaling_k_lookahead=30`) — sanity re-check

| kappa_max_abs | effective floor (vs baseline 0.5) |
|---|---|
| 0.00 | 0.50 |
| 0.02 | 0.69 |
| 0.05 | 0.80 |
| 0.10 | 0.88 |
| 0.17 | 0.93 |

Unlike anti-hunt's k=15, this gate (k=30) relaxes noticeably faster even at
gentle kappa_max_abs (0.69 at 0.02 vs anti-hunt's 0.77 — actually anti-hunt
IS a bit faster here per-value, but anti-hunt's ceiling being 6x vs this
floor's range 0.5-1.0 makes anti-hunt's absolute swing much larger). This
one looks reasonably tuned already for gentle corners; anti-hunt (3a) is
the one that looks under-relaxed by comparison.

### 3c. Structural gap: R_rate's "entering" floor (0.85) is much shallower than anti-hunt's ceiling (6x)

Even at PERFECT anti-hunt relaxation (scale→1.0) and full `entering_scale`
floor (0.85x), the best-case R_rate[0,0] on approach is only 15% cheaper
than baseline. Compare to R[0,0] (effort, not rate) which can reach 0.5x
via `_lookahead_steer_effort_relax`. If the real bottleneck for early
turn-in is the RATE at which steering is allowed to change (plausible,
since "gradual ramp-in" requires several ticks of rate-limited change to
accumulate into a real angle), a 0.85x floor may simply not be enough
headroom regardless of how well anti-hunt is gated — these are two
different knobs (entering_floor's value vs anti-hunt's gate sharpness) and
raising the WRONG one would look like "boost didn't help" even though the
mechanism is directionally correct.

Candidate check (not yet done): compute, from the lap1 log
(`fsae_logs/mpc_standalone_control_1786516297.csv`, t=3.9-4.9 window
already analysed), what R_rate[0,0] value would have been NEEDED to reach
the actual steering-rate lap 2 achieved, and see whether that's within
reach of `entering_floor` even at 1.0 anti-hunt scale, or whether
`adaptive_r_rate_entering_floor` itself (0.85) is the binding constraint.

### 3d. `_corner_demand` scoring at gentle curvature — checked, NOT under-scoring

Computed `demand_frac` across kappa=0.01-0.17 x speed=8/12/17 m/s (the
demand-normalised curve shared by `_lookahead_approach_boost`,
`_lookahead_epsi_approach_boost`, `_lookahead_yaw_rate_relax`, and
`_lookahead_steer_effort_relax` — all four mechanisms use this exact
function, so a single miscalibration here would hit all four at once):

| kappa | v=8 frac | v=12 frac | v=17 frac |
|---|---|---|---|
| 0.01 | 0.146 | 0.262 | 0.356 |
| 0.02 | 0.254 | 0.416 | 0.525 |
| 0.05 | 0.460 | 0.640 | 0.734 |
| 0.10 | 0.631 | 0.780 | 0.847 |
| 0.17 | 0.744 | 0.858 | 0.904 |

This is the OPPOSITE of what 3a's hypothesis assumed: because
`_corner_demand` is speed-aware (`kappa / kappa_limit(v)`, and `kappa_limit`
shrinks as v rises), the SAME curvature scores much higher demand at
cruise speed than at low speed — a gentle kappa=0.02 bend at 17 m/s already
reads as 0.525 demand_frac, more than halfway to every boost's ceiling.
**This mechanism is not the bottleneck for gentle corners at speed** — if
anything it's already quite generous there. Ruled out; do not spend more
time on `_corner_demand`'s calibration without a specific counter-example
from a log.

This sharpens where the real gap must be: since Q/R gain-scheduling
(confirmed not under-scoring) AND three independent "raise the ceiling"
live tests (Part 2, #1/#4/#5) all failed to help, the remaining live
candidates are (a) anti-hunt's `k_lookahead=15` still being too
conservative specifically for the STEERING-RATE cost, not the Q-side
weights (3a, this is a magnitude/gate-sharpness question on a DIFFERENT
mechanism than the demand-normalised ones just ruled out), and (b) the
untested `adaptive_q_scaling_k_lookahead` fix from Part 2 #3. Both are
about removing an existing fight/damping rather than raising a ceiling —
consistent with the pattern that "raise the boost" keeps failing while
"stop something else from cancelling it" (the anti-hunt curvature-forcing
gate fix, situationally) was the one change in this whole investigation
that measurably helped anything.

## Part 4 — Design: precomputed per-corner segmentation (not yet implemented)

**Motivation.** Today, "what corner am I in / how far through it am I" is
never known directly — it's re-approximated every tick from two local
geometric proxies (`kappa`, `kappa_max_abs`) plus a small amount of
tick-to-tick state (`_dist_since_peak`, `_armed_for_next_peak`). 3f-pre
below found a concrete bug that falls directly out of this: the exit-decay
tracker can't distinguish "13m past the apex, corner over" from "13m past
the apex, still fighting a bad exit" because it only ever sees local
curvature, never the corner's actual identity or how that specific
approach is going. Since the path is a known, precomputed array
(`_static_path`, when `path_map_path` is set — see caveat below), corner
identity/extent doesn't need to be inferred at all: it can be computed
ONCE, when the path loads, and looked up by index every tick instead of
re-derived.

**Caveat — this only fully applies in one of two path-source modes.**
`mpc_controller_standalone.py` has two paths: `_static_path` (loaded once
from `path_map_path`, a CSV — genuinely fixed for the whole run, matches
the "record a lap, then drive it precisely" workflow this whole
investigation's logs come from) vs. the live planner topic
(`/fsae/planning/selected_trajectory`, SLAM-built, growing/changing tick
to tick when no `path_map_path` is given — e.g. a genuine first lap on an
unmapped track). Precomputing corner metadata for the WHOLE path only
makes sense for `_static_path`. For the live/growing path, either (a) fall
back to today's live heuristics unchanged (simplest — the live-planner
case is a fundamentally different, harder problem: you can't precompute
what you haven't scanned yet), or (b) incrementally segment only the
newest tail of the path each tick a new segment is appended, keeping a
rolling metadata array that only grows monotonically — deferred, not
needed for the near-term goal of fixing behaviour on a recorded track.
**Design below targets the `_static_path` case only; live-planner mode
keeps its current heuristics untouched (out of scope, not broken by this).**

### 4a. Data structure: per-waypoint corner metadata

Computed once (a new function, e.g. `_segment_corners(path) -> CornerMap`),
called when `_static_path` is set (once, not per-tick):

```python
@dataclass
class CornerMap:
    corner_id:       np.ndarray  # (n,) int, -1 = on a straight
    dist_into_corner: np.ndarray # (n,) float, m from THIS corner's start (0 if straight)
    dist_to_apex:     np.ndarray # (n,) float, m to this corner's peak |kappa| (0 at/past apex)
    dist_since_apex:  np.ndarray # (n,) float, m since this corner's peak (0 before apex)
    corner_length:    np.ndarray # (n,) float, total arc length of the corner this point is in
    peak_kappa_abs:   np.ndarray # (n,) float, this corner's peak |kappa| (same value for every
                                  #      point in one corner -- for demand/severity scoring)
    total_heading_change: np.ndarray  # (n,) float, this corner's total |heading change| end to
                                  #      end (replaces the U-turn detector's live accumulator)
```

One row per path waypoint, indexed identically to `path` itself — so a
lookup is just `corner_map.dist_since_apex[base_idx]`, no scan.

### 4b. Segmentation algorithm (runs once, O(n))

1. Compute `kappa[i] = _curvature(path, i)` for every waypoint (this
   function already exists, already used per-tick today — just run it
   once over the whole array instead of at 1-2 points per tick).
2. Threshold `|kappa[i]|` against a small "on a straight" epsilon (reuse
   `adaptive_q_lookahead_peak_hysteresis`'s existing value, 0.01 — already
   tuned as the "genuinely straight" threshold for the live peak-detector,
   no new constant needed) to split the path into straight / corner runs.
3. For each corner run: find its apex index (max `|kappa|` within the
   run), record `corner_length` (arc length of the run), `peak_kappa_abs`,
   and `total_heading_change` (`sum(|kappa[i]| * ds[i])` over the run —
   exactly `_lookahead_curvature_profile`'s existing heading-change
   integral, just computed over the corner's true extent instead of a
   speed-scaled sliding window).
4. Fill `dist_into_corner`/`dist_to_apex`/`dist_since_apex` by walking arc
   length forward/backward from the apex within each run.
5. Assign sequential `corner_id`s; `-1` for straight runs.

Edge cases to handle explicitly (not hand-waved): a corner at the very
start/end of the path (no full run on one side — clamp, don't crash); two
corners separated by a straight shorter than any lookahead distance used
elsewhere (already handled naturally since segmentation is purely
geometric, doesn't know about speed-scaled lookahead windows at all); a
closed-loop track where the last waypoint's "next corner" wraps to the
first (check whether `path` already encodes a closed loop or is treated
as point-to-point elsewhere in `mpc_core.py` before assuming wraparound is
needed — don't add wraparound speculatively if nothing else in the file
does).

### 4c. How each existing mechanism migrates

| Today | Replacement | Why it's better |
|---|---|---|
| `kappa_max_abs` via `_lookahead_curvature_profile`'s per-tick forward scan | `corner_map.peak_kappa_abs[base_idx]` if `dist_to_apex[base_idx] < lookahead_dist` else look ahead to the NEXT corner's peak if within range — still needs a SPEED-SCALED lookahead distance (this part is legitimately dynamic, speed determines how far ahead matters), but no longer needs to re-scan curvature values every tick; it's an index lookup into precomputed peaks. | Removes an O(lookahead_dist / waypoint_spacing) scan from every single tick; exact instead of sampled. |
| `_update_lookahead_peak` / `_dist_since_peak` (local-peak rising-edge detector, stateful, tick-to-tick) | `corner_map.dist_since_apex[base_idx]` — direct lookup, no state, no rising-edge logic needed at all. | Eliminates an entire stateful mechanism (`_armed_for_next_peak`, the rising-edge-after-clear logic) — this was only ever an approximation of "which corner, how far past its apex," which the corner map now answers exactly. |
| `_lookahead_exit_boost`'s fixed-distance decay (today's bug, 3f-pre) | Same decay-vs-distance SHAPE, but gate it on `corner_map.corner_id[base_idx] == the corner that produced the peak` (so it can't decay into a DIFFERENT corner's territory) AND — the actual fix — hold the boost at its current value (don't let decay_frac advance) while `|e_psi|` remains above some threshold, only resuming the distance-based decay once heading error has genuinely recovered. | Fixes 3f-pre's confirmed bug: today's boost can wear off before recovery is done because it only tracks distance, never checks whether the thing it's supposed to help with (heading error) is actually resolved. |
| U-turn accumulated heading-change (`_lookahead_curvature_profile`'s `heading_change_abs`, scanned live over the speed-scaled window) | `corner_map.total_heading_change[base_idx]` — the CORNER's true total rotation, known exactly, not estimated from whatever fraction of it currently fits in a speed-scaled window (today's mechanism can under-see a U-turn at low speed, where the lookahead window is clamped to only 3-11m and may not span the whole turn). | Removes a known limitation (3-11m minimum lookahead window may not contain a whole slow U-turn) rather than just working around it. |
| Demand-normalised scoring (`_corner_demand`, `_demand_frac`) | UNCHANGED — this is correctly a live, speed-dependent quantity (same corner, different speed, different demand) and has no fixed/precomputable answer. Corner geometry is precomputable; how hard THIS corner is to take at THIS instant's speed is not. | N/A — flagging explicitly so the redesign doesn't try to over-reach into things that are correctly live today. |

### 4d. What does NOT change

- `_adaptive_R_scaling`, `_adaptive_Q_scaling`, `_steer_rate_anti_hunt`'s
  current-position terms (`boost_kappa`, `boost_ey`, `boost_epsi`) — these
  are about the car's CURRENT tracking error and speed, not corner
  geometry; nothing to precompute.
- `_error_state`'s single-point preview `kappa` (drives `_adaptive_R_rate`'s
  "during" floor) — could technically become a corner-map lookup too, but
  it's already an O(1) computation today (walks forward ~1m from
  `base_idx`), so there's no performance or correctness case for touching
  it. Precompute the things that are currently APPROXIMATED, not
  everything that happens to touch curvature.
- The QP itself, `Q`/`R` base weights, every base tunable in
  `mpc_params.py` — this is purely a signal-quality upgrade for the
  lookahead/anticipation layer, not a re-tune of any weight.

### 4e. Validation plan (once implementable/testable)

1. Offline unit-style check first (cheap, no live test needed): run
   `_segment_corners()` against the actual recorded track CSV already used
   for logging (`tracks/<name>/cone_map.json` derived centreline) and
   print corner count / lengths / peak curvatures; sanity-check against
   the known track by eye (e.g. does it find the same "first corner" this
   whole investigation has been analysing, at roughly the arc-length
   position implied by the existing logs?) before trusting it near a QP.
2. Re-run 3f-pre's exact check (`m_Q_epsi_exit` vs `e_psi` through lap1's
   hard corner) against the NEW exit-boost gating, offline, using the
   already-recorded log's `car_pos`/`car_yaw` replayed through
   `_error_state` with the new corner map — confirms the fix closes the
   gap found without needing FSDS running at all.
3. Only then a live A/B, once lag is resolved — same before/after
   discipline as everything else in this file, on the SAME corner already
   characterised in Part 1/3e so results are directly comparable to
   today's baseline.

## Part 5 — Integration plan: toggles, refactor points, docs/launch updates

Design-only, per user's "design now, implement later" scope choice — no
code written yet. Goal here specifically: (a) a way to A/B the new
corner-map mechanism against today's live heuristics without deleting the
old code path, mirroring the existing `map_path`/`path_map_path` on/off
pattern the user already knows, and (b) a complete list of every doc/launch
file that touches the fields this would add, so nothing drifts the way
`fsae_params.yaml`/`fsds_simulator` already have a couple of times this
session.

### 5a. New toggle — VERIFIED against the actual existing mechanism (not the MPCParams pattern)

Checked directly (not guessed): `path_map_path`/`use_precomputed_path` are
**NOT** `MPCParams` fields — they're hand-written `DeclareLaunchArgument`s
in `sim.launch.py`/`control.launch.py`, wired through a real two-layer
toggle, and `use_precomputed_speed`/`use_precomputed_path` already exist
as exactly the kind of boolean-gates-a-CSV-path toggle the user described.
The actual mechanism, read from `control.launch.py`:

```python
effective_path_map_path = IfElseSubstitution(
    EqualsSubstitution(use_precomputed_path, 'true'),
    path_map_path,   # if true: use the configured CSV path
    '',              # if false: pass empty string -> node's `if path_map_path:` sees "unset"
)
```

i.e. the boolean doesn't gate a separate code path inside the node at
all — it gates WHETHER THE CSV PATH STRING EVEN REACHES THE NODE. The
node itself only ever sees "a path was given" or "" ("live planner
mode"); it has no idea a boolean flag exists upstream. This matters
because it changes where the new toggle has to live: **a new boolean at
this same launch-argument layer, gating a NEW hand-written launch
argument**, following the `use_precomputed_path`/`path_map_path` PAIR
exactly — not a single `MPCParams` field as originally sketched.

**Revised design**:

```python
# New DeclareLaunchArgument in BOTH sim.launch.py and control.launch.py,
# declared next to path_map_path/use_precomputed_path (NOT in the
# MPC_PARAM_FIELDS-generated block just below them):
DeclareLaunchArgument(
    'use_precomputed_corner_map', default_value='false',
    description="true -> segment path_map_path's STATIC path into "
                "per-corner metadata once at load (see mpc_core.py's "
                "CornerMap), replacing the live kappa_max_abs lookahead "
                "scan / exit-decay tracker / U-turn accumulator with "
                "exact lookups. Has no effect unless use_precomputed_path "
                "is ALSO true (nothing static to segment otherwise) -- "
                "same dependency shape as use_precomputed_path itself "
                "depending on path_map_path being set."),
```

**Verified**: `use_precomputed_speed`/`use_precomputed_path` do NOT appear
anywhere in `mpc_controller_standalone.py`/`mpc_controller.py` — grepped
both node files directly, zero hits. They are PURELY launch-file sugar:
`control.launch.py`'s `IfElseSubstitution` resolves them into an
empty-or-real `map_path`/`path_map_path` STRING before the node ever
starts, and the node only ever declares/reads those two string
parameters (confirmed at `mpc_controller_standalone.py`'s
`declare_parameters` block, `('map_path', '')`/`('path_map_path', '')`).

`use_precomputed_corner_map` has no equivalent string to blank away at the
launch layer — it's a pure behavior switch, not "which path string to
use," so unlike its siblings it CANNOT be launch-layer-only. It must be
declared as a genuine new node-level boolean parameter (its own
`declare_parameters` entry, e.g. `('use_precomputed_corner_map', False)`,
read via `get_parameter(...).get_parameter_value().bool_value`, same as
`enable_dynamic_speed_cap` already is in that same node). The
`DeclareLaunchArgument` in `sim.launch.py`/`control.launch.py` still needs
to exist (to expose it on the `ros2 launch` command line and give
`launch_all.sh` something to forward), but it passes straight through
as a plain arg forward — no `IfElseSubstitution` needed for this one,
since there's no downstream string to conditionally blank.

`default_value='false'`: same rollout posture as every other new mechanism
this session (`curvature_forcing_enabled` was `True` by default and that
bit us) — land it OFF, prove it live, then consider flipping the default.

**Behavior matrix** (mirrors `use_precomputed_path`'s existing dependency
on `path_map_path` being non-empty, one level up):

| `use_precomputed_path` | `use_precomputed_corner_map` | Behaviour |
|---|---|---|
| `false` (live planner) | any | `path_map_path` never reaches the node (blanked by the existing `IfElseSubstitution`) — `_static_path` is None, corner map never built, today's live heuristics run unchanged. Log a one-time info message if `use_precomputed_corner_map=true` was passed anyway, so a launch typo doesn't silently do nothing. |
| `true` | `false` (default) | Static path loaded as today; corner map NOT built; every mechanism in 4c behaves exactly as it does right now. This is today's behaviour, unchanged — the safe default. |
| `true` | `true` | Corner map built once at load (in the same place `_static_path` is currently loaded); `kappa_max_abs` lookup, exit-decay gating, and U-turn scoring switch to the corner-map lookups from 4c. Falls back to live heuristics with a logged warning if segmentation fails (e.g. a malformed/degenerate CSV) rather than crashing the node — same defensive posture as the existing `except (OSError, ValueError)` blocks around `load_path_profile_csv`/`load_speed_profile_csv`. |

### 5b. Refactor points in existing code (what actually has to change, file by file)

**`mpc_core.py`:**
- New module-level function `_segment_corners(path) -> CornerMap` (a small
  `@dataclass`, see 4a) — pure function, no controller state, easy to unit
  test in isolation before it ever touches `compute()`.
- `MPCController.__init__` gains `self._corner_map: CornerMap | None =
  None` (mirrors `self._static_path`'s own None-until-set pattern one
  layer up, in the node).
- New method `MPCController.set_static_path(path)` (or extend an existing
  entry point if one already exists for this) that calls
  `_segment_corners` and stores the result — called ONCE, from the node,
  not from `compute()`. `compute()` must NEVER call `_segment_corners`
  itself; that would silently reintroduce a per-tick cost this whole
  design exists to remove.
- `_error_state`/`compute()`: the three call sites in 4c's table
  (`kappa_max_abs` computation, `_update_lookahead_peak` call,
  `_lookahead_exit_boost` call) each need an `if self._corner_map is not
  None:` branch alongside the existing live-heuristic branch — NOT a
  replacement of the old branch, so `use_precomputed_corner_map=False`
  (or no static path) is a byte-for-byte no-op versus current behaviour.
  This mirrors how `curvature_forcing_enabled`'s dynamics-constraint `w`
  parameter was added earlier this session: new code path fully gated,
  old path untouched and still the default.
- `_update_lookahead_peak`, `self._last_peak_kappa_abs`,
  `self._dist_since_peak`, `self._armed_for_next_peak`: KEEP, do not
  delete — still the live-planner-mode code path (5a's first row). Only
  bypassed, never removed, as long as live-planner mode exists.

**`mpc_controller_standalone.py` / `mpc_controller.py`** (both nodes,
mirror each other today and must keep doing so):
- Declare `('use_precomputed_corner_map', False)` in the SAME
  `declare_parameters` block as `('map_path', '')`/`('path_map_path', '')`
  — verified this is a plain node-parameter block, not
  `declare_mpc_params`'s separate `MPCParams`-generated call one line
  below it, so this is unambiguous: it's a wiring/path-mode concern like
  `path_map_path` itself, not a numeric MPC gain, and goes with that
  parameter, not with `MPCParams`.
- Read it via `get_parameter('use_precomputed_corner_map').get_parameter_value().bool_value`
  (same pattern as `enable_dynamic_speed_cap`, a few lines above
  `map_path`/`path_map_path` in that same method).
- Where `self._static_path = load_path_profile_csv(...)` succeeds, add the
  `self._mpc.set_static_path(self._static_path)` call gated on the new
  bool being True — one new call, same success branch, same try/except
  already there.

**`fsae_MPCTest` side** — per the standing "don't do offline, live-test
this" instruction that's been in effect all session for the adaptive-gain
work, this design does NOT propose touching `fsae_MPCTest`/`settings.py`/
`model_utils.py`/`rollout_core.py` yet. Flag this explicitly as a decision
point, not an oversight: CLAUDE.md's parity rule normally requires every
planning/control change on both sides, but this investigation has been
entirely live-only by explicit user instruction throughout. **Before
implementing**, confirm whether this specific feature should break that
pattern (a structural mechanism like this might be worth prototyping
offline first, unlike the weight tweaks that came before it) — do not
assume either way without asking.

### 5c. Docs that need updating (checklist, once implemented)

All in `fsae_MPCTest/docs/` (the authoritative docs per CLAUDE.md, even
though the CODE changes are live-only for now — the docs describe the
live mechanism and must stay accurate regardless of which side changed):

- **`planning_control_sync.md`**:
  - `use_precomputed_corner_map` is a node-level launch parameter (5a),
    NOT an `MPCParams` field like every other flag in the existing
    numeric-parity table — it does not belong in that table as-is. Either
    add a new, clearly-separated "path-mode flags" mini-table alongside it
    (matching how `path_map_path`/`use_precomputed_path` themselves are
    documented, if they appear in this file at all — check first) or fold
    it into whatever section already explains `path_map_path`'s existing
    true/false behaviour, rather than mixing node-parameter and
    `MPCParams`-field rows in one table.
  - New dated section (`## Precomputed corner segmentation — replaces N
    live curvature heuristics with exact lookups (<date>)`) covering: the
    3f-pre bug that motivated it, the CornerMap structure, the 4c
    migration table, and which mechanisms were fully replaced vs. left
    live (4d).
  - Update the existing "Curvature-forcing term" section's cross-references
    if any of `_update_lookahead_peak`/`kappa_max_abs` plumbing it
    references changes shape.
- **`architecture.md`**: the existing lookahead-mechanism summary sections
  (added when curvature forcing / steer-effort-relax were built, per
  CLAUDE.md's own log of this session) need a new subsection describing
  the corner-segmentation layer and updating the "structural limit" callout
  if this closes any part of the gap it describes.
- **`tuning.md`**: new field-reference entry for
  `use_precomputed_corner_map` (following the existing table format used
  for e.g. `curvature_forcing_enabled`/`anti_hunt_k_lookahead`), plus an
  explicit note in whichever existing sections describe
  `_lookahead_exit_boost`/U-turn detector/`kappa_max_abs` that their
  behaviour now depends on this flag.
- **`junior_project_mpc_docs.md`**: if this doc's curvature-blindness
  explanation (added earlier this session) references the live lookahead
  mechanism's limitations, it needs a note that a precomputed variant now
  exists for the static-path case — keep the pedagogical explanation of
  WHY the live version has to approximate, don't just delete it, since the
  live-planner path source still needs that explanation.

This working file (`late_turn_in_investigation.md`) itself is NOT one of
the authoritative docs — fold its final content into the above once
implemented, then delete it, per its own header note.

### 5d. Launch file / config updates (checklist, once implemented) — VERIFIED, no open questions

- **`fsae_params.yaml`**: NO CHANGE. Confirmed `path_map_path`/
  `use_precomputed_path` don't appear in this file at all (grepped
  `fsae_params.yaml` for both — zero hits); they're launch-argument-only.
  `use_precomputed_corner_map` follows the same rule: no yaml entry.
- **`ros2/launch_all.sh`**: this file already has the EXACT pattern to
  extend — `USE_PRECOMPUTED_SPEED=true` / `USE_PRECOMPUTED_PATH=true`
  shell variables (line 105-106) feeding the `[ "$USE_PRECOMPUTED_PATH" =
  true ] && echo "$PATH_CSV"` conditional (line 114-115) and the
  `use_precomputed_speed:=$USE_PRECOMPUTED_SPEED
  use_precomputed_path:=$USE_PRECOMPUTED_PATH` forwarding on the `ros2
  launch` command lines (lines 402, 409). Add
  `USE_PRECOMPUTED_CORNER_MAP=false` next to the other two `USE_PRECOMPUTED_*`
  variables (line ~105-106, not in the `MPC_*` weight shortlist further
  down — this is a path-mode switch grouped with its siblings, not a
  tunable weight) and
  `use_precomputed_corner_map:=$USE_PRECOMPUTED_CORNER_MAP` alongside the
  other two on both `ros2 launch` lines (402 and 409 — both container and
  host variants must get it, they're kept in sync today).
- **`control.launch.py` / `sim.launch.py`**: add the new
  `DeclareLaunchArgument('use_precomputed_corner_map', default_value='false', ...)`
  in both files, in the same block as `path_map_path`/`use_precomputed_path`'s
  existing declarations (confirmed both declare these three: `map_path`,
  `use_precomputed_speed`, `path_map_path`, `use_precomputed_path` as a
  visually grouped set — `sim.launch.py` lines 103-150,
  `control.launch.py`'s equivalent block). Unlike `use_precomputed_path`,
  this new flag needs NO `IfElseSubstitution` (5a) — just declare it and
  forward the raw `LaunchConfiguration` straight through to
  `control.launch.py` (from `sim.launch.py`) and down into the controller
  node (from `control.launch.py`), the same way `enable_dynamic_speed_cap`
  is already forwarded as a plain bool with no conditional resolution.
- **Verify-parity script**: NO CHANGE NEEDED. Confirmed
  `use_precomputed_corner_map` is not and should not become an `MPCParams`
  field (5a), so the existing `MPCParams`-fields-vs-yaml-keys parity
  script has nothing new to check here — it was never checking
  `path_map_path`/`use_precomputed_path` either, and this follows them.

### 5e. Order of operations for the actual implementation session

1. Write `_segment_corners` + `CornerMap` as a standalone, testable unit —
   validate against the real track CSV per 4e step 1 BEFORE touching
   `compute()` at all.
2. Wire the `MPCController` side (4c/5b) with the flag defaulting False,
   confirm `use_precomputed_corner_map=False` reproduces today's behaviour
   bit-for-bit on the existing recorded log (a regression check, not just
   "it doesn't crash").
3. Wire the node-level parameter (both `mpc_controller_standalone.py` and
   `mpc_controller.py`, kept in sync per usual) + `set_static_path` call.
4. Wire the launch-file layer (5d): `DeclareLaunchArgument` in both
   `.launch.py` files, plain forward (no `IfElseSubstitution`), plus
   `launch_all.sh`'s `USE_PRECOMPUTED_CORNER_MAP` variable and its two
   `ros2 launch` call-site additions.
5. Docs (5c) updates, in the same change, not deferred — this session's
   own CLAUDE.md-documented history of yaml/`fsds_simulator` drift
   happened specifically because doc/config updates were treated as a
   follow-up rather than part of the same commit.
6. Offline validation (4e steps 1-2), THEN live A/B (4e step 3) once lag
   is resolved.

### 3f-pre. Does the lookahead window "see the corner ending" and revert early mid-corner?

User question, checked directly rather than assumed. Two distinct
mechanisms use forward-looking curvature, and they behave differently:

**`kappa_max_abs` (drives the approach boosts / yaw-rate relax): NOT
premature.** The scan window is `[base_idx, base_idx+lookahead_dist]`,
always anchored at the car's OWN current position — never anchored at the
corner's start. Checked against both a constant-radius corner (kmax stays
pinned at the corner's own curvature for the car's entire time inside it —
can't be seen as "ending" until the car's own position passes the exit)
and a realistic rising-then-falling curvature profile (kmax tracks current
curvature exactly, point for point, once past the apex — because nothing
ahead is ever sharper than "right now" past the peak). This mechanism is
sound: it never reads a corner as finished while the car is still in it.

**`_lookahead_exit_boost` (Q[2,2] only): YES, this one reverts early —
confirmed on a real log.** This is a DIFFERENT signal: not
`kappa_max_abs`, but `_dist_since_peak`, which starts counting the moment
CURRENT curvature (`kappa`, at the car's own position) stops rising — i.e.
right at the apex — and decays linearly to 1.0 (no boost) over
`exit_decay_dist = clip(car_speed*2.5, 5, 25)` metres, independent of
whether the car has actually finished correcting its heading. Checked
against `mpc_standalone_control_1786516297.csv` (lap1's hard corner,
apex ~t=4.9-5.0): `m_Q_epsi_exit` is already back to `1.00` (fully
decayed) by **t=5.92** — while the car is still at `e_psi=-13.96°`, speed
has dropped to 5.3 m/s (`exit_decay_dist` at that speed is only 13.25m),
and the WORST of the heading correction is still ahead of it (steering
saturates at ±25° repeatedly through t=6.5-7.3 as `e_psi` swings from
-14° through +25° and back, `e_y` still recovering from -1.7m). The exit
boost — specifically designed to help heading-error recovery after a
corner's peak — had already worn off well before the correction it exists
to help with was anywhere near resolved.

This is a real, evidenced instance of exactly what was asked: a
lookahead/decay mechanism reverting to baseline while the corner (in the
sense of "still actively correcting from it") isn't functionally over yet.
It's keyed on DISTANCE since the geometric apex, not on the actual
magnitude of remaining tracking error — so a corner that produces a bigger
excursion than usual (as this one did) gets LESS exit-boost help exactly
when it needs more, because the fixed distance-based decay doesn't know
the recovery is taking longer than normal.

**Candidate fix** (not yet applied — flagging for a decision): gate
`_lookahead_exit_boost`'s decay on remaining `|e_psi|` as well as
distance, e.g. holding the boost active until BOTH the distance has
elapsed AND `|e_psi|` has dropped below some threshold — same "continuous,
no discontinuity" style as every other mechanism here (e.g. multiply the
existing decay_frac by an `|e_psi|`-based saturating term so the boost
can't reach 1.0 while heading error is still large, but decays exactly as
today once e_psi is small). This is a genuinely new mechanism, not a
constant retune, so it should go through the same before/after live-test
discipline as everything else, and is a candidate for the SAME test
session as 3f's disable-and-compare list below rather than a separate one.

### 3f. Simplification audit — which mechanisms have actual evidence, which are just "enabled, untested"

Every one of these mechanisms was added to fix a specific reported symptom,
but several carry an explicit "NOT VALIDATED" / "TEMPORARY/EXPERIMENTAL"
tag in their own comments — meaning they were turned on and left on, not
shown to help via any before/after comparison. With three independent
"raise the boost" attempts all measured WORSE today (Part 2), it's worth
asking whether some of these are actively part of the problem rather than
just under-tuned. Evidence status, mechanism by mechanism:

| Mechanism | Evidence FOR | Evidence AGAINST / status |
|---|---|---|
| `delay_compensation_enabled` | **Yes** — disabled outright and compared: rmse/sat/e_psi all got WORSE without it. | None. Keep — this is the one mechanism in the whole file with a real disable-and-compare result. |
| `adaptive_r_rate_enable_in_corners` | Indirect — the alternative (discontinuous cutoff) was tried and caused "severe lag specifically in corners" (solver-iteration spikes at the threshold crossing). | Not a same-mechanism A/B, just "the other option was worse." Keep as-is; not a simplification target. |
| `adaptive_q_scaling_enabled` (centred Q[0,0] softening) | None. | **A live A/B test found it enabled coincided with a bad late-turn-in episode** (own comment, mpc_core.py:1327). "Not proven solely causal" but never re-validated either. Its ENTIRE justification is "reduce small-error hunting on a straight" — a chatter/comfort concern, not a lap-time one — while its documented failure mode is directly implicated in the exact symptom under investigation (late turn-in). Fixed today via a kappa_max_abs gate, but the gate only patches the corner-approach interaction; the mechanism's core premise (soften Q[0,0] near centreline) has never been shown to help anything. **Strong removal/disable candidate** — see recommendation below. |
| `steer_effort_straight_boost_enabled` | None — comment says "NOT VALIDATED -- enabled for live testing; treat results as unconfirmed." | Makes R[0,0] up to 1.5x MORE expensive on a straight, fading fast (k=20) as a corner is detected. If it's not fading fast enough at the moment a gentle corner's signal first appears (same k=20 sharpness question as anti-hunt's k=15 in 3a), this could be ANOTHER mechanism making early corrective steering artificially expensive right when Part 1/3e's "small early error, no commitment" ceiling is most sensitive to added cost. Never isolated/compared. **Candidate for a disable-and-compare test**, cheap to try (single bool flag, no interacting state). |
| `steer_rate_anti_hunt_enabled` | None — comment says "Temporary experiment requested to suppress low-error steering hunt; NOT validated against a live log or VALIDATION_SUITE." | This is the SAME mechanism already found today to be actively cancelling curvature-forcing's correction (Part 2 #1/#2) and still under-relaxed for gentle corners (3a). Its own justification was never validated even before today's finding. Given it can multiply R_rate[0,0] by up to 6x specifically when the car looks "centred and calm" — exactly the state right before a good, early turn-in — and its lookahead gate is only a same-day patch, not a redesign, this is a **strong candidate for a disable-and-compare test**, not just a further re-gate. If disabling it live-tests neutral-to-better, that's a big complexity reduction (one fewer 4-factor multiplicative mechanism) with no loss. |
| `adaptive_q_lookahead_enabled` (approach/exit Q boosts) | Weak/circumstantial — comment notes that disabling it to isolate an unrelated jitter regression did NOT fix that regression (rules this OUT as that regression's cause, which is evidence of innocence, not evidence of benefit), and that without it steering pins at 25° for over a second in at least one corner. | "NOT VALIDATED" tag still present. This is the CORE anticipation mechanism the whole investigation has been trying to strengthen, though — not a candidate for removal, just for the ordering/gating fixes already in progress (3a/3b). |
| U-turn detector (`adaptive_q_uturn_*`) | Motivated by one specific logged U-turn where steering was saturated 1.3s with no other mechanism able to help before the corner. | Checked today against a full 2-lap log: fires on 14.6% of ticks but never exceeds severity 0.29 (of 0-1) on this track — i.e. its ceiling values (1.6x boosts) are essentially never reached here. Not obviously harmful, but also not doing much on ordinary tracks — low complexity/low current relevance. Not a priority either way. |
| `low_speed_steer_rate_boost_enabled` | N/A — already disabled by default, correctly, after live-testing worse (documented incident, "struggling to turn in early after this was enabled"). | Already off. Non-issue, already resolved correctly. |
| `curvature_forcing_enabled` | N/A — already disabled today after being found structurally unsound (Part 2 #1). | Already off, code kept for a future redesign. Non-issue. |

**Recommendation, ranked by expected value**: the next live test (once
lag is resolved) should be a **disable-and-compare** on
`adaptive_q_scaling_enabled` and `steer_rate_anti_hunt_enabled` — TWO
mechanisms with zero positive evidence, one with actual negative A/B
evidence, both directly theorised (today, with numbers) to fight early
turn-in specifically. This is cheap to test (two bool flags, no new code)
and, unlike the three ceiling-raises tried today, tests REMOVING
complexity rather than adding more of it — directly opposite in kind to
what's already failed three times. If disabling either (or both) tests
neutral-or-better, that is a genuine simplification: fewer interacting
multiplicative gates, less surface area for exactly the kind of
cross-mechanism cancellation already found twice today (anti-hunt vs
curvature-forcing; adaptive_Q_scaling vs lookahead approach-boost).

Suggested test order (isolate one variable at a time):
1. `adaptive_q_scaling_enabled=False` alone (today's kappa_max_abs gate
   becomes moot/no-op — simpler than re-gating further).
2. If (1) doesn't fully resolve it, `steer_rate_anti_hunt_enabled=False`
   alone, on top of (1).
3. Compare against today's un-reverted baseline
   (`anti_hunt_k_lookahead=15`, both scaling mechanisms still on) using the
   same corner/log methodology as Part 1's lap1-vs-lap2 analysis.

### 3e. Lap1-vs-lap2 S-curve — RESOLVED, same root cause one corner earlier

Checked: yes, the gentle left bend itself is turned into late on lap 1,
same as the hard corner. From `mpc_standalone_control_1786516297.csv`,
during the gentle-bend-only window (kappa_max_abs~0.007-0.03, well before
the hard corner's signal ramps):

- **Lap 1** (t≈2.9-3.1s): steering pairs `+4.43/-4.50`, `+4.50/-4.50` —
  net ≈0 per pair. `car_yaw` climbs only 0.150→0.163→0.208 rad over ~0.6s
  (slow, matches a car barely turning).
- **Lap 2** (t≈56.9-57.7s): steering pairs `+9.33/+0.33`, `+9.33/+0.33`,
  `+9.07/+0.48`, `+8.73/+0.07` — strongly asymmetric, net ≈+4.9 per pair
  (one big positive tick, one near-zero, essentially never negative).
  `car_yaw` climbs 0.113 rad in under 0.2s at one point — much faster net
  rotation from the SAME kappa_max_abs≈0.007 input as lap 1's stalled pair.

Same oscillation-amplitude, different net-bias signature already
identified at the hard corner (Part 1 investigation, mean steer 1.96° vs
3.75° for near-identical |steer| oscillation) — reproduces one corner
earlier, at lower stakes (a gentle bend tolerates a slow, oscillating,
non-committal response without running wide; the hard corner right after
it does not).

**Root cause of the bias, checked directly (not a gain-schedule
difference)**: at the exact tick each window starts, `e_psi` is
**~+0.2° (lap 1)** vs **~-4.5° (lap 2)** — lap 2 already carries a real,
substantial existing heading error at that point (inherited from further
back — see the original lap1-vs-lap2 note on the connected corner before
this one), lap 1 does not. The QP's asymmetric response in lap 2
(alternating "correct hard" ~+9-11° / "barely anything" ~0-2° commands) is
the QP correctly reacting to a genuine, nonzero `e_psi` it's being asked to
fix — not a mysterious bias from identical inputs. Lap 1's near-symmetric
oscillation around `e_psi≈0` is *also* the QP behaving correctly: with
almost no real heading error yet and only a very mild `kappa_max_abs`
(~0.007-0.03) lookahead signal, there is genuinely very little for the
weights to act on, so the command has little reason to commit to a
direction. **This is the SAME structural ceiling already diagnosed
elsewhere in this investigation** (a small/gradual corner produces no real
tracking error early enough for cost-reweighting to bite), now confirmed
to reproduce identically on a gentle bend as well as a sharp corner — not
a second, independent bug. It does NOT point at a gain-schedule
miscalibration between the two laps; it points at the lookahead-boost
mechanisms (Part 1) simply not being strong/early enough, on their own, to
manufacture commitment before real error exists — consistent with Part 2's
repeated finding that raising Q/R ceilings further hasn't helped, and with
the curvature-forcing avenue (Part 2 #1) already being tried and found
unsound as an attempted structural fix for exactly this gap.

## Part 6 — Implementation complete (2026-08-12)

Implemented per Part 5e's order of operations. Live-only (user chose this
explicitly when asked — see decision point flagged in 5b — over prototyping
in `fsae_MPCTest` first).

**Done:**
1. `CornerMap`/`_segment_corners`/`_corner_map_lookahead` written in
   `mpc_core.py`, offline-validated against `comp_test_map_3/raceline.csv`
   (9 runs found, 7 real corners with arc-length/speed-profile alignment
   confirmed physically sane; 2 single-waypoint blips at path start/end,
   expected).
2. `MPCController.set_static_path()` + 3 gated call sites wired
   (`kappa_max_abs` lookup, peak-tracker replacement, exit-boost hold-fix).
   Regression-checked: flag off / no `set_static_path()` call reproduces
   `compute()`'s output bit-for-bit (200-tick synthetic replay, exact
   float equality). Flag on measurably changes output (max abs diff 0.072
   over the same replay) — confirming the new path is genuinely exercised.
3. `use_precomputed_corner_map` node parameter wired in both
   `mpc_controller_standalone.py`/`mpc_controller.py`, mirroring
   `path_map_path`'s load-and-log pattern.
4. Launch layer wired: `DeclareLaunchArgument` in both `sim.launch.py`/
   `control.launch.py` (plain forward, no `IfElseSubstitution`, per 5a),
   `USE_PRECOMPUTED_CORNER_MAP=false` + both `ros2 launch` call sites in
   `launch_all.sh`.
5. Docs updated same-session: `planning_control_sync.md` (new dated
   section + numeric-parity table row, explicitly flagged NOT MIRRORED),
   `architecture.md`, `tuning.md` (§4.5b), `junior_project_mpc_docs.md`.
6. **3f-pre's exit-boost bug fix validated offline against the real log**
   that found the bug (`mpc_standalone_control_1786516297.csv`, lap 1's
   hard corner). Replayed `e_psi`/`last_peak_kappa`/`dist_since_peak`
   through the hold-fix at several `epsi_hold_thresh` values (10°/20°/30°):
   at every value tested, `m_fixed` stays above 1.0 throughout the
   `t=5.88-7.03` window (peak `|e_psi|` ~29°) where the ORIGINAL mechanism
   (`m_base`, matching the log's own `m_Q_epsi_exit=1.000`) had already
   fully decayed. Confirmed the fix also releases cleanly: at `t=6.17`,
   once `e_psi` drops below 5°, `hold_frac→0` and `m_fixed→m_base` — no
   permanent latch.

**Both new flags land at their off/no-op defaults**
(`use_precomputed_corner_map=false`, `adaptive_q_lookahead_epsi_hold_thresh_rad=0.0`)
— NOT YET LIVE-TESTED. Next step when lag resolves: live A/B on the same
corner characterised throughout this file, same before/after discipline as
everything else here.

Parts 1-3/3e/3f/3f-pre's findings (mechanism walkthrough, evidence audit,
disable-and-compare recommendation, lap1-vs-lap2 root cause) describe the
lookahead gain-scheduling family that `mpc_core.py`/`mpc_params.py` later
deleted wholesale (see those files' own "removed" comments) — of limited
practical use now that the mechanism is gone, but the methodological
lesson (disable-and-compare over raise-the-ceiling) isn't recorded
elsewhere. Part 4/5/6 (the corner-segmentation feature, itself later
deleted by the same rewrite) is summarized in `planning_control_sync.md`'s
"Precomputed corner segmentation" section.

## Part 6b — Curvature-forcing term: the QP's own prediction was blind to the path bending ahead (2026-08-12)

**Implemented on both sides, offline smoke-tested (no crash), live symptom
not yet re-confirmed fixed at the time this section was first written.**
Reported symptom, after the steering-effort relax (Part 0 below) and the
`ey_k` fix were both live-tested: the car still turned in late on
sudden/sharp corners.

**Root cause (deeper than either fix above)**: every existing lookahead
mechanism (`adaptive_Q_lookahead`, `lookahead_steer_effort_relax`, etc.)
only reweights an *existing* tracking error — it changes how expensive
being off-line already is, never the QP's own predicted trajectory. The
QP's internal dynamics model (`Ad`/`Bd`) has *no path-curvature term at
all*: with `e_y ≈ e_psi ≈ 0` (car dead on-line on the still-straight
approach, exactly the state before any real corner), the QP's own 35-step
rollout predicts staying at `≈0` for the whole horizon regardless of how
sharply the real path bends ahead. No amount of cheaper steering effort
can fix this, because there is no predicted error yet for a cheaper weight
to act on — this is why the steering-effort relax fix, while correctly
implemented and firing, could not fully close the gap on its own.
Confirmed directly: `m_R_steer_relax` and `Q_ey_eff` moved correctly and
over a full second early in live telemetry, while `steer_deg` stayed at
≈0° the whole time, because there was no `e_y`/`e_psi` for those
reweighted costs to react to.

**Mechanism**: `_curvature_horizon_profile`/`curvature_horizon_profile`
walks the reference path forward from the car's current position by the
PREDICTED arc-length at each of the QP's `N` steps (`v_x·k·dt`), returning
signed curvature at each step (distinct from `_lookahead_curvature_profile`,
which only returns the single peak value used for the Q/R reweighting in
Part 0/Part 2). That per-step curvature feeds a new forcing term added
directly to the dynamics constraint — `x[:,1:] == Ad@x[:,:-1] + Bd@u + w`,
where `w` is zero everywhere except the `e_psi` row: `w[2,k] =
-v_x·κ(s_k)·dt·gain`. The sign follows directly from `e_psi = car_yaw -
path_yaw` and `path_yaw_rate = v_x·κ`: a path curving left (`κ>0`) drives
`e_psi` negative over the horizon even if the car holds a constant
heading. `w=None` (the default for any caller that doesn't pass it) sends
an all-zero array, making this an exact no-op — existing callers are
unaffected.

**Verified with a synthetic constant-curvature path** before trusting the
sign: with `curvature_forcing_enabled=False`, commanded steering is
exactly `0.000°` 24 m before a 20 m-radius left bend with zero tracking
error (reproducing the diagnosed bug); with it `True`, steering correctly
leans toward the bend (`+1.5°` at 17 m/s, before any `e_y`/`e_psi` error
exists) — and mirrored for a right bend (`-1.5°`). A closed-loop
simulation (the car actually driving toward the bend using its own
commands, not a single frozen snapshot) confirms the controller commits to
real, sustained left steering (ramping through `+2°` → `+8.6°`) well
before reaching the curved section.

**A genuinely confusing artifact was found and ruled out during
verification**: at the exact instant `x0=0` with no history, the very
first commanded step can be a tiny (hundredths-of-a-degree), momentarily
WRONG-SIGN "flinch" before the plan settles into the correct direction —
e.g. `-0.01°` at the first tick, then correctly positive from the second
tick onward. This is a real, known linear-MPC phenomenon (the coupled
`e_y`/`e_psi`/`r` state dynamics briefly trade off a different combination
of costs at `x0` exactly zero) and is negligible in magnitude compared to
the real turn-in signal — confirmed via the closed-loop simulation above,
where it never shows up as a problem once the car is actually driving
(nonzero `x0` history from the previous tick's plan).

**A second problem was found DURING live testing of this fix**: even with
the forcing term firing correctly and early (`w_epsi_sum` already
`-1.0` to `-1.35` more than 2 s before a sharp corner, confirmed in
telemetry), the car still turned in late. Root cause: `_steer_rate_anti_hunt`
was tuned before this forcing term existed — it reads "`e_y`/`e_psi`/current
`kappa` all near zero" as "nothing happening, dampen any steering-rate
change," which is now exactly the state the forcing term deliberately
produces on approach. Live telemetry showed steering oscillating ±10° with
no net commitment while `m_Rrate_antihunt` sat at `1.2×`–`3.3×` throughout
the approach — anti-hunt was actively cancelling the forcing term's whole
effect, every tick, right up until real curvature arrived and steering had
to snap to the 25° stop anyway. Fixed by adding a fourth, `kappa_max_abs`-gated
factor to `_steer_rate_anti_hunt`/`steer_rate_anti_hunt` (`anti_hunt_k_lookahead`,
default `60.0`, same `k` as the existing current-curvature term) that
relaxes the anti-hunt boost once a real corner is detected in the
lookahead window — not just once the car is already turning through it.

**Implemented in**: live `mpc_core.py`/`mpc_params.py`
(`curvature_forcing_enabled`, `curvature_forcing_gain`,
`anti_hunt_k_lookahead`), offline `controller/optimiser.py`'s
`init_parameterized_mpc`/`solve_mpc` (new `w` parameter) and
`controller/model_utils.py` (`curvature_horizon_profile`,
`steer_rate_anti_hunt`'s new `kappa_max_abs`/`k_lookahead` params),
`sim/rollout_core.py`, `settings.py` (`CURVATURE_FORCING_ENABLED`,
`CURVATURE_FORCING_GAIN`, `ANTI_HUNT_K_LOOKAHEAD`), `fsae_params.yaml`,
`launch_all.sh`'s shortlist, and the `fsds_simulator` mirror.

### Re-tested live (2026-08-12) — regression found and fixed: `anti_hunt_k_lookahead=60.0` was itself too aggressive

The combined fix made things *worse*: still no earlier net turn-in, plus a
new symptom — brief wrong-direction (rightward) steering flicks right
before some left corners, occasionally costing enough line to miss the
corner.

Log analysis (`mpc_standalone_control_1786509640.csv`) found the actual
mechanism has nothing to do with curvature-forcing's sign or magnitude —
`w_epsi_sum` was firing correctly and its tick-to-tick drift was negligible
(±0.01–0.02) throughout the approach. Instead, `steer_deg` was swinging
±9° almost every tick with an exactly-repeating pattern, the signature of a
**pre-existing, already-documented mechanism**: `predict_ahead()`'s
rollforward (see `mpc_core.py`'s "Delay compensation" comment) compounds
pose noise through `n_delay` steps with no ground-truth correction, and
that noise reaches the steering command directly at small `e_y`/`e_psi` —
"steer swinging +-5-10 deg per tick... causing late turn-in and running
wide," exactly as that comment already warned, unrelated to this session's
work.

What changed this session is how much that pre-existing noise gets damped.
`boost_lookahead = 1/(1 + k_lookahead·|kappa_max_abs|)` with `k=60` already
cuts anti-hunt's damping in half at `kappa_max_abs=0.02` — a corner still
far outside the window curvature-forcing actually needs help fighting
through (`kappa_max_abs` there is usually 0.06+ by the time forcing's
correction matters). Comparing the same approach-window statistics before
vs. after the gate: mean anti-hunt multiplier fell at **every** curvature
level (e.g. `kappa_max_abs<0.02`: `2.19×→1.50×`), and mean `|Δsteer_deg|`
per tick rose correspondingly (`2.74→3.46`, `2.63→2.93`, `3.59→4.26`) —
confirming the gate was relaxing damping broadly, not selectively where
curvature-forcing's early correction needed room, letting the pre-existing
noise oscillate more everywhere. Reversal *rate* alone looked unchanged
(0.171 vs 0.174) precisely because this is amplified-noise, not new noise —
the amplitude grew, not the frequency, which is why it read as "still late"
(bigger, noisier swings, no more net commitment) rather than a raw increase
in direction changes.

**Fix**: lowered `anti_hunt_k_lookahead` from `60.0` to `15.0` on both
sides (`mpc_params.py`, `settings.py`, `fsae_params.yaml`,
`launch_all.sh`'s shortlist, `fsds_simulator` mirror). At `k=15`,
`boost_lookahead` stays close to `1.0` (little relaxation) until
`kappa_max_abs` is around `0.05`+, and only drops substantially past
`0.1`–`0.15` — inert during the very early, faint lookahead signal, active
once a corner is close/sharp enough for curvature-forcing's correction to
actually need the room. This is the change recorded as Part 2, row 2
above.

**While investigating this, the `MPC_ANTI_HUNT_K_LOOKAHEAD` comment in
`launch_all.sh` was found to have the tuning direction backwards** ("Lower
= relaxes anti-hunt sooner/more" — actually the opposite: higher `k` means
more relaxation at a given `kappa_max_abs`, since `k` multiplies inside the
denominator). Fixed alongside this change.

### Re-tested live at `k=15` (2026-08-12) — curvature forcing itself found structurally unsound, disabled

`anti_hunt_k_lookahead=15.0` worked as intended: the same approach-window
statistics from the regression above returned close to their original
(pre-`k=60`) values (`mean_antihunt` at low `kappa_max_abs`: `2.19×` →
(regression) `1.50×` → (fixed) `2.01×`; `mean|Δsteer|`: `2.74` → `3.46` →
`2.90`). But the user still reported no real improvement in turn-in
timing, which meant the anti-hunt interaction was never the main story —
it was a second-order effect on top of a first-order problem in the
forcing term itself.

**Isolated QP testing (not a live log this time — a controlled synthetic
test, `x0=0`, no noise, no other adaptive mechanism, using the actual
`init_parameterized_mpc`/`solve_mpc`/`curvature_horizon_profile` code)
found the forcing term is unsound at any gain that matters:**

At `curvature_forcing_gain=1.0` (the "physically exact" default) with a
realistic corner (radius 13 m, 17 m/s, corner start ~15 m / ~0.9 s ahead —
comparable to the live log's approach windows), the QP's own predicted
`e_psi` trajectory only deviates a few tenths of a degree from zero across
the whole horizon, even though `w_epsi_sum` (the accumulated forcing)
reaches -1.2. The reason: `Ad`'s own `e_psi` decay (`Ad[2,2]≈0.946`/step)
bleeds off a small per-step forcing almost as fast as it's added, so it
never builds into a state deviation big enough to be worth paying real
steering-rate cost to counter. The resulting `delta_cmd` response is
sub-1° — noise-scale, exactly matching live telemetry's slightly-negative
mean steer (`-1.05°`) buried in `±4.4°` of pre-existing oscillation during
this same window. This is the mechanism behind "still doesn't turn early."

**Raising `curvature_forcing_gain` to compensate does not fix this — it
makes it worse in a new way.** Sweeping gain against the same synthetic
corner:

| gain | w_epsi_sum | steer_cmd |
|---|---|---|
| 1.0 | -1.19 | -0.27° |
| 3.0 | -3.56 | -0.96° |
| 6.0 | -7.13 | **-25.00°** (saturated AWAY from the corner) |
| 9.0–15.0 | -10.7 to -17.8 | **-25.00°** (still saturated away) |
| 20.0 | -23.8 | +25.00° (finally correct direction, but saturated, 20x the physical value) |

Inspecting the QP's full predicted trajectory at `gain=6.0` shows why:
`u[0,:]` commits immediately to `-25°` (steering AWAY from the corner) for
the first several steps, driving predicted `e_psi` to `-24°` and `e_y` to
`-2.3 m`, before reversing to `+25°` around step 6 onward. This is the
"drifts right before some left corners" symptom, reproduced exactly in a
clean, noise-free, single-mechanism test — not a live-noise artifact and
not an anti-hunt interaction.

**Root cause**: the forcing term is implemented as a disturbance on the
dynamics constraint itself (`x[:,1:] = A@x[:,:-1] + B@u + w`), which is
the same recursion the QP is minimizing total quadratic cost over. That
gives the solver freedom to choose *how* to spend/absorb the disturbance
across the whole horizon — it is not "tracking a path that bends," it is
finding the cheapest predicted trajectory subject to an artificial forcing
term, and nothing in that formulation prevents the cheapest trajectory
from being a transient swing away before correcting. This is a structural
property of forcing-via-dynamics-disturbance, not a magnitude/tuning
question — no gain between the two extremes tested is both large enough
to matter and free of the wrong-direction transient.

**Fix**: disabled `curvature_forcing_enabled` on both sides (`mpc_params.py`,
`settings.py`, `fsae_params.yaml`, `launch_all.sh`'s shortlist, `fsds_simulator`
mirror). This reverts to the pre-2026-08-12 baseline: no early anticipation,
but also no wrong-direction transient — the better-understood failure mode.
`anti_hunt_k_lookahead` was left at `15.0` (harmless no-op with forcing off,
and a real improvement over `60.0` if forcing is ever revisited). This is
the state recorded as Part 2, row 1 above.

**Code kept in place at the time, not deleted**: `curvature_horizon_profile`
(both sides), the `w` parameter threaded through `_build_qp`/`_solve_qp`
(live) and `init_parameterized_mpc`/`solve_mpc` (offline), and the
`kappa_max_abs`-gated anti-hunt factor. A future redesign should likely
make curvature shift the *reference*/error definition (e.g. curving the
reference heading `e_psi` is measured against, so the QP's cost directly
penalises deviation from a bending reference) rather than perturbing the
same recursion the QP optimizes trajectories over. Do not re-enable
`curvature_forcing_enabled` by flipping the flag alone without re-deriving
the mechanism — the gain sweep above shows the current formulation has no
safe operating point. (This is exactly the lesson the NMPC formulation in
Part 16 acts on: curvature as a function of a state the solver is actively
choosing, not external data it's free to defer absorbing. The mechanism
itself, along with everything else in this lookahead family, was later
deleted wholesale in the corner-factor-scheduler rewrite — see
`removed_mechanisms.md` Section 8.)

**Late turn-in on sudden corners was therefore still an open problem at
this point in the investigation** — this session's attempted fix was
reverted, not replaced. The reference-heading-lead mechanism (§12.8 in
`sim_to_real_investigation.md`, also flagged in CLAUDE.md's "Still open"
list) was investigated as the next avenue — **but it applies to the
live-planner branch specifically** (the planner's per-tick, FOV-limited
centreline rebuild), not to driving against a precomputed
`map_path`/`path_map_path` raceline, which is the actual live driving
configuration. On a precomputed path there is no per-tick rebuild to swing
unpredictably; the reference is fixed in advance. Re-measured anyway out
of thoroughness (see below) before this scope mismatch was caught — the
re-measurement itself is real and worth keeping on file, but is not the
fix for the precomputed-path late-turn-in symptom.

**Re-measurement note (2026-08-12, for the live-planner branch only, scope
above notwithstanding)**: `tuner/reference_heading_geometry_check.py` and
`tuner/reference_excess_mechanism_check.py` (§26/§27 of
`sim_to_real_investigation.md`) were deleted in the same-day `tuner/`
reorg (`c182a05`) as "concluded one-off investigation scripts," but the
investigation's own "Open — mechanism confirmed real, first candidate fix
tried live and FAILED" status (its last recorded state) was never actually
closed, and — separately — an 2026-08-08 parity bug fix (§31, `SimPlanner`
never passing live-tuned smoothing/blend params) invalidated every number
§26/§27 measured, with no re-measurement ever done. Restored both scripts
from git history (`c182a05^`) and re-ran against the corrected planner: the
mechanism now measures **larger**, not smaller — planner/geometric
reference-rate ratio 1.66/1.76/6.91/12.84 (mean/p90/p99/max), correlation
with true track geometry down to 0.336 (was 1.22/1.87/3.51 and 0.80
pre-fix), and 110 ticks (of 1045) show >30°/s excess swing, only 9%
explained by the known seed-jump artifact — unchanged from before, so the
remaining ~91% is still an unlocalized, distinct planner mechanism. This is
a real, larger-than-previously-known open item **for the live-planner
branch** — restored scripts are back in `tuner/` for whoever next drives
with `use_planner=True` instead of a precomputed path.

## Part 7 — Design: lead-heading reference (not yet implemented)

User's idea, checked directly rather than assumed: instead of biasing the
QP's *prediction* (curvature-forcing, found structurally unsound in Part
2) or the *cost target inside the horizon* (also tested this session, same
failure — see below), measure `e_psi` against the path's heading a bit
**ahead** of the car's actual nearest point, at k=0, before the QP ever
runs. This is structurally different from both prior attempts and — in
synthetic testing — does NOT reproduce their failure mode.

### Why curvature-forcing and cost-target-shift both failed (recap, now with a firm reason)

Tested (this session, offline, synthetic QP): shifting the horizon-internal
cost target (`cost = Q_epsi*(e_psi[k] - target[k])^2` where `target[k]`
tracks the path's expected heading advance) produces the SAME
wrong-direction transient as curvature-forcing (`w` dynamics disturbance) —
steering goes negative for ~7 steps before committing positive, on the
identical synthetic corner that broke curvature-forcing. Root cause is
general, not specific to the disturbance-vs-cost distinction: kappa[k] is
exogenous data (known at solve time, doesn't depend on x/u), so ANY way of
telling the QP "you'll owe this deviation LATER, within the horizon" gives
the solver freedom to choose *when* to pay it — an early wrong-direction
dip can integrate to lower total quadratic cost than committing immediately,
and the solver will take that trade. This applies identically whether the
future obligation enters via the dynamics constraint or the cost function.

### Why the lead-heading idea avoids this

It doesn't create a future obligation at all — it changes what's true
**right now**, at `k=0`. `e_psi(0)` becomes a real, already-existing
nonzero error (car's actual heading vs. the path's heading a bit further
down-track) the instant `_error_state` computes it, before the QP ever
sees it. There is no "spend this later" freedom because there is nothing
scheduled for later — it's the starting condition, exactly as if the car
had actually drifted off the (lead) heading.

**Verified via synthetic test** (`x0 = [0, 0, -lead_epsi, 0]`, unmodified
dynamics/cost, `lead_epsi` computed from a real curvature ramp — no `w`
term, no cost-target modification, ONLY x0 changed): steering responds
*directly and monotonically* toward the lead heading (e.g. `e_psi`:
`+4.0° → +3.37° → +2.37° → ... → 0.05° → -1.01° → ...`, no dip-then-
reverse) at every speed tested (8/14/20 m/s). This is the QP behaving
exactly as it should for a real, current error — not exploiting anything.

### Critical tuning finding — lead distance is narrow between "no effect" and "instant full-lock"

Tested against a realistic corner entry ramp (curvature rising linearly
over 15m to a R=13m corner, then holding):

| lead distance | steer[k=0] |
|---|---|
| 0 m (baseline, no lead) | 0.00° |
| 3 m | 7.3-7.7° (speed-dependent, smooth) |
| 8 m | **25.00° — full lock, immediately** |
| 15 m | 25.00° — full lock |

This held at 8, 14, and 20 m/s alike. **8m of lead is already enough to
saturate steering at k=0 on this corner shape** — there is a narrow band
between "lead too short to matter" and "lead long enough to slam to full
lock the instant a corner is detected," which is itself a plausible NEW
failure mode (an abrupt full-lock commit is not obviously better than a
late one — needs testing, not assumed better). This is the single biggest
open risk before implementing: the lead distance is doing double duty as
both "how early" and "how hard," and those may need to be decoupled (e.g.
by NOT feeding the full lead-point heading directly, but blending it with
the near-point heading — see Open design questions below).

### Design sketch (NOT implemented)

- **Where**: `_error_state`'s existing `path_yaw = math.atan2(seg[1], seg[0])`
  computation (mpc_core.py, ~line 1842) — evaluate the path tangent at
  `base_idx + lead_offset_idx` instead of `base_idx`, where `lead_offset_idx`
  is derived from a new speed-scaled lead distance (same
  `clip(car_speed * time_s, dist_min, dist_max)` pattern as
  `adaptive_q_lookahead_dist_*`, NOT reusing those exact constants —
  this needs its own, much SHORTER distance given the full-lock finding
  above; 8m already saturates vs. the corner-anticipation boosts' 3-25m
  range).
- **`e_y` is NOT affected** — stays projected onto the near point
  (`path[base_idx]`), unchanged. Only the heading reference moves ahead.
  This matches `ref_heading_rate_limit_enabled`'s existing precedent of
  "only e_psi is recomputed from a modified reference; e_y is left
  untouched" (mpc_core.py's own comment at that mechanism).
- **Interaction with `ref_heading_rate_limit_enabled`**: that mechanism
  caps how FAST the reference heading may change per tick — built for the
  OPPOSITE problem (the live planner's rebuilt path swings faster than the
  car can yaw, on the SLAM/live-planner branch). A lead-heading target
  would interact with it directly (both modify `path_yaw` before `e_psi`
  is computed) but currently `ref_heading_rate_limit_enabled` defaults
  `False` and targets a different path-source mode than the precomputed-
  path driving this investigation's logs — likely a non-issue for initial
  testing, but must be explicitly checked, not assumed compatible, if both
  are ever enabled together.
- **Interaction with `CornerMap`**: `dist_to_apex`/`corner_id` could gate
  the lead distance (e.g. no lead on a straight, lead grows approaching a
  known corner) rather than a flat always-on lookahead — needs its own
  design pass, not assumed.

### Open design questions (must be resolved before implementation)

1. **Blend vs. hard-switch**: feed the QP the FULL lead-point heading
   directly (as tested above — simple, but produces the instant-full-lock
   effect at only 8m), or blend near-point and lead-point heading by some
   weight that ramps in gradually? The latter decouples "how early" from
   "how hard" but is a new mechanism to design and validate on its own.
2. **Lead distance formula**: fixed constant, speed-scaled
   (`car_speed * time_s`, clipped), or `CornerMap`-gated (ramp in only
   approaching a known corner, zero on straights/mid-corner)? The
   full-lock finding suggests this needs to be considerably shorter than
   `adaptive_q_lookahead_dist_*`'s existing 3-25m range, or gated by
   demand/corner-severity rather than flat.
3. **Should this REPLACE or SUPPLEMENT the existing lookahead Q-boost
   mechanisms** (`_lookahead_approach_boost` etc.)? They solve a related
   but distinct problem (reweighting cost given an EXISTING error) — this
   creates the error itself early. Worth checking whether combining both
   double-counts or conflicts before assuming they compose cleanly.
4. **Validation plan** (once design questions above are resolved): (a)
   offline replay against the same real log/corner used throughout this
   investigation, comparing lead-heading on/off; (b) sweep lead distance
   against the full-lock threshold found above to find a value that helps
   turn-in timing WITHOUT slamming to full lock every corner; (c) only
   then a live A/B, same discipline as everything else in this file.

**Status: idea validated as structurally sound (avoids the QP-freedom trap
that killed curvature-forcing and the cost-target variant), NOT
implemented. Three open design questions above need resolving first —
particularly the full-lock risk, which was NOT anticipated at proposal
time and only surfaced from testing a realistic corner shape rather than a
single synthetic step.**

## Part 8 — Corrected design: precomputed SHAPED heading profile (supersedes Part 7's flat lead)

User corrected Part 7's framing: not a flat "look ahead N metres" lead
applied live, but a full **heading profile precomputed once per waypoint**,
alongside `x,y,psi,v_target` in the SAME offline export step that already
produces `raceline.csv` — shaped by how much yaw authority the car
actually has at each point's *planned* speed, not a fixed distance.

### The gap this fills

`raceline.csv` already has a `psi` column — but it's discarded today.
`load_path_profile_csv`'s own docstring says so explicitly: "`psi` is
exported but not returned here: `MPCController._error_state()` already
derives path heading from consecutive waypoints (atan2 of the segment
direction)... no interface change to `mpc_core.py`." So today's `psi` is
purely geometric (path tangent) and there is no path at all for a SHAPED
heading profile to reach the controller — this isn't a live-runtime
mechanism to build, it's an EXPORT-TIME computation plus one small
consumption change in `_error_state`.

### Why this avoids the full-lock cliff Part 7's flat lead hit

Part 7's flat lead (fixed lookahead distance, e.g. 8m) hit a cliff:
immediate full-lock steering, because the lead magnitude had no relationship
to what the car could actually achieve — it was just "how far ahead the
geometric path happens to have turned," which can be arbitrarily large
right at a sharp corner's entry.

**The fix: derive the lead from achievable yaw, not fixed distance.**
Walk the profile BACKWARD from each corner's true (geometric) heading,
at each waypoint capping how far the target can lead the true heading by
`max_yaw_rate(v_target) * authority_frac * dt_to_next_waypoint`, where
`dt_to_next_waypoint = ds / v_target` (arc length to the next waypoint
divided by THIS waypoint's planned speed — the speed-awareness the user
asked for explicitly: a corner taken slower gets a proportionally larger
lead per metre of arc length, since more TIME is available per metre at
lower speed). `max_yaw_rate(v)` reuses the same kinematic relationship
already in `_discrete_model` (`A_kin[2,6] = v/(lf+lr)`, i.e. yaw rate per
unit steering angle) at `delta_max` (25°) — no new vehicle-model constant
needed, this is exactly the model already trusted for prediction.

```python
def max_yaw_rate(v, lf=0.70, lr=0.85, delta_max=math.radians(25.0)):
    return v / (lf + lr) * delta_max

def build_shaped_heading_profile(s, geometric_psi, v_target, authority_frac=1.0):
    """
    s: (n,) cumulative arc length, geometric_psi: (n,) atan2 path tangent
    (today's psi column), v_target: (n,) planned speed (same array
    speed_profile.csv/raceline.csv already carries).
    Returns psi_target: (n,) shaped heading profile, same length/indexing.
    authority_frac in (0,1]: fraction of max achievable yaw rate to
    "pre-spend" as lead -- 1.0 = as aggressive as physically achievable,
    lower = gentler nudge. THE tuning knob for this mechanism.
    """
    n = len(s)
    psi_target = geometric_psi.copy()
    for i in range(n - 2, -1, -1):
        ds = s[i+1] - s[i]
        dt_seg = ds / max(v_target[i], 0.5)
        max_dpsi = max_yaw_rate(v_target[i]) * authority_frac * dt_seg
        diff = psi_target[i+1] - geometric_psi[i]
        step = np.clip(diff, -max_dpsi, max_dpsi)
        psi_target[i] = geometric_psi[i] + step
    return psi_target
```

**Verified in synthetic testing** (60° corner, 15m entry ramp, speed
decelerating 17->9 m/s into it — realistic entry, not a toy step):

| `authority_frac` | lead at corner entry | resulting steer[k=0] |
|---|---|---|
| 1.0 (max achievable) | 1.62° | **9.36°**, decaying smoothly |
| 0.5 | 0.81° | **4.68°**, decaying smoothly |
| 0.25 | 0.40° | **2.34°**, decaying smoothly |

No full-lock cliff at any tested value — this is the qualitative
difference from Part 7's flat-distance lead. The profile ALSO naturally
decays the lead to zero once the corner's constant-curvature section
begins (nothing left to "pre-achieve" once the car is expected to already
be mid-turn), which Part 7's fixed-distance version had no equivalent to.

### Where this plugs in

- **Export time** (`fsae_MPCTest/tuner/raceline_optimizer.py` or
  wherever `psi`/`v_target` are currently written to the CSV — NOT
  verified yet which file owns this, see Open questions): compute
  `psi_target` via `build_shaped_heading_profile` from the already-computed
  geometric `psi` and `v_target` columns, write it as an ADDITIONAL column
  (don't overwrite geometric `psi` — `_curvature()`/`_segment_corners()`
  elsewhere in `mpc_core.py` need the true geometric tangent, only the
  heading-ERROR reference should use the shaped version).
- **Load time** (`load_path_profile_csv`, `control_utils.py`): currently
  discards `psi` entirely (`_path_Psi` prefixed with `_`, per the grep
  above). Needs to start returning the new shaped-heading column instead
  (or alongside) — the function's return shape changes, so every call site
  (`mpc_controller.py`, `mpc_controller_standalone.py`) needs updating in
  the same change.
- **Use time** (`_error_state`, mpc_core.py ~line 1842): today,
  `path_yaw = math.atan2(seg[1], seg[0])` (geometric, recomputed every
  tick from consecutive waypoints). New: if a shaped-heading array is
  loaded, use `shaped_psi[base_idx]` (a lookup, like `CornerMap`'s
  lookups) for the value that FEEDS `e_psi`'s computation, while `e_y`
  keeps using the geometric tangent for its projection (unchanged, same
  precedent as `ref_heading_rate_limit_enabled`'s existing "only e_psi
  changes" pattern).
- **CornerMap interaction**: `_segment_corners`'s straight/corner
  thresholding should run on GEOMETRIC curvature (unchanged) — the shaped
  heading profile is a derived, secondary array, not a replacement for the
  geometry CornerMap already characterises exactly.

### Open questions (must resolve before implementing)

1. **Which file currently writes `psi` into `raceline.csv`?** Not yet
   confirmed — likely `fsae_MPCTest/tuner/raceline_optimizer.py`, needs
   verification before writing to it (`fsae_MPCTest` scope decision below
   applies here too).
2. **fsae_MPCTest scope, again**: like the corner-map feature, this
   spans an offline EXPORT step (naturally `fsae_MPCTest`-side, since
   that's where `raceline.csv`/`speed_profile.csv` are produced) and a
   live CONSUMPTION step (`mpc_core.py`, live-side). Unlike the corner-map
   feature, the export half of this idea has no live equivalent to be
   "live-only" about — the shaping computation only makes sense done once,
   offline, on a track already fully known. Recommend asking explicitly
   before implementing, not assuming, same as the corner-map decision
   point.
3. **`authority_frac` default/tuning**: no live evidence yet for which
   value is good — 1.0/0.5/0.25 were arbitrary sweep points, not a
   recommendation. Needs the same offline-then-live validation discipline
   as everything else (see Part 4e's validation-plan pattern).
4. **Interaction with existing lookahead Q-boost mechanisms**: same open
   question as Part 7's #3 — does a shaped-heading lead make
   `_lookahead_approach_boost`/`_lookahead_epsi_approach_boost` partially
   redundant (both are trying to help the same "commit early" goal, via
   different mechanisms), or do they compose cleanly? Not yet checked.
5. **Validation plan** (once open questions above are resolved, same
   escalating discipline as Part 4e/Part 7's #4): (a) generate the shaped
   profile for `comp_test_map_3` and sanity-check it by eye against the
   known corners (does the lead grow approaching the same hard corner this
   whole investigation has analysed, and shrink to ~0 on straights?); (b)
   offline replay through `_error_state` with the shaped profile swapped
   in, compare against today's baseline on the same log positions; (c)
   sweep `authority_frac` offline before ever going live; (d) live A/B,
   same discipline as everything else in this file.

**Status: idea validated as sound and DISTINCT from Part 7's flat-lead
version (avoids its full-lock cliff via speed/authority-aware shaping),
NOT implemented. Five open questions above, particularly the
`fsae_MPCTest` file-ownership and scope questions, need resolving first.**

## Part 9 — Refined design: heading profile as a DERIVED third pass (not fully coupled), extended with slip

Builds on Part 8, after reading the actual `raceline_optimizer.py` (not
assumed) and two follow-up user requests: (a) the heading profile must
account for acceleration/braking physics, ideally with path/speed/heading
planners "working together"; (b) all three should also minimise slip.

### What `raceline_optimizer.py` already does (verified by reading it, not assumed)

- **Path and speed are ALREADY jointly converged**: `optimize_raceline`'s
  main loop calls `_lap_time_and_speed` (the speed pass) INSIDE every
  path-curvature-reduction iteration (line 541), so the exported `v_target`
  is not independent of the exported path — each iteration's candidate path
  is scored by the lap time ITS OWN matching speed profile would produce.
- **`_lap_time_and_speed` already does forward/backward accel/brake
  propagation** (lines 640-651): a genuine two-pass (accel-limited forward,
  brake-limited backward) fixed-point speed solve against
  `params.max_accel`/`params.max_accel_brake` — this IS the "acceleration
  planner" the user asked about; it already exists, just not exposed
  outside this file's `v_target` output.
- **`ALAT_MARGIN=0.85`/`BRAKE_MARGIN=0.85` already reserve headroom**
  specifically so the reference doesn't ask the car for its absolute limit
  with nothing left for the MPC to correct with — i.e. this file's authors
  already solved a version of "don't plan a reference the car structurally
  cannot achieve" for lateral/longitudinal acceleration. Extending the same
  philosophy to heading-rate and slip is consistent with, not foreign to,
  this file's existing design.
- **`path_Psi` today is purely geometric** (`np.arctan2(dy, dx)` on the
  finished path, line 591) — no shaping, exactly like the live
  `_error_state`'s equivalent computation. Confirms Part 8's finding: the
  gap is real and this is the file that would need to change.
- This is heavily empirically tuned code (`_smooth_step`'s
  `PUSH_STEP_M`/headroom-taper, `CURVATURE_SOFT_MAX`, `ALAT_MARGIN` are
  each backed by a specific measured failure mode in their own comments,
  not guessed) — a reason for caution before restructuring its core loop.

### Coupling-depth decision: DERIVED third pass, not fully coupled into the optimizer loop

Asked directly: should heading (and slip) become a co-equal variable
`_candidate_score`/`_smooth_step` optimizes FOR (potentially reshaping the
path/speed themselves), or a pass computed AFTER path+speed already
converge, derived from their final output?

**Decision: derived third pass first.** Reasoning: `optimize_raceline`'s
existing constants are each tied to a specific measured failure mode (see
above) — coupling a new objective into `_candidate_score`/`_smooth_step`
invalidates that tuning against the new objective and risks regressing a
carefully-debugged optimizer. `ALAT_MARGIN` already reserves lateral
headroom for exactly the class of problem heading/slip achievability would
also flag — a derived pass computed AFTER convergence, from the path/speed
that already has that headroom built in, is likely to find few or no
violations it can't handle, which is itself the right way to find out
whether full coupling is actually necessary: if the derived pass can
produce a clean profile without hard clamping, coupling isn't needed yet;
if it can't, THAT is the evidence to justify the bigger, riskier change.
Measure before coupling, not the reverse — same discipline as everything
else in this investigation.

### Design: one new pass, two derived quantities (heading profile + slip check), sharing one bicycle-model evaluation

Both heading achievability and slip depend on the same underlying
quantities (yaw rate, speed, steering) via the SAME linear bicycle model
already used in two other places in this codebase (`mpc_core.py`'s
`_discrete_model`, `model/vehicle_physics.py`'s `VehicleParams`/`Cf`/`Cr`)
— one pass can compute both rather than two independent mechanisms.

```python
def shape_heading_and_check_slip(
    path_xy, kappa, v, params, authority_frac=1.0,
):
    """
    Runs ONCE, after optimize_raceline()'s path_xy/kappa/v have already
    converged (path+speed already jointly optimal per the existing loop).
    Does NOT feed back into path_xy/v -- see coupling-depth decision above.

    Returns (psi_target, slip_flags):
      psi_target: (n,) shaped heading profile (see Part 8's
        build_shaped_heading_profile -- same algorithm, now fed the REAL
        v_target this file already computed, not a hand-waved value, so
        it already reflects the existing accel/brake propagation).
      slip_flags: (n,) bool -- True where the profile's implied yaw rate
        would require a slip angle beyond a safe bound (see below) at
        that station's v -- diagnostic output for now (see status below),
        not yet fed back into re-shaping anything.
    """
    n = len(path_xy)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path_xy[:,0], append=path_xy[0,0]),
                                                     np.diff(path_xy[:,1], append=path_xy[0,1])))])
    geometric_psi = np.arctan2(np.gradient(path_xy[:,1]), np.gradient(path_xy[:,0]))

    psi_target = build_shaped_heading_profile(s, geometric_psi, v, authority_frac)

    # Slip check: at each station, the YAW RATE implied by kappa*v (the path's
    # own geometric turning rate) vs. the max yaw rate the linear bicycle
    # model can deliver at that v/that slip angle before Cf/Cr's linear
    # region is exceeded (same Cf/Cr VehicleParams already uses for
    # alat_ceiling_at's own tyre-force reasoning -- not a new tyre model).
    r_implied = kappa * v
    # Linear bicycle model's steady-state slip angle at the REAR axle (a
    # standard understeer-gradient quantity, not a new derivation):
    #   beta_r = lr * r / v  (small-angle, steady-state cornering)
    # Flag where this exceeds a conservative bound (e.g. 4-5 deg is
    # typically the edge of a tyre's linear region for a small FS car --
    # NOT YET VALIDATED, placeholder pending a real measurement, same
    # posture as every other new constant in this file).
    SLIP_LIMIT_RAD = math.radians(5.0)  # PLACEHOLDER, unvalidated
    beta_r = params.lr * r_implied / np.maximum(v, 0.5)
    slip_flags = np.abs(beta_r) > SLIP_LIMIT_RAD

    return psi_target, slip_flags
```

### Where this plugs into `raceline_optimizer.py` (file/line references from the actual read above)

- Call `shape_heading_and_check_slip` in `export()` (line ~710-712, right
  after `optimize_raceline` returns `path_X, path_Y, path_Psi, path_v`) —
  REPLACE the geometric `path_Psi` in the CSV write with the shaped
  `psi_target`, or write BOTH as separate columns (recommend both: keep
  geometric `psi` for anything downstream that still wants the true
  tangent — e.g. `CornerMap`'s curvature segmentation should stay on
  geometric curvature per Part 8 — and add a new `psi_target` column for
  the live controller's heading-error reference).
- `slip_flags`: print a summary at export time (count/locations of
  stations exceeding the placeholder limit), same pattern as
  `_assert_clearance`'s existing clearance report — diagnostic first, not
  a hard failure, since `SLIP_LIMIT_RAD` is unvalidated.
- CSV format changes from `x,y,psi,v_target` to `x,y,psi,psi_target,v_target`
  (or similar) — this is a BREAKING change to `load_path_profile_csv`'s
  expected format and every call site of it (`mpc_controller.py`,
  `mpc_controller_standalone.py`) — must update the loader and both nodes
  in the same change, and re-export every existing track's `raceline.csv`
  (just `comp_test_map_3` today) or the loader needs backward-compat
  handling for old 4-column files.

### Open questions (still unresolved, growing list from Part 8)

1. `SLIP_LIMIT_RAD` is a placeholder guess (5°) — needs a real value, same
   validated-not-guessed discipline as every other constant in this
   codebase. Possibly derivable from `Cf`/`Cr`'s own linear-region bound
   rather than a flat angle — not yet investigated.
2. CSV format migration (backward compat vs. re-export requirement) not
   yet decided.
3. Part 8's open questions #2-5 (fsae_MPCTest scope confirmation -- now
   answered implicitly by reading the file, but still not IMPLEMENTED --
   authority_frac tuning, interaction with existing lookahead Q-boost
   mechanisms, full validation plan) still apply unchanged.
4. Should `slip_flags` ever become a HARD failure (like
   `_assert_clearance`'s `RuntimeError`) once `SLIP_LIMIT_RAD` is
   validated, or stay diagnostic-only? Not yet decided — depends on
   whether derived-pass violations turn out to be common or rare on real
   tracks (the measurement that would also decide the coupling-depth
   question above).

**Status: design refined with real file references (not speculative),
slip folded in via the same bicycle-model quantities already used
elsewhere in the codebase. STILL NOT IMPLEMENTED. SLIP_LIMIT_RAD in
particular must not be treated as a real constant until validated.**

## Part 10 — Implementation complete (2026-08-12), with an honest caveat on comp_test_map_3

Implemented per Part 9's design, live-only for `mpc_core.py`/nodes/launch
(consistent with the corner-map feature's scope), offline-only for the
shaping/slip computation itself (`fsae_MPCTest/tuner/tools/raceline_optimizer.py`
— there is no live equivalent to build, per Part 8's open question #2,
now resolved by there being nothing to resolve: the shaping only ever
makes sense done once, offline).

**Done:**
1. `build_shaped_heading_profile`/`check_slip`/`max_yaw_rate` written in
   `raceline_optimizer.py`, run as a derived third pass in `export()` AFTER
   `optimize_raceline()`'s path+speed already converge (confirmed via
   direct comparison: x/y/psi/v_target are BYTE-IDENTICAL before/after
   this change — the new pass only adds the `psi_target` column, doesn't
   touch anything it's derived from).
2. CSV format extended to 5 columns (`x,y,psi,psi_target,v_target`),
   backward-compatible: `_load_profile_csv` (control_utils.py) detects
   column count per-row and returns `psi_target == psi` (a no-op) for any
   4-column file (old raceline.csv exports, or speed_profile.csv, which
   never needs this column).
3. New loader `load_path_heading_profile_csv` (separate from
   `load_path_profile_csv`, whose `(n,2)` return shape is unchanged, so
   `CornerMap`/everything else touching `_static_path` is unaffected).
4. `MPCController.set_heading_profile()` + one gated call site in
   `_error_state`: substitutes `path_yaw` with the loaded profile's
   value at `base_idx`, ONLY for `e_psi`'s reference — `e_y`'s projection
   still uses the geometric `path_yaw`, unchanged (mirrors
   `ref_heading_rate_limit_enabled`'s existing "only e_psi changes"
   precedent). Regression-checked: flag unset reproduces `compute()`'s
   output bit-for-bit (200-tick synthetic replay); flag set measurably
   changes it (max abs diff 0.266).
5. `use_precomputed_heading_profile` node parameter wired in both
   controller nodes + both launch files + `launch_all.sh`
   (`USE_PRECOMPUTED_HEADING_PROFILE=false`, default OFF — untested).
6. `comp_test_map_3/raceline.csv` re-exported with the new column
   (verified x/y/psi/v_target unchanged from the pre-existing file before
   overwriting it, per user's explicit go-ahead).

**Honest finding from sanity-checking the profile shape (Part 9's
validation step 1) — flagging before live testing, not glossing over it:**
`comp_test_map_3` does not actually have long straights. Checked directly:
`psi` (the geometric heading) climbs from 0.4° to 16° over just the first
40m of what looks like a "straight" in the corner-segmentation sense (this
is part of the gentle S-curve documented in Part 3e) — the whole lap is a
sequence of gentle bends and sharper corners with very little truly
constant-heading track. Consequence: at `authority_frac=0.5`, the
achievable-yaw-rate backward walk propagates a NON-TRIVIAL heading lead
(~3.5-3.75°) across nearly the entire lap — only 2 of 1000 exported points
ever reach near-zero lead. This is NOT a bug in
`build_shaped_heading_profile` (Part 8's isolated single-corner synthetic
test correctly showed decay-to-zero on an ACTUAL straight-then-corner
shape) -- it's a structural property of this specific track's geometry,
which the algorithm is correctly responding to. But it means the live
test on `comp_test_map_3` will exercise a heading lead that's active
almost everywhere, not just "approaching corners" as originally pictured
-- closer to a constant offset than a corner-triggered anticipation
signal on this particular track. Worth knowing before reading the first
live result: if it's confusing or produces odd wide-open-road behaviour,
this is why, and lowering `HEADING_LEAD_AUTHORITY_FRAC` (currently 0.5,
in raceline_optimizer.py) is the first lever to try, not assuming the
mechanism itself is broken.

**Status: implemented, offline-validated (segmentation/slip logic
correct, no-op-when-off confirmed, profile shape sanity-checked and
found track-geometry-dependent as described above), NOT YET LIVE-TESTED.**
Both `USE_PRECOMPUTED_HEADING_PROFILE` and every constant in
`raceline_optimizer.py`'s new code (`HEADING_LEAD_AUTHORITY_FRAC=0.5`,
`SLIP_LIMIT_RAD=5deg` placeholder) are unvalidated against a live run —
first test should compare against baseline on the SAME corner this whole
investigation has characterised, same discipline as everything else here.

## Part 11 — NEW finding from latest live log: severe speed-tracking lag causes a near-spin (distinct from turn-in timing)

Live run `mpc_standalone_control_1786532395.csv` (2026-08-12, 62.6s
completed lap, `USE_PRECOMPUTED_CORNER_MAP=true`,
`adaptive_q_lookahead_epsi_hold_thresh_rad=0.35`,
`adaptive_q_scaling_enabled=false` — i.e. the corner-map/exit-boost fix
plus step 1 of Part 3f's disable-and-compare test, `USE_PRECOMPUTED_
HEADING_PROFILE` still false, NOT part of this run).

**The corner this whole investigation has focused on (t=2.9-6.0s, first
corner) shows real improvement**: peak `|e_psi|` 19.4° (down from ~29°
in the original 3f-pre log), steering saturation only 7/61 ticks in that
window, `m_Q_ey_soften` pinned at exactly `1.000` throughout (confirms
`adaptive_q_scaling_enabled=false` is correctly inert, no more fighting
the approach boost).

**But a DIFFERENT, more serious failure appears later (t=10.6-14.0s),
unrelated to turn-in timing**: `e_y` reaches **-4.21m** (off track),
`e_psi` swings past **54.8°**, `corner_demand` peaks at **3.9x** (the
reference is briefly asking for ~4x more lateral grip than the FSDS
ceiling permits at the car's actual speed there). Root cause, traced via
`v_actual`/`v_desired`/`a_cmd`: **`v_desired` starts falling in good time**
(18.0 -> 17.3 -> 16.6 -> ... -> 13.7 m/s over `t=9.5-11.3`) **but
`v_actual` barely moves** (pinned at 16.4-16.6 m/s) and **`a_cmd` stays
POSITIVE (still commanding acceleration, +1.3 to +0.5) for the entire
window** — the car does not start actually braking (`a_cmd` negative)
until `t=11.29`, by which point `v_desired` has already fallen to 13.68.
This ~1.8s of near-zero braking response while the speed target drops
6.3 m/s is what turns an otherwise-planned deceleration into an
unrecoverable entry speed, triggering the near-spin that follows.

**This is a speed-tracking/longitudinal-authority problem, not a
heading/turn-in problem** — a structurally different bug from everything
else in this file. `q_e_v` (speed-error weight) is currently `4.0`,
`r_a_brake` is already cheaper than `r_a_accel` (0.4 vs 1.0), so the
sluggishness isn't trivially explained by those two weights being
miscalibrated in the "obvious" direction — needs its own dedicated
investigation (delay compensation's `predict_ahead` interaction? some
other mechanism suppressing braking authority right as this corner's
demand ramps? not yet checked). **NOT YET INVESTIGATED FURTHER — flagging
for a separate line of work, not folding into the turn-in investigation.**

Reproduce: this log's `t=8.5-14.5` window, watch `v_actual` vs `v_desired`
divergence and `a_cmd`'s sign.

## Part 12 — First live test of use_precomputed_heading_profile: WORSE, not better (2026-08-12)

Tested live at `HEADING_LEAD_AUTHORITY_FRAC=0.5` (the shipped default) on
`comp_test_map_3`, `use_precomputed_corner_map=true`,
`adaptive_q_lookahead_epsi_hold_thresh_rad=0.35`,
`adaptive_q_scaling_enabled=false` unchanged from the prior test batch, plus
the same-day base-weight retune (Part 13's sync). Two completed laps
compared against three same-day baseline laps (heading profile OFF, all
other flags identical).

**Result: consistently worse, not better, on both the specific corner this
investigation has tracked throughout AND whole-run averages.**

| | corner window (t=2.5-6.5s) | whole run |
|---|---|---|
| baseline (3 runs, profile OFF) | peak `\|e_psi\|` 19.4° | mean `\|e_psi\|` 6.1-8.2°, mean `\|e_y\|` 0.27-0.45 m |
| profile ON (2 runs) | peak `\|e_psi\|` 27.0° / 31.4° | mean `\|e_psi\|` 9.5° / 10.5°, mean `\|e_y\|` 0.39 / 0.52 m |

Steering saturation on the same corner window roughly doubled-to-tripled
(10/80 baseline vs 19/80 and 26/76 with the profile on). Confirmed the
mechanism was genuinely active (not a no-op/wiring bug): every run with
the flag on starts at `e_psi=-4.12°` at `t=0` while `car_yaw=0`, exactly
the lead Part 10 predicted from `HEADING_LEAD_AUTHORITY_FRAC=0.5` on this
track's near-total lack of straights.

**Likely explanation, consistent with Part 10's own caveat (not yet
separately confirmed)**: because this track has almost no genuine
straights, the shaped lead is active essentially everywhere (per Part 10:
998/1000 exported waypoints carry a non-trivial lead), not selectively
approaching corners. That means through much of a REAL corner's interior
— where the geometric heading is already changing fast and correctly —
the car is ALSO carrying an extra few degrees of lead on top, which can
push the reference ahead of where the actual apex is, rather than helping
the car commit earlier to a bend it hasn't reached yet. This would show up
exactly as observed: higher heading error and lateral error, not lower,
because the "help" is present continuously rather than gated to the
approach phase specifically.

**Action: reverted to OFF.** `USE_PRECOMPUTED_HEADING_PROFILE=false` in
`launch_all.sh`. Do not re-enable at `authority_frac=0.5` without changes.

**Candidates for next attempt, not yet tried:**
1. Lower `HEADING_LEAD_AUTHORITY_FRAC` substantially (e.g. 0.1-0.2) so the
   lead is small enough not to fight the corner interior, per the
   mechanism's own design intent — first thing to try, per Part 10's own
   note.
2. Gate the lead to ONLY the approach phase (e.g. zero inside
   `CornerMap.dist_into_corner > 0`, i.e. only active on straights/before
   `dist_to_apex` reaches some threshold) rather than letting the backward
   walk propagate it into the corner's own interior — this was flagged as
   an open design question in Part 9 (#3, whether this should
   REPLACE/SUPPLEMENT the lookahead Q-boost mechanisms) but not resolved;
   this result is evidence toward needing that gating, not just a
   `authority_frac` tuning issue.
3. Re-derive on a track with actual straights before concluding the
   MECHANISM (not just this track/this constant) is unsound — `comp_test_
   map_3` may simply be a bad test case for this specific idea, per Part
   10's caveat.

Do not re-attempt live without addressing (2) at minimum — the "active
almost everywhere, not gated to approach" behaviour is a plausible root
cause, not a guess, and testing the same unGated mechanism again at a
different `authority_frac` alone may just move where on the
too-strong/too-weak spectrum it fails.

## Part 13 — Correction to Part 12: more runs show high variance, not a clean regression

Two more completed laps arrived after Part 12 was written (same config,
`use_precomputed_heading_profile=true`, `authority_frac=0.5`):
`composite_score` 0.79 and 0.93 (vs. Part 12's two runs at 1.14/1.24) —
both BETTER than every baseline run recorded so far. On the tracked
corner specifically:

| run | peak `\|e_psi\|` | sat ticks (of ~80) |
|---|---|---|
| baseline 1/2/3 (profile off) | 19.4° / 21.9° / 22.7° | 10 / 16 / 22 |
| profile-on run 1 (Part 12) | 27.0° | 19 |
| profile-on run 2 (Part 12) | 31.4° | 26 |
| profile-on run 3 | **11.1°** | **0** |
| profile-on run 4 | 29.5° | 27 |

**Revised read: this is NOT a clean, consistent regression — it's high
run-to-run variance, both with the profile on and (per the 19.4-22.7°
spread across three "identical" baseline runs) without it.** Four
profile-on runs span from the single best result recorded all session
(run 3: zero saturation, lowest peak error of any run logged) to some of
the worst (runs 2/4). Sample size (4 on, 3 off) is too small to conclude
either "helps" or "hurts" from this data alone — Part 12's "consistently
worse" framing was written after only 2 profile-on runs and should be
treated as premature, not as an established negative result.

**Left ON** (`use_precomputed_heading_profile=true`) per direct user
instruction, based on their own read of how it was driving live (not
solely this log analysis) — continue collecting runs before drawing a
firm conclusion either way. If a clear pattern emerges with more data
(e.g. profile-on's WORST cases being worse than baseline's worst cases,
even if its best cases are also better), that would point toward the
Part 12 gating hypothesis (lead active in the corner interior, not just
approach) explaining the high-variance failure mode specifically, while
still being a net-neutral-or-positive change on average. Re-assess once
more runs accumulate rather than acting on either Part 12's or this
section's read alone.

## Part 13b — Lookahead corner-anticipation window widened, approach side

`adaptive_q_lookahead_dist_max` was raised `17.0` → `25.0` (both sides,
`fsae_params.yaml`, `fsds_simulator` mirror) to match
`adaptive_q_lookahead_exit_decay_dist_max`, which was already `25.0` — the
approach-side ceiling was tighter than the exit-side one for no documented
reason (each field's own git history showed it was only ever set once, at
`MPCParams` centralization, ported over from whatever value predated that
with no dedicated tuning record).

At typical corner-approach speed (16-17 m/s, from
`mpc_standalone_control_1786513486.csv`) the intended lookahead
(`car_speed * adaptive_q_lookahead_time_s` = 18.3-19.4 m) was being silently
clamped to 17.0 m — under 1 s of lead time regardless of actual speed.

**This was not expected to fix late turn-in on its own.** `adaptive_
q_lookahead` (§4.4 in `tuning.md`) only reweights `Q[0,0]`/`Q[2,2]` on an
*existing* tracking error — with `e_y ≈ e_psi ≈ 0` on approach, a wider
window lets the boost detect the corner's curvature a little sooner, but
multiplying a still-near-zero error by a bigger number is still near-zero.
This is the same "reweighting cannot manufacture an error" ceiling documented
at length in the curvature-forcing postmortem (Part 6b above) and in
`junior_project_mpc_docs.md`. Expected effect, if any: the boosts (and
downstream steering commitment) trigger marginally sooner and larger once
real error/curvature *does* appear inside the window, not before.

`sim/rollout_core.py`'s hardcoded `(1.13, 3.0, 17.0)` literal was updated to
`(1.13, 3.0, 25.0)` to match — kept as a plain literal, not sourced from a
new `settings.py` constant, since `fsae_MPCTest` and `fsae_planning` must
never share an import dependency in either direction (`fsae_planning`'s
standing "no settings.py-on-the-car" constraint applies symmetrically here).
Not live-tested at the time of this change.

A genuine fix for before-any-error anticipation on a precomputed path would
need to change what the reference/error is measured against — e.g. compute
`e_psi` against a lookahead point on the path rather than the nearest point —
not reweight costs on the current-position error. This was left unexplored;
no design or implementation existed yet at this point. (Part 14, immediately
below, found that even this widened 25m ceiling is still short of some
real corner-approach gaps.)

## Part 14 — Root cause of the mid-track corner, precisely confirmed: gap LENGTH exceeds max lookahead distance, not a signal-quality problem

Traced directly against `comp_test_map_3`'s real `CornerMap` (not assumed):
the corner group at `t=10.6-25.7s` (peak kappa 0.211, corner_id=2) follows
corner_id=1 (the already-tracked first corner) with a genuine **42m gap**
between them (`s=59` to `s=101`) — CONFIRMED by direct query, not
estimated.

**`_corner_map_lookahead`'s walk-forward search finds NOTHING (kmax=0.0000)
from `s=59` to `s=84`, then detects the corner starting exactly at
`s=85`** — the precise point where `lookahead_dist` (speed-scaled,
`clip(v*1.13, 3, 25)`) finally reaches or exceeds the REMAINING distance
to the corner (16m at that point, matching a 16.0m lookahead_dist at
v=14.2 m/s). This is not a bug in the scan/lookup logic -- confirmed by
running it directly offline against the real map and real speeds -- it is
the search correctly reporting "nothing within this bounded distance,"
because there genuinely is nothing within that distance for the first 26m
of the 42m gap.

**Why the growth-rate feature (Part 8's adaptive_q_growth_rate_weight)
could not help here, live-tested and confirmed**: growth-rate can only
distinguish "small but nonzero" from "unambiguously large" WITHIN a
window that already contains some part of the rising corner. When the
scan finds a hard `kappa_max_abs=0.0000` (nothing at all within
lookahead_dist), there is no growth-rate signal to blend in either — the
gap here is a genuine SEARCH-RANGE limitation, not a WEAK-SIGNAL
limitation like the earlier corner Part 8/9 was built against (where
kappa_max_abs was small-but-real the whole approach, just under-weighted).
Confirmed on the latest live log: `kappa_max_abs` reads exactly `0.0000`
(not 0.001, not 0.005 -- exactly zero) for the entire 1.8s/28m the car
covers before detection, consistent with the offline trace above.

**The car's own maximum lookahead distance never gets close to spanning
this gap.** `adaptive_q_lookahead_dist_max=25.0` is the hard ceiling
regardless of speed; even at this track's top speed (16.7 m/s),
`lookahead_dist = clip(16.7*1.13, 3, 25) = 18.9m` -- 2.2x SHORTER than the
42m gap. The car can only ever detect this corner once already within
~19m of its start, no matter how the car is driving beforehand.

**This reframes the fix target entirely.** Every mechanism investigated
so far (approach boosts, growth-rate blending, heading-lead profile) only
ever act on a signal that already exists within the scan window -- none of
them can help during a window where the scan finds LITERALLY NOTHING,
because there is nothing to reweight, blend, or lead from. The actual gap
is `adaptive_q_lookahead_dist_max`'s 25m ceiling being shorter than a real
42m straight on this track.

**Also confirmed: the CSV telemetry columns for the growth-rate feature
were missing from the log** (`kappa_growth_rate`/`kappa_max_abs_effective`
were computed into `adapt` but never added to
`telemetry_logger.ADAPTIVE_COLUMNS`, so they were silently dropped from
every CSV write). Fixed same-session -- see the `mpc_core.py`/
`telemetry_logger.py` diff; both columns are now always written
(unconditionally, not just when the weight is nonzero) so future logs can
directly verify this mechanism instead of having to infer it from
`kappa_max_abs`/`corner_demand` alone.

### Candidate fixes (not yet implemented, ranked by directness)

1. **Raise `adaptive_q_lookahead_dist_max` above 25m specifically to close
   this gap** -- most direct, but changes the ceiling for EVERY corner on
   the track, not just this one; a much longer lookahead window at low
   speed could also start reaching INTO an unrelated, distant corner and
   scoring against the wrong one. Needs checking against the whole track's
   other corner-to-corner gaps before raising blindly.
2. **CornerMap already knows the corner is there and how far away, even
   past the live scan's own bound** -- `dist_to_apex`/`corner_id` for the
   NEXT corner are precomputed and available regardless of `lookahead_dist`
   (the `_corner_map_lookahead` walk is bounded only because it's
   deliberately mirroring the live scan's own behaviour, Part 4c's design
   intent, NOT because CornerMap itself lacks the data). A precomputed-path
   -only fix: let `_corner_map_lookahead` report the NEXT corner's true
   peak/distance unconditionally (no lookahead_dist bound at all, since a
   static path's full geometry is already known), and gate any RESULTING
   boost by DEMAND/distance instead of by whether the scan happened to
   reach it. This targets the actual asymmetry: the live-scan bound made
   sense when curvature had to be re-derived every tick (expensive to scan
   arbitrarily far); it no longer needs to, now that CornerMap exists.
3. **Do nothing new; treat this as an inherent, currently-uncloseable gap**
   on THIS track's specific geometry (42m straight) and accept it -- not
   recommended without first trying #2, which is a small, targeted change
   that doesn't touch every other corner's behaviour.

**Status: root cause precisely identified and confirmed (not the same
category as the earlier gradual-corner problem this session already
addressed) -- candidate #2 is the most promising next step, not yet
implemented.**

## Part 15 — Checked: extending the heading-lead profile to a full per-horizon-step reference is UNSAFE (reproduces curvature-forcing's trap)

User asked whether `use_precomputed_heading_profile`'s k=0-only lead could
be extended to a full per-horizon-step reference (using
`_curvature_horizon_profile`'s existing walking-index mechanism to sample
the known path's heading at each of the QP's N predicted steps, then
target the COST against that per-step array instead of a single value).

**Checked directly, not assumed: this is unsafe.** Synthetic test (car
starts genuinely on-line, `e_psi(0)=0`, no real error yet, per-step target
active): the QP produces `+1.5deg` at k=0, then swings NEGATIVE and keeps
growing more negative through k=5 -- a real sign flip, the exact
wrong-direction-transient signature that killed `curvature_forcing_enabled`
(Part 2) and the cost-target-shift variant (Part 7). Root cause is the
same in all three cases, confirmed to generalize rather than assumed:
ANY mechanism that tells the QP about a future deviation from a reference
gives the solver freedom to choose WHEN to pay that cost across the whole
horizon, regardless of whether the future obligation is phrased as a
dynamics disturbance (`curvature_forcing`) or a cost target (this idea,
and Part 7's variant) -- a wrong-direction dip now, correction later, can
integrate to lower total quadratic cost than committing directly.

**Why `use_precomputed_heading_profile`'s actual (k=0-only) design stays
safe**: it changes what's true AT THE CURRENT TICK, not a future one --
`e_psi(0)` becomes a real, already-existing error the moment `_error_state`
computes it, with nothing scheduled for later. There is no obligation for
the solver to defer. This is a narrow, specific safety margin (current
tick only) that a per-step extension directly erases.

**Conclusion: do not extend `use_precomputed_heading_profile` (or any
similar mechanism) to a per-horizon-step reference.** The k=0-only design
is not an arbitrary scope limitation to be lifted later -- it is the
specific property that makes the mechanism safe. Any future idea in this
family must preserve "only the CURRENT tick's error changes" or it risks
reproducing this exact failure mode. Also directly answers a related
question asked this session (matching the QP's own PREDICTED trajectory to
nearest-point-on-path at each step): that would require abandoning the
linear QP for nonlinear MPC (see the earlier "full nonlinear trajectory
tracking" discussion) -- a much larger, riskier change than this
cost-target extension, which itself already failed at a much smaller
scope. Not pursued further.

## Part 15b — adaptive_q_extended_lookahead_dist_max live-tested WORSE, reverted (2026-08-13)

Live-tested at the shipped defaults (`dist_max=60.0`, `k=0.1`), per direct
user instruction: "performed horribly." **Reverted same day**
(`adaptive_q_extended_lookahead_dist_max` 60.0 -> 0.0, both `mpc_params.py`
and `fsae_params.yaml` — 0.0 is a confirmed exact no-op, see the
regression check in this file's earlier implementation notes).

No detailed log analysis done yet for THIS specific regression (user
reported the qualitative result directly; not yet traced tick-by-tick the
way earlier regressions in this file were). Root cause not established —
flagging explicitly so a future session doesn't assume the mechanism's
design is understood to be flawed, only that these specific constants
performed badly live. Candidates worth checking before any retry:
- `k=0.1`'s fade may still be too gentle/aggressive in a direction not
  caught by the offline sanity check (Part 14 checked the fade's SHAPE
  against one specific 42m gap in isolation, not its interaction with
  every OTHER corner-to-corner gap on the track, some of which may be
  much shorter and could now be getting a spuriously-early, faded boost
  from the WRONG corner if two corners are close together).
- The extended search has no upper bound on which corner it reports --
  it returns the FIRST corner found in the walk, but on a track with
  short straights this may sometimes be reporting a corner's fading
  signal while the car is still resolving a DIFFERENT nearer concern,
  interacting badly with other simultaneously-active mechanisms
  (`adaptive_q_growth_rate_weight=1.0` also blends into the same
  `kappa_max_abs` this feeds, compounding rather than isolated in the
  live test).
- Should be isolated (test `adaptive_q_growth_rate_weight=0.0` alongside
  this, one variable at a time) before concluding which mechanism, or
  their interaction, caused the regression.

**Status: DISABLED (`adaptive_q_extended_lookahead_dist_max=0.0`). Do not
re-enable at dist_max=60.0/k=0.1 without new evidence per the field's own
code comment. Root cause of the regression is an open question, not yet
investigated.**

## Part 16 — NMPC: research, formulation survey and recommendation (2026-08-13)

Scope of this Part: replace the "the QP cannot see the road bend" structural
limit (Parts 2/7/15) with a genuinely nonlinear MPC, as a NEW, separately
selectable controller — the existing `MPCController` LTV-QP stays the default
and is not modified. §16.1-16.3 are the research/decision record written
BEFORE any code; §16.4 onward record the implementation and measurements.

### 16.1 What the structural gap actually is, restated in model terms

Today's prediction model (`_discrete_model`) is
`x[k+1] = Ad x[k] + Bd u[k]` on `x = [e_y, e_yd, e_psi, r, e_v, e_a, delta_act,
a_act]`. Ad/Bd are built from the bicycle model in *error* coordinates with the
path's own rotation dropped. The exact missing term is one line of the Frenet
kinematics:

```
e_psi_dot = r - kappa(s) * s_dot          <-- the "- kappa*s_dot" is absent
```

Because it is absent, `e_psi` can only change if the CAR yaws; the reference
frame is treated as if it never rotates. With `e_y = e_psi = 0` the whole
35-step rollout predicts `0` forever, so no cost weighting on any of those
states can produce turn-in before real error exists. That is the same gap
Parts 1/3e diagnosed behaviourally.

Parts 2/7/15 each tried to bolt that term on as **exogenous data indexed by
horizon step** — as a dynamics disturbance `w[k]` (Part 2), as a shifting cost
target (Part 7), as a per-step reference array (Part 15). All three reproduced
a wrong-direction transient, and Part 15 established the reason generally: a
future obligation known at solve time lets the solver choose *when* to pay,
and paying it with an early wrong-way dip can integrate to lower total
quadratic cost.

The distinction that matters for NMPC: in a Frenet-frame nonlinear model,
`kappa` is **not** indexed by horizon step. It is `kappa(s)` where `s` is
itself a *state* driven by the car's own predicted motion. The "obligation" is
therefore not schedulable — steering the wrong way early makes `e_psi` and then
`e_y` grow immediately, from step 1, and every later step inherits the worse
state through the dynamics. Whether that closes the failure mode in practice is
an empirical question and is tested explicitly in §16.6, NOT assumed (nonlinear
MPC is not automatically safe here — the exogenous-vs-state distinction is an
argument, not a proof).

### 16.2 Candidate formulations surveyed

Four families are used for this problem in the literature; the two credible
here are compared in full, the other two recorded with why they were dropped.

**A. Frenet-frame tracking NMPC (curvilinear coordinates, direct multiple/single
shooting, Gauss-Newton SQP).** States `[s, e_y, e_psi, v_x, v_y, r, delta_act,
a_act]`, dynamics as in §16.3 below, `kappa(s)`/`v_ref(s)` looked up from the
already-loaded path. Cost = quadratic tracking of `(e_y, e_ydot, e_psi,
e_psi_dot, e_v)` + input effort + input rate. This is the standard
"trajectory-tracking NMPC in a space-dependent frame" formulation (Frenet
representations for automotive NMPC, arXiv:2212.13115; the racing tracking-MPC
variant in the F1TENTH survey, arXiv:2402.18558).
*Pros*: the missing `- kappa*s_dot` term is the whole point of the frame, so
the gap closes by construction; the cost, the weights, the actuator-lag states
and the error definitions all map 1:1 onto what `MPCController` already uses,
so `MPCParams`' tuned weights transfer with one documented exception (§16.3);
the reference stays the raceline the team already tunes offline.
*Cons*: `1 - kappa*e_y` singular at `e_y = 1/kappa` (irrelevant here: 3.5 m
track half-width vs the tightest corner's `1/0.21 = 4.8 m`, and it is guarded);
needs a curvature array that is not spike-ridden (a known open planner defect,
CLAUDE.md) — handled in §16.3.

**B. Model Predictive Contouring Control (MPCC).** Progress-maximising: the
cost trades contouring/lag error against `+ progress`, with track boundaries as
inequality constraints (Liniger's MPCC, github.com/alexliniger/MPCC; MPCC++ for
the same idea in flight, ResearchGate 383893254).
*Pros*: no speed reference needed at all — the controller picks the speed; is
the state of the art for *time-optimal* racing rather than *tracking*.
*Cons*: it deliberately abandons the reference the rest of this stack is built
on. This car's speed profile is optimised offline by `raceline_optimizer.py`
(forward/backward accel-brake passes, `ALAT_MARGIN`, measured `alat_ceiling`),
its score is a *tracking* score (`scoring.py`), and Part 11 shows the current
open longitudinal problem is failing to TRACK a good reference, not having a
bad one. Swapping to progress-maximisation would discard all of that offline
work and make every existing log incomparable. It also has no natural home for
`q_e_v`, so no tuned weight transfers. Deferred, not rejected on merit: it is
the right thing to try *after* tracking works, and the §16.3 model is directly
reusable for it (same dynamics, different cost).

**C. Cartesian-frame NMPC with online nearest-point projection.** States
`[X, Y, psi, ...]`, tracking error computed inside the solver by projecting
onto the path each iteration. Dropped: the projection is non-smooth
(argmin over waypoints), which is exactly what Frenet coordinates exist to
avoid, and it buys nothing here — the path is known and already arc-length
parameterisable.

**D. Keep the QP, add an outer SQP loop that re-linearises the EXISTING
error-coordinate model.** Dropped as unsound for this specific goal: no amount
of re-linearising a model that structurally lacks the `- kappa*s_dot` term adds
that term. Re-linearisation only improves accuracy *of the wrong model*.

**Recommendation: A (Frenet-frame tracking NMPC), Gauss-Newton SQP, condensed
dense QP subproblem solved by OSQP, real-time-iteration-style (few iterations
per tick, warm-started from the previous tick's shifted solution).**
Reasons, in priority order: (1) it is the only candidate that actually removes
the structural gap; (2) it preserves the reference, the score, the weights and
the offline tooling, so results stay comparable to every previous run; (3) it
needs no new runtime dependency (below); (4) it lands as a separate class, so
the existing controller is untouched.

### 16.3 Environment check, dependency decision, and model choices

**Checked directly, not assumed:**
- ROS distro is **Jazzy** (`$ROS_DISTRO`), not Humble; workspace Python is
  system 3.12.
- Installed and importable in the ROS interpreter today: `numpy 2.5.1`,
  `scipy 1.18.0`, `cvxpy 1.9.2`, **`osqp 1.1.3`** (plus CLARABEL/SCS/HIGHS via
  cvxpy). `fsae_control/package.xml` already documents cvxpy/osqp/clarabel as
  required pip installs for the MPC, so **OSQP is an existing dependency, not a
  new one**.
- **CasADi is NOT installed** and cannot be `pip install`ed into the system
  interpreter without `--break-system-packages` (Ubuntu 24.04 / PEP 668). A
  `cp312` manylinux wheel does exist and installs fine into a private
  `--target` directory, which is how it was used for the cross-check in §16.5 —
  but requiring it *on the car* means either breaking the system interpreter or
  standing up a venv that also has to see `rclpy`. **Decision: no CasADi (and
  no acados, which additionally needs a C toolchain + template codegen, and no
  FORCES PRO, which is commercially licensed) in the shipped code path.** The
  SQP + Jacobians are hand-rolled on numpy, and OSQP solves the subproblem.
  This is the more conservative, more reversible choice: nothing new to install
  on the vehicle, and the whole thing is one importable module.
- Cost of that decision, stated honestly: no algorithmic differentiation and no
  interior-point fallback. Mitigated by (a) computing Jacobians by *vectorised
  finite differences* over the horizon rather than by hand, so a model change
  cannot silently desynchronise its derivative, and (b) cross-checking the
  converged solution against CasADi+IPOPT offline (§16.5).

**Model (continuous, `x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`,
`u = [delta_cmd, a_cmd]`):**

```
s_dot       = (v_x*cos(e_psi) - v_y*sin(e_psi)) / (1 - kappa(s)*e_y)
e_y_dot     =  v_x*sin(e_psi) + v_y*cos(e_psi)
e_psi_dot   =  r - kappa(s)*s_dot                     <-- the missing term
v_x_dot     =  a_act + r*v_y
v_y_dot     =  (F_yf*cos(delta_act) + F_yr)/m - r*v_x
r_dot       =  (lf*F_yf*cos(delta_act) - lr*F_yr)/Iz
delta_act_dot = (delta_cmd - delta_act)/tau_delta
a_act_dot     = (a_cmd     - a_act    )/tau_a
F_yf = -2*Cf*alpha_f,  alpha_f = atan((v_y + lf*r)/v_safe) - delta_act
F_yr = -2*Cr*alpha_r,  alpha_r = atan((v_y - lr*r)/v_safe)
```

Every constant is taken from `MPCController.__init__` unchanged — `lf=0.70`,
`lr=0.85`, `m=255.0`, `Iz=150.0`, `Cf=29155.478`, `Cr=19512.342`,
`tau_delta=0.08`, `tau_a=0.02`, `MAX_STEER_RAD`, `MAX_ACCEL`, `MAX_BRAKE`, and
the same `du_max` slew limit. The `2*Cf`/`2*Cr` axle convention matches
`_discrete_model`'s `(2*Cf + 2*Cr)/m` terms exactly. **No new physical
constant is introduced.** `e_y_dot` is the same expression `_error_state`
already uses (`v_x*sin(e_psi) + v_y*cos(e_psi)`), so `q_e_yd` keeps its
meaning.

Three model choices that are NOT inherited and are called out as decisions:

1. **`q_r` now weights `e_psi_dot` (heading-error rate), not absolute yaw rate
   `r`.** In the curvature-blind model these were interchangeable, because the
   reference never rotated. In the Frenet model they are not: penalising
   absolute `r` would penalise the yaw rate the car *must* hold to follow a
   constant-radius corner (`r = kappa*v`), i.e. it would actively fight
   cornering, which is precisely the failure this Part exists to remove.
   Penalising `r - kappa*s_dot` is zero for a car correctly tracking a corner
   and nonzero exactly when the car is rotating *relative to the path*. Same
   weight value, redefined regressor — flagged because it means `q_r` is the
   one weight whose numerical value does not transfer with identical meaning.
2. **Low-speed handling.** The dynamic-tyre terms are singular as `v_x -> 0`.
   `_discrete_model` blends kinematic->dynamic over `v_x in [1.0, 2.5]` m/s;
   the same blend is applied here (identical `alpha = clip((v_x-1)/1.5, 0, 1)`
   breakpoints), with the kinematic branch using `r = v_x*tan(delta_act)/(lf+lr)`,
   `v_y = lr*r`. Reusing the existing breakpoints rather than inventing new
   ones.
3. **`v_ref` is the caller's single `desired_speed`, held constant over the
   horizon.** The node already low-passes it; using a *profile* over the
   horizon would need new plumbing and would change longitudinal behaviour at
   the same time as the lateral change, making the live A/B unreadable. Left
   for later, deliberately (and noted as the obvious next lever for Part 11's
   braking-lag problem, which this does NOT attempt to fix).

**Curvature reference.** `kappa(s)` is built once per distinct path (cached;
rebuilt each tick only in live-planner mode) by resampling the path at 0.5 m,
moving-averaging with a 3-wide kernel and then differencing headings — the
*existing* denoise precedent from `control_utils.curvature_speed()`
(`dense_step = 0.5`, `w = 3`), not a new smoothing constant. This matters more
for NMPC than for the QP: with `kappa` inside the prediction, a spurious
centreline spike (the known open planner defect, CLAUDE.md) would be predicted
as a real bend and steered for.

**Solver.** Gauss-Newton SQP, each iteration: (1) roll the nonlinear model
forward from the measured state under the current input guess — so the
linearisation point is always *dynamically feasible* and the QP's dynamics
defect is exactly zero; (2) finite-difference `A_k, B_k` and the output
Jacobians, vectorised across the horizon; (3) condense to a dense QP in the
input deviations only (`nu*N + N` slack variables — 105 for N=35, small enough
that dense is faster than sparse); (4) solve with OSQP, warm-started, fixed
sparsity so only the data arrays are updated; (5) step with `Delta u` box-
constrained as a trust region. Iterations per tick are capped by a parameter
and by a wall-clock budget, RTI-style: a tick that runs out of time ships the
best feasible iterate rather than missing the tick.

Sources consulted for the survey: arXiv:2212.13115 (Frenet-Cartesian NMPC
representations), arXiv:2402.18558 (F1TENTH survey — tracking-MPC vs MPCC),
github.com/alexliniger/MPCC (MPCC reference implementation), arXiv:1901.08184
(learning/predictive racing control).

### 16.4 What was built (all live-side, `ros2/src/fsae_planning/` only)

| File | Change |
|---|---|
| `control/fsae_control/fsae_control/nmpc_core.py` | **NEW.** `PathReference` (arc length, smoothed `kappa(s)` AND `psi_ref(s)`), the nonlinear model (vectorised `_f` + a hand-mirrored scalar `_f_scalar`/`_step_scalar` fast path), `NMPCController` (Gauss-Newton SQP, condensing, OSQP). ~1050 lines incl. commentary. |
| `control/fsae_control/fsae_control/nmpc_params.py` | **NEW.** `NMPCParams` + `declare_nmpc_params`/`nmpc_params_from_node`/`NMPC_PARAM_FIELDS`, mirroring `mpc_params.py`'s pattern exactly. 24 fields. |
| `control/fsae_control/fsae_control/mpc_controller.py`, `..._standalone.py` | Declare `NMPCParams`; construct `NMPCController` instead of `MPCController` iff `use_nmpc`. Nothing else changed — every downstream call site (`compute`, `reset`, `set_static_path`, `set_heading_profile`, `last_telemetry`, `a_max_brake`) is satisfied by both classes. |
| `common/fsae_bringup/launch/control.launch.py`, `sim.launch.py` | `NMPC_PARAM_FIELDS` fed through the SAME generated-launch-arg mechanism `MPC_PARAM_FIELDS` already uses (no hand-written blocks). |
| `common/fsae_bringup/config/fsae_params.yaml` | 24 new `controller:` keys, defaults identical to `NMPCParams`. Explicitly marked as NOT part of CLAUDE.md's numeric-parity table (no `settings.py` counterpart exists yet). |
| `ros2/launch_all.sh` | `USE_NMPC=false` + a commented-out NMPC shortlist, forwarded via the existing `_append_mpc_arg` helper; startup echo says which controller is running. |
| `control/fsae_control/fsae_control/telemetry_logger.py` | 8 `nmpc_*` columns APPENDED to `ADAPTIVE_COLUMNS` (empty cells on LTV-QP runs, so no existing parser changes). |
| `control/fsae_control/test/nmpc_offline_check.py` | **NEW.** The whole validation suite below, runnable in one command with no ROS/FSDS session. |

**`mpc_core.py` and `mpc_params.py` were not touched at all** — verified by
file mtime (both last modified 2026-08-12, before this work started). That is
a stronger no-op guarantee than a numerical regression test: with
`use_nmpc=false` the LTV-QP path is byte-identical code reached by an
unchanged constructor call, so there is nothing for a regression to differ on.
The only added risk to the existing path is the new module-level import
(`osqp`, `scipy.sparse` — both already required by `mpc_core` via cvxpy) and
24 extra declared ROS parameters; both nodes were imported under ROS Jazzy
and a `declare`/read round-trip confirmed to reproduce `NMPCParams()` exactly.

### 16.5 Correctness checks (all reproducible: `python3 ros2/src/fsae_planning/control/fsae_control/test/nmpc_offline_check.py`)

1. **Scalar/vectorised model parity** — the sequential rollout uses a scalar
   fast path (numpy's per-call overhead makes the vectorised form cost 17 ms
   per rollout vs 2 ms); the two are hand-mirrored, so they are asserted equal
   over 300 randomised states x 3 substep counts. **Worst relative difference
   7.2e-16** (i.e. bit-level agreement). `kappa_scalar` vs `kappa_at`: exactly 0.
2. **Jacobians** — the forward finite differences the SQP actually uses vs
   central differences at a 100x larger step: **worst relative discrepancy
   6.1e-05**, and the only column anywhere near that is `s` (expected: it
   differences the local slope of a piecewise-linear `kappa(s)`).
3. **SQP convergence** — cost decreases monotonically to a plateau at every
   operating point tried, in **3-9 iterations from a COLD start** (zero input
   trajectory): e.g. corner approach 23.91 -> 0.0675 in 3, mid-corner
   360 -> 1.80 in 6. Live it never starts cold (the previous tick's shifted
   solution is the warm start), which is why 1 iteration/tick suffices.
4. **Independent solver cross-check (CasADi 3.7.2 + IPOPT, exact AD
   derivatives, interior point, tol 1e-9)** — the identical OCP rebuilt
   symbolically and solved to convergence, versus this module's
   finite-difference Gauss-Newton SQP:

   | state | SQP cost | IPOPT cost | gap | SQP u0 | IPOPT u0 |
   |---|---|---|---|---|---|
   | straight, on-line | 0.00000 | 0.00000 | 0.00% | 0.000 deg | -0.000 deg |
   | e_y = +1.0 m | 58.37291 | 58.37235 | 0.00% | -9.000 deg | -9.000 deg |
   | corner approach | 0.26264 | 0.26263 | 0.00% | **-0.333 deg** | **-0.327 deg** |
   | mid-corner, v=9 | 1.79938 | 1.79907 | 0.02% | +9.000 deg | +9.000 deg |
   | e_psi = -5 deg | 123.84159 | 124.11642 | 0.22% | +9.000 deg | +9.000 deg |

   The hand-rolled solver finds the same optimum as IPOPT. Note CasADi was
   used ONLY here, from a private `pip install --target` directory; it is not
   a dependency of the shipped code (§16.3).

### 16.6 Does it close the late-turn-in gap? (the actual question)

**(a) The structural gap, reconfirmed directly.** Car placed EXACTLY on the
line (`e_y = e_psi = 0`) at 2-20 m before a known bend, at 8 and 14 m/s:
`MPCController` commands **0.000 deg at every single one of the 8 test
points**. Not "small" — exactly zero, as its model implies. This is Parts
1/3e's behavioural finding reduced to one line of evidence.

**(b) The NMPC plans steering for the same states**, whenever the bend is
inside its horizon's reach (`v*N*dt`): horizon-peak steering 0.96-4.88 deg,
rising as the bend gets closer. The 3 test points where the bend is FURTHER
away than the horizon reaches correctly plan 0.00 deg — no controller can
anticipate past its own horizon, and asserting otherwise was a bug in the
first version of this check, not in the controller.

**(c) The wrong-direction failure mode of Parts 2/7/15 does NOT reproduce, but
it is not exactly zero either — the honest numbers.** Converged, the deepest
wrong-direction command anywhere in the horizon is **-0.33 deg** (at 14 m/s,
2 m before the bend) against a +4.88 deg correct-direction peak in the same
plan, and it lasts **one step** — the "consecutive steps below -0.5 deg"
counter reads 0 at every test point. IPOPT reproduces it (-0.327 deg), so it
is the genuine optimum of this cost, not a solver artifact. For scale:
one 50 ms tick at -0.33 deg moves the actual road wheel about 0.15 deg
through the `tau_delta = 0.08 s` lag. Compare Parts 2/7/15, which failed with
sign-flipped excursions **sustained over ~7 consecutive steps at a magnitude
comparable to the eventual correct-direction command**. Different in kind and
~15-30x different in magnitude. **Caveat recorded rather than glossed:** at
`N=35` with only 2 iterations the same probe read -3.0 deg. The shipped
default is `N=20`, but if the horizon is ever lengthened this is the number
to re-measure first.

**(d) Closed loop on the real track — the result that matters.** The offline
harness drives **fsae_MPCTest's 25-state Pacejka plant** (suspension, tyre
relaxation, wheel dynamics, load transfer, and the modelled FSDS `alat_ceiling`)
along `comp_test_map_3/raceline.csv`, with the SAME `MPCParams` weights, the
same plant, the same speed reference and the same dt for both controllers —
so unlike the live logs in Parts 11-13 (which mix several weight sets and are
NOT comparable to each other), this is a clean single-variable A/B:

| | \|e_y\| mean / p90 / max | \|e_psi\| mean / p90 | steer sat | lap | solve mean / p95 / max |
|---|---|---|---|---|---|
| LTV-QP (as shipped) | 0.400 / 1.451 / 2.323 m | 5.92 / 15.41 deg | **12.5%** | 43.1 s | 7.4 / 9.9 / 40.5 ms |
| **NMPC (defaults)** | **0.277 / 0.686 / 1.150 m** | **5.84 / 14.50 deg** | **0.8%** | **42.0 s** | 8.9 / 11.6 / **14.7 ms** |

Lateral error p90 halves, max drops 50%, heading error is slightly better,
the lap is 1.1 s faster, and **steering saturation falls from 12.5% to 0.8%** —
a 15x reduction in exactly the symptom ("pins +-25 deg through corners") this
whole investigation has been chasing. Solve time is comparable on average and
far better in the tail (the LTV-QP's own worst tick in this run was 40.5 ms).

**(e) Turn-in timing, measured per corner.** Turn-in point = the arc length at
which |steering| first exceeds 25% of that corner's own peak, measured relative
to the corner's geometric start, with the search window starting at the
previous corner's end so a neighbour cannot contaminate it:

| corner (s) | radius | LTV-QP | NMPC | NMPC earlier by |
|---|---|---|---|---|
| 38.0 | 5.9 m | +2.9 m | -4.5 m | 7.4 m |
| 101.0 | 6.9 m | +11.7 m | -39.9 m* | 51.6 m |
| 160.0 | 4.8 m | +2.4 m | -1.8 m | 4.2 m |
| 197.5 | 25.3 m | -0.2 m | -1.1 m | 0.9 m |
| 251.0 | 7.6 m | +3.9 m | -35.1 m* | 39.1 m |
| 306.0 | 8.2 m | +28.0 m | -2.2 m | 30.2 m |
| 390.5 | 10.9 m | +2.4 m | -23.2 m* | 25.6 m |

**Earlier on 7/7 corners, median 25.6 m.** Sign matters as much as magnitude:
the LTV-QP's turn-in is POSITIVE on 6/7 corners (it starts turning after the
corner has already begun); the NMPC's is negative on 7/7 (before). *Three
rows hit the 40 m search-window edge, so those are lower bounds, not exact.

**(f) Two bugs and one modelling gap found by this testing, all fixed — worth
recording because each was invisible in the synthetic tests and only appeared
in closed loop:**

1. **Infeasible subproblem accepted as a step direction.** OSQP returns a
   finite-but-meaningless `x` on a primal-infeasible QP; the first version
   checked only for NaN. Combined with (2) this built a divergent wrong-way
   full-lock ramp over ~20 ticks. Fixed by checking status, and — the real
   fix — by projecting the warm start onto the slew-feasible set so `dU = 0`
   is always feasible and the subproblem is unconditionally feasible by
   construction.
2. **Backtracking accepted a non-improving step** ("so a tick always makes
   progress"), writing a bad direction into the warm start and carrying it
   into the next tick. Now only genuine improvements are kept.
3. **The reference heading was quantised.** Measuring `e_psi` against the raw
   segment tangent (as the LTV-QP does) steps by `ds/R` — 5.7 deg per 0.5 m
   waypoint on a 5 m-radius hairpin. The NMPC reads each step as real state
   error and corrects it within a tick or two, producing a hard period-2
   +-25 deg steering limit cycle. Fixed by deriving `psi_ref(s)` from the SAME
   smoothed samples `kappa(s)` comes from, so the measured Frenet state and the
   predicted one describe one identical reference. **This is a real behavioural
   difference from the LTV-QP's `e_y`/`e_psi` definitions** (smoothed reference
   direction rather than raw tangent), not the bit-identical measurement §16.3
   originally intended; the difference is bounded by the 1.5 m smoothing window.
4. **The prediction had no grip limit.** Linear tyres produce unbounded
   lateral force, so the model believed it could hold any corner at any speed;
   the plant cannot. The NMPC demanded yaw that never arrived and **spun at
   s ~ 191 m** (`e_psi` 94 deg). Fixed by saturating the predicted lateral
   forces at FSDS's measured sustained `a_lat` ceiling, reusing
   `MPCParams.alat_ceiling_flat/_slope/_intercept`[^alat-moved] — the same law
   `mpc_core._alat_ceiling_at` and `vehicle_physics.alat_ceiling_at` already
   carry, no new constant. `nmpc_alat_ceiling_enabled=false` recovers the
   unconstrained plant for real-vehicle work. **This single change is what
   turned a spin into the §16.6(d) result.**

   [^alat-moved]: True when written. The corner_factor rewrite later the same
   day removed these from `MPCParams`; `nmpc_core.py`'s `_Plant` now hardcodes
   the same numbers (7.5/0.47/2.46) as its own class defaults instead. See
   `planning_control_sync.md`'s "Nonlinear MPC (`use_nmpc`)" section.

### 16.7 Solve time, and how the defaults were chosen

Per-iteration cost, N=35, measured on this machine: rollout 2.2 ms, one-step
Jacobians 4.1 ms (the dominant term, 10 batched integrations), output
Jacobians 0.3 ms, condensing + OSQP 1.3 ms. The first version cost 42 ms/tick
because the rollout went through the vectorised model (16.7 ms) — hence the
scalar fast path (§16.5.1), and hence `nmpc_jac_substeps=1` (the sensitivities
only set a step direction; the rollout, which defines the prediction, keeps
`nmpc_rk_substeps=2`).

Horizon/iteration sweep, closed loop, identical weights:

| N | iters | \|e_y\| mean / p90 | \|e_psi\| mean | sat | lap | solve mean / p95 |
|---|---|---|---|---|---|---|
| 35 | 1 | 0.437 / 1.200 | 6.45 | 0.2% | 40.7 s | 14.1 / 20.1 ms |
| 35 | 2 | 0.520 / 1.372 | 6.85 | 1.1% | 40.4 s | 24.3 / 34.1 ms |
| 25 | 1 | 0.314 / 0.775 | 5.99 | 1.3% | 41.5 s | 10.3 / 13.6 ms |
| **20** | **1** | **0.277 / 0.686** | **5.84** | **0.8%** | **42.0 s** | **9.1 / 12.4 ms** |
| 20 | 2 | 0.302 / 0.752 | 5.86 | 0.4% | 41.6 s | 17.9 / 23.5 ms |
| 15 | 1 | 0.254 / 0.597 | 5.73 | 0.9% | 42.8 s | 8.1 / 10.5 ms |
| 15 | 2 | 0.266 / 0.655 | 5.80 | 0.0% | 42.4 s | 15.1 / 18.5 ms |

Two results worth stating explicitly because both contradict the obvious
assumption:
- **A longer horizon tracked WORSE.** N=35 (matching `MPCController`'s
  horizon, which was the original plan) is the worst tracking row in the
  table. The prediction model is optimistic — linear tyres, no suspension, no
  tyre relaxation — and that mismatch compounds over 1.75 s. N=35 does produce
  the fastest lap (40.7 s), so if lap time rather than tracking is the
  objective this trade is worth revisiting deliberately.
- **More SQP iterations per tick was slightly worse as well as ~2x more
  expensive.** Consistent with the real-time-iteration literature: the warm
  start carries convergence across ticks, and solving harder against an
  optimistic model exploits its errors.

Defaults therefore: **N=20, 1 SQP iteration, 25 ms budget** — chosen from this
table, not from theory. `nmpc_solve_budget_ms` is a hard stop: a tick that runs
out of time ships its best feasible iterate rather than overrunning.

### 16.8 STATUS and what is left for the live test

**STATUS: implemented, wired end-to-end, offline-validated, NOT YET LIVE-TESTED.
Defaults land it OFF (`use_nmpc=false` / `USE_NMPC=false`), and the LTV-QP path
is byte-unchanged.**

To live-test: set `USE_NMPC=true` in `ros2/launch_all.sh` and run as usual.
The intended configuration is the one already in that file —
`USE_PRECOMPUTED_PATH=true` + `USE_PRECOMPUTED_SPEED=true` (a static raceline
and its speed profile), which is also the only mode the NMPC has been tested
in. Live-planner mode works (it rebuilds the reference per tick, cached on
change) but is untested here.

Known differences to expect versus every previous run in this file, so a live
log is not misread:
- **`use_precomputed_heading_profile` has NO effect** with the NMPC. The shaped
  heading lead (Parts 8-13) approximates the curvature this model carries
  exactly; applying both would double-count it. One log line says so at startup.
- **The whole adaptive gain schedule is inactive**: no `m_*` telemetry columns,
  no lookahead boosts, no anti-hunt, no exit-boost hold. Those mechanisms exist
  to synthesise anticipation this model does structurally. The 8 new `nmpc_*`
  columns replace them diagnostically.
- **`q_r` means something different** (heading-error rate `r - kappa*s_dot`, not
  absolute yaw rate) and is the weight most likely to need a live sweep. Every
  weight can be overridden NMPC-only via `nmpc_q_*` (-1.0 = inherit from
  `MPCParams`) without touching `MPCParams`/`settings.py` parity.
- Part 11's longitudinal problem (speed target falls, `a_cmd` stays positive
  ~1.8 s) is **not addressed**: `v_ref` is still one scalar held across the
  horizon. Feeding the precomputed speed PROFILE over the horizon is the
  obvious next step and is the single most promising extension of this work,
  but it was deliberately left out so the live A/B changes one thing.

**Addendum (2026-08-13, later same day): offline port added.** The scope
note below was written before this changed — `fsae_MPCTest` now has its own
NMPC port (`controller/nmpc_optimiser.py`, selected by `settings.USE_NMPC`,
wired into `sim/rollout_core.py`'s `run_core_rollout(..., use_nmpc=...)`),
and the NMPC weight overrides were moved from a separate `NMPCParams` onto
`MPCParams` itself (`nmpc_q_e_y` etc., `-1.0` = inherit), removing the parity
obligation this section originally anticipated. See
`planning_control_sync.md`'s "Nonlinear MPC (`use_nmpc`)" section for the
current, authoritative description of both sides.

**Original note (superseded by the addendum above):** not mirrored into
`fsae_MPCTest` — consistent with this whole session's live-only scope, and
flagged as the user's decision, not an omission. If it is mirrored later:
`nmpc_core.py`/`nmpc_params.py` have no offline counterpart at all, and
`NMPCParams` would then acquire a `settings.py` parity obligation that it
deliberately does not have today (see `nmpc_params.py`'s module docstring).

### 16.9 Live test: matched same-day LTV-QP vs. NMPC pair

A same-day live pair on `comp_test_map_3` (`mpc_standalone`, identical
weights: `q_e_y=6.35, q_e_yd=0.5, q_e_psi=1.65, q_r=1.0, q_e_v=5.40,
r_delta=1.8, r_a_accel=2.25, r_a_brake=0.5, r_rate=[2.5, 2.25]`) was logged in
`fsae_logs/Linear mpc/mpc_standalone_control_1786568958.csv` (`use_nmpc=0`)
and `fsae_logs/NMPC/mpc_standalone_control_1786571019.csv` (`use_nmpc=1`):

| | LTV-QP (live) | NMPC (live) |
|---|---|---|
| lap time | 54.72 s | 52.35 s |
| composite score | 0.695 | 0.532 |
| RMSE (lateral) | 0.455 m | 0.378 m |
| peak lateral error | 1.636 m | 1.179 m |
| \|e_psi\| mean / p90 / max | 7.85° / 15.53° / 28.16° | 5.06° / 11.71° / 21.22° |
| steering saturation | 6.45% | 0.58% |
| steering reversals | 299 | 226 |

Same direction and similar magnitude as §16.6's offline A/B (steering
saturation drops sharply, tracking error improves across the board), on a
single matched pair — not the same n=multiple-runs rigor as the offline
sweep. NMPC judged live-tested and performing well on the strength of this
result.

### 16.10 Three MPCC-inspired additions: two rejected on first live test

Prompted by a comparison against Alexander Liniger's Model Predictive
Contouring Control (MPCC — "C" is Contouring; `https://github.com/alexliniger/MPCC`).
MPCC's headline idea — treating progress along the track `θ` as a free
decision variable the solver *maximises*, with a contouring-error/lag-error
split against a parametric spline — was assessed and not adopted: `θ̇`-
maximisation is a more aggressive version of exactly the "exogenous,
schedulable future obligation" failure mode this NMPC's own `kappa(s)`-as-state
design exists to avoid (§16.1), and adopting it would need the same
falsification testing (§16.6's dead-on-line synthetic corner approach) before
being trusted. Three narrower, self-contained ideas survived that filter, all
NMPC-only (`mpc_core.py` untouched by any of them) and implemented identically
in both `nmpc_core.py` (live) and `controller/nmpc_optimiser.py` (offline
port).

**1. Spline-based path reference** (`nmpc_spline_reference_enabled` /
`NMPC_SPLINE_REFERENCE_ENABLED`, default `true`). `PathReference` fits
`x(s)`/`y(s)` as two independent `scipy.interpolate.CubicSpline` objects over
cumulative arc length, deriving `kappa(s)`/`psi_ref(s)` analytically
(`kappa = (x'y'' - y'x'') / (x'^2+y'^2)^1.5`) instead of the old
dense-resample + moving-average + finite-difference-headings pipeline. This is
MPCC's reference-parametrisation mechanism (a continuous spline in arc length)
adopted on its own, decoupled from the contouring/progress apparatus built on
top of it in the original paper. Defaults on, unlike the other two: a strict
numerical-quality improvement with no new coupling to solver dynamics, and it
directly targets the open "centreline curvature spikes" defect (CLAUDE.md) — a
proper spline fit was one of that defect's two previously-named-but-unattempted
remedies. The old moving-average path is kept intact behind the flag for A/B.
`kappa_at`/`kappa_scalar`/`psi_ref_at`/`project` needed no changes.

**2. Horizon speed profile** (`nmpc_horizon_speed_profile_enabled` /
`NMPC_HORIZON_SPEED_PROFILE_ENABLED`, default `false`, experimental). Before
this, `v_ref` (the cost's speed target, `H[:,4] = v_x - v_ref`) was a single
scalar held constant across the whole horizon — deliberately, so a live A/B of
feature 1's lateral-model change alone would not be confounded by a
simultaneous longitudinal change. This flag samples a precomputed per-lap
speed profile at each horizon stage's own predicted arc length `s_k` via a new
`PathReference.v_ref_at(s)`, the same way `kappa_at(s)` is already looked up
against the predicted state rather than scheduled by horizon step — deliberate,
so the feature would inherit `kappa(s)`'s non-schedulability property instead
of reproducing the earlier curvature-scheduling failures (§16.1, Parts 2/7/15)
in longitudinal form. It targets Part 11's still-open braking-lag problem.
Fully wired in `sim/rollout_core.py`'s NMPC construction/`compute_step()` call
(offline) and `mpc_controller_standalone.py`'s `set_static_path()` call (live
mirror); deliberately not wired in `mpc_controller.py` (the LTV-QP-parity
node). With no speed-profile array supplied, or with the flag off, `v_ref`
remains the exact same frozen scalar as before.

Live-tested enabled alone (spline reference also on, friction circle off) on
`comp_test_map_3`, `mpc_standalone_control_1786585464.csv`: showed the exact
predicted failure mode, worse than the curvature case it was modelled on. At
t≈58–61s approaching a corner, `v_actual` climbed from ~5.7 to **16.7 m/s**
while `v_desired` **dropped** to 3.3–5 m/s and stayed there for ~2s — `a_cmd`
stayed strongly positive (0.6 to 2.5+ m/s²) throughout, i.e. the controller
was actively accelerating, not merely failing to brake. `e_y` grew to
**-3.6 m** (car off-track; `nmpc_track_halfwidth` is 3.5 m) before the corner
geometrically opened back up and it recovered. Mechanism: unlike `kappa(s)`,
whose obligation only exists once the predicted trajectory's own `s` has
actually reached it, summing `v_x - v_ref(s_k)` across all 20 horizon stages
let a high `v_ref` at a later stage (the straight after this corner) offset the
cost of a low `v_ref` at an earlier stage (the corner itself) in the same
solve — the QP's net gradient could favour accelerating now against a target
that was about to rise, even though the current target was low. Same
"solver pre-pays a future obligation" trap as the curvature-scheduling
failures (Parts 2/7/15), but surfacing as real off-track excursions rather
than a self-correcting steering wobble, because a wrong-direction speed
decision carries real kinetic energy into the corner for the lateral
controller to then fight. Reverted to `false`. The non-schedulability
property this feature was designed to inherit from `kappa(s)` does not
transfer to a summed cost over the horizon — `kappa(s)`'s safety came from the
state coupling, not from being state-indexed per se, and speed's cost
structure (a plain sum of squared errors across all stages) breaks that
coupling in a way curvature's structure did not. Do not re-enable without
addressing the underlying mechanism (e.g. bounding how far ahead along `s`
the sampled `v_ref` may rise, or some other per-stage clamp preventing a later
high-speed stage from outvoting an earlier low-speed one) and revalidating
offline first.

**3. Friction-circle hard constraint** (`nmpc_friction_circle_enabled` /
`NMPC_FRICTION_CIRCLE_ENABLED`, default `false`, experimental). Adds a hard
`|F_yf|, |F_yr| <= F_max` bound to the condensed QP, additional to — not a
replacement for — the existing soft `tanh` lateral-force saturation already
inside `_f`/`_f_scalar` (per CLAUDE.md's standing caution against
re-litigating that soft mechanism without new measurement evidence). `F_max`
is derived from the same measured ceiling law
(`alat_ceiling_flat/_slope/_intercept`) via `F_max = m * ceiling(v_x) / 2` per
axle. Mechanically: `_outputs()` grows two extra, cost-unweighted rows (tyre
forces, computed post-soft-saturation) that ride through the existing
`_output_jacobians` finite-differencing for free, reusing the condensing
step's own `S = dx/dU_flat` rather than needing a second rollout;
`_build_qp`/`_solve_step` add `2*N` new hard rows only when the flag is on.
When off, `_build_qp`/`_outputs`/`_output_jacobians`/`_solve_step` are
identical — same array shapes, same QP dimensions — to before this feature
existed. Telemetry exposes `nmpc_fyf_max_abs`/`nmpc_fyr_max_abs` only when
enabled, but see the telemetry gap below — these never actually reached a CSV
column. Loosely inspired by MPCC's friction-ellipse tyre constraints, adapted
to bound the same tyre-force quantity this NMPC's plant already computes.

Live-tested enabled alone (spline reference also on, horizon speed profile
off) on `comp_test_map_3`, `mpc_standalone_control_1786585910.csv`: much more
severely broken than feature 2. The SQP subproblem failed to solve
(`nmpc_status=0`) on **77.5% of all 614 ticks**, steering sat at the
mechanical hard lock (±25°) on **30.8% of ticks** starting as early as
t=0.65s, and the run ended in a full stall — car stopped (`v_actual≈0`),
4.94 m off-track, heading error -52°, every column frozen tick-for-tick for
the final ~1 s of the log. This was enabled with no prior offline A/B (unlike
features 1/2), against explicit caution given before testing it live; the
result confirmed that caution was warranted. Reverted to `false` in
`launch_all.sh` immediately.

Root cause: unlike the soft `tanh` saturation it sits alongside (which only
engages once beyond the ceiling, and is a smooth penalty the solver can trade
off against), the new hard `|F_yf|,|F_yr| <= F_max` rows have no slack
variable — there is nothing for the QP to give up if ordinary cornering
geometry and the force bound conflict simultaneously, which they evidently do
under completely normal driving on this track, not just extreme conditions.
When that happens the subproblem goes infeasible, `_solve_step` correctly
refuses to act on the resulting garbage direction (per its own documented
safety logic), and the practical effect is the controller stops updating its
steering command tick after tick — exactly the "pretty much doesn't work
anymore" symptom observed, compounding into a spin/stall as the geometry
degrades further with no correction being applied. This means
`F_max = m * ceiling(v_x) / 2` per axle is measurably tighter than what normal
cornering actually needs on this plant/track, not merely a
conservative-but-workable bound.

**Bug found alongside this rejection** (separate from it, worth fixing
regardless of whether the feature is ever revisited): `nmpc_fyf_max_abs`/
`nmpc_fyr_max_abs` never actually appeared in the CSV — `telemetry_logger.py`'s
`NMPC_COLUMNS` is a hand-maintained tuple, unlike `build_config_lines()`'s
`dataclasses.asdict()`-based config dump, and nobody added the two new field
names to it when the feature was implemented. This silently dropped exactly
the diagnostic that would have shown, tick-by-tick, how close to (or over)
`F_max` the solve was running — the post-mortem above had to rely on
`nmpc_status`/steering/stall behaviour alone because of this gap. Fix
`NMPC_COLUMNS` before ever re-attempting this feature.

Neither feature 2 nor 3 has offline A/B numbers (feature 1 is a numerical
improvement to an existing mechanism and is default-on; features 2 and 3 are
default-off, both live-tested-and-rejected in their current form) —
reproduce a comparison with `python -m tuner.nmpc_offline_check` once one
exists. Do not re-enable 2 or 3 without first fixing the identified
mechanism, adding an offline A/B, and (for feature 3) the missing telemetry
columns above.

### 16.11 Fixed: NMPC steered hard-right at a standstill on every run

**Symptom:** every NMPC run from a standing start — independent of any of the
three §16.10 features, reproducing with only feature 1 (spline reference,
default-on) active — commanded a hard, transient right-steer excursion in the
first ~0.5-0.7s: steering reached the full ±25° mechanical lock by
t≈0.59-0.65s, `nmpc_pred_ey_end` (the horizon's own predicted terminal
lateral error) swung to roughly -3.3 m, while `v_actual` was still essentially
zero. The car recovered once genuinely moving and did not repeat the
excursion on later laps through the same map location.

**Not the track/spawn geometry.** The car spawns at `(0,0)` facing `+x`; the
track's first recorded waypoint sits at `x≈1.76, y≈-0.18` with
`psi≈0.006-0.009` rad — a small, ordinary offset (`e_y≈0.16` m at t=0,
`nmpc_s0≈-1.06` m, i.e. the Frenet projection lands just behind the path's
recorded start). Ruled out as the cause by a same-day, same-spawn A/B: both
the LTV-QP (`mpc_core.py`, `Linear mpc/mpc_standalone_control_1786594804.csv`)
and Stanley (`stanley_control_1786594710.csv`) see the identical `e_y≈0.16-0.17`
at t=0 and show no equivalent hard-lock snap — Stanley's initial command is
-8.9°, converging smoothly; the LTV-QP's is a few degrees, also smooth. Since
both react to the same starting error without incident, the cause was
specific to the NMPC's own model, not the track/spawn setup — track padding
was considered and rejected as a fix for this reason (the same v_x=0 startup
condition would recur at any new start point).

**Root cause: `_f`/`_f_scalar`'s tyre-slip-angle formula manufactured a
lateral force from steering alone at `v_x≈0`.** `alpha_f = arctan((v_y +
lf*r)/v_safe) - d`, where `v_safe = max(|v_x|, v_blend_hi)` floors the
denominator to avoid a divide-by-zero as `v_x -> 0`. That floor is necessary,
but its side effect was that at `v_x=0` with `v_y, r` small, `alpha_f ≈ -d` —
the slip angle tracked the commanded steering angle directly, producing a
substantial `F_yf` (and `F_yr`) purely from steering, with zero forward speed.
A real tyre generates ~zero lateral force with no rolling contact velocity,
regardless of steering angle — this was backwards. The `blend` factor already
computed for the kinematic/dynamic mix (0 at `v_x <= v_blend_lo`, 1 at
`v_x >= v_blend_hi`) already excluded `v_y_dot_dyn`/`r_dot_dyn` from the state
derivative at low speed, but that happened too late: the force itself already
existed and was available to the SQP's cost/Jacobians before that exclusion,
and (for `nmpc_friction_circle_enabled`, when re-enabled) would have been
visible to the friction-circle constraint too.

**Fix:** scale `F_yf`/`F_yr` by the same `blend` factor immediately after
they're computed (and after the existing `alat_ceiling` soft saturation), in
all three copies — `nmpc_core.py` (live and mirror) and
`controller/nmpc_optimiser.py`'s `_f`, `_f_scalar`, and its
`nmpc_friction_circle_enabled`-only `_tyre_forces()` helper (which computes
the same force independently for telemetry/constraint rows and must stay a
line-by-line mirror of `_f`'s own computation per its own docstring). At
`v_x=0` with any commanded steering, every lateral-dynamics derivative
(`e_y_dot`, `v_y_dot`, `r_dot`) is now confirmed exactly zero (previously
`v_y_dot`/`r_dot` were nonzero purely from `d`). Verified: `_f`/`_f_scalar`
parity holds at 1e-13/1e-14 (both live/mirror and offline, well under the
1e-12 bar `test_scalar_matches_vectorised` requires), and
`python -m tuner.nmpc_offline_check`'s full suite (model parity, SQP
convergence, turn-in/wrong-direction, closed-loop) passed with no regression
in the closed-loop `|e_y|`/`|e_psi|`/saturation numbers.

**Live-tested and confirmed fixed**, with one intervening false alarm caused by
a stale build rather than the fix itself. The first live attempt after
copying the fix to the live checkout (`mpc_standalone_control_1786595389.csv`)
still showed the hard-lock snap — traced to the ROS 2 workspace not having
been rebuilt, not a flaw in the fix: this project's `--symlink-install` setup
symlinks `ros2/build/fsae_control/fsae_control` back to
`src/fsae_planning/control/fsae_control/fsae_control`, but the running Python
process had an older module already loaded/cached from before the edit, so
the source-level edit was invisible until the workspace was rebuilt and the
nodes restarted. This is exactly the class of issue `ros2/launch_all.sh`'s
own commented-out `--symlink-install` rebuild step exists to catch (see that
file: "an edit to `src/` after the last build is silently invisible to
`ros2 launch` until rebuilt" — this was the third time it had bitten in one
session, distinct from §52's earlier `zip_safe`-related install-mode
incident). After rebuilding, `mpc_standalone_control_1786595530.csv` showed
the fix working exactly as predicted: steering never exceeded ~18° in the
first two seconds (previously pegged at the full ±25° lock for most of a
second), and the full run (1107 ticks, a complete lap) posted the best
numbers of the whole session — `|e_y|` mean 0.195 m, max 0.968 m, steering
saturation 0.09%.

### 16.12 Fixed (partially): NMPC standstill steering via speed-cost leaking into the steering channel

**A DIFFERENT bug from §16.11 above** — same top-level symptom (large
steering excursion from a standing start, small tracking error), different
mechanism, found independently on 2026-08-20 during a live test that still
showed steering saturation on liftoff despite §16.11's fix already being in
place. Do not conflate the two: §16.11 was a manufactured tyre force in the
plant model at `v_x=0`; this one is a coupling in the NMPC's own cost
function that exists regardless of the plant model.

**Symptom:** `v_desired` jumped straight to the raw target speed (the full
track speed, e.g. 18 m/s) on the very first control tick, because the
speed-target rise limiter's own state variable (`v_des_prev` live,
`v_des_prev` offline) started as `None` and the limiter's guard
(`if v_des_prev is not None:`) skipped applying the limit entirely on that
first tick — the one tick where the gap between `v_x=0` and the raw target
is largest and the limiter matters most. The same `None`-guard pattern also
gated `nmpc_core.py`'s own internal `_v_des_filtered`, so it provided zero
smoothing on its first use too.

**Root cause of why this leaks into STEERING specifically, not just
throttle:** the NMPC's per-stage cost includes `e_v = v_x - v_ref`
(`w_out[4]`, `q_e_v`/`nmpc_q_e_v`). At a standing start with the raw target
applied, `e_v` is 1-2 orders of magnitude larger than `e_y`/`e_psi`,
becoming the dominant residual in the Gauss-Newton least-squares cost.
Steering has no DIRECT effect on `v_x`'s dynamics, but the sensitivity
matrix `S = dx/dU` (built via finite-difference `A_k`/`B_k` Jacobians) does
carry a real, if numerically small per-stage, coupling — the kinematic
branch's `r_dot_kin` depends on steering, which feeds `v_y`/`r` into
`v_x_dot` at subsequent stages. A large enough `e_v` residual, projected
through that coupling by `grad = G.T @ g`, is enough to move the steering
control even though steering cannot actually reduce that residual.
Confirmed directly: with `e_v` active, a single SQP step from a state with
`e_y=0.15 m`/`e_psi=-1.0°` commanded a `-9°` steering change; with `q_e_v`
zeroed, the same state gave `-0.004°`.

**Fix:** seed `v_des_prev`/`_v_des_filtered` from the car's actual current
speed the first time each is used, instead of leaving the rate limiter
disabled for that first tick. Applied to both controller nodes
(`mpc_controller.py`, `mpc_controller_standalone.py`) and
`sim/rollout_core.py`. This closes the WORST case (a full jump to the raw
target) but does not fully close the underlying `e_v`→steering coupling —
see below.

**Residual, NOT fully fixed:** even with the rise limiter correctly seeded,
a smaller version of the same coupling can still occur during the ramp
itself, if achieved acceleration lags the ramped target. Observed live: a
brief window (2 ticks, max ~22°, well under the 25° mechanical lock) where
steering built up smoothly while `e_y`/`e_psi` were still small, driven by
the same `e_v` mechanism at a smaller magnitude. Three fixes for this
residual were tried and rejected, in order:

1. **Capping the horizon's speed target using the controller's OWN previous
   acceleration output**, projected forward. Created a feedback loop: low
   achieved accel → low cap → weaker incentive to accelerate → accel stays
   low. Produced a permanent low-speed stall (~2 m/s) rather than fixing
   anything. Any fix for this residual must not depend on the controller's
   own prior output.
2. **Soft-saturating the `e_v` residual itself** (`tanh`) inside `_outputs()`.
   Flattens the cost GRADIENT everywhere past the saturation knee, not just
   in the extreme case, which weakened real acceleration authority even at
   moderate, legitimate speed gaps.
3. **Boosting `r_delta` (steering effort) when the speed gap is large.**
   Ineffective: the SQP's trust-region step-size limit (`trust_delta_rad`,
   9°) was the actual binding constraint in both boosted and unboosted
   cases, so reweighting `r_delta` shifted the gradient's direction but not
   the trust-region-clipped magnitude of the resulting step.

All three were reverted; none are present in the shipped code. The
residual is small enough (well under the mechanical limit, brief) that it
was left as an open, documented gap rather than risk shipping a fix with a
worse side effect. A durable fix likely needs to change how the cost
couples steering and speed-tracking (e.g. a genuinely decoupled residual
weighting, or restructuring which stages carry `e_v`), not a scalar
scaling of one existing term — worth exploring, not yet attempted.

## Part 0 (background) — how the accel/brake effort split (`r_a_accel`/`r_a_brake`) came about, 2026-08-12

This predates Part 1 above chronologically (same day) and is why Part 1's
snapshot table already shows the split in place. Recorded here since it was
never written up elsewhere.

Reported symptom, after the `r_a 0.85 -> 0.77` cut (see
`planning_control_sync.md`'s history via `sim_to_real_investigation.md`) was
confirmed live: the same shared weight that freed up acceleration also
weakened braking by the same amount, since `R_diag[1]` was applied
symmetrically to `|a_cmd|` regardless of sign. Live telemetry after the
`r_a` cut showed the resulting asymmetry directly — a corner-entry log
(`mpc_standalone_control_1786444690.csv`) had the car arriving faster into
corners (accel side now eager) while `a_cmd` still floored around -1.4 m/s²
during a sustained 2-second, 3-5 m/s speed deficit (braking side unchanged
and still weak) — too hot into corners, followed by steering saturation and
an unstable post-saturation recovery.

Fix: `R_diag[1]`'s single scalar weight was replaced with two independent
weights, `r_a_accel` (`a_cmd >= 0`) and `r_a_brake` (`a_cmd < 0`), applied
via `cp.pos(u[1,:])`/`cp.neg(u[1,:])` in the QP cost —
`R_a_accel * sum(pos(a_cmd)²) + R_a_brake * sum(neg(a_cmd)²)` — rather than
new slack variables or constraints. A slack-variable design was considered
first and replaced with this simpler `cp.pos`/`cp.neg` rewrite once it was
confirmed DCP-valid and numerically identical to the old single-weight cost
when `r_a_accel == r_a_brake`, since `pos(x)²+neg(x)² == x²` for any real
`x`. Implemented in both `mpc_core.py`/`mpc_params.py`/`fsae_params.yaml`/
`launch_all.sh` (live) and `controller/optimiser.py`/`settings.py` (offline)
— see `planning_control_sync.md`'s "Accel/brake effort weight split"
section for the current file-by-file mapping.

Checked before implementing: no adaptive gain (`_adaptive_R_scaling`,
`_adaptive_R_rate`, etc.) touches index 1 of `R`/`R_rate` anywhere in this
codebase (confirmed by direct grep), so the split composed cleanly with
every existing adaptive mechanism with no interaction to account for.

Values were adjusted directly during live testing the same day, moving
0.35/0.2 -> 0.5/0.2 -> 0.5/0.1 -> 1.0/0.6 across one session, before
settling at the 1.0/0.4 shown in Part 1's snapshot table above (further
retuned since — see `mpc_params.py`'s actual current defaults).

## Part 0c (background) — how `_lookahead_steer_effort_relax` (R[0,0] approach-side relief) came about, 2026-08-12

Also predates Part 1 chronologically (same day) — Part 1's `R[0,0]`
walkthrough and Part 3c/3d already describe this mechanism as an existing,
implemented part of the gain-scheduling family. Recorded here since the
diagnosis itself was never written up elsewhere. Implemented on both sides
(live `mpc_core.py`/`mpc_params.py`, offline `model_utils.py`/
`sim/rollout_core.py`/`settings.py`, and the `fsds_simulator` mirror), not
live-tested in isolation before Part 1 folded it into the broader
mechanism inventory.

Reported symptom (a follow-on diagnosis from the same live session that
produced the `r_a` cut and the accel/brake split above): the car was slow
to commit to turn-in specifically at higher corner-entry speed.

Root cause: `_adaptive_R_scaling`'s speed-dependent steering-effort penalty
(`R[0,0] *= 1 + 1.5*vx/(6+vx)`, e.g. ~2.07x at 15 m/s) had no lookahead
relief at all — it stayed at full strength right through an approaching
corner regardless of curvature. The only other mechanism touching
`R[0,0]`, `_steer_effort_straight_boost`, only ever raised it (on a clear
straight) or relaxed back to the unscaled baseline as a corner was
detected — neither one ever pushed `R[0,0]` below baseline for an
approaching corner. A car entering a corner hot therefore paid the full
speed-based steering-effort penalty at exactly the moment it most needed
to commit to turn-in. (`_lookahead_yaw_rate_relax` already did the
equivalent relief for `Q[3,3]`/yaw-rate — its own docstring explicitly
names "turns late/slowly" as the failure mode it exists to prevent — but
no `R[0,0]` counterpart existed until this fix.)

Fix: added `_lookahead_steer_effort_relax(kappa_max_abs, car_speed,
floor=0.5, ...)`, mirroring `_lookahead_yaw_rate_relax`'s shape exactly
(same demand-normalised corner-severity curve), falling from `1.0` (no
corner ahead) toward `floor=0.5` as corner demand rises, composing
multiplicatively with `_adaptive_R_scaling` and
`_steer_effort_straight_boost`'s existing `R[0,0]` scalings. `floor=0.5`
was set to match `_lookahead_yaw_rate_relax`'s own default floor (same
magnitude as the sibling mechanism this was modelled on, not independently
fitted). Implemented in `mpc_core.py`/`mpc_params.py`
(`lookahead_steer_effort_relax_enabled`,
`adaptive_q_lookahead_steer_relax_floor`), mirrored in `model_utils.py`
(`lookahead_steer_effort_relax`) / `sim/rollout_core.py` / `settings.py`
(`LOOKAHEAD_STEER_EFFORT_RELAX_ENABLED`,
`ADAPTIVE_Q_LOOKAHEAD_STEER_RELAX_FLOOR`), and the `fsds_simulator` mirror.

## Addendum (2026-08-11): dynamic speed cap — closing the gap between the oracle profile and live tracking

`precomputed_speed_at()` (the oracle-profile lookup used when a track is
already mapped — `USE_PRECOMPUTED_SPEED_PROFILE=True` / `map_path` set) is a
static, position-indexed nearest-point lookup: it has no notion of the car's
*actual current speed* relative to how much runway is left to brake for the
upcoming corner.

Combined with the frozen-`e_v`-horizon characteristic described elsewhere in
this log (the MPC has no internal mechanism to anticipate a corner and ease
off early — it only ever tracks whatever `desired_speed` scalar it's handed
*this* tick), a car that enters a fast straight even slightly ahead of the
oracle profile's own pace has no predictive braking margin: the target speed
only starts dropping once the car reaches the position the profile
associates with braking, by which point there may not be enough distance
left.

This showed up directly in live telemetry
(`fsae_logs/mpc_standalone_control_*.csv`): at a corner entry, `v_actual`
was measured at ~9 m/s against a `v_desired` already at ~4.6 m/s, with
steering saturated at the 25° stop for over a second while speed caught up
to the target — i.e. the corner was recognised too late to brake for
smoothly.

Fix: `control_utils.dynamic_speed_cap()`, a thin wrapper over the
already-existing `curvature_speed()` (see that function's own docstring for
the full mechanism: it scans ~24 m of live path ahead, converts curvature to
a lateral-accel-limited corner speed, then propagates a braking-distance
constraint back from each corner), layered *underneath* the oracle lookup
rather than replacing it: every tick, when a track is mapped, the controller
takes `min(precomputed_speed_at(...), dynamic_speed_cap(...))`. The oracle
profile remains the trusted primary target (it encodes the whole lap's
raceline optimisation); the dynamic cap only ever pulls the target *down*
from that, catching the case where live tracking has drifted from the plan
(e.g. exiting the previous corner faster than expected).

Deliberately **not** a call to `curvature_speed()` with its own default
`a_lat_max=4.0`/`safety=1.0` (the values already tied to numeric parity with
the offline `use_planner=True` branch): the dynamic cap uses separate,
tighter defaults (`DYNAMIC_CAP_A_LAT_MAX=3.2`, `DYNAMIC_CAP_SAFETY=0.9`) so
it engages a little before the oracle profile would actually be violated,
rather than exactly at the edge — since it is a safety net under an
already-tuned profile, not a second opinion on the racing line.

Made tunable on/off (`enable_dynamic_speed_cap` ROS param /
`ENABLE_DYNAMIC_SPEED_CAP` in `settings.py`, both default `True`) so it
could be A/B'd against the oracle-profile-only baseline without code
changes — `ros2/launch_all.sh`'s MPC tuning shortlist got a commented-out
`ENABLE_DYNAMIC_SPEED_CAP=false` line for a one-off disable. With the flag
off, behaviour is byte-identical to before this change. Has no effect when
no track is mapped (the live-`curvature_speed()`-only branch already runs
predictive lookahead speed control with no separate oracle target to cap).

Downstream of this cap, `tracking_error_speed_gate()` and
`SPEED_TARGET_RISE_RATE` apply exactly as before — the dynamic cap only
changes what `v_curv` feeds into that existing pipeline, reusing it rather
than adding a second gate/rate-limiter.

**Measured offline, mixed result.** `python -m tuner.recorded_map_rollout
--planner` (the `USE_PLANNER=True` branch this cap actually runs in; the
default oracle-only `recorded_map_rollout` invocation never reaches this
code path at all, since it has no live centreline to scan) on
`comp_test_map_3`, cap default (`a_lat_max=3.2`, `safety=0.9`) vs.
`ENABLE_DYNAMIC_SPEED_CAP=False`:

| metric | cap off | cap on |
|---|---|---|
| steering sat % | 4.37 | **5.54** |
| `\|e_psi\|` mean / p90 (deg) | 7.04 / 15.94 | **8.01 / 16.82** |
| a_lat max | 10.06 | **10.61** |
| a_lat > ceiling % | 2.92 | **0.62** |
| score (lower better) | 0.503 | **0.520** |

The cap does what it's narrowly designed to do — `a_lat > ceiling %` drops
4.7x, confirming it's genuinely holding the car below the lateral-accel
ceiling more often. But steering saturation and heading error both got
**worse**, and the composite score regressed. This is the opposite of the
predicted effect on saturation and was not diagnosed at the time — a likely
candidate is interaction with `SPEED_TARGET_RISE_RATE`/the tracking-error
gate (braking earlier/harder for one corner may leave the car in a worse
heading position entering the next one), but this was never confirmed.
Left enabled by default in code (`enable_dynamic_speed_cap`/
`ENABLE_DYNAMIC_SPEED_CAP` default `True`) at this point because the
underlying mechanism (a real-time lookahead cap under a static oracle
profile with no notion of the car's actual current speed) is the one the
sim-to-real investigation identifies as structurally missing — but
`DYNAMIC_CAP_A_LAT_MAX`/`DYNAMIC_CAP_SAFETY` were flagged as unresolved
tuning, not validated defaults.

**Tested live same day: disabled again.** Subjectively performed worse on
the live car, consistent with the offline score regression above (0.503 →
0.520). `ros2/launch_all.sh` was updated to uncomment
`ENABLE_DYNAMIC_SPEED_CAP=false`, so a plain `launch_all.sh` run has the cap
off, overriding the code-level `True` default — the mechanism and its
launch-arg/YAML/settings.py plumbing were left in place (not reverted) for
whoever picks up the tuning next, but the cap should not be assumed on by
default in the repo's actual driving configuration; check
`launch_all.sh`'s shortlist first. Do not re-enable for a live run without
first understanding why it made steering saturation and heading error
worse, not just the a_lat ceiling metric it was targeted at.

## Addendum (2026-08-11): exit-heading boost was firing at the wrong time

**Fixed, small measured offline improvement, confirmed performing well on
the live car same day** (dynamic speed cap left OFF for this run — so this
result is attributable to the exit-boost fix alone). Reported symptom:
after exiting a corner, the car is sometimes left pointed slightly off the
path tangent, and accelerating out of the corner in that state produces
drift/slip.

Root cause: `_lookahead_exit_boost()`'s 5 m decay window (default
`MPCParams.adaptive_q_lookahead_exit_decay_dist`) is meant to boost `Q[2,2]`
(heading-error cost) right after the car passes a corner's peak curvature,
so the MPC works harder to straighten out on exit. Its decay clock
(`_dist_since_peak`) was reset by `_update_lookahead_peak()` off
`kappa_max_abs` — the **lookahead-window** peak curvature, detected the
moment a corner enters the speed-scaled scan window (10-17 m ahead of the
car at speed, per `MPCParams.adaptive_q_lookahead_time_s`/`_dist_max`).
That means the decay clock started counting down 10-17 m before the car
physically reached the corner. Verified on live telemetry
(`fsae_logs/mpc_standalone_control_1786436674.csv`): at the corner starting
around t=3.28s, `dist_since_peak` reset to 2.56 at the moment the lookahead
first saw rising curvature, then counted up continuously through the car's
actual apex (t≈5.2s, where current-position `kappa` itself peaked at 0.175
and heading error was worst) — by then `dist_since_peak` was already past
28 m, decades beyond the 5 m decay window. The exit-heading boost had
already decayed to a no-op (`1.0`, i.e. contributing nothing) before the
car was anywhere near its physical exit. This is not merely "a bit early" —
on a continuous chain of corners (curvature never drops back to the
`adaptive_q_lookahead_peak_hysteresis` re-arm threshold between them), the
detector effectively never gets a chance to fire again mid-sequence either,
per the `dist_since_peak` trace above.

Fix: `_update_lookahead_peak()` (`mpc_core.py`, mirrored
`model_utils.update_lookahead_peak()`) is now keyed on **current-position**
`kappa` (the same near-instantaneous, ~1 m preview curvature
`_adaptive_R_rate`/`_steer_rate_anti_hunt` already use) instead of
`kappa_max_abs`. `kappa` peaks at the car's own physical apex by
construction, so the decay clock — and therefore the exit-heading boost —
now actually covers the physical exit instead of having already elapsed
before it. The re-arm/hysteresis check was moved onto the same signal for
internal consistency (armed once the car itself is on a straight, not just
once the far lookahead window is clear). `_lookahead_exit_boost()` itself,
and its `k_exit_norm`/`boost_max` parameters, are unchanged — only the
timing of when its decay countdown starts.

Measured (`tuner.recorded_map_rollout --planner`, the `USE_PLANNER=True`
branch where this mechanism is actually exercised — see the dynamic-speed-cap
discussion elsewhere in this log for why the default oracle-only invocation
never reaches this code path at all):

| metric | before | after |
|---|---|---|
| steering sat % | 5.54 | **5.46** |
| `\|e_psi\|` mean / p90 (deg) | 8.01 / 16.82 | **7.60 / 15.74** |
| a_lat max | 10.61 | **10.55** |
| score (lower better) | 0.520 | **0.517** |

Small but consistently in the right direction on every metric offline
(unlike the dynamic speed cap and the abandoned heading-misalignment accel
gate below, both of which improved one metric while regressing others).
This is a timing bug fix to an existing, previously-inert mechanism, not a
new tunable — no new `MPCParams`/`settings.py` fields were added.

**Live test, 2026-08-11 (same day): confirmed performing well**, with
`ENABLE_DYNAMIC_SPEED_CAP=false` (that mechanism stayed disabled for this
run), so the improvement is attributable to this exit-boost timing fix
specifically, not a combination. No quantitative before/after live log pair
was captured for this change specifically (unlike the dynamic-speed-cap
A/B, which had matched control logs) — "performing well" is
qualitative/subjective from this run. The residual gap is still expected to
be non-zero: this fixes one specific mistiming, not the broader
reference-heading-lead issue flagged as open in the sim-to-real
investigation — re-read that section before assuming this closes the
corner-exit-misalignment complaint entirely.

**v2 fix, same day: the 5m decay window was still too short even after the
timing fix above.** After the `r_a` cut (0.85 → 0.77) made the car
accelerate harder out of corners, a live log
(`fsae_logs/mpc_standalone_control_1786443033.csv`) showed 10 stretches of
`|e_y| > 0.5` m with `m_Q_epsi_exit == 1.00` (i.e. NO boost applied) at
every single one — `dist_since_peak` at the moment `|e_y|` peaked was
11.7-20.6 m (mean ~15.7 m, one cluster of outliers at ~45 m attributable to
a different corner's peak already having re-armed), decades past the
now-correctly-timed 5 m window. Root cause: even keyed on the true apex,
`|e_y|`/`|e_psi|` don't peak instantaneously AT the apex — the car is still
sliding wide/yawing back through the exit for 1.5-2.7 s of travel
afterward, and at 5-8 m/s that is well over 5 m. The fixed decay window was
simply too short for how long a real exit disturbance actually takes to
play out, independent of the timing-origin bug already fixed.

Fix: `adaptive_q_lookahead_exit_decay_dist` is now a FLOOR, not the whole
story — the actual decay window used is
`car_speed * adaptive_q_lookahead_exit_decay_time_s` (default `2.5` s, fit
to the ~15.7 m mean from the cluster above), clamped to
`[adaptive_q_lookahead_exit_decay_dist, adaptive_q_lookahead_exit_decay_dist_max]`
(`5`-`25` m default) — same shape as the approach-side `lookahead_dist`
(`adaptive_q_lookahead_time_s`/`_dist_min`/`_dist_max`). Applied in
`mpc_core.py`'s `compute()` (computed at the `_lookahead_exit_boost` call
site, `_lookahead_exit_boost()` itself unchanged) and offline as three new
`adaptive_Q_lookahead()` kwargs (`exit_decay_dist_floor`/`_time_s`/`_max`)
threaded from `settings.py`'s new `ADAPTIVE_Q_LOOKAHEAD_EXIT_DECAY_DIST`/
`_TIME_S`/`_DIST_MAX` constants — mirrored to `fsds_simulator`.

While implementing this, found (but did NOT fix, out of scope) that offline
`adaptive_Q_lookahead()`'s call to `lookahead_exit_boost()` had never
threaded `k_exit_norm`/`boost_max` from `settings.py` either — it silently
used that function's own hardcoded defaults the whole time, unlike every
other lookahead call in the same function. Pre-existing drift, not
introduced by this change; flagging for whoever next touches this function.

Measured (`tuner.recorded_map_rollout --planner`, same track/config as the
v1 table above, now on top of the `r_a=0.77` cut): score 0.499→0.497, lap
time 55.15s→54.95s, steering sat 4.35%→4.37% (flat). Small further
improvement, no regression on the oracle-path baseline (0.408→0.409). Not
tested live.

**Investigated and explicitly rejected as a further fix for the same
symptom (2026-08-11):** raising `R[1,1]` (acceleration effort) and/or
relaxing `Q[4,4]` (`e_v`, speed-tracking urgency) when CURRENT `|e_psi|` is
large, so the MPC is reluctant to accelerate hard while still visibly
misaligned. Implemented in both `mpc_core.py`/`model_utils.py` as
`_epsi_misalignment_accel_gate`/`_epsi_misalignment_speed_relax` (`R[1,1]`
boost) and their `Q[4,4]` counterpart, gated by a new
`epsi_accel_gate_enabled` flag defaulted off. At the first-tried gains
(`boost_max=3.0`, `speed_floor=0.4`, `k_epsi=15.0`, half-effect ~3.8° of
`e_psi`), offline testing showed a clear regression, not an improvement:
steering sat 5.54%→7.70%, `|e_psi|` mean 8.01°→9.50°, score 0.520→0.660.
Reverted entirely (no trace of `epsi_accel_gate_*` remains in any file) —
this was not committed as a disabled feature the way the dynamic speed cap
was, because unlike that mechanism it showed no redeeming metric at all,
only a uniform regression. If revisiting a steering/accel exit penalty in
the future, do not re-derive this same current-|e_psi|-gated `R[1,1]`
approach without accounting for why it made things worse — a likely
candidate (never diagnosed) is that relaxing `Q[4,4]` let the car dawdle at
the wrong speed while the already-existing heading-correction machinery
(the exit-boost fix above, and the anti-hunt `boost_epsi` term) was doing
its own uncoordinated thing on the same `e_psi` signal.

(Note: this entire mechanism — `_lookahead_exit_boost`,
`_update_lookahead_peak`, `dist_since_peak`, and the
`adaptive_q_lookahead_exit_decay_*` family above — was later removed
altogether along with the rest of the lookahead gain-scheduling family and
replaced by the corner-factor scheduler; see
`planning_control_sync.md`'s "Corner-factor scheduler" section and
`removed_mechanisms.md`. This addendum is kept for the historical record of
the bug and its fix while the mechanism was still live.)

## Gradual-corner accel oscillation is genuine track geometry, not a bug

**No fix applied — confirmed correct behaviour.** Reported symptom: through
mild/gradual turns, the car accelerates, slows, accelerates, slows
repeatedly rather than holding a smooth speed or steady acceleration.

Traced on the same live log used for the exit-boost v2 fix (see the
Addendum above, `mpc_standalone_control_1786443033.csv`), t=39.7–42.9s:
`v_desired` genuinely oscillates (12.62 → 9.79 → 12.94 → 8.91 m/s over ~3s)
and `a_cmd` faithfully tracks it (+3.0 to −3.4 m/s² swings) — but this is
NOT the adaptive-gain machinery misbehaving. Cross-referenced against
`speed_profile.csv` (the precomputed oracle) at the car's actual position
(`tracks/comp_test_map_3/speed_profile.csv`, idx 656–680): `v_target` rises
smoothly to a local peak of 13.18 m/s then dips back to 11.79 m/s, tracking
a real change in the raw path geometry — Menger curvature computed directly
from the CSV's own `(x, y)` points crosses through ~0 in SIGN (not just
magnitude) at idx≈662-664, confirmed by checking the signed cross product
of consecutive segments. This is a genuine S-curve/chicane: a left-hand
bend straightens briefly, then curves right for the next bend. The
`kappa_max_abs`-driven lookahead correctly speeds the car up through the
brief straight and slows it back down anticipating the next corner — the
oscillation is the plan working as intended on a real varying-radius
feature, not spurious jitter to be filtered out.

Checked `a_lat_max` (`4.0`, `sim/speed_profile.py`'s
`compute_speed_profile()` default, well under the car's measured
~6.45-7.5 m/s² ceiling per the sim-to-real investigation) as a possible
source of excess conservatism at this specific corner, but did not raise
it: this would be a track-wide, not corner-specific, change, and the
sim-to-real investigation explicitly warns the measured ceiling is
speed-dependent and not to assume a tuning change closes that gap without
per-corner validation — raising `a_lat_max` broadly was flagged as a
follow-up to actually test, not applied here.

**Do not attempt to smooth this away via `R_rate_diag[1]`
(acceleration-rate-of-change cost) or similar** — that would make the MPC
slower to respond to a real, upcoming tightening corner, trading a
correctly-anticipated slowdown for a late, harder one. If the oscillation
"feels wrong" on a specific track, the right lever is the speed profile's
own generation parameters (`a_lat_max`, scan window) or the path geometry
itself (smoothing a genuinely spurious kink), never the live adaptive gains
— but confirm the geometry is actually spurious first, the way this
investigation did, rather than assuming it.

## Appendix — Low-speed steering-rate boost: full incident (added and disabled same day, 2026-08-12)

Referenced in passing from Part 1, item 3 above ("`_low_speed_steer_rate_boost`
— DISABLED... No-op currently"); this appendix records the full incident
that produced that disabled state.

**Added, live-tested, found to have an unwanted side effect, disabled by
default — code stayed in place at the time for a future rework** (it has
since been removed entirely along with the rest of the lookahead
gain-scheduling family — see `planning_control_sync.md`'s "Corner-factor
scheduler" section). Reported symptom: after exiting a corner at low speed
(3-4 m/s), steering swung through a large, fast, under-damped correction
while accelerating — confirmed on `mpc_standalone_control_1786483673.csv`,
t=6.9-7.7s: `steer_deg` swings +25° → -9° → 0° over ~1.5s while `a_cmd`
climbs 0 → 2.15 m/s², with `Rrate_steer_eff` essentially flat (~1.8-2.2)
throughout — neither `_adaptive_R_rate` nor `_steer_rate_anti_hunt` (both
gated on curvature/tracking-error, not speed) meaningfully reacted, since
the wobble's `kappa` was already small (car past the apex) by the time it
happened.

**Mechanism (as implemented)**: `_low_speed_steer_rate_boost(vx, ...)` —
INVERTED from Stanley's `k/(v+eps)` correction-gain shape (cheap correction
at low speed): this instead made steering-RATE changes MORE expensive at
low speed (`R_rate[0,0] *= 1 + (boost_max-1)/(1+k*vx)`, `boost_max=2.5,
k=0.35` → ~1.73× at 3 m/s, ~1.0× by race speed), on the theory that a fast
swing matters more when the car has little momentum to resist it. A literal
Stanley-shaped (cheap-at-low-speed) mechanism was explicitly considered and
rejected before implementing this one — see the mechanism note that was in
`mpc_core.py`'s `_low_speed_steer_rate_boost` docstring for why.

**Live-tested same day, found to regress turn-in**: because this gated
purely on speed with no curvature/lookahead signal, it could not distinguish
"post-exit overcorrection at low speed" (the case it was built for) from
"turn-in at low speed" (also low speed, also needs a fast steering-rate
change, but wanted) — live driving reported the car "struggling to turn
early in turns" after this was enabled, i.e. it suppressed the two cases
identically. **Disabled** (`low_speed_steer_rate_boost_enabled=False` in
both `mpc_params.py` and `settings.py`, plus the `fsds_simulator` mirror and
`fsae_params.yaml`) the same day. The function, its `MPCParams`/`settings.py`
fields, and its telemetry column (`m_Rrate_lowspeed`) were initially left in
place at their designed values (`boost_max=2.5, k=0.35`) rather than
removed, so a future lookahead-curvature-gated rework (fire only when NOT
approaching/inside a corner) would not need to re-derive the shape from
scratch — but the mechanism was ultimately removed outright along with the
rest of the lookahead gain-scheduling family when the corner-factor
scheduler replaced it (see `planning_control_sync.md`'s "Corner-factor
scheduler" section for that later removal and its rationale).

## Part 17 — Straight-line lateral-error snap-back was too sharp

Implemented on both sides (`adaptive_q_straight_ey_k` 20.0 → 8.0), not live-tested in isolation. Reported symptom, raised alongside the turn-in diagnosis in Part 1: the car sometimes enters a corner at the wrong lateral position relative to the planned path, as if it drifted off-line on the approach.

**Root cause**: `_lookahead_straight_lateral_reduce` softens `Q[0,0]` (lateral-error cost) to `ey_floor=0.7` on a clear straight, and previously snapped back to full weight very sharply (`k=20`, deliberately much sharper than the `k=8` shared by the `Q[2,2]`/`Q[3,3]` straight-line boosts) as soon as any curvature entered the lookahead window. That snap-back could still be incomplete by the time the car needed to be precisely positioned for turn-in, so the car could still be mid-recovery from the straight-line relaxation exactly when the corner arrived — entering already offset from the intended line rather than from the intended centreline point.

**Fix**: lowered `adaptive_q_straight_ey_k` from `20.0` to `8.0`, matching `adaptive_q_straight_k` (the `Q[2,2]`/`Q[3,3]` boosts' shared fade sharpness) — the straight-line relaxation benefit itself is unchanged (`ey_floor` still `0.7`), only the speed of the transition back to full lateral weight as a corner approaches.

Two alternative fixes were considered and rejected in favour of this one:
- Raising `ey_floor` — would reduce the straight-line-hunting benefit this mechanism exists for, and would change behaviour even far from any corner.
- Leaving `k` alone while addressing this some other way — not pursued.

Untested live in isolation — if straight-line hunting reappears after this change, that is the first thing to re-check, per this section and the `adaptive_q_straight_ey_k` field comment in `mpc_params.py`/`settings.py`.
