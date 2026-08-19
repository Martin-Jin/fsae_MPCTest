# Error-State Reference: How Tracking Error Is Calculated, by Hand

This is a **from-scratch, derive-it-yourself reference** for how both
controllers turn "the car is here, the path is there" into the numbers
(`e_y`, `e_psi`, `kappa`, ...) their cost functions actually penalise. It
assumes no prior MPC or vehicle-dynamics background.

**What this doc is not:** a system architecture overview (that's
[`architecture.md`](architecture.md), which explains *what each module does
and how they connect*), and not a project-status narrative (that's
[`junior_project_mpc_docs.md`](junior_project_mpc_docs.md), which covers the
same ideas at a higher level with less arithmetic). This doc exists so you
can take a concrete `(car position, car heading, path)` triple and reproduce
every downstream number by hand, with a worked numeric example at each step.

**Where the real code lives**, if you want to check this doc against ground
truth:
- Live LTV-QP controller: `MPCController._error_state()` in
  [`mpc_core.py`](../../ros2/src/fsae_planning/control/fsae_control/fsae_control/mpc_core.py)
  (lines ~833-945 as of this writing).
- Live NMPC controller: `PathReference.project()`/`_f()`/`_outputs()` in
  [`nmpc_core.py`](../../ros2/src/fsae_planning/control/fsae_control/fsae_control/nmpc_core.py).
- Offline plant-side equivalent: `vehicle_physics.plant_to_tracking_error()`.

---

## Table of contents

- [1. The absolute basics: what "error" means here](#1-the-absolute-basics-what-error-means-here)
- [2. LTV-QP (`mpc_core.py`) error calculation, step by step](#2-ltv-qp-mpc_corepy-error-calculation-step-by-step)
  - [2.1 Worked example](#21-worked-example)
  - [2.2 Is this current-error or forward-looking?](#22-is-this-current-error-or-forward-looking)
- [3. Why the LTV-QP can't just "re-project at every future step"](#3-why-the-ltv-qp-cant-just-re-project-at-every-future-step)
  - [3.1 Your instinct is basically right — here's the part that breaks it](#31-your-instinct-is-basically-right--heres-the-part-that-breaks-it)
  - [3.2 What was actually tried, and why it failed anyway](#32-what-was-actually-tried-and-why-it-failed-anyway)
- [4. NMPC (`nmpc_core.py`) error calculation, step by step](#4-nmpc-nmpc_corepy-error-calculation-step-by-step)
  - [4.1 Worked example](#41-worked-example)
  - [4.2 Why this structure survives where curvature-forcing didn't](#42-why-this-structure-survives-where-curvature-forcing-didnt)
- [5. Side-by-side summary](#5-side-by-side-summary)
- [6. Where each formula lives in code](#6-where-each-formula-lives-in-code)

---

## 1. The absolute basics: what "error" means here

Forget control theory for a second. You're driving, and there's a line
painted on the road (the racing line / planner's centreline) that you're
trying to follow. At any instant, two numbers describe how well you're
doing:

- **`e_y`** (lateral error): how far *sideways* you are from the line, in
  metres. Positive means one side, negative the other.
- **`e_psi`** (heading error): the angle between which way you're *pointing*
  and which way the line *points* at the nearest point, in radians (or
  degrees, for humans).

Everything else (rate of change of `e_y`, speed error, curvature) is a
straightforward derivative or lookup built on top of those two ideas. The
two controllers in this repo compute `e_y`/`e_psi` using the **same
geometric convention** (front-axle projection, perpendicular distance, wrapped
angle) — this is deliberate, so a number logged by one controller means the
same physical thing as the same-named number logged by the other. Where they
differ is *what reference direction* `e_psi` is measured against, and
*whether the prediction horizon knows the reference direction changes* — that
difference is the whole subject of [Section 3](#3-why-the-ltv-qp-cant-just-re-project-at-every-future-step).

---

## 2. LTV-QP (`mpc_core.py`) error calculation, step by step

Source: `MPCController._error_state()`.

**Step 1 — Find the front axle's position.**

The car's raw pose is usually given as a rear-axle reference point
(`car_pos`) plus a heading (`car_yaw`). Error is measured at the **front
axle** instead, since that's the end of the car that's actually steered:

```
front_axle = car_pos + lf * [cos(car_yaw), sin(car_yaw)]
```

`lf` is the distance from the car's centre of mass to the front axle (0.70 m
in this project's model).

**Step 2 — Find the nearest waypoint on the path.**

The path is a list of `(x, y)` waypoints. Compute the straight-line
(Euclidean) distance from the front axle to *every* waypoint, and take
whichever is smallest:

```
base_idx = argmin_i ( distance(front_axle, path[i]) )
```

This is a brute-force nearest-neighbour search — no cleverness, just "which
point is closest right now."

**Step 3 — Find the path's direction at that point (`path_yaw`).**

Take the *next* waypoint along the path from `base_idx`, subtract the
current one, and use `atan2` to turn that little vector into an angle:

```
segment = path[base_idx + 1] - path[base_idx]
path_yaw = atan2(segment.y, segment.x)
```

This is a plain two-point finite difference — no smoothing, no lookahead.
It only ever considers the *one* segment nearest the car.

**Step 4 — Project the offset onto "sideways to the path" (`e_y`).**

This is the step people most often get wrong by intuition, so it's worth
being explicit about *why* it isn't simply "the distance to the nearest
point."

```
dx, dy = front_axle - path[base_idx]
e_y = dy * cos(path_yaw) - dx * sin(path_yaw)
```

What this formula does: it takes the raw offset `(dx, dy)` and **rotates it
into the path's own local coordinate frame** — "how far along the path" and
"how far to the side of the path" — then keeps only the "to the side" part.
Rotating a vector `(dx, dy)` by `-path_yaw` and taking the resulting
y-component is exactly `dy·cos(path_yaw) - dx·sin(path_yaw)`; that's all
this line is.

**Why not just use the raw distance to the nearest point?** Because that
number conflates "sideways off the line" with "further down the line than
the nearest waypoint happens to be." A car that is 5 m further along a dead
straight path, but only 0.2 m sideways off it, has a Euclidean
nearest-point distance of `sqrt(5² + 0.2²) ≈ 5.004` — reporting the car as
5 metres off the racing line, when it's really almost exactly on it. The
rotation above throws away the "along the path" component (`dx·cos +
dy·sin`, not used here) and keeps only the perpendicular component, which is
what a lateral-error term is actually supposed to measure.

**Step 5 — Heading error (`e_psi`).**

```
e_psi = atan2( sin(car_yaw - path_yaw), cos(car_yaw - path_yaw) )
```

The naive version, `e_psi = car_yaw - path_yaw`, breaks the moment the two
angles straddle the ±180° wraparound (e.g. car at 179°, path at -179° — a
1° actual difference, but a naive subtraction reports 358°). Wrapping
through `atan2(sin(Δ), cos(Δ))` is the standard trick: it always returns the
equivalent angle in `(-180°, 180°]`, regardless of how the two raw angles
happened to be represented.

**Step 6 — Rate of lateral error (`e_y_dot`).**

```
e_y_dot = car_speed * sin(e_psi) + car_vy * cos(e_psi)
```

Plain English: if you're pointed slightly off from the path direction
(`e_psi ≠ 0`) while moving forward, your sideways position drifts at a rate
proportional to `sin(e_psi)` times your speed — the faster you go, or the
more misaligned you are, the faster `e_y` changes. The second term accounts
for genuine sideways slip (`car_vy`, body-frame lateral velocity) on top of
that.

**Step 7 — Speed error (`e_v`).**

```
e_v = car_speed - desired_speed
```

Just a plain difference; `desired_speed` comes from the planner or a
precomputed speed profile, low-pass filtered before this point so a sudden
step change in the request doesn't shock the controller.

**Step 8 — Curvature preview (`kappa`), used only for gain scheduling, not
the dynamics.**

Walk forward along the path from `base_idx` accumulating distance until you
hit roughly 1 metre, then estimate curvature there via a second
finite-difference (angle change between two adjacent segments, divided by
distance):

```
yaw_before = atan2(segment_before.y, segment_before.x)
yaw_after  = atan2(segment_after.y,  segment_after.x)
dpsi = wrap(yaw_after - yaw_before)
kappa = dpsi / average_segment_length
```

This number is **not** fed into the prediction model at all — it's only
used to retune the cost weights *for this tick's solve* (see
[architecture.md's adaptive gain scheduling section](architecture.md#adaptive-gain-scheduling-controllermodel_utilspy)).
Section 3 explains exactly why it can't also be used to make the *prediction*
curvature-aware.

**Step 9 — Assemble the 8-state vector.**

```
x0 = [ e_y, e_y_dot, e_psi, car_yaw_rate, e_v, 0.0, delta_act, a_act ]
```

`delta_act`/`a_act` (the actuator-lag states) aren't measured — they're
carried over from the controller's own memory of what it last commanded,
since a real steering rack/throttle doesn't reach a commanded value
instantly.

### 2.1 Worked example

Say the path near the car, in map coordinates, is a straight line running
due East:

```
path[10] = (10.0, 0.0)
path[11] = (10.5, 0.0)
```

The car's rear-axle position is `(9.35, 0.3)`, heading `car_yaw = 5°`
(slightly rotated counter-clockwise from due East), speed `car_speed = 8
m/s`, `car_vy = 0`, `desired_speed = 10 m/s`, `lf = 0.70`.

**Step 1 — front axle:**

```
front_axle = (9.35, 0.3) + 0.70 * (cos5°, sin5°)
           = (9.35, 0.3) + 0.70 * (0.9962, 0.0872)
           = (9.35 + 0.697, 0.3 + 0.061)
           = (10.047, 0.361)
```

**Step 2 — nearest waypoint:** by inspection, `path[10] = (10.0, 0.0)` is
closest (distance `sqrt(0.047² + 0.361²) ≈ 0.364`).

**Step 3 — path direction:**

```
segment = (10.5, 0.0) - (10.0, 0.0) = (0.5, 0.0)
path_yaw = atan2(0.0, 0.5) = 0°
```

**Step 4 — `e_y`:**

```
dx = 10.047 - 10.0 = 0.047
dy = 0.361  - 0.0  = 0.361
e_y = 0.361*cos(0°) - 0.047*sin(0°) = 0.361 - 0 = 0.361 m
```

Since the path is exactly due East here, `cos(0°)=1, sin(0°)=0`, so the
formula reduces to "just the `dy` component" — matching intuition, the car
is `0.361` m north of the line.

**Step 5 — `e_psi`:**

```
e_psi = wrap(5° - 0°) = 5° (0.0873 rad)
```

**Step 6 — `e_y_dot`:**

```
e_y_dot = 8 * sin(5°) + 0 * cos(5°) = 8 * 0.0872 = 0.698 m/s
```

The car is currently drifting further from the line at about 0.7 m/s,
because it's pointed slightly away from it while moving.

**Step 7 — `e_v`:**

```
e_v = 8 - 10 = -2 m/s   (2 m/s slower than target)
```

**Result:**

```
x0 = [0.361, 0.698, 0.0873, car_yaw_rate, -2.0, 0.0, delta_act, a_act]
```

You can now plug this into the cost function
(`Q_0*e_y² + Q_1*e_y_dot² + ...`, see
[`architecture.md`'s cost-function section](architecture.md#the-cost-function-and-qp-controlleroptimiserpy))
and reproduce exactly what the solver is penalising this tick.

### 2.2 Is this current-error or forward-looking?

**Entirely current-instant.** Every step above uses only `car_pos`,
`car_yaw`, `car_speed`, and the path geometry *right now*. Nothing about
where the path goes 2 seconds from now enters this calculation — the
horizon-forward prediction happens afterward, using this single `x0` as
the starting point and the linear model from
[architecture.md's "Building the prediction model" section](architecture.md#building-the-prediction-model-modelbicycle_modelpy).
That distinction — *one snapshot, then a linear rollout with no path-shape
awareness* — is exactly the limitation Section 3 explains.

---

## 3. Why the LTV-QP can't just "re-project at every future step"

A natural question once you understand Section 2: the horizon prediction
just applies a formula 35 times in a row to roll `x0` forward. Why not, at
each of those 35 steps, take the model's predicted `(x, y)` position, find
the nearest path point *there*, and recompute `e_y`/`e_psi` against that new
nearest point — exactly like Section 2 does at `t=0`, just repeated at every
future step too?

**Short answer: that idea is correct in spirit — it's a real fix, and it's
*conceptually* what the nonlinear controller in Section 4 actually does.**
The obstacle is purely about **how the solver works**, not whether the idea
is sound geometrically. The rest of this section explains exactly what
breaks (§3.1), then walks through what the team actually tried instead and
why even *that* still failed (§3.2) — both are worth reading, since §3.2's
failure is what motivates Section 4's specific design.

### 3.1 The part that breaks it: this search can't live inside a QP

The LTV-QP's speed comes from one specific trick: the relationship "if
you're in state `x` and apply input `u`, you move to state `x'`" is
expressed as a single fixed matrix multiplication, `x' = Ad·x + Bd·u`
(see [architecture.md](architecture.md#building-the-prediction-model-modelbicycle_modelpy)).
Because that relationship is *linear* (state times a fixed number, added
up — no state multiplied by another state, no branching, no lookups), the
solver (OSQP) can find the mathematically *provably best* answer among
millions of candidate steering sequences in 1-5 milliseconds.

"At each step, find the nearest point on the path" is a **search** — a
`min` over every waypoint — not a fixed formula. Two problems that causes:

1. **It isn't smooth.** As the predicted position moves, the nearest
   waypoint can suddenly *jump* from one point to a different, non-adjacent
   one (imagine the predicted position crossing the midpoint between two
   waypoints — the "nearest point" flips discontinuously). A fixed-multiplier
   linear model has no jumps like that anywhere; a solver built to exploit
   that smoothness has no way to represent one.
2. **It would need to happen 35 times per solve, for every one of the many
   candidate steering sequences the solver explores internally while
   searching for the optimum** — not once. Baking a live nearest-point
   search into the *dynamics themselves* destroys the fixed-matrix structure
   that makes OSQP fast in the first place; the problem stops being a
   Quadratic Program at all and becomes a much harder, generally much slower
   Nonlinear Program (see
   [architecture.md's linear-vs-nonlinear section](architecture.md#linear-vs-nonlinear-in-plain-english)).

So the honest answer to "why not just re-project every step" is: **you're
allowed to, but the moment you do, you're no longer solving a QP, you're
solving something structurally different** — which is exactly what Section 4
is. The LTV-QP specifically avoids this because staying a QP is what lets it
finish reliably inside the 50 ms tick budget.

### 3.2 What was actually tried, and why it failed anyway

Before building the full nonlinear controller, the team tried a
middle-ground fix that's worth understanding, because it explains *why*
Section 4's specific design (curvature as a **state**, not injected data) is
the part that actually works.

**The attempt ("curvature forcing"):** instead of a live per-step
nearest-point search, precompute curvature at each of the 35 horizon steps
*before* solving (assuming constant speed: `distance_k = v_x · k · dt`), and
add it as a fixed "nudge" term directly into the heading-error prediction at
each step:

```
predicted_e_psi[k+1] = (normal LTV-QP prediction) + w[k]
where  w[k] = -v_x * kappa(distance_k) * dt * gain
```

This is essentially your idea, but with the "search" replaced by a
**precomputed, fixed lookup** — done once, outside the solver, so the QP's
linear structure survives.

**Result: it made the car steer the wrong way first, then correct.**
Verified directly: with the car dead-centre and pointed correctly, 24 m
before a bend, adding this term produced steering that swung the *wrong*
direction for several consecutive ticks before turning the correct way.

**Why, mechanically:** the injected term `w[k]` is a **known fact about the
future** the moment the solver starts — it doesn't depend on what steering
sequence the solver actually picks. Since it's already "priced in"
regardless of the chosen inputs, the solver has complete freedom to decide
*when within its 35-step plan* to actually respond to it. If a brief
wrong-direction wiggle happens to produce a slightly lower total squared-cost
than committing immediately (because of how the cost trades off against
steering-rate penalties elsewhere in the horizon), the solver takes it — it
is mathematically optimizing the numbers it's given, and nothing in those
numbers *forces* early commitment, only *permits* it. Every variant of
"tell the solver about a future obligation as external data" tested this way
hit the same failure mode.

**Worked example, showing why the gain can't be fixed by tuning it.** Take
the car dead-centre and pointed correctly, `v_x = 15` m/s, approaching a
bend whose curvature 24 m ahead is `kappa = 0.05` (1/m). The `e_psi` state's
own natural decay in the linear model is `Ad[2,2] ≈ 0.946` per 0.05 s tick
(i.e. any nonzero `e_psi` shrinks by about 5.4% every tick even with zero
input) — that decay is fighting the forcing term every step.

- **At `gain = 1.0`** (the physically exact value, from `path_yaw_rate =
  v_x·κ`): the injected nudge per tick is roughly
  `w = -v_x·kappa·dt·gain = -15·0.05·0.05·1.0 ≈ -0.0375` rad/step
  (≈ -2.15°/step) — but the QP's own decay bleeds off about 94.6% of
  whatever accumulates from the *previous* tick before this tick's nudge
  even lands. Net effect measured on this test: resulting commanded
  steering under 1°, indistinguishable from ordinary solver noise. **Too
  weak to produce any real anticipation.**
- **Raise `gain` to compensate (say `gain ≈ 6`)**: now the nudge is large
  enough to matter, but it also makes the *cheapest total path* through the
  QP's cost landscape change character. Picture the cost as a landscape the
  solver is finding the lowest point of: a large forcing term partway
  through the horizon can make a brief dip in the *wrong* direction earlier
  in the horizon (cheap in steering-rate terms, since it's a small early
  correction) followed by a bigger *correct*-direction swing later, sum to
  a lower total squared-cost than committing to the correct direction
  immediately. This was confirmed directly: at this gain, the solver's
  chosen trajectory steers measurably away from the corner for the first
  few steps before reversing.
- **Push `gain` to ≈ 20** to force early correct-direction commitment to
  win outright: by this point the forcing term is so large it saturates
  steering at the mechanical limit for the *entire* horizon regardless of
  actual tracking error, which defeats the purpose (the controller is now
  just always turning at maximum, not responding to the actual corner).

There is no gain in between that is simultaneously large enough to produce
meaningful anticipation and free of the wrong-direction transient — this
isn't a case of "try harder to tune it," it's the geometry of the cost
landscape itself changing shape as gain increases, which is exactly the
structural property described above (the solver optimizes whatever total
cost the numbers describe, and for a wide middle range of gains, a
wrong-direction dip is genuinely cheaper).

**The fix that actually worked:** make the curvature obligation impossible
to defer, by making it a **structural consequence of the car's own predicted
motion**, not a fact injected from outside. That's Section 4.

---

## 4. NMPC (`nmpc_core.py`) error calculation, step by step

Source: `PathReference` class + `_f()` in `nmpc_core.py`.

The key structural change: the state vector gains a new entry, **`s`** — arc
length travelled along the path — and curvature is looked up **at whatever
`s` the model currently predicts**, not at a fixed step index. Since `s`
itself evolves as a normal state (driven by the car's own predicted speed
and heading), a bend at some future distance along the path is automatically
"there" the instant the horizon reaches it — there's no separate slot for
the solver to defer paying into.

State vector: `x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`.

**Step 1 — Build a smooth, continuous description of the path (once, not
per-tick, for a static path).**

Rather than reading curvature from raw waypoint-to-waypoint differences
(Section 2, Step 8's approach), fit two independent cubic splines through
the waypoints, one for `x(s)` and one for `y(s)`, parameterised by
cumulative arc length `s` along the path:

```
arc[0] = 0
arc[i] = arc[i-1] + distance(path[i-1], path[i])     for i = 1..n-1

x_spline = CubicSpline(arc, path.x)
y_spline = CubicSpline(arc, path.y)
```

A cubic spline is a smooth curve that passes exactly through every waypoint
while keeping its first and second derivatives continuous — which matters
here because curvature *is* a second derivative (see Step 2). This
replaces the raw two-point differencing Section 2 uses, because differencing
raw waypoints makes curvature "step" abruptly between waypoints in a way
that doesn't reflect the path's true smooth shape.

**Step 2 — Curvature and reference heading, as continuous functions of `s`.**

Standard calculus result for a parametric curve `(x(s), y(s))`:

```
psi_ref(s) = atan2( y'(s), x'(s) )

kappa(s) = ( x'(s)*y''(s) - y'(s)*x''(s) ) / ( x'(s)² + y'(s)² )^1.5
```

`x'`, `y'`, `x''`, `y''` are the spline's own analytic derivatives (a cubic
spline's derivatives are themselves simple polynomials, so this is exact,
not a numerical approximation). Plain English: `psi_ref(s)` is "which
direction is the path pointing at distance `s` along it," and `kappa(s)` is
"how sharply is it curving there" — both evaluated from the *same* smooth
curve, which matters for the reason in the callout below.

> **Why `psi_ref` and `kappa` must come from the same smoothed curve.**
> Early testing measured `e_psi` against the *raw* segment-to-segment
> tangent (Section 2's method) while curvature came from the new smooth
> spline. That combination produced a **period-2 steering oscillation**
> (commanded steering alternating roughly +25°, then -25°, every tick,
> indefinitely) through tight corners. Cause: raw waypoint tangents jump in
> discrete steps of about `segment_length / corner_radius` — 5.7° per 0.5 m
> waypoint spacing on a 5 m-radius hairpin — and the controller read every
> one of those artificial steps as real heading error to correct within a
> single tick. Deriving `e_psi`'s reference from the *same* smoothed spline
> that produces `kappa(s)` removed the artificial steps entirely.

**Step 3 — Project the car onto the path (once, at `t=0`, same idea as
Section 2 Steps 1-5, with two refinements).**

```
front_axle = car_pos + lf * [cos(car_yaw), sin(car_yaw)]
base_idx = argmin_i ( distance(front_axle, path[i]) )
s_base = arc[base_idx]
path_yaw = psi_ref(s_base)                     # from the spline, not raw segment

dx, dy = front_axle - path[base_idx]
e_y = dy*cos(path_yaw) - dx*sin(path_yaw)       # identical formula to Section 2, Step 4
along = dx*cos(path_yaw) + dy*sin(path_yaw)     # the "along the path" component, discarded in Section 2
s0 = s_base + along                              # refine WHICH arc-length station the car is actually at

path_yaw = psi_ref(s0)                           # re-evaluate heading at the REFINED station
e_psi = wrap(car_yaw - path_yaw)
```

Two differences from Section 2, both small but deliberate:
1. `path_yaw` comes from the spline (`psi_ref`), not a raw two-point
   difference — this is what avoids the period-2 oscillation above.
2. The "along the path" component (`along`), which Section 2 computes but
   throws away, is kept here and used to refine `s0` — the arc-length
   station — beyond just "whichever waypoint happened to be nearest." On a
   tight corner this correction can be a metre or more, over which the
   reference heading genuinely changes, so the heading is re-evaluated a
   second time at the refined `s0`.

**Step 4 — How error *evolves* over the horizon (this is the actual fix).**

This is the part with no equivalent in Section 2 at all — Section 2's model
has no term describing how `e_psi` changes due to the path itself curving.
Here, at every predicted stage of the horizon, given whatever `(s, e_y,
e_psi, v_x, v_y, r)` the model currently predicts:

```
kap = kappa(s)                                    # curvature AT THE PREDICTED s
denom = 1 - kap * e_y                             # Frenet-frame distortion term, see note below
s_dot     = (v_x*cos(e_psi) - v_y*sin(e_psi)) / denom
e_y_dot   = v_x*sin(e_psi) + v_y*cos(e_psi)        # same form as Section 2 Step 6
e_psi_dot = r - kap * s_dot                        # <-- the term Section 2's model is entirely missing
```

Plain English for each line:
- `s_dot`: how fast the car is advancing *along the path* (not just through
  the world) — this needs the `1/denom` correction because "distance along
  a curving path" and "distance in a straight line" aren't quite the same
  thing once you're offset from the centreline (a car on the inside of a
  bend covers less arc-length per metre travelled than one on the outside;
  `denom` captures exactly that geometric effect, and is clamped away from
  zero to avoid dividing by zero on an extreme case).
- `e_y_dot`: identical idea to Section 2's version.
- `e_psi_dot`: **your heading error changes at a rate equal to your own yaw
  rate, MINUS how fast the path itself is turning underneath you
  (`kap * s_dot`).** This is the literal mathematical statement of "the
  road is bending." Section 2's model effectively assumes `kap = 0` always,
  which is exactly true on a straight and increasingly wrong the sharper the
  corner.

Because `s` is a **state that the rollout itself predicts forward** (via
`s_dot`, which depends on the car's own predicted speed/heading), `kappa(s)`
automatically changes as the horizon advances — a bend 10 steps ahead is
already shaping today's plan, with no separate signal that needs to be
"paid for later." This is the structural difference from Section 3.2's
failed curvature-forcing attempt: there, `w[k]` was a fixed number computed
before the solve and handed to the solver as data; here, `kappa(s)` is
looked up **inside** the same equation the solver is actively solving,
using a state (`s`) the solver is actively choosing.

### 4.1 Worked example

Take a gentle constant-curvature left bend, radius 20 m, so `kappa = 1/20 =
0.05` (1/m) everywhere on it, starting at arc length `s = 50` m. Car is
exactly on the bend's start, `s = 50`, `e_y = 0`, `e_psi = 0`, driving at
`v_x = 15` m/s, `v_y = 0`, current yaw rate `r = 0` (not yet turning).

**At this instant (`t=0` in the horizon):**

```
kap = kappa(50) = 0.05
denom = 1 - 0.05*0 = 1.0
s_dot = (15*cos(0) - 0*sin(0)) / 1.0 = 15 m/s
e_y_dot = 15*sin(0) + 0*cos(0) = 0
e_psi_dot = 0 - 0.05*15 = -0.75 rad/s
```

Compare to Section 2's model at the same instant: `e_psi_dot = r = 0`
(unchanged, forever, since nothing there depends on `kappa`).

**The NMPC's rollout predicts `e_psi` will start becoming nonzero
immediately** — specifically, drifting at -0.75 rad/s (about -43°/s) the
very first instant, purely because the path is curving underneath the car,
even though the car hasn't turned its wheels yet. That's the entire
mechanism: the model tells the solver "if you do nothing, you're about to
develop heading error," and the solver reacts to that *predicted* error the
same way it reacts to *real* error today, using the normal cost function —
no bolted-on lookahead heuristic required.

A few steps later (say the rollout has driven forward ~0.35 s, advancing `s`
to roughly `50 + 15*0.35 ≈ 55.25` and picking up some real `e_psi` and
countering yaw rate `r` from the QP's own chosen steering), you'd plug the
*new* `s`, `e_y`, `e_psi`, `v_x`, `v_y`, `r` back into the same formulas
above — this is a rollout, so each step's output becomes the next step's
input, exactly like Section 2's `x' = Ad·x + Bd·u`, except this update rule
is nonlinear (it multiplies `kap` — itself a function of the state `s` — by
`s_dot`, another state-dependent quantity, so it isn't "state times a fixed
number" any more; see [architecture.md's linear-vs-nonlinear section](architecture.md#linear-vs-nonlinear-in-plain-english)
for what that distinction costs computationally).

### 4.2 Why this structure survives where curvature-forcing didn't

Directly contrasting with Section 3.2:

| | Curvature forcing (failed) | NMPC (works) |
|---|---|---|
| How curvature enters | A fixed number, `w[k]`, computed once before the solve and added to the prediction as external data | `kappa(s)`, looked up **inside** the dynamics equation using `s`, a state the solver is actively predicting |
| Can the solver defer it? | Yes — `w[k]` is true no matter what input sequence gets chosen, so paying for it "now" vs "later in the plan" is a free choice | No — `e_psi_dot` at a given predicted moment mechanically depends on wherever `s` has gotten to by that moment; there's no separate slot to schedule around |
| Measured failure mode | Brief wrong-direction ("wrong way then correct") steering, ~7 consecutive ticks at comparable-to-peak magnitude | A single, negligible wrong-direction step (~-0.33°, confirmed as the true mathematical optimum, not a solver bug) against a real correction peak of +4.9° |

---

## 5. Side-by-side summary

| | **LTV-QP** (`mpc_core.py`) | **NMPC** (`nmpc_core.py`) |
|---|---|---|
| When is error computed? | Once, at `t=0`, from the measured pose | Once at `t=0` for the *initial* state, then **continuously re-derived** at every horizon step as `s`, `e_y`, `e_psi` evolve |
| Reference heading source | Raw two-point segment tangent | Analytic cubic-spline fit, evaluated at the (refined) arc-length station |
| Does the *prediction* know the path curves? | No — `e_psi_dot = r` only | Yes — `e_psi_dot = r - kappa(s)*s_dot` |
| How is curvature used at all? | Only to retune cost weights for *this* tick's solve (gain scheduling) — never enters the predicted dynamics | Enters the predicted dynamics directly, every step |
| Structural class of problem | Convex Quadratic Program (QP) — one solve, guaranteed global optimum, ~1-5 ms | Sequence of QPs via Gauss-Newton SQP — no guaranteed global optimum, ~9 ms mean |
| Consequence of the difference | Needs adaptive gain-scheduling machinery to approximate anticipation it structurally can't have; still turns in late on sharp/sudden corners | No corner-anticipation machinery needed; the whole adaptive gain-schedule family is inactive/inapplicable |

---

## 6. Where each formula lives in code

| Formula | Live code | Offline equivalent |
|---|---|---|
| Front-axle projection, `e_y`/`e_psi` (LTV-QP) | `mpc_core.MPCController._error_state()` | `model/vehicle_physics.plant_to_tracking_error()` |
| Curvature preview (gain-scheduling only) | `mpc_core._curvature()` | `controller/model_utils.py`'s equivalent |
| Linear bicycle model (`Ad`/`Bd`) | `mpc_core.MPCController._discrete_model()` | `model/bicycle_model.get_8state_discrete_model()` |
| `PathReference` (spline `kappa(s)`/`psi_ref(s)`) | `nmpc_core.PathReference` | `controller/nmpc_optimiser.py`'s port |
| NMPC dynamics (`_f`, includes `e_psi_dot = r - kappa*s_dot`) | `nmpc_core._f()` / `_f_scalar()` | `controller/nmpc_optimiser.py`'s port |
| NMPC cost-facing outputs (`_outputs`) | `nmpc_core._outputs()` | `controller/nmpc_optimiser.py`'s port |
| The rejected curvature-forcing attempt (Section 3.2) | historical, see `planning_control_sync.md`'s "Curvature-forcing term" section | `controller/model_utils.curvature_horizon_profile()` (historical) |

For the cost function itself (`Q`/`R`/`R_rate`, how these error terms turn
into a single number to minimise), see
[`architecture.md`'s "The cost function and QP" section](architecture.md#the-cost-function-and-qp-controlleroptimiserpy)
or the more plain-English
[`junior_project_mpc_docs.md` Section 1.4](junior_project_mpc_docs.md#14-the-cost-function) —
not repeated here since that part is identical in spirit for both
controllers and already well covered.
