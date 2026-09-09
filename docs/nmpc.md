# The Nonlinear MPC Controller (NMPC)

Full technical reference for the second, separately selectable controller,
`nmpc_core.NMPCController`, chosen by the node parameter `use_nmpc` (default
false). Split out of `architecture.md` because this material is large enough
to be its own document; that file now only summarises and links here.

For the default linear controller, see [`lmpc.md`](lmpc.md). For the
worked-by-hand arithmetic behind `e_y`/`e_psi`, see
[`error_state_reference.md`](error_state_reference.md).

Everything in [`lmpc.md`](lmpc.md) describes `mpc_core.MPCController`:
a linear time-varying MPC solved as one convex QP per tick. The
live workspace carries a **second, separately selectable** controller,
`nmpc_core.NMPCController`, described here. This repo has its own offline
port, `controller/nmpc_optimiser.py`,
selected by `settings.USE_NMPC`; this doc is a pointer to the live design,
not a mirror of the offline code, see `docs/reference/control_mechanisms.md`'s
"Nonlinear MPC (`use_nmpc`)" section for the offline port's specifics.

## Table of Contents

1. [The structural difference, in one line](#the-structural-difference-in-one-line)
2. [Structure and solve method, in brief](#structure-and-solve-method-in-brief)
3. [The state vector and the Frenet metric factor](#the-state-vector-and-the-frenet-metric-factor)
4. [The nonlinear model (`_f`)](#the-nonlinear-model-_f)
5. [Linearising the rollout: finite-difference Jacobians](#linearising-the-rollout-finite-difference-jacobians)
6. [Condensing and the QP](#condensing-and-the-qp)
7. [Testing the math](#testing-the-math)
8. [Feature comparison: LTV-QP vs. NMPC, at a glance](#feature-comparison-ltv-qp-vs-nmpc-at-a-glance)

---

## The structural difference, in one line

In plain terms: the LTV-QP
plans ahead as though the road stays pointed the same direction for the
whole horizon, even if a corner is coming up; the NMPC's internal model
actually knows the road bends, and where. The LTV-QP predicts how the car's
current error (`e_y`, `e_psi`) drifts under its own dynamics, against a
reference direction it treats as fixed for the whole horizon. The NMPC
predicts that same error's evolution **relative to a path whose bend is
itself part of the prediction**, the model knows the reference direction
changes with `s`, not just the car's state.

**Both controllers measure their current-tick error the same way**: the
Frenet-frame projection described in
[`lmpc.md`'s "How the error vector is measured"](lmpc.md#how-the-error-vector-is-measured-frenet-frame-projection)
(`_error_state()` in `mpc_core.py`, `PathReference.project()` in
`nmpc_core.py`, same nearest-point-plus-perpendicular-offset arithmetic).
Frenet-frame measurement is not what tells them apart. What differs is
what happens to that error **over the prediction horizon**, after this
tick's measurement:

The LTV-QP takes its one Frenet measurement at the current tick, then
predicts forward in fixed error coordinates with the reference frame's
rotation dropped, so `e_psi_dot = r` instead of
`e_psi_dot = r - kappa(s)*s_dot`. Arc length `s` never appears as a
predicted state, curvature is sampled once (the ~1 m preview lookup in
`_error_state()`) and held fixed for the whole horizon. With the car on
line and a corner ahead, its 35-step rollout predicts staying on line
forever (measured: exactly 0.000 deg commanded at 8 dead-on-line states),
which is why the "structural limit" callout in `removed_mechanisms.md`
exists and why the adaptive lookahead layer had to be invented.

The NMPC instead carries `s` itself as a horizon *state*: at every one of
its 20 predicted steps, `kappa(s)`/`psi_ref(s)` are looked up fresh at that
step's predicted `s`, not sampled once at the current tick. So the road's
bend is re-evaluated at every future point along the plan, not frozen at
one lookahead distance the way the LTV-QP's preview curvature is. As `s`
advances along the predicted horizon, `kappa(s)` changes with it, so a
bend 10 steps out is already shaping the plan today, not just once the
car arrives there.

## Structure and solve method, in brief

**Structure**: states `[s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`, inputs
`[delta_cmd, a_cmd]`, linear-tyre bicycle dynamics with the same constants and
the same low-speed kinematic blend as the LTV-QP, plus a `tanh` saturation of
the predicted lateral force at FSDS's measured `a_lat` ceiling.

**How it's solved (Gauss-Newton SQP)**, step by step each tick, in one line
each (the full derivation of every step is below):

1. **Roll the nonlinear model forward** from the car's actually-measured
   state, not an approximation, using the real nonlinear equations (see
   ["The nonlinear model"](#the-nonlinear-model-_f) below).
2. **Linearise around that rollout**: compute how a small change in each
   input would change the predicted trajectory, via finite-difference
   Jacobians (see ["Linearising the rollout"](#linearising-the-rollout-finite-difference-jacobians)
   below).
3. **Condense into a QP**: fold the whole 20-step problem down into one
   solved for input *changes* only (see ["Condensing and the QP"](#condensing-and-the-qp)
   below).
4. **Solve with a trust region**: cap how large a step OSQP is allowed to
   take from this tick's rollout, since the linearisation from step 2 is
   only accurate near it.
5. **One iteration per tick, warm-started from last tick's answer**
   ("real-time iteration"), rather than looping steps 1-4 until full
   convergence within a single tick, which would risk missing the 50 ms
   deadline.

Horizon 20 steps (1.0 s); measured solve time mean 8.9 ms, p95 11.6 ms.

**Consequences for the rest of the architecture**: when `use_nmpc=true` the
entire adaptive gain schedule, the precomputed corner map and the shaped
heading-lead profile are all inactive (each was a workaround for the missing
curvature term), and the telemetry CSV's `m_*` columns are empty while eight
`nmpc_*` columns carry solver/prediction diagnostics instead. The composite
score, the scoring pipeline, the path/speed-profile plumbing and the delay
compensation are unchanged.

## The state vector and the Frenet metric factor

Recall the 8-state vector and 2-input command vector:

```
x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]ᵀ
u = [delta_cmd, a_cmd]ᵀ
```

The first three states are exactly the Frenet quantities described in
[`lmpc.md`'s "How the error vector is measured"](lmpc.md#how-the-error-vector-is-measured-frenet-frame-projection):
arc length along the path (`s`), lateral offset from it (`e_y`), and heading
error against its tangent (`e_psi`). The remaining five are the same kind of
physical state the LTV-QP tracks (speed, lateral velocity, yaw rate, and the
two lagged-actuator states), just expressed once (as true quantities, not
errors against a frozen target) rather than duplicated as both a raw state
and an error state.

**Why `s` is a state here and not in the LTV-QP.** The LTV-QP's error
coordinates implicitly assume the reference frame itself doesn't rotate
under the prediction (see `lmpc.md`). Carrying `s` explicitly is what lets
`kappa(s)` and `psi_ref(s)` be looked up **fresh at the predicted `s` of
every horizon stage**, instead of being sampled once at the current tick and
held fixed. This is the mechanism behind the "structural difference"
described above.

**The kinematics of moving along a curved reference: the metric factor
`1 - kappa*e_y`.** Converting straight-line, global-frame motion into
"progress along a curving path" isn't a plain unit conversion, because
a point offset to one side of a bend covers a different arc length than a
point on the bend itself for the same physical displacement (walk the
inside of a curved corridor and you cover less ground than someone walking
its outside edge, for the same number of steps forward). This is exactly the
same idea `kappa` and `e_y` describe elsewhere in this stack, applied to the
*rate* of `s` rather than to a single measurement. The exact relationship
(a standard result in Frenet-frame vehicle models) is:

```
s_dot = (v_x*cos(e_psi) - v_y*sin(e_psi)) / (1 - kappa(s)*e_y)
```

The numerator is just the car's forward-progress speed resolved into the
path-tangent direction (a car facing away from the tangent, `e_psi != 0`, or
carrying sideways velocity `v_y`, doesn't turn all of its speed into
progress along the path). The denominator, `1 - kappa*e_y`, is the metric
factor above: on a straight (`kappa = 0`) it's exactly `1` regardless of
`e_y` and `s_dot` is just the ordinary forward speed; in a corner, being
offset toward the inside (`e_y` and `kappa` the same sign) makes the
denominator less than 1, so `s_dot` is *larger* than the raw forward speed
for the same physical motion, because the car is covering the same physical
ground while advancing further along a shorter (inside) arc. In code
(`nmpc_core.py`'s `_f()`):

```python
kap = ref.kappa_at(s)
denom = 1.0 - kap * e_y
s_dot   = (v_x * cos_ep - v_y * sin_ep) / denom
e_y_dot = v_x * sin_ep + v_y * cos_ep
e_psi_dot = r - kap * s_dot
```

`e_y_dot` needs no metric correction (a lateral offset is measured
perpendicular to the path, which is a locally flat direction regardless of
curvature). `e_psi_dot`, the heading-error rate, is yaw rate `r` **minus**
how fast the reference direction itself is rotating as the car advances
along it (`kappa(s) * s_dot`, curvature times progress rate is the standard
identity `dpsi_ref/dt = kappa * ds/dt`). This is the exact term the LTV-QP's
fixed-frame prediction drops (see `lmpc.md`), reintroduced here because `s`
and hence `kappa(s)` are now genuinely time-varying predicted quantities,
not one frozen sample.

**The denominator is floored, not left to blow up or flip sign**
(`_DENOM_FLOOR = 0.25` in code): if a prediction step ever puts `e_y` far
enough to the "inside" that `1 - kappa*e_y` approaches zero, the true
physical picture is the car is nearly orbiting a point (`s_dot` genuinely
diverges), which is not a regime the SQP's linearisation should be asked to
represent. The floor keeps `denom` bounded away from zero **without ever
flipping its sign**, since a sign flip would reverse the predicted direction
of travel along the path, a far more misleading failure than a merely
under-estimated `s_dot`.

## The nonlinear model (`_f`)

This is the same purpose as [`lmpc.md`'s "Building the prediction
model"](lmpc.md#building-the-prediction-model-modelbicycle_modelpy): "given
the current state `x` and a chosen input `u`, what is the state's
instantaneous rate of change?" The difference is that here the answer is
genuinely **nonlinear** (curvature, trig terms and a smooth saturation all
depend on the current state itself), so it's written directly as `ẋ = f(x,
u)`, a function, rather than reduced to a fixed matrix `ẋ = A·x + B·u`.

### Tyre forces and the lateral-acceleration ceiling

The lateral tyre forces use the same linear-tyre slip-angle model as the
LTV-QP's cornering-stiffness terms in `lmpc.md`, just written per-axle
rather than folded into the `A_dyn` matrix:

```
alpha_f = atan((v_y + lf*r) / v_x) - delta_act      (front slip angle)
alpha_r = atan((v_y - lr*r) / v_x)                  (rear slip angle)
F_yf = -2*Cf*alpha_f
F_yr = -2*Cr*alpha_r
```

`v_x` in these two `atan()` terms is floored at `v_blend_hi` (2.5 m/s)
purely to keep the slip-angle expression finite as the car approaches a
standstill; the low-speed kinematic blend below is what actually governs
behaviour down there, not this floor.

**Why the linear-tyre force is then saturated with a `tanh`.** A linear
tyre model has no upper bound on lateral force: double the slip angle,
double the force, forever. FSDS's actual car does not behave this way (see
CLAUDE.md's "The offline sim does not yet fully predict the car": a
measured, sustained lateral-acceleration ceiling of roughly 7.5 m/s²,
mildly speed-dependent). Without representing that ceiling **inside the
prediction itself**, the NMPC's internal model believes it can hold any
corner at any speed; when the real car can't keep up, heading error grows,
the solver demands even more force for an already-saturated tyre, and the
error persists or grows rather than resolving, a genuine offline failure
mode (measured spin, see `late_turn_in_investigation.md` Part 16 §16.6).

```
a_y = (F_yf*cos(delta_act) + F_yr) / m
ceil = max(alat_ceiling_flat, alat_ceiling_slope*|v_x| + alat_ceiling_intercept)
ratio = |a_y| / ceil
sat = tanh(ratio) / ratio          (both axle forces scaled by this factor)
```

`tanh(x)/x -> 1` as `x -> 0` (to second order), so at small `ratio` (well
inside the ceiling) `sat ≈ 1` and the linear-tyre force is returned
unchanged, exactly the regime the cornering-stiffness weights were tuned
against. As `ratio` grows past 1, `tanh` saturates and `sat` shrinks,
smoothly bending the force curve over rather than clipping it with a hard
corner (a hard clip would make the model's sensitivity to steering
discontinuously jump to zero right at the bound, which is a poor thing to
linearise around). Both axle forces are scaled by the *same* factor, which
preserves the front/rear force **ratio** (and hence the model's understeer
character) while limiting the overall magnitude.

### The kinematic/dynamic blend, and why the tyre force itself must be blended out

The same low-speed blend as the LTV-QP (`lmpc.md` §"Blending kinematic and
dynamic models"), same breakpoints (`v_blend_lo = 1.0`, `v_blend_hi = 2.5`):

```
blend = clip((v_x - v_blend_lo) / (v_blend_hi - v_blend_lo), 0, 1)
```

`blend = 0` below 1 m/s (pure kinematic), `blend = 1` above 2.5 m/s (pure
dynamic), linearly interpolated between. Unlike the LTV-QP, where blending
only needs to combine two already-computed matrices, here the **tyre force
itself** has to be scaled by `blend` before it enters the dynamic-branch
equations below, not just the dynamic branch's final output:

```python
F_yf = F_yf * blend
F_yr = F_yr * blend
```

**Why this matters, precisely** (a real bug found and fixed, see the code
comment in `_f`): the slip-angle expressions above use a speed-floored
denominator (`v_safe`) purely to avoid dividing by zero as `v_x -> 0`. Left
otherwise unguarded, that floored denominator makes a *stationary* car's
computed slip angle track the steering command almost directly
(`alpha_f ≈ -delta_act` when `v_y`, `r` are both small), so the linear-tyre
formula predicts a large cornering force from steering alone even though a
real tyre with no rolling contact velocity generates approximately zero
force. That fictitious force would otherwise propagate through
`v_y_dot`/`r_dot` into a large, entirely imaginary predicted `e_y`/`e_psi`
excursion over the horizon while the car has not physically moved. Measured
effect before the fix: the NMPC's steering command snapped to the full
±25° mechanical lock in the first 0.5-0.7 s of every run from a standing
start, with the predicted `e_y` at the end of the horizon reaching -1.9 m
while the car's actual speed was still ~0. Scaling `blend` into the force
itself, at the source, removes the fictitious force before it can
contribute to anything downstream, rather than trying to blend away its
consequences after the fact (which is too late: the dynamic branch's
*output* is blended out downstream too, but by then the force has already
been computed and would still leak in through the intermediate terms
below).

### Assembling the state derivatives

Dynamic branch (used once `blend > 0`):

```
v_x_dot     = a_act + blend * r * v_y
v_y_dot_dyn = (F_yf*cos(delta_act) + F_yr) / m  -  r * v_x
r_dot_dyn   = (lf*F_yf*cos(delta_act) - lr*F_yr) / Iz
```

These are the same Newton's-law relationships as `lmpc.md`'s dynamic model
(`A_dyn`), just evaluated at the actual current nonlinear force rather than
a linearised coefficient times the state.

Kinematic branch (used once `blend < 1`), obtained by differentiating the
Ackermann relationship `r_kin = v_x*tan(delta_act)/L` and `v_y_kin =
lr*r_kin` with respect to time, and letting the steering actuator's own lag
supply `delta_act`'s rate:

```
L = lf + lr
delta_act_dot = (delta_cmd - delta_act) / tau_delta      (actuator lag, same as lmpc.md)
r_dot_kin   = (a_act*tan(delta_act) + v_x*sec^2(delta_act)*delta_act_dot) / L
v_y_dot_kin = lr * r_dot_kin
```

Blended exactly as `lmpc.md`'s `A_c` matrices are:

```
v_y_dot = (1 - blend)*v_y_dot_kin + blend*v_y_dot_dyn
r_dot   = (1 - blend)*r_dot_kin   + blend*r_dot_dyn
```

The remaining two states are the same first-order actuator lag as
`lmpc.md`'s shared rows, `d(delta_act)/dt = (delta_cmd - delta_act)/tau_delta`
and `d(a_act)/dt = (a_cmd - a_act)/tau_a`, and `s_dot`/`e_y_dot`/`e_psi_dot`
are exactly the Frenet-kinematics equations derived above. Together, all
eight rates form `ẋ = f(x, u)`, evaluated by `nmpc_core.py`'s `_f()`
(vectorised over every horizon stage at once) and `_f_scalar()` (a
hand-mirrored scalar copy used by the sequential rollout, checked against
`_f()` to machine precision by
`test_nmpc_core_math.py::test_step_scalar_matches_step_vectorised`, see
["Testing the math"](#testing-the-math) below).

### From continuous to discrete: RK4, not Zero-Order Hold

`lmpc.md`'s `ẋ = A·x + B·u` is discretised **exactly** via a matrix
exponential (Zero-Order Hold), because it's linear, an exact closed form
exists. `ẋ = f(x, u)` here has no such closed form (`f` is nonlinear), so
discretisation instead uses **4th-order Runge-Kutta (RK4)**, a standard
numerical integrator that evaluates `f` several times per step (at the
start, twice at the midpoint, and once at the end) and combines them into a
step estimate accurate to 4th order in the step size, far tighter than a
single-evaluation (Euler) step for the same `dt`. Each control tick's
`dt = 0.05 s` is itself subdivided into `n_sub` RK4 sub-steps
(`nmpc_rk_substeps`, default 2, for the rollout) for extra accuracy on a
fast-changing state; see ["Two different sub-step counts, and why that's
safe"](#two-different-sub-step-counts-and-why-thats-deliberately-safe)
below for why the Jacobian pass uses a different, coarser count.

## Linearising the rollout: finite-difference Jacobians

Once the nonlinear rollout `X = [x_0, x_1, ..., x_N]` exists for the current
guess `U`, the SQP needs to know: *if input `u_k` at stage `k` were nudged
slightly, how would that change the predicted state at every later stage?*
That sensitivity is exactly what `lmpc.md`'s fixed `Ad`/`Bd` matrices
provide for the linear model; here, because the model is nonlinear, the
equivalent matrices have to be **recomputed fresh, around this tick's
specific rollout**, rather than looked up once and reused.

**Why finite differences, not a symbolic/analytic derivative.** The model
above involves `atan`, `cos`, `sin`, a `tanh` saturation and a piecewise
kappa-lookup, differentiable in principle but tedious and error-prone to
differentiate by hand and keep in sync with `_f` as it changes. A forward
finite difference approximates the same derivative numerically instead:

```
A_k[:, j] = (f(x_k + eps_j * e_j, u_k) - f(x_k, u_k)) / eps_j     (one column of A_k, state j)
B_k[:, j] = (f(x_k, u_k + eps_j * e_j) - f(x_k, u_k)) / eps_j     (one column of B_k, input j)
```

i.e. nudge one state (or input) component at a time by a small amount
`eps_j`, re-evaluate `f`, and divide the change in the output by `eps_j`,
recovering the local slope in that one direction. Repeating this for every
one of the 8 states and 2 inputs builds the full one-step Jacobians
`A_k = d(x_{k+1})/d(x_k)` and `B_k = d(x_{k+1})/d(u_k)` at every horizon
stage `k`. This is done **vectorised over all stages at once**
(`nmpc_core.py`'s `_jacobians()`): perturbing state `j` at every stage
simultaneously costs one batched call to `_step()` across the whole
horizon, so the whole Jacobian pass costs 10 such batched calls (8 states +
2 inputs) rather than `10 * N` individual ones.

### Two different sub-step counts, and why that's deliberately safe

The Jacobian pass uses `nmpc_jac_substeps` (default **1**), a **separate,
coarser** RK4 sub-step count from the rollout's own `nmpc_rk_substeps`
(default 2). This is a real, easily-missed distinction (an earlier
investigation round swept the wrong one of the two and measured no effect
at all, see `nmpc_low_speed_accel_stall_investigation.md`).

**Why the asymmetry is safe rather than a shortcut that quietly degrades
accuracy.** `A_k`/`B_k` only ever supply the QP's *step direction* for this
iteration, never the predicted trajectory itself, that's what the rollout
computes, and the rollout is exact to full RK4 at `nmpc_rk_substeps`
regardless of how coarse the Jacobian is. A coarser sensitivity estimate can
at most produce a slightly worse direction to step in; the backtracking
line search in `_solve_step`'s caller validates every candidate step
against the true nonlinear cost before accepting it (see
["The Gauss-Newton iteration as a whole"](#the-gauss-newton-iteration-as-a-whole)
below), so a poor direction costs at most one wasted iteration, never a bad
command. Halving the sub-step count on the Jacobian pass alone (the
dominant per-iteration cost) is exactly the kind of asymmetry this
buys real solve-time budget without touching prediction accuracy.

**The numerical-stability caveat this asymmetry runs into.** RK4's
real-axis stability limit is `|eigenvalue * dt| ≈ 2.78`; at
`nmpc_jac_substeps = 1` the effective per-substep `dt` is large enough that
the linearised (v_y, r) sub-dynamics' own eigenvalues push
`|eigenvalue| * dt` up to roughly 10.5 around 2.0 m/s (falling to about 2.7
by 8 m/s), i.e. *outside* RK4's stable region specifically in the low/mid
speed band. This does not corrupt the rollout (which uses the finer
`nmpc_rk_substeps = 2`), but it can make the Jacobian estimate itself
numerically unstable exactly in that speed band, which is the root cause
investigated at length in
`nmpc_low_speed_accel_stall_investigation.md`; that document is the
canonical reference for the finding, not repeated in full here.

## Condensing and the QP

The SQP subproblem is: find the sequence of input *changes*
`dU = [du_0, ..., du_{N-1}]` that minimises a quadratic approximation of the
true nonlinear cost, subject to the *linearised* dynamics
`dx_{k+1} = A_k dx_k + B_k du_k` (with `dx_0 = 0`, since the rollout already
starts at the true measured state, so there is no linearisation defect to
correct for at stage 0).

**Condensing** eliminates the state-deviation variables `dx_k` from the
problem entirely, expressing each one purely as a function of the
input-change decision variables via forward substitution:

```
S[0] = 0
S[k+1] = A_k @ S[k] + [0 ... B_k ... 0]     (B_k in the k-th input-change slot)
dx_k = S[k] @ dU_flat
```

`S[k]` (shape `NX x n_du`) is the sensitivity of stage-`k` state deviation
to the *whole* flattened input-change vector; building it recursively this
way costs one matrix multiply per stage rather than repeatedly composing
Jacobians from scratch. This is the same condensing idea used in
`lmpc.md`'s "parameterised" QP trick, just applied per-tick to a freshly
linearised, nonlinear-in-origin problem rather than once to a fixed linear
one.

**The cost, in condensed form**, mirrors `lmpc.md`'s QP almost exactly
(weighted output tracking, `Q`-analogue; input effort, `R`; input-rate
smoothness, `R_rate`), but with the output residual and its sensitivity now
coming from the *linearised, condensed* rollout rather than a fixed `Ad·x +
Bd·u`:

```
h(x) = [e_y, e_y_dot, e_psi, e_psi_dot, v_x - v_ref]      (the 5-row output vector, "H" in code)
G[k] = sqrt(W) @ C[k] @ S[k]        (sensitivity of the weighted output to dU_flat)
g[k] = sqrt(W) @ h(x_k)             (the current, unimproved weighted residual)

minimise over dU_flat:
    ||G @ dU_flat + g||^2                    (output tracking, condensed)
  + ||sqrt(ru) * (u_flat + dU_flat)||^2       (input effort, evaluated at u_flat + dU, not just dU)
  + ||sqrt(R_rate) * (E @ dU_flat + e_rate)||^2   (input-rate smoothness)
```

`C[k] = d h/d x` at stage `k` is built by exactly the same finite-difference
technique as `A_k`/`B_k` (`_output_jacobians`), riding along on the same
per-stage perturbations. `E` is a fixed differencing matrix so that
`E @ dU_flat` gives consecutive input-change differences directly (the same
role `cp.diff` plays in `lmpc.md`'s CVXPY formulation), and `e_rate` carries
the *current* iterate's own consecutive differences (including against
`u_prev`, the last command actually sent), so that `E @ dU_flat + e_rate` is
the *true* rate of change the true, unlinearized cost would see once `dU`
is applied, not just the change in the change.

Expanding the quadratic norms gives the QP's `Hess`/`grad` (`P`/`q` in
standard QP notation):

```
Hess = G'G + diag(ru_flat) + E'(R_rate)E
grad = G'g + ru_flat * u_flat + E'(R_rate)(E @ u_flat - u_prev_row)
```

exactly the Gauss-Newton approximation to the true nonlinear Hessian: `G'G`
(first-order-accurate curvature from the output sensitivity alone, dropping
second-derivative terms of `h` itself, the standard Gauss-Newton
simplification that keeps the subproblem a QP rather than a general
nonlinear program) plus the exactly-quadratic effort and rate terms, which
need no approximation since they are already quadratic in the true problem.

**Constraints on `dU_flat`** (`_solve_step`'s `A_dense`/`l`/`u`): a box
bound (stay within `u_min`/`u_max` overall) intersected with a **trust
region** (`nmpc_trust_delta_rad`, `nmpc_trust_a`, capping how far this one
step may move from the current iterate, since the linearisation is only
locally accurate); a slew-rate bound identical in spirit to `lmpc.md`'s
`du_max`; and, when enabled, a soft-slacked track-boundary bound and a hard
per-axle friction-circle bound, both built by projecting the same
condensed `S[k]` sensitivity onto the relevant state row (`e_y`, or the two
extra friction-circle output rows), exactly the same "sensitivity times
decision variable" pattern as the cost terms above, just used as a
constraint instead.

### The Gauss-Newton iteration as a whole

Putting the pieces together, one call to `_solve_step` does exactly one
Gauss-Newton step:

1. Roll out (already done by the caller) → `X`.
2. Linearise (`_jacobians`, `_output_jacobians`) → `A_k`, `B_k`, `C_k`.
3. Condense (`S`, `G`, `g`) → a QP in `dU_flat` alone.
4. Solve the QP (OSQP) → a candidate `dU`.
5. **Backtracking line search**: try `U + step*dU` for `step = 1, 0.5, 0.25,
   ...`, rolling the *true nonlinear* model forward each time (`_rollout`)
   and evaluating the *true nonlinear* cost (`_cost`, which mirrors the
   QP's objective term for term, not an approximation of it), accepting the
   first `step` that does not increase the true cost.
6. If no backtracking step improves, keep the previous iterate: a wasted
   iteration, never a step in a bad direction, since step 5 only ever
   compares against the true cost, never the QP's own (possibly optimistic)
   quadratic approximation of it.

**"Real-time iteration" means stopping after exactly one Gauss-Newton step
per control tick** (steps 1-5 above run once, not looped to convergence),
warm-started from the previous tick's converged-so-far `U`. Consecutive
ticks differ by only one horizon step sliding forward, the same argument
`lmpc.md` makes for OSQP's own warm start, so one step per tick tracks a
slowly-moving optimum closely enough in practice, and the offline
`nmpc_offline_check` explicitly verifies the cost decreases monotonically
from a cold start (see ["Testing the math"](#testing-the-math) below) as a
check that the iteration is behaving correctly, even though a live tick
never actually runs it to convergence.

## Testing the math

Two test suites exist for this controller's numerics specifically, both
referenced from `docs/lmpc.md`'s testing pointers and CLAUDE.md's "Testing"
section:

- **`tuner.nmpc_offline_check`** (offline repo): re-verifies `_step_scalar
  == _step` model parity, forward-vs-central-difference Jacobian agreement,
  and SQP cost-monotonic-convergence from a cold start at several
  representative operating points, on every call.
- **`fsae_autonomous`'s `test_nmpc_core_math.py`** (production port): the
  same three checks, ported so the production copy's own transcription can
  be verified independently rather than assumed identical to the sim
  original; see that file's module docstring for exactly which of the
  sim tree's checks are and are not ported.

A divergence in the first check (`_step_scalar` vs `_step`) is a **silent
wrong-prediction bug**: both the scalar rollout and the vectorised Jacobian
path would be consistently wrong the same way, so no closed-loop behavioural
test would catch it, only this direct numerical comparison does.

## Feature comparison: LTV-QP vs. NMPC, at a glance

Every feature below is verified against actual read-sites in the code, not
inferred from a docstring or field name, see
`docs/reference/README.md`'s "Which settings affect which controller" map
for the exhaustive, field-by-field version this table summarises.

| Feature | LTV-QP (`mpc_core.py`) | NMPC (`nmpc_core.py`) | Why |
|---|---|---|---|
| Adaptive gain scheduling (`_corner_factor`, anti-hunt, `adaptive_Q_scaling`, `adaptive_R_scaling`, `adaptive_R_rate`) | **Yes** | **No** (inert, none of these fields have any read site in `nmpc_core.py`) | Every one of these mechanisms exists to compensate for the LTV-QP's blind spot (it can't predict the path curving). NMPC's model has that built in structurally, so reweighting the cost on top would double-count an effect that's now already handled, see [`removed_mechanisms.md` §1](removed_mechanisms.md#1-the-structural-limit-the-argument-that-motivates-nmpc). |
| `steer_rate_anti_hunt` (steering-rate damping when centred/aligned/uncurving) | **Yes**, on by default | **Opt-in**, off by default (`nmpc_steer_rate_anti_hunt_enabled`) | The one exception to the row above: it only ever makes steering *more* damped in a specific narrow case, the opposite direction from anticipation, so it doesn't fight NMPC's structural fix the way the rest of the gain schedule would. Reuses the LTV-QP's own function verbatim (imported, not reimplemented). |
| Precomputed corner map (`use_precomputed_corner_map`) | Removed from both | Removed from both | Served the deleted lookahead gain-scheduling family, gone from both controllers, not an LMPC/NMPC difference. See [`removed_mechanisms.md` §7](removed_mechanisms.md#7-precomputed-corner-segmentation-cornermap). |
| Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`) | **Yes** | **Accepted but ignored** (`set_heading_profile()` exists so the node needs no branch, logs a one-time warning) | Same reasoning as gain scheduling: the shaped lead is a workaround for the same missing curvature term NMPC closes structurally. Applying both would double-count the anticipation. |
| Delay/latency compensation (rolling `x0` forward through recently-issued commands) | **Yes** (`predict_ahead()`, linear rollforward) | **Yes** (rolls `x0` forward through the nonlinear model instead) | Both need this, it's about *sensor/actuation lag*, a problem that exists regardless of which prediction model is used. Different implementation, same four gating fields (`delay_compensation_enabled`, `max_delay_compensation_steps`, `pose_age_lp_alpha`, `n_delay_hysteresis`), shared `MPCParams` fields, read by both. One exception: `predict_epsi_clip` is LTV-QP only (a small-angle bound specific to the *linear* rollforward; NMPC's nonlinear rollforward has no such bound to set). |
| Tracking-error speed gate (slow down when `e_y`/`e_psi` are large) | **Yes** | **Yes** | This lives in `control_utils.py`, called by the **node** (`mpc_controller.py`) *before* either controller's `.compute()` is invoked, neither `MPCController` nor `NMPCController` is even aware it exists. Controller-agnostic by construction. |
| Curvature-based speed profile (`curvature_speed()`) | **Yes** | **Yes** | Same reason as the row above: computed by the node, handed to whichever controller is selected as `desired_speed`. |
| Cone-proximity emergency braking, GO-gating, stale-path fail-safes | **Yes** | **Yes** | All node-level (`mpc_controller.py`'s `_control_step` phases, `standalone_output=true` only), not part of either controller class. `NMPCController` exposes the same `compute()`/`reset()`/`set_static_path()` surface as `MPCController` specifically so the node doesn't need a branch. |
| FSDS lateral-acceleration ceiling | **Yes**, as a plain speed-profile input (`curvature_speed()`'s friction-circle cap) | **Yes**, AND inside the prediction itself (`tanh` saturation on predicted tyre force) | NMPC's version is strictly more: the ceiling shapes what the *solver itself* believes is achievable, not just the requested speed. Without it, NMPC's linear-tyre model believes it can hold any corner at any speed and the car spins (measured). |
| Horizon length | 35 steps (1.75 s) | 20 steps (1.0 s) | Independent tuning choices, not a structural requirement, NMPC's shorter horizon reflects its per-tick solve cost (Gauss-Newton SQP is more expensive per step than one convex QP). |
| Solve method | One convex QP per tick (OSQP) | Real-time-iteration SQP: one Gauss-Newton step per tick, warm-started, condensed dense QP (OSQP) | See [`lmpc.md`'s "The solver"](lmpc.md#the-solver) for what a QP is; NMPC needs the extra linearize-and-resolve step because its own model is nonlinear (curvature is now a function of a state, not a fixed matrix entry). |

**Three further, NMPC-only additions**, assessed against
Alexander Liniger's Model Predictive Contouring Control (MPCC) but narrower
than it: full MPCC's progress-maximisation apparatus was considered and
rejected as too close to a failure mode already eliminated here (see
`docs/reference/README.md`'s writeup for why). One is on by default, two are
off:

- `nmpc_spline_reference_enabled` (default **true**): `PathReference`'s
  `kappa(s)`/`psi_ref(s)` come from an analytic cubic-spline fit to the
  waypoints instead of moving-average-smoothed finite differences. A
  numerical-quality fix, not a new coupling to the solver.
- `nmpc_horizon_speed_profile_enabled` (default **false**, experimental):
  samples a precomputed speed profile at each horizon stage's own predicted
  arc length, the same state-keyed pattern `kappa(s)` already uses, instead
  of holding one frozen speed target across the horizon.
- `nmpc_friction_circle_enabled` (default **false**, experimental): a hard
  per-axle tyre-force bound in the QP, additional to (not replacing) the
  existing soft `alat_ceiling` saturation.

All three are implemented identically in `nmpc_core.py` and the offline
`controller/nmpc_optimiser.py`; none touch `mpc_core.py` (the LTV-QP).

Full detail: `docs/reference/control_mechanisms.md`'s "Nonlinear MPC (`use_nmpc`)" section
(what it is, what it reuses, what is inactive, offline A/B numbers, the offline
port, a matched same-day LIVE A/B (steering saturation 6.45% → 0.58%,
lap 54.72s → 52.35s) and the "Which settings affect which controller" map
and the three MPCC-inspired additions above), `tuning.md` §4.5d (tuning
surface), and `late_turn_in_investigation.md` Part 16 (research survey,
formulation choice, validation, the four bugs found in testing).
