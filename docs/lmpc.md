# The LTV-QP Controller (LMPC)

Full technical reference for the default controller, `mpc_core.MPCController`:
a linear time-varying MPC (LTV-QP) solved as one convex Quadratic Program per
tick. Split out of `architecture.md` because this material (state vector,
every matrix entry, the cost function, the solver) is large enough to be its
own document; that file now only summarises and links here.

For the nonlinear alternative controller, see [`nmpc.md`](nmpc.md). For the
worked-by-hand arithmetic behind `e_y`/`e_psi` specifically (more detail than
this doc's own measurement section below), see
[`error_state_reference.md`](error_state_reference.md).

This section explains the controller in full: the state vector, where every
entry of every matrix comes from, the cost function, the solver, and the two
runtime adaptive features layered on top. The implementation is split across
three files that must be kept in numeric agreement: `model/bicycle_model.py`
(the prediction model), `controller/optimiser.py` (the QP formulation, used
by the simulator/tuner), and `mpc_core.py` (a self-contained duplicate of
both, used by the live ROS 2 node so it has no simulator dependencies).

This section assumes no prior background: every term below (model, state,
input, cost function, solver) is defined in plain English the first time it
appears.

## Table of Contents

1. [Four terms needed before anything else makes sense](#four-terms-needed-before-anything-else-makes-sense)
2. [What "MPC" means here](#what-mpc-means-here)
3. [The 8-state error vector](#the-8-state-error-vector)
4. [The 2-input control vector](#the-2-input-control-vector)
5. [Building the prediction model](#building-the-prediction-model-modelbicycle_modelpy)
6. [The cost function and QP](#the-cost-function-and-qp-controlleroptimiserpy)
7. [The solver](#the-solver)
8. [Adaptive gain scheduling](#adaptive-gain-scheduling-controllermodel_utilspy)
9. [Where this is duplicated, and why](#where-this-is-duplicated-and-why)

---

### Four terms needed before anything else makes sense

- **State**: the set of numbers that describe "where things stand right
  now." For this controller, state doesn't mean the car's raw (X, Y)
  position, it means *how far off the intended path the car currently is*
  (see [The 8-state error vector](#the-8-state-error-vector) below for the
  actual list). Written as a vector, `x`.
- **Input** (also called a **command**): the numbers the controller
  actually gets to choose each tick, steering angle and
  acceleration/braking. Written as a vector, `u`.
- **Model**: a mathematical rule that answers *"given the current state `x`
  and a chosen input `u`, what will the state be a moment later?"* It's the
  controller's internal, simplified stand-in for "how the car behaves,"
  used purely for planning, not the real car itself. This project's model
  is a **bicycle model**: instead of simulating all four wheels, the car is
  approximated as one wheel on the front axle and one on the rear, both on
  the centreline. That's a standard simplification in vehicle control, and it
  captures the two things that matter most for path tracking (how the
  front wheel steers, and how the whole car rotates/slides) while staying
  simple enough to evaluate thousands of times a second.
- **Cost function**: a single number that scores "how bad" a candidate plan
  is, where bigger tracking error, more control effort, or jerkier commands all
  make this number bigger. The controller's whole job each tick is to
  search for the sequence of inputs that makes this number as small as
  possible. The thing doing that search is called the **solver** (see [The
  solver](#the-solver) below). Rather than writing a search algorithm from
  scratch, the cost function and the model are handed to an existing,
  purpose-built solver library, which finds the best answer.

With those four ideas in hand, the rest of this section builds up exactly
what this controller's state, input, model, and cost function actually are,
in full detail.

### What "MPC" means here

At every control tick (20 Hz), the controller:

1. Measures the current tracking error (`x0`).
2. Predicts, using a simplified **linear** model, what the tracking error
   would do over the next `N_HORIZON` steps (1.75 s) for every possible
   sequence of steering/throttle commands.
3. Solves for the sequence that minimises a cost (tracking error + control
   effort + smoothness), subject to hard limits (max steering angle, max
   acceleration, a soft lane boundary).
4. Applies **only the first command** in that sequence to the real
   (nonlinear) plant.
5. Throws the rest of the plan away and repeats from measurement at the next
   tick.

This "solve a plan, use only the first step, replan" pattern is the
*receding horizon* principle, and it's what makes MPC robust to the fact
that its internal model (linear, 8-state) is not a perfect match for the
real vehicle (nonlinear, 24-state, Pacejka tyres, suspension, aero). Any
mismatch between what the model predicted and what the plant actually did
shows up as tracking error at the next measurement, and gets corrected on
the next solve. The controller never needs its internal model to be
perfectly accurate, only good enough to plan a *reasonable* next step.

### The 8-state error vector

The MPC does not track the car's raw position (X, Y). It tracks **error
relative to the path**, how far off, and in what way, the car currently is.
This keeps the model's behaviour independent of where on the map the car
happens to be.

```
x = [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
```

| # | Symbol | Meaning | Units |
|---|---|---|---|
| 0 | `e_y` | Lateral (sideways) distance from the path centreline | m |
| 1 | `e_y_dot` | Rate of change of `e_y` | m/s |
| 2 | `e_psi` | Heading error, car's yaw minus the path's tangent direction | rad |
| 3 | `e_psi_dot` | Yaw rate (how fast the car's heading is currently changing) | rad/s |
| 4 | `e_v` | Speed error, current speed minus the planner's target speed | m/s |
| 5 | `e_a` | Unused acceleration-error placeholder, always driven toward 0 | m/s² |
| 6 | `delta_act` | The steering angle the actuator has *actually* reached so far (after lag) | rad |
| 7 | `a_act` | The acceleration command the actuator has *actually* reached so far (after lag) | m/s² |

States 6 and 7 exist because a real steering rack / throttle doesn't jump
instantly to a commanded value, there's a first-order lag (see
`tau_delta`, `tau_a` in `model/vehicle_physics.py`). Tracking the *actual*
(lagged) actuator state, not just the commanded value, lets the model
correctly predict how the car will move over the horizon.
State 5 is purely for consistency, there is a rate of change for each state.

**`e_v`'s target speed is frozen for the whole horizon, not a per-step
profile.** In plain terms: the controller picks one target speed at the
start of each solve and holds it fixed across the whole 1.75 s look-ahead,
rather than asking for a different speed at each future point along that
horizon. Concretely, `desired_speed` is looked up/computed once per solve,
from `speed_profile.curvature_speed()` when the live planner is in the
loop, or from the precomputed `path_v_profile` array otherwise, and baked
into `x0[4]` as a single scalar. Nothing in `Ad`/`Bd` re-references it at
later horizon steps, so the cost function penalises deviation from *the
same* target speed across all `N` steps, not the true curvature-limited
speed at each predicted future position.

This is a deliberate receding-horizon simplification rather than an
oversight: the controller re-solves every 50 ms with a freshly recomputed
`desired_speed`, so a stale in-horizon reference self-corrects within one
tick. It is still worth knowing about when debugging speed-tracking
behaviour approaching a corner whose onset falls inside the current
horizon. There is currently no acceleration profile, so there is no
acceleration error either.

#### How the error vector is measured (Frenet-frame projection)

The error states above (`e_y`, `e_psi`, etc.) aren't things the car can
read off a sensor directly, they only make sense *relative to a point on
the path*. Every control tick, `vehicle_physics.plant_to_tracking_error()`
(`model/vehicle_physics.py`) has to answer: "of all the points along the
reference path, which one is
the car currently 'at', and how far off is it from that point?"

This is a **Frenet-frame** conversion: instead of describing the car's
position in the usual global (X, Y) map coordinates, it's re-described
relative to the path itself, as a longitudinal position *along* the path
plus a lateral offset *perpendicular* to it. Concretely, the code:

1. Finds the nearest reference point on the path to the car's current
   (X, Y) position (`get_interpolated_ref_point()`), giving a reference
   `(ref_x, ref_y, ref_psi)`, the path's position and tangent heading at
   that point.
2. Projects the car's offset from that point onto the direction
   perpendicular to the path's tangent, which gives the signed lateral
   error `e_y` (positive/negative = left/right of the centreline).
3. Takes the difference between the car's heading and the path's tangent
   heading at that point, giving `e_psi`.

This is the same idea used throughout path-tracking control (and in the
planner's centreline/curvature calculations, see `architecture.md`'s
"Architecture Overview"): re-expressing "where am
I" as "how far along the path, and how far off to the side," which is a
much more useful frame for a controller whose whole job is to stay close
to a curve, rather than reaching a specific (X, Y) point.

**NMPC measures its own current-tick error the same way** (a Frenet-frame
projection, `PathReference.project()` in `nmpc_core.py`, same
nearest-point-plus-perpendicular-offset arithmetic). Frenet-frame
*measurement* is not what tells the two controllers apart; see
[`nmpc.md`](nmpc.md#the-structural-difference-in-one-line) for what
actually differs between them, which is what happens to that error over
the prediction horizon, not how it's measured this tick. For the full
worked-by-hand arithmetic, see
[`error_state_reference.md`](error_state_reference.md).

### The 2-input control vector

```
u = [delta_cmd, a_cmd]
```

`delta_cmd` (rad) and `a_cmd` (m/s²) are the raw commands sent to the
actuator lag filters, not the actual steering angle / acceleration
themselves (those are states 6 and 7 above, which lag behind `u`).

### Building the prediction model (`model/bicycle_model.py`)

Before the MPC can plan anything, it needs a way to answer the question:
*"if the car is currently in error state `x`, and a steering/throttle
command `u` is applied, what will the error state be a tiny fraction of a
second later?"* That question, answered mathematically, is the **prediction
model**. This section builds it up from scratch: the general form, the two
physical models that get blended into it, and finally how it's converted
into the exact numbers the solver uses.

The car itself is approximated as a **bicycle model**: instead of four
separate wheels, it's treated as one wheel on the front axle and one wheel
on the rear axle, both sitting on the car's centreline. This is a standard
simplification in vehicle control: it captures the two things that matter
most for path tracking (how the front wheel steers, and how the whole car
rotates and slides sideways) while staying simple enough to solve fast,
20 times a second.

#### The general continuous-time form

Every linear model in control theory is written the same way:

```
ẋ = A·x + B·u
```

Read this as: **"the rate of change of the state vector (ẋ) is some fixed
mixture of the current state (x) plus some fixed mixture of the current
command (u)."** `A` and `B` are just tables of numbers (matrices) that say
*how much* of each state and each command feeds into the rate of change of
every other state. This is "continuous-time" because `ẋ` is a true
instantaneous rate of change (like a speedometer reading), not a per-tick
step; that comes later.

Recall the 8-state error vector and 2-input command vector from above:

```
x = [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]ᵀ
u = [delta_cmd, a_cmd]ᵀ
```

So `A` is an **8×8** grid of numbers and `B` is an **8×2** grid of numbers.
Reading the grid: **entry `A[row, col]` is a multiplier saying "how much
does the current value of state `col` contribute to the rate of change of
state `row`."** Most entries are zero, because most states have no direct
physical influence on most other states, only a handful of meaningful
physical relationships exist, and those are the only non-zero numbers in
the grid. Two different physical assumptions produce two different sets of
numbers for `A` (`B` turns out to be the same in both), described next.

#### 1. Kinematic model (used below ~1 m/s)

At very low speed, the tyres haven't built up any real sideways
(cornering) grip yet, so the car turns purely by geometry, the same way
pushing a shopping trolley by its handle makes it pivot. The physical
relationships are:

```
ė_y   = v_x · e_psi
ė_psi = v_x · delta_act / L        (L = wheelbase = lf + lr)
```

In plain words: *"how fast the car drifts sideways off the path depends on
how much it's currently pointing the wrong way, scaled by speed"* (turn
your wheels while stationary and nothing happens, sideways drift needs
forward motion to convert into it), and *"how fast the car's heading is
changing depends on the current steering angle and speed, via the
wheelbase"* (standard Ackermann steering geometry: a longer car turns more
slowly for the same steering angle).

Every other state either isn't affected in this simple model, or follows
the same "shared" behaviour described in section 3 below (actuator lag,
speed error). Written out as the full 8×8 matrix `A_kin` (blank cells are
zero):

```
        e_y   e_y_dot  e_psi  e_psi_dot   e_v    e_a   delta_act  a_act
e_y   [  0      0      v_x       0         0      0        0        0   ]
e_y_dot[ 0      0       0        0         0      0        0        0   ]
e_psi [  0      0       0        0         0      0      v_x/L      0   ]
e_psi_dot[0     0       0        0         0      0        0        0   ]
e_v   [  0      0       0        0         0      1        0        0   ]
e_a   [  0      0       0        0         0      0        0        1   ]
delta_act[0     0       0        0         0      0     -1/tau_δ    0   ]
a_act [  0      0       0        0         0      0        0    -1/tau_a]
```

In code:

```python
A_kin[0, 2] = v_x_safe          # ė_y = v_x * e_psi
A_kin[2, 6] = v_x_safe / L      # ė_psi = v_x/L * delta_act  (Ackermann geometry)
```

(rows 4-7 are the shared rows, covered in section 3.)

#### 2. Dynamic model (used above ~2.5 m/s)

At higher speed, tyre grip (cornering stiffness × slip angle) dominates
over pure geometry, and this is the regime a real car spends most of its time
in. It's the standard linearised bicycle model, derived from Newton's laws
for a rigid body sliding and rotating in a plane, assuming small slip
angles:

```
ë_y   = -(2Cf+2Cr)/(m·vx) · ė_y  +  (2Cf+2Cr)/m · e_psi
        + (-2Cf·lf+2Cr·lr)/(m·vx) · e_psi_dot  +  (2Cf)/m · delta_act

ë_psi = (-2Cf·lf+2Cr·lr)/(Iz·vx) · ė_y  +  (2Cf·lf-2Cr·lr)/Iz · e_psi
        - (2Cf·lf²+2Cr·lr²)/(Iz·vx) · e_psi_dot  +  (2Cf·lf)/Iz · delta_act
```

`Cf`/`Cr` are the front/rear cornering stiffnesses (N/rad, from
`VehicleParams`, how much sideways force a tyre generates per radian of
slip angle), `lf`/`lr` are the distances from the car's centre of mass to
each axle, `m` is mass, and `Iz` is yaw inertia (how hard it is to make the
car spin, similar to how a figure skater with arms out spins slower). The
`1/vx` terms exist because at higher speed, the same sideways drift
produces a *smaller* slip angle. The tyre has rolled further forward for
the same amount of sideways motion, so it "notices" the slide less, and
grip builds up more gradually rather than instantly.

As the full 8×8 matrix `A_dyn`:

```
         e_y  e_y_dot          e_psi           e_psi_dot         e_v  e_a  delta_act    a_act
e_y     [ 0     1                0                 0              0   0       0           0   ]
e_y_dot [ 0  -(2Cf+2Cr)/(m·vx) (2Cf+2Cr)/m  (-2Cf·lf+2Cr·lr)/(m·vx) 0   0    (2Cf)/m        0   ]
e_psi   [ 0     0                0                 1              0   0       0           0   ]
e_psi_dot[0 (-2Cf·lf+2Cr·lr)/(Iz·vx) (2Cf·lf-2Cr·lr)/Iz -(2Cf·lf²+2Cr·lr²)/(Iz·vx) 0 0  (2Cf·lf)/Iz  0 ]
e_v     [ 0     0                0                 0              0   1       0           0   ]
e_a     [ 0     0                0                 0              0   0       0           1   ]
delta_act[0     0                0                 0              0   0    -1/tau_δ       0   ]
a_act   [ 0     0                0                 0              0   0       0        -1/tau_a]
```

In code:

```python
A_dyn[0, 1] = 1.0                                          # ė_y = e_y_dot
A_dyn[1, 1] = -(2*Cf + 2*Cr) / (m * v_x_safe)              # Lateral damping
A_dyn[1, 2] = (2*Cf + 2*Cr) / m                             # Heading error → lateral accel
A_dyn[1, 3] = (-2*Cf*lf + 2*Cr*lr) / (m * v_x_safe)         # Yaw rate → lateral accel
A_dyn[1, 6] = (2*Cf) / m                                    # Steering → lateral force
A_dyn[2, 3] = 1.0                                           # ė_psi = e_psi_dot
A_dyn[3, 1] = (-2*Cf*lf + 2*Cr*lr) / (Iz * v_x_safe)        # Lateral velocity → yaw moment
A_dyn[3, 2] = (2*Cf*lf - 2*Cr*lr) / Iz                      # Heading error → yaw moment
A_dyn[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x_safe)  # Yaw damping (both axles)
A_dyn[3, 6] = (2*Cf * lf) / Iz                               # Steering → yaw moment
```

Notice row 1 (`e_y_dot`) here isn't just `ė_y = ...` like the kinematic
model, it's a *second-order* relationship (`ë_y`, acceleration of lateral
error), so the state `e_y_dot` itself needs its own row saying `ė_y = 
e_y_dot` (row 0, entry `[0,1] = 1`) before row 1 can describe how
`e_y_dot` itself accelerates. This is the standard trick for turning a
second-order physical equation into two coupled first-order ones, which is
why the dynamic model needs both `e_y` *and* `e_y_dot` as genuinely
separate states, while the kinematic model above barely used `e_y_dot` at
all.

#### 3. Shared rows (identical in both models)

Four rows don't depend on which physical regime is active, they're either
structural bookkeeping or simple decay behaviour, so both `A_kin` and
`A_dyn` set them identically:

```python
A_kin[4, 5] = A_dyn[4, 5] = 1.0             # ė_v = e_a
A_kin[5, 7] = A_dyn[5, 7] = 1.0             # ė_a = a_act (structural; e_a itself is unused)
A_kin[6, 6] = A_dyn[6, 6] = -1.0 / tau_delta  # dδ_act/dt = -δ_act/tau_delta (decays toward 0 with no input)
A_kin[7, 7] = A_dyn[7, 7] = -1.0 / tau_a      # da_act/dt = -a_act/tau_a
```

The last two rows describe **actuator lag**: a real steering rack or
throttle doesn't jump instantly to a commanded value, it eases toward it.
Left alone (no new command), `delta_act` and `a_act` naturally decay back
toward zero over a time constant `tau_delta`/`tau_a`, like a stretched
spring relaxing. What actually *drives* them toward the commanded value is
the input matrix `B` (8×2, one column per command, `delta_cmd` and
`a_cmd`), which is identical for both the kinematic and dynamic models:

```
           delta_cmd   a_cmd
e_y       [   0          0   ]
e_y_dot   [   0          0   ]
e_psi     [   0          0   ]
e_psi_dot [   0          0   ]
e_v       [   0          0   ]
e_a       [   0          0   ]
delta_act [ 1/tau_δ       0   ]
a_act     [   0        1/tau_a]
```

```python
B[6, 0] = 1.0 / tau_delta   # delta_cmd drives the steering lag integrator
B[7, 1] = 1.0 / tau_a       # a_cmd drives the acceleration lag integrator
```

Together, row 6 of `A` and row 6 of `B` combine into the classic
first-order lag equation `dδ_act/dt = (delta_cmd − δ_act) / tau_delta`:
the actuator moves toward the command, at a rate proportional to how far
away it still is (the `-δ_act/tau_delta` self-decay term lives in `A`,
the `+delta_cmd/tau_delta` "pull toward the target" term lives in `B`).

#### What the matrix multiplication actually produces

Putting `A` and `B` together, `ẋ = A·x + B·u` means: multiply each row of
`A` by the entire state vector `x` (a dot product), then add the matching
row of `B` multiplied by `u`; the result is the rate of change of that one
state. Spelling out just the two most important rows, using the dynamic
model's `e_y_dot` row and the kinematic model's `e_psi` row as concrete
examples, the matrix multiplication `A·x` expands into exactly the
physical equations from sections 1 and 2:

```
Row 1 (e_y_dot) of A_dyn · x  =
    0·e_y + [-(2Cf+2Cr)/(m·vx)]·e_y_dot + [(2Cf+2Cr)/m]·e_psi
    + [(-2Cf·lf+2Cr·lr)/(m·vx)]·e_psi_dot + 0·e_v + 0·e_a
    + [(2Cf)/m]·delta_act + 0·a_act

  = -(2Cf+2Cr)/(m·vx)·e_y_dot + (2Cf+2Cr)/m·e_psi
    + (-2Cf·lf+2Cr·lr)/(m·vx)·e_psi_dot + (2Cf)/m·delta_act

  = ë_y      ← exactly the dynamic-model equation from section 2
```

```
Row 2 (e_psi) of A_kin · x  =  [v_x/L]·delta_act  =  ė_psi
  ← exactly the kinematic-model equation from section 1
```

Every zero entry in the row simply means "this state has no effect here",
the dot product just drops those terms out. This is the whole point of
writing the physics as a matrix: instead of writing eight separate
equations by hand, `Ad @ x + Bd @ u` (one line of code) computes all eight
rates of change at once, which is exactly what lets the solver evaluate the
model quickly, thousands of times, while searching for the best control
sequence.

#### 4. Blending kinematic and dynamic models

A single linear model can't represent the car well across its whole speed
range: the kinematic model breaks down once tyres start sliding, and the
dynamic model's `1/vx` terms blow up as speed approaches zero. Rather than
switching abruptly between the two (which would cause a visible jump/jerk
in the car's predicted behaviour right at the switch-over speed), the two
matrices are blended smoothly:

```python
alpha = clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
A_c   = (1.0 - alpha) * A_kin + alpha * A_dyn
```

`alpha` ramps linearly from 0 to 1 as speed goes from 1 m/s to 2.5 m/s:
pure kinematic model below 1 m/s, pure dynamic model above 2.5 m/s, and a
proportional mix of the two matrices' numbers in between (e.g. at
`alpha = 0.5`, every entry of `A_c` is exactly halfway between the
matching entry of `A_kin` and `A_dyn`). `B` is identical in both models, so
it doesn't need blending, it's used unchanged regardless of `alpha`.

#### 5. From continuous to discrete: Zero-Order Hold (ZOH)

Everything above describes `ẋ = A_c·x + B_c·u`, an instantaneous,
continuous-time rate of change. But the MPC doesn't operate continuously;
it makes one decision every `dt = 0.05 s` and holds that decision fixed
until the next tick. What it actually needs is a **discrete** one-step
prediction:

```
x[k+1] = Ad·x[k] + Bd·u[k]
```

In plain terms, "given the state right now (`x[k]`) and the command I'm about to hold for
the next 0.05 s (`u[k]`), what will the state be exactly one tick later
(`x[k+1]`)?" Converting the continuous equation into this discrete one is
called **discretisation**, and the method used here is **Zero-Order Hold
(ZOH)**, the exact, mathematically correct discretisation for a system
where the input is held constant between updates (a "zero-order hold" on
the input), which is precisely how MPC applies its commands. This is more
accurate than a simpler method like Euler's approximation, which introduces
compounding error at every step.

Both `Ad` and `Bd` are computed together via one matrix exponential (`expm`,
the matrix equivalent of `e^x`) on an augmented matrix, which sidesteps
having to directly invert `A_c` (a numerically risky operation if `A_c` is
close to singular):

```
exp( [A_c  B_c] · dt )  =  [Ad  Bd]
     [ 0    0 ]            [ 0  I ]
```

```python
M[:8, :8] = A_c
M[:8, 8:] = B_c
Md = scipy.linalg.expm(M * dt)
Ad, Bd = Md[:8, :8], Md[:8, 8:]
```

`Ad` and `Bd` are what actually get handed to the solver, the continuous
matrices `A_c`/`B_c` above exist only as an intermediate step to build them
correctly.

#### Linear vs nonlinear, in plain English

Now that `Ad`/`Bd` exist concretely, it's worth being precise about what
"linear" actually means here, since the word gets used constantly below. A
model is **linear** if every output is just a fixed multiple of each input,
added together, double an input and its contribution exactly doubles, and
no input's effect depends on the current value of another input. Every
matrix built above (`A`, `B`, `Ad`, `Bd`) is exactly this: a table of fixed
multipliers, so `x[k+1] = Ad·x[k] + Bd·u[k]` is always "this state times a
fixed number, plus that state times a fixed number, ...", never anything
that bends or saturates depending on where the car currently is.

That rigid structure is what lets the solver treat "find the best control
sequence" as a **Quadratic Program (QP)**, see [The cost function and
QP](#the-cost-function-and-qp-controlleroptimiserpy) and [The
solver](#the-solver) below, with a fast, predictable solve and a
guaranteed global optimum every tick.

The real plant (`model/vehicle_physics.py`) has no such structure: its
tyre forces, weight transfer, and heading kinematics genuinely curve and
saturate (see `architecture.md`'s "Configuring the Vehicle" section for
concrete numbers). If the MPC
tried to plan against those equations directly, the relationship between
`x[k+1]` and `(x[k], u[k])` would no longer reduce to a fixed multiplier
table, and the problem would stop being a QP and become a much harder
nonlinear program (NLP), no guaranteed optimum, no fast off-the-shelf
solver, and solve times that can balloon unpredictably.

That's exactly why the MPC doesn't plan against the real nonlinear plant
directly: it builds the much simpler **linear** approximation derived above
(good near the car's *current* operating point, thanks to the
kinematic/dynamic blend) and re-linearises it fresh every single tick as
speed and conditions change. The nonlinear plant is reserved for simulating
what actually happens to the "real" car in response to a command, see
`architecture.md`'s "Configuring the Vehicle" section.

#### Note on OSQP sparsity

`Ad` and `Bd` are consumed a few sections down by **OSQP**, the QP
(Quadratic Program, see [The solver](#the-solver) below) solver that
actually computes the steering/throttle command every tick. OSQP has a
quirk that affects how these matrices must be initialised, explained here
since it's decided at model-construction time even though it only matters
once the solver is involved.

All matrices start as `1e-12` (not exact `0.0`) rather than `np.zeros(...)`.
Here's why:

- **What a sparsity pattern is:** OSQP analyses which matrix entries are
  nonzero on its *first* solve, then caches that pattern (the *set* of
  matrix positions holding a nonzero value) for speed on every later
  solve.
- **The bug this avoids:** if a later solve produces an entry that rounds
  exactly to zero where it was previously nonzero (which can happen as
  `vx` changes and terms like `1/vx` shrink), OSQP's cached factorisation
  becomes invalid and it throws a reallocation error.
- **The fix:** filling every entry with a tiny nonzero epsilon keeps the
  sparsity pattern identical at every speed, so OSQP never needs to
  re-analyse it mid-run.

See
[The solver](#the-solver) for what OSQP is doing with these matrices and
why sparsity matters to it in the first place.

### The cost function and QP (`controller/optimiser.py`)

The cost function is the concrete answer to "how bad is this candidate
plan" (see the plain-English definition at the top of this section). Each
solve minimises, over the predicted `N`-step horizon:

```
min  Σᵢ ‖√Q ⊙ x[:,i]‖²   (state/tracking cost, all N+1 predicted states)
   + Σᵢ ‖√R ⊙ u[:,i]‖²   (control effort cost, all N inputs)
   + Σᵢ ‖√R_rate ⊙ Δu[:,i]‖²   (smoothness cost, penalises step-to-step change)
   + W_slack · ‖slack‖²   (soft lane-boundary violation penalty)

subject to:
   x[:,0] = x0                           (must start at the measured state)
   x[:,k+1] = Ad·x[:,k] + Bd·u[:,k]       (obey the linear model, all N steps)
   u_min ≤ u[:,k] ≤ u_max                 (hard actuator limits)
   -3.5 - slack ≤ x[0,k] ≤ 3.5 + slack    (soft ±3.5 m lane corridor on e_y)
```

`Q`, `R`, `R_rate` are diagonal weight matrices, one number per state/input
dimension, controlling how much the solver cares about minimising that
particular quantity relative to the others (see the
[Manual Tuning Guide](junior_project_mpc_docs.md#26-manual-tuning-guide) and
the comments in `settings.py` for what each entry means practically). They're
expressed and injected as
square roots (`sqrtQ`, `sqrtR`, `sqrtR_rate`) so the cost can be written with
`cp.sum_squares`, which CVXPY maps efficiently onto OSQP's internal
quadratic-cost matrix, this is a numerical-stability/implementation choice,
not a change in what's being penalised (`‖√w·x‖² = w·x²`).

**Why states 5-7 (`e_a`, `delta_act`, `a_act`) are never tuned:** only the
first 5 diagonal entries of `Q` (`e_y` through `e_v`) and both entries of
`R`/`R_rate` are exposed to the offline tuner (`TUNABLE_Q_IDX = [0,1,2,3,4]`
in `tuner/offline_tuner.py`), for two different reasons:

- **`Q[5,5]` (`e_a`)** stays at 0 because that state is a structural
  placeholder with no independent target. Penalising it would just add
  noise to the cost with no corresponding control lever to actually fix it.
- **`Q[6,6]`/`Q[7,7]` (`delta_act`, `a_act`)** also stay at 0, for a
  different reason: those are *measurements* of where the actuator
  currently is, not tracking errors. There's no "correct" value for them
  to be pulled toward: the actual steering/acceleration commands are
  already penalised directly through `R` and `R_rate` instead.

**The rate-of-change (smoothness) cost is split into two pieces** because
the first horizon step needs a different "previous command" than every
step after it:

```python
# Step 0: compare against the last command actually sent to the real plant
cost += sum_squares(sqrtR_rate * u[:,0] - sqrtR_rate * u_prev)

# Steps 1..N-1: compare each step against the previous *predicted* step
du = cp.diff(u, axis=1)
cost += sum(sum_squares(sqrtR_rate * du))
```

**The soft lane boundary** (`±3.5 m` on `e_y`, matching `TRACK_HALF_WIDTH`)
uses a slack variable rather than a hard constraint. `W_slack = 10000.0` is
large enough that the solver will essentially never choose to violate the
corridor when a compliant solution exists, but because it's *soft*
(penalised, not forbidden), the QP stays solvable even when the car is
already outside the corridor (e.g. mid-recovery from an off-track excursion),
where a hard constraint would make the problem infeasible and the solver
would return nothing at all.

**The "parameterised" trick:** the QP's variables, constraints, and cost
expression are built **once** using `cp.Parameter` placeholders rather than
plain numbers. Every subsequent solve only updates the parameter *values*
(`Ad`, `Bd`, `x0`, weights, etc.) and re-invokes the same compiled problem.
This lets OSQP reuse its cached factorisation and warm-start from the
previous solution, rebuilding the whole CVXPY expression graph from scratch
every tick would be roughly 10× slower and is unnecessary since the
problem's *structure* (which variables relate to which) never changes,
only the numbers plugged into it.

### The solver

**What kind of problem is being solved?** The cost function above (state
error + control effort + smoothness, all squared) is a **quadratic**
function of the unknowns (`x` and `u` over the whole horizon), and every
constraint (dynamics, actuator limits, lane boundary) is **linear**. A
quadratic cost with linear constraints is called a **Quadratic Program
(QP)**, a well-studied category of optimisation problem for which fast,
reliable, purpose-built solvers exist. This is precisely why the cost
function was built the way it was (squared errors, not e.g. absolute
values or something more exotic), it's what keeps the whole problem inside
this fast-to-solve category rather than needing a slower, more general
optimiser.

**What does "solving" it actually mean?** The solver is handed the fully
built-out cost expression and constraint list from the previous section,
and searches for the one sequence of steering/throttle values (`u[0]`
through `u[N-1]`) that makes the total cost as small as possible, while
never violating a hard constraint (actuator limits) and only softly
violating the lane boundary if truly necessary. It does this by starting
from a guess, checking whether nudging that guess in some direction reduces
the cost while respecting the constraints, and repeating until no further
nudge helps. This iterative process is what OSQP's `max_iter`/`eps_abs`
settings control (how many nudges it's allowed, and how small a nudge
counts as "close enough" to stop).

**Primary: OSQP.** Exploits the QP's sparsity (most matrix entries are
zero, so the solver skips work on them) and supports warm-starting,
reusing the *previous* tick's solution as this tick's starting guess. Since
consecutive MPC solves in a receding horizon differ by only one step (the
horizon just slides forward by 0.05 s each time), the previous answer is
already an excellent starting guess, so warm-started solves typically
converge in ~50-200 nudges instead of 500-2000 from a cold start, and this is
what makes solving a QP fast enough to happen 20 times per second. Typical
solve time is 1-5 ms at `N=25`.

**Fallback: Clarabel.** A different (interior-point) solving strategy that
is generally slower per solve but more numerically robust on
poorly-behaved problems. It's only invoked if OSQP itself fails to reach a
usable answer: returning infeasible, unbounded, or hitting numerical
trouble or its iteration cap.

**If both fail**, the simulator/tuner returns `None` and the caller holds
the previous command; the live `mpc_core.MPCController` instead
returns a full-brake command (`[u_prev[0], -a_max_brake]`), braking is the
safer default for a real vehicle than continuing to coast on a stale plan.

**`OPTIMAL_INACCURATE`** (OSQP found an answer, but not to its full
precision tolerance) is still accepted and used. Refusing it and holding
the previous command would generally be worse than using a
slightly-under-converged-but-still-reasonable solution at 20 Hz. The
offline tuner counts these occurrences and applies a scoring penalty (see
`architecture.md`'s "The Composite Score" section) so weight sets that cause
frequent `OPTIMAL_INACCURATE` are still discouraged, without discarding the
run outright.

### Adaptive gain scheduling (`controller/model_utils.py`)

The tuned `Q`, `R`, `R_rate` weights are optimised as if for a single
"average" operating point. A handful of functions rescale `Q`/`R`/`R_rate`
*every tick* to compensate for known, predictable ways the required control
authority changes with speed and curvature, without needing a separate
tuned weight set for every regime.

**This section describes two generations of that idea.** The mechanism used
to be a family of ~15 interacting functions that scanned *forward* along the
path (a "lookahead" scan producing a peak curvature ahead, `kappa_max_abs`)
and reweighted the cost matrices in anticipation of a corner not yet
reached. That whole family was deleted and replaced by three simpler,
**current-state-only** factors, no forward scan at all.

"Current-state gain scheduling" below documents what runs today;
"Historical: the lookahead gain-scheduling family" documents what it
replaced, kept for the reasoning: why it was tried, what it got right, and
the structural argument for why reweighting *today's* cost based on a
*future* corner cannot substitute for a prediction model that actually
represents the road bending. See [`nmpc.md`](nmpc.md) for how that
argument plays out fully.

#### Current-state gain scheduling

Every function below reacts only to curvature/error the car is measuring
*right now*, none of them scan the path ahead. `adaptive_R_scaling` and
`adaptive_R_rate` predate the corner-factor rewrite (below) and carry over
unchanged; the corner-factor blend and the heading-error accel/brake
asymmetry were introduced by that rewrite.

**`adaptive_R_scaling(vx, R)`** increases steering cost with speed:

```
steer_scale = 1 + (1.5 · vx) / (6.0 + vx)      # → 1.0 at vx=0, → 2.5 as vx→∞
accel_scale = 1 + 0.05 · vx                     # gentler linear scale
```

At higher speed, the same steering angle produces much more lateral
acceleration (`a_lat ≈ vx² · κ`), so the same-magnitude steering command is
more destabilising. This Hill-function form was chosen over a straight
linear ramp because it *saturates*: steering cost approaches but never
exceeds 2.5× base, so the controller is never effectively locked out of
steering at very high speed. The half-saturation point (`vx_half = 6.0`)
sits in the same speed range where the kinematic→dynamic model blend
transitions (1-2.5 m/s), so extra steering conservatism ramps up exactly
where the internal prediction model itself becomes less certain.

**`adaptive_R_rate(kappa, R_rate, enable_in_corners=True)`** softens the
steering *jerk* penalty in tight corners, via a floor on the current-position
curvature alone:

```
during_scale = max(0.625, 1 / (1 + 3·κ))     # current-position curvature only
```

`κ` (curvature) is estimated causally from the plant's own current yaw rate
and speed (`curvature_estimate()`: `κ = |yaw_rate| / vx`), it reflects the
curvature the car is *currently experiencing*. In a straight, the full
smoothness penalty applies. In a tight corner, the penalty is floored rather
than removed entirely, enough softening to let the controller make the fast
steering changes a tight corner demands, without ever allowing the rate cost
to vanish completely (which would permit arbitrarily rapid, oscillatory
steering). (This function used to also combine a second, lookahead-driven
floor via `min()`, see "Historical" below; the corner-factor rewrite
removed that half, leaving only the current-position floor shown above.)

Both functions return a **copy** of the base matrix, the tuned weights in
`settings.py` are never mutated, only scaled per-tick on top of.

#### Corner-factor scheduler

Everything from here through `steer_rate_anti_hunt` above reacts to
curvature the car is *at* right now, none of it anticipates a corner ahead.
`mpc_core.py`'s `_corner_factor`/`_low_speed_corner_boost`/`_blend` (and the
offline mirror, `controller/model_utils.py`'s functions of the same name)
turn that same current-position `κ` into a single continuous 0→1 "how much
in a corner am I" fraction, then blend several weights between a straight
endpoint and a corner endpoint on that fraction, replacing the entire
forward-scanning lookahead family documented under "Historical" below.

```
corner_factor = 1 - 1 / (1 + corner_factor_k · |κ|)      # 0 (straight) -> 1 (full corner)
low_speed_boost = corner_factor · max_extra · v_half / (v_half + |car_speed|)
corner_frac = clip(corner_factor + low_speed_boost, 0, 1)
```

- **`corner_factor`** is a saturating curve of current curvature only (no
  forward scan, no decay-distance timer, no hysteresis state), entry and
  exit are exactly symmetric, driven purely by how `κ` itself rises and
  falls tick to tick. `corner_factor_k` (default 8.0) sets the curve's
  sharpness.
- **`low_speed_corner_boost`** adds extra weight toward the "full corner"
  endpoint specifically at low speed, but **only while `corner_factor > 0`**.
  The multiplicative gate on `corner_factor` makes this an exact no-op on
  a straight regardless of speed. That gate is what makes it safe where a
  since-deleted mechanism (`low_speed_steer_rate_boost`, gated on speed
  alone) was not: with no curvature signal to distinguish the two, that
  older mechanism suppressed a car's own turn-in exactly as often as it
  damped the post-exit wobble it was built for.
- **`corner_frac`** (the two combined, clipped to `[0,1]`) drives a shared
  linear blend (`_blend(straight_val, corner_val, corner_frac)`) across four
  weights, each with its own straight/corner endpoint pair in `MPCParams`:
  `Q[0,0]` (`q_ey_straight`/`q_ey_corner`), `Q[2,2]`
  (`q_epsi_straight`/`q_epsi_corner`), `Q[3,3]`
  (`q_r_straight`/`q_r_corner`, which *relaxes* in-corner, unlike the other
  two), and `R_rate[0,0]` (`rrate_steer_straight`/`rrate_steer_corner`,
  likewise relaxing). `R[0,0]` (steering effort) blends toward a **middle**
  value instead (`r_steer_corner_mid`) rather than either extreme, so
  turn-in isn't made maximally cheap right when saturation risk is highest.
- Every quantity above is written into the per-tick telemetry trace
  (`corner_factor`, `low_speed_corner_boost`, `corner_frac`, `Q_ey_base`,
  `Q_ey_eff`, `Rrate_steer_corner_blend`, `R_steer_corner_blend`, ...) so a
  log shows exactly where each blend landed, not just the final QP weights.

**Heading-error-driven accel/brake asymmetry (always-on)**: independent of
`corner_frac` above, a continuous fraction of
current `|e_psi|` scales `r_a_accel` (acceleration effort) toward a boost
ceiling, making the MPC less willing to keep accelerating through a
heading error it should be correcting, and scales `r_a_brake` toward a
floor, freeing up braking authority specifically when heading error is
large:

```
frac_epsi = |e_psi| / (|e_psi| + epsi_ra_half_rad)
r_a_accel_eff = r_a_accel · (1 + (epsi_ra_accel_boost_max - 1) · frac_epsi)
r_a_brake_eff = r_a_brake · (1 - (1 - epsi_ra_brake_floor) · frac_epsi)
```

Not a replacement for `adaptive_R_scaling`'s current-speed-driven `R[0,0]`
scaling above, which this leaves untouched: the two compose.

`adaptive_R_rate`'s current-position floor above used to combine with a
second, lookahead-driven floor via `min()` (whichever was more aggressive
won), see "Historical" below for that half.

**`adaptive_Q_scaling(e_y, Q, enabled)`** softens the lateral-error cost
`Q[0,0]` when the car is already close to the centreline, to reduce
small-error hunting/chatter:

```
scale = floor                                            |e_y| <= ey_lo (0.05 m)
scale = floor + (1-floor)*(|e_y|-ey_lo)/(ey_hi-ey_lo)     ey_lo < |e_y| < ey_hi (0.3 m)
scale = 1.0                                               |e_y| >= ey_hi
```

- **Why:** steering sign-reversal rate was observed rising as `|e_y|` gets
  *smaller* live, the car darting across the centreline rather than
  settling onto it. A quadratic cost pulls toward zero error with the same
  proportional strength no matter how small the error already is, a
  plausible contributor to a correct-overcorrect cycle right where the
  controller should be settling, not correcting.
- **Status:** `ADAPTIVE_Q_SCALING_ENABLED = True` in `settings.py`
  (**enabled by default**, to match the live controller). Still not
  reproduced on the offline recorded-map rollout, there, reversal rate
  rises *with* `|e_y|`, the opposite direction, so it may be a live-only
  symptom of sensor noise, delay-compensation dynamics, or the plant
  behaving differently from the linear model near zero slip. Re-validate
  against `VALIDATION_SUITE`/the recorded map before any further re-tuning
  around it.

**`enable_in_corners` (an `adaptive_R_rate` parameter, on by default)** is
renamed from `disable_in_corners`, whose `True`/`False` polarity was inverted
from what the name suggested. Setting it `False` *undoes* `adaptive_R_rate`'s
softening once estimated curvature exceeds a small "cornering" threshold
(`kappa_straight = 0.03`), restoring the full unscaled `R_rate[0,0]` baseline
instead. Tried disabled and reverted the same day: it caused severe lag
specifically in corners, most likely because the discontinuous cost jump at
the threshold crossing spikes QP solver iterations and invalidates
warm-starts on ticks straddling it. Kept in the code, gated on (softening
active, the setting that avoids the discontinuity), as a documented dead end
rather than deleted, so it isn't accidentally re-tried without this context.

**`steer_rate_anti_hunt(kappa, e_y, R_rate, enabled, e_psi=0.0)`** stacks on
top of `adaptive_R_rate` (not a replacement): multiplies `R_rate[0,0]` **up**
by a fixed boost ceiling (6.0×) instead of softening it, strongest when the
car is simultaneously straight (`κ` near zero), centred (`|e_y|` small), *and*
well-aligned (`|e_psi|` small):

```
boost_kappa = 1 / (1 + 60·|κ|)
boost_ey    = 1 / (1 + 30·|e_y|)
boost_epsi  = 1 / (1 + 23·|e_psi|)
scale = 1 + (6.0 - 1) · boost_kappa · boost_ey · boost_epsi
```

- Each factor saturates independently toward 1.0 as its input shrinks, so
  the full ceiling only applies when all three are near their "straight,
  centred, aligned" ideal, fading smoothly (never snapping) as any one of
  them grows.
- **Why `e_psi`:** guards against a car that enters a straight *misaligned*
  (large `|e_psi|`, small `|e_y|`, e.g. just exited a corner still pointed
  the wrong way). Without it, `κ`/`e_y` alone can't distinguish "straight
  and correctly aligned" from "straight but needs to yaw back into line",
  making exactly the correction it needs artificially expensive.
- **What it covers:** the "already on the line, not cornering" regime
  `adaptive_R_rate` alone doesn't address, that function only ever softens
  the rate cost for corners, never stiffens it for straights.
- **Status:** `STEER_RATE_ANTI_HUNT_ENABLED = True` in `settings.py`
  (**enabled by default**). Experimental, not validated.

#### Historical: the lookahead gain-scheduling family (removed)

An entire family of mechanisms, lookahead corner anticipation, demand
normalisation, U-turn detection, straight-line adjustments, precomputed
corner segmentation (`CornerMap`), curvature forcing, and the low-speed
steering-rate boost, scanned forward along the path every tick and
reweighted `Q`/`R`/`R_rate` in anticipation of a corner not yet reached.
All of it was deleted by the corner-factor rewrite.

It's moved to its own doc, **[`removed_mechanisms.md`](removed_mechanisms.md)**,
because the elimination reasoning (see that doc's "structural limit"
section) is the direct motivation for the nonlinear MPC below, and because
several of the ideas were tried, measured, and rejected for specific,
non-obvious reasons worth not re-discovering by accident.

**Precomputed shaped heading-lead profile (`use_precomputed_heading_profile`)**
is a related but distinct idea, unaffected by the corner-factor rewrite
above: instead of reweighting Q/R given an existing tracking error,
precompute a heading REFERENCE (`psi_target`, a new `raceline.csv` column)
that already leads the geometric path tangent by however much yaw the car
can achieve at its planned speed between here and the next station.
`_error_state` measures `e_psi` against this instead of the geometric
tangent (only `e_psi`, `e_y` is unaffected). Structurally different from
curvature-forcing above: it changes what's true at `k=0` (a real, current
error) rather than telling the QP about a future obligation it's free to
satisfy however is cheapest; synthetic testing found this avoids
curvature-forcing's wrong-direction transient entirely.

See `docs/reference/control_mechanisms.md`'s "Precomputed shaped heading-lead
profile" section for the full design, the fixed-lookahead version that was
tried and rejected first (immediate full-lock steering), and an important
caveat about this track's lack of true straights. Live-tested with a
high-variance, inconclusive result and currently shipped off by default,
see `tuning.md` §4.5c for the full status.

**Delay compensation (`mpc_core.py`'s `predict_ahead()` / `_update_n_delay()`,
live controller only)**: the live car's perception→planning→control→actuation
latency is unknown and time-varying, unlike the offline simulator's fixed
`DELAY_STEPS`.

- **Mechanism:** each solve is told how old the pose it's using actually is
  (`pose_age_s`, from the pose message's own timestamp), converts that into
  an integer step count, and rolls the error state forward through that
  many recently-issued commands before solving, so the QP plans against
  the state the car will actually be in, not a stale measurement.
- **Why filtered first:** the raw step count is low-pass filtered and
  hysteresis-gated before use. Ordinary control-loop jitter would otherwise
  flip the raw `round(pose_age_s / dt)` between adjacent integers tick to
  tick, and each flip discontinuously changes how far the state gets rolled
  forward, injecting a step disturbance into the QP at the control rate
  purely from measurement noise, independent of any real latency change.

**Tracking-error speed gate (`control_utils.tracking_error_speed_gate()`,
both live nodes)**: `curvature_speed()`'s target only looks at path shape,
with no way to know whether the car is actually near that path right now.

- **Mechanism:** scales the speed target down (linear ramp, floored so the
  car always retains enough speed to steer) once lateral or heading
  tracking error grows large.
- **Rise-rate limiter:** paired with a cap on how fast the resulting speed
  target may *rise*, braking is never delayed, only the "speed up"
  direction is capped, so the target doesn't jump around tick to tick.

### Where this is duplicated, and why

`mpc_core.py`'s `MPCController` re-implements `_discrete_model`
(mirrors `model/bicycle_model.py`), `_adaptive_R_scaling`/`_adaptive_R_rate`
(mirrors `controller/model_utils.py`), and `_build_qp` (mirrors `controller/optimiser.py`'s
`init_parameterized_mpc`, including the same `±3.5 m` soft boundary,
`W_slack=10000`, and step-0/subsequent rate-cost split) as self-contained
local copies, rather than importing the shared modules. This is deliberate:
`mpc_core.py` runs inside a ROS 2 node on the real/FSDS vehicle and
must have zero simulator dependencies. **Any change to the cost/constraint
structure in one location must be mirrored in the other**, or weights tuned
by `tuner/offline_tuner.py` will not transfer faithfully to the live controller.

Both QPs enforce a hard per-step slew-rate limit (`du_max`) on top of the soft
`R_rate` cost. This used to be a live-only constraint, which meant the tuner
was optimising against a plant that could change steering arbitrarily fast
while the real car was clamped, a silent parity break independent of any
weight choice.

`controller/optimiser.py` now takes a `du_max` too (baked into
the cached QP alongside `u_min`/`u_max`, and participating in the same
cache-staleness check), and `sim/rollout_core.py` derives it from
`VehicleParams.max_steer_rate * DT` so both sides agree. See
[`docs/reference/`](reference/)'s "Slew-rate limit"
section for the measurement behind the current 180 deg/s value and why the
previous 80 deg/s was the direct cause of live steering chatter.
