# Error States: How Tracking Error Is Defined and Computed

How both controllers turn "the car is here, the path is there" into the numbers their cost functions penalise (`e_y`, `e_psi`, `kappa`, and so on). Written so a concrete triple of car position, car heading and path can be taken and every downstream number reproduced by hand, with a worked example at each step. No prior MPC or vehicle-dynamics background is assumed.

This is not the architecture overview ([architecture.md](architecture.md)) and not the controller math ([lmpc.md](../controllers/lmpc.md), [nmpc.md](../controllers/nmpc.md)). It covers only the error definitions, the differences between the controllers and between live and offline, and why the difference between LTV-QP and NMPC matters.

## The two quantities everything is built from

Picture a line painted on the road (the planner's centreline or a precomputed raceline). At any instant two numbers say how well the car follows it:

- **`e_y`, lateral error**: how far sideways the car is from the line, in metres. Positive is left of the path, negative is right.
- **`e_psi`, heading error**: the angle between where the car points and where the line points at the nearest point, in radians, wrapped to `(-pi, pi]`.

Every other error term (the rate of `e_y`, speed error, curvature) is a derivative or lookup built on those two. Both controllers use the same geometric convention (perpendicular distance, wrapped angle) so a logged `e_y` means the same thing from either. They differ in which reference direction `e_psi` is measured against, and in whether the prediction horizon knows the reference direction changes. That difference is covered in [Why the LTV-QP cannot re-project every future step](#why-the-ltv-qp-cannot-re-project-every-future-step).

## Where each definition lives

| Definition | Live code (`ros2/src/fsae_planning/control/fsae_control/fsae_control/`) | Offline code (`fsae_MPCTest/`) |
|---|---|---|
| LTV-QP `e_y`, `e_psi`, `e_y_dot`, `x0` | `MPCController._error_state()` in `lmpc/controller.py` | `plant_to_tracking_error()` in `model/vehicle_physics/tracking.py`, called from `compute_reference()` in `sim/rollout/reference.py` |
| LTV-QP curvature (gain scheduling only) | `_curvature()` in `lmpc/adaptive_gains.py` | `curvature_estimate()` in `controller/model_utils.py` |
| Linear bicycle model `Ad`, `Bd` | `MPCController._discrete_model()` in `lmpc/controller.py` | `model/bicycle_model.py` |
| NMPC projection, `PathReference` (spline `kappa(s)`, `psi_ref(s)`) | `PathReference` in `nmpc/reference.py` | `PathReference` in `controller/nmpc/reference.py` |
| NMPC dynamics (`_f`, includes `e_psi_dot = r - kappa * s_dot`) | `_f()`, `_f_scalar()` in `nmpc/dynamics.py` | `_f()`, `_f_scalar()` in `controller/nmpc/dynamics.py` |
| NMPC cost rows (`_outputs`) | `nmpc/outputs.py` | `controller/nmpc/outputs.py` |

The live tree and the mirror in `fsds_simulator/` hold identical files.

## Live and offline do not measure the LTV-QP error identically

| | Live LTV-QP | Offline LTV-QP | NMPC, live and offline |
|---|---|---|---|
| Point measured | `car_pos + lf * [cos(yaw), sin(yaw)]`, labelled the front axle | The plant's own `X`, `Y` (centre of mass), with no `lf` offset | `car_pos + lf * [cos(yaw), sin(yaw)]`; offline `car_pos` is the plant `X`, `Y` as estimated (sensor noise applies) |
| Reference point | Nearest waypoint, no interpolation | Nearest waypoint, then projected onto the nearer adjacent segment (`get_interpolated_ref_point`) | Nearest waypoint, refined along the path by the `along` component |
| Reference heading | Raw tangent of the one segment ahead of the nearest waypoint | Linear interpolation of per-segment headings | Spline `psi_ref(s)` at the refined arc length |
| Curvature for gain scheduling | Finite difference about 1 m ahead | `abs(yaw_rate / v_x)` from the plant state | Spline `kappa(s)` at the projected arc length |

Offline, `plant_to_tracking_error()` supplies `e_y`, `e_psi` and `e_psi_dot`, and the rollout computes `e_y_dot` from `v_x`, `v_y` and `e_psi` directly. Whether the offline LTV-QP should measure at the front axle like the live one is not recorded. The live docstring calls `car_pos` the rear-axle reference point, while the code adds `lf` (0.70 m), a distance that lands on the centre of mass from a rear-axle origin, not on the front axle. Which point the FSDS odometry reports is not verified here.

## LTV-QP live error calculation, step by step

Source: `MPCController._error_state()`.

**Step 1. Find the point being measured.**

```
front_axle = car_pos + lf * [cos(car_yaw), sin(car_yaw)]
```

`lf` is the distance from the centre of mass to the front axle (0.70 m). The code names the result the front axle, the end that steers. See the note above on the reference point.

**Step 2. Find the nearest waypoint.**

```
base_idx = argmin_i ( distance(front_axle, path[i]) )
```

A brute-force search over every waypoint.

**Step 3. Find the path direction there (`path_yaw`).**

```
segment  = path[base_idx + 1] - path[base_idx]       # at the last waypoint, the previous segment
path_yaw = atan2(segment.y, segment.x)
```

A plain two-point finite difference, with no smoothing and no lookahead.

**Step 4. Project the offset onto "sideways to the path" (`e_y`).**

```
dx, dy = front_axle - path[base_idx]
e_y = dy * cos(path_yaw) - dx * sin(path_yaw)
```

This rotates the offset `(dx, dy)` into the path's own frame ("along" and "sideways") and keeps only the sideways part. It is not the distance to the nearest waypoint. A car 5 m further along a straight path but only 0.2 m sideways has a nearest-point distance of `sqrt(5² + 0.2²)`, about 5.004, which would report it as 5 m off a line it is almost on. The rotation discards the along-path component (`dx * cos + dy * sin`) and keeps the perpendicular one.

**Step 5. Heading error (`e_psi`).**

```
e_psi = atan2( sin(car_yaw - path_yaw), cos(car_yaw - path_yaw) )
```

Plain subtraction breaks across the plus and minus 180 degree wrap (car at 179 degrees, path at -179 degrees is a 1 degree error, not 358). Passing the difference through `atan2(sin, cos)` always returns the equivalent angle in `(-pi, pi]`.

Two optional substitutions change only the reference `e_psi` is measured against, never `e_y`:

- `ref_heading_rate_limit_enabled` (off): limits how fast `path_yaw` may change per tick (`ref_heading_rise_rate_deg_s`, 90 degrees/s). Offline: `settings.REF_HEADING_RATE_LIMIT_ENABLED` in `sim/rollout/reference.py`, applied on the planner branch only.
- `use_precomputed_heading_profile` (off): replaces `path_yaw` with the shaped `psi_target` at `base_idx` ([control_mechanisms.md](control_mechanisms.md#precomputed-shaped-heading-lead-profile-ltv-qp-only-off)).

**Step 6. Rate of lateral error (`e_y_dot`).**

```
e_y_dot = car_speed * sin(e_psi) + car_vy * cos(e_psi)
```

A car pointed slightly off the path while moving forward drifts sideways at a rate proportional to `sin(e_psi)` times speed. The second term adds real sideslip (`car_vy`, body-frame lateral velocity, 0.0 when the caller does not measure it).

**Step 7. Speed error (`e_v`).**

```
e_v = car_speed - desired_speed
```

`desired_speed` is first low-pass filtered with a first-order filter (`alpha = 0.08` in the LTV-QP, so a step in the request does not shock the controller). The NMPC uses `nmpc_v_des_filter_alpha` (0.09).

**Step 8. Curvature preview (`kappa`), used only for gain scheduling.**

Walk forward from `base_idx` until about 1 m of path has accumulated, then take the angle change between the two segments there divided by their mean length:

```
dpsi  = wrap(yaw_after - yaw_before)
kappa = dpsi / average_segment_length
```

This never enters the prediction model. It only reweights the cost for this tick's solve ([control_mechanisms.md](control_mechanisms.md#corner-factor-scheduler-ltv-qp-weights-follow-the-current-curvature)).

**Step 9. Assemble the 8-state vector.**

```
x0 = [ e_y, e_y_dot, e_psi, car_yaw_rate, e_v, 0.0, delta_act, a_act ]
```

`x0[3]` is the measured absolute yaw rate, which equals `e_psi_dot` only on a straight. `x0[5]` is an unused placeholder. `delta_act` and `a_act` are not measured. They are the controller's own record of the last steering and acceleration commands, because a real rack and throttle do not reach a command instantly. Delay compensation (`predict_ahead`) then rolls `x0` forward through the commands still in flight.

### Worked example

The path near the car is a straight line running due East:

```
path[10] = (10.0, 0.0)
path[11] = (10.5, 0.0)
```

The car reports position `(9.35, 0.3)`, heading `car_yaw = 5` degrees, speed 8 m/s, `car_vy = 0`, `desired_speed = 10` m/s, `lf = 0.70`.

```
front_axle = (9.35, 0.3) + 0.70 * (0.9962, 0.0872) = (10.047, 0.361)
nearest    = path[10]                  (distance about 0.364)
path_yaw   = atan2(0.0, 0.5) = 0 degrees
e_y        = 0.361 * cos(0) - 0.047 * sin(0) = 0.361 m      (car is north of the line)
e_psi      = wrap(5 - 0) = 5 degrees = 0.0873 rad
e_y_dot    = 8 * sin(5 degrees) + 0 = 0.698 m/s             (drifting away at about 0.7 m/s)
e_v        = 8 - 10 = -2 m/s
x0 = [0.361, 0.698, 0.0873, car_yaw_rate, -2.0, 0.0, delta_act, a_act]
```

The cost then penalises these numbers (`Q_0 * e_y² + Q_1 * e_y_dot² + ...`, see [lmpc.md](../controllers/lmpc.md)).

### The LTV-QP error is one snapshot

Every step uses only the pose, speed and path geometry at this instant. Nothing about where the path goes in two seconds enters. The horizon prediction then starts from this single `x0` and rolls it forward with the linear model, with no awareness of the path's shape. That is the limitation the next section explains.

## Why the LTV-QP cannot re-project every future step

The horizon prediction applies one formula 35 times to roll `x0` forward. A natural question is why the prediction does not, at each future step, take the predicted position, find the nearest path point there, and recompute `e_y` and `e_psi` against it, as Steps 1 to 5 do at `t = 0`.

The idea is sound and is what the NMPC does. The obstacle is how the solver works.

The LTV-QP is fast because "state `x` plus input `u` gives next state `x'`" is one fixed matrix multiplication, `x' = Ad * x + Bd * u`. That relation is linear, with no state multiplied by another state, no branching and no lookups, so OSQP can find the provably best steering sequence in one solve. "Find the nearest waypoint" is a search, not a fixed formula:

- It is not smooth. As the predicted position moves, the nearest waypoint can jump to a non-adjacent one, and a linear model cannot represent a jump.
- Putting the search inside the dynamics destroys the fixed-matrix structure. The problem stops being a Quadratic Program and becomes a Nonlinear Program, which is what the NMPC solves.

The middle-ground attempt was to precompute the curvature at each horizon step before the solve and add it as a fixed forcing term in the `e_psi` prediction. It failed structurally: the term is a known fact about the future, independent of the chosen inputs, so the solver is free to defer paying for it, and at every gain that produced a response it steered away from the corner first. The gain sweep and numbers are in [retired_mechanisms.md](retired_mechanisms.md#curvature-forcing-term-removed-the-solver-defers-external-data).

## NMPC error calculation, step by step

Source: `PathReference` (`project()`, `kappa_at()`, `psi_ref_at()`) and `_f()`.

The state vector gains **`s`**, arc length travelled along the path, and curvature is looked up at whatever `s` the model currently predicts, not at a fixed step index. Since `s` evolves as a normal state (driven by the car's predicted speed and heading), a bend at some future distance is "there" the instant the horizon reaches it, and there is no separate slot to defer paying into.

State: `x = [s, e_y, e_psi, v_x, v_y, r, delta_act, a_act]`.

**Step 1. Build a smooth description of the path, once for a static path.**

```
arc[0] = 0
arc[i] = arc[i-1] + distance(path[i-1], path[i])
x_spline = CubicSpline(arc, path.x)        y_spline = CubicSpline(arc, path.y)
```

A cubic spline passes through every waypoint with continuous first and second derivatives. Curvature is a second derivative, so this avoids the abrupt steps that raw waypoint differencing gives. Trailing zero-length segments (a live planner path padded with its last point) are dropped first, because they break the spline and make the horizon predict the corner stopping.

**Step 2. Curvature and reference heading as functions of `s`.**

```
psi_ref(s) = atan2( y'(s), x'(s) )
kappa(s)   = ( x'(s) * y''(s) - y'(s) * x''(s) ) / ( x'(s)² + y'(s)² )^1.5
```

The derivatives are the spline's own analytic polynomials, evaluated on a `nmpc_curvature_dense_step` (0.5 m) grid and `kappa` is clipped to `nmpc_kappa_clip` (0.5). With `nmpc_spline_reference_enabled=false` the older dense-resample, moving-average and finite-difference pipeline is used instead.

**`psi_ref` and `kappa` must come from the same smoothed curve.** Measuring `e_psi` against the raw segment tangent while `kappa` came from the spline produced a period-2 steering limit cycle of about plus and minus 25 degrees through tight corners. Raw tangents step by about `segment_length / radius` (5.7 degrees per 0.5 m waypoint on a 5 m radius hairpin), and the controller read each artificial step as real error. This is a deliberate behavioural difference from the LTV-QP's raw tangent, bounded by the 1.5 m smoothing window ([late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), section 16.6).

**Step 3. Project the car onto the path, once at `t = 0`.**

```
front_axle = car_pos + lf * [cos(car_yaw), sin(car_yaw)]
base_idx   = argmin_i ( distance(front_axle, path[i]) )
s_base     = arc[base_idx]
path_yaw   = psi_ref(s_base)

dx, dy = front_axle - path[base_idx]
e_y    = dy * cos(path_yaw) - dx * sin(path_yaw)          # same formula as LTV-QP Step 4
along  = dx * cos(path_yaw) + dy * sin(path_yaw)          # discarded by the LTV-QP
s0     = s_base + along                                   # refined arc-length station
path_yaw = psi_ref(s0)                                    # heading re-read at the refined station
e_psi  = wrap(car_yaw - path_yaw)
```

Two refinements over the LTV-QP: `path_yaw` comes from the spline, and the `along` component is kept to refine `s0` (on a tight corner the correction can exceed a metre, over which the reference heading changes, so it is re-read at `s0`).

**Step 4. How error evolves across the horizon (the actual fix).** At every predicted stage, given the model's predicted `(s, e_y, e_psi, v_x, v_y, r)`:

```
kap       = kappa(s)                                      # curvature AT THE PREDICTED s
denom     = 1 - kap * e_y                                 # Frenet distortion, floored at 0.25 in magnitude
s_dot     = (v_x * cos(e_psi) - v_y * sin(e_psi)) / denom
e_y_dot   = v_x * sin(e_psi) + v_y * cos(e_psi)           # same form as LTV-QP Step 6
e_psi_dot = r - kap * s_dot                               # the term the LTV-QP model lacks
```

- `s_dot` is how fast the car advances along the path, not through the world. A car on the inside of a bend covers less arc per metre travelled than one on the outside, and `denom` captures that. It is floored so the model cannot divide by zero on an extreme offset.
- `e_psi_dot` says the heading error changes at the car's own yaw rate minus how fast the path turns underneath it. That is the literal statement that the road is bending. The LTV-QP model effectively sets `kap = 0` always, which is exact on a straight and increasingly wrong in a sharp corner.
- Because `s` is a state the rollout predicts forward, a bend ten steps ahead is already shaping today's plan. Curvature is looked up inside the equation the solver is solving, using a state it is choosing. In the failed forcing attempt it was a fixed number handed in as data.

### Worked example

A constant-curvature left bend, radius 20 m (`kappa = 0.05` per metre), the car at its start (`s = 50`, `e_y = 0`, `e_psi = 0`), `v_x = 15` m/s, `v_y = 0`, `r = 0`:

```
kap = 0.05        denom = 1 - 0.05 * 0 = 1.0
s_dot     = 15 * cos(0) / 1.0 = 15 m/s
e_y_dot   = 15 * sin(0) = 0
e_psi_dot = 0 - 0.05 * 15 = -0.75 rad/s
```

The LTV-QP model at the same instant gives `e_psi_dot = r = 0`, forever. The NMPC's rollout predicts `e_psi` drifting at -0.75 rad/s (about -43 degrees/s) immediately, purely because the path curves under a car that has not turned its wheels. The solver reacts to that predicted error through the normal cost, with no bolted-on lookahead heuristic. A few steps later each step's output becomes the next step's input, but the update is nonlinear (`kap`, a function of `s`, multiplies `s_dot`, another state-dependent quantity), so it is no longer "state times a fixed number".

### Why this survives where curvature forcing did not

| | Curvature forcing (removed) | NMPC |
|---|---|---|
| How curvature enters | A fixed `w[k]` computed before the solve, added to the prediction as data | `kappa(s)` looked up inside the dynamics using the predicted state `s` |
| Can the solver defer it? | Yes, it is true whatever inputs are chosen, so paying now or later is free | No, `e_psi_dot` at a predicted moment depends on where `s` has got to by then |
| Measured wrong-direction steering | About 7 consecutive ticks at magnitude comparable to the correct command | One step of -0.33 degrees against a +4.88 degree correct-direction peak, the true optimum (IPOPT gives -0.327), 15 to 30x smaller |

The NMPC figure is from a probe with the car dead on the line before a known bend, at the shipped horizon of 20 steps. At a horizon of 35 with two iterations the same probe read -3.0 degrees, so re-measure it if the horizon is lengthened ([late_turn_in_investigation.md](../logs/late_turn_in_investigation.md), section 16.6).

## Side-by-side summary

| | LTV-QP | NMPC |
|---|---|---|
| When error is computed | Once at `t = 0` from the measured pose | Once at `t = 0`, then re-derived at every horizon step as `s`, `e_y`, `e_psi` evolve |
| Reference heading source | Raw two-point segment tangent | Analytic cubic-spline fit at the refined arc-length station |
| Does the prediction know the path curves? | No: `e_psi_dot = r` only | Yes: `e_psi_dot = r - kappa(s) * s_dot` |
| How curvature is used | Only to reweight this tick's cost; never enters the prediction | Enters the predicted dynamics at every step |
| Problem class | Convex QP, one solve, guaranteed global optimum | Sequence of QPs by Gauss-Newton SQP, no global guarantee, about 9 ms mean solve at horizon 20 |
| Consequence | Needs a gain-scheduling layer to approximate anticipation it cannot have, and turns in late on sharp corners | No anticipation machinery needed; the adaptive gain schedule is inactive |

For the cost function that turns these terms into one number to minimise, see [lmpc.md](../controllers/lmpc.md). It is the same in spirit for both controllers, with `q_r` weighting yaw rate in the LTV-QP and heading-error rate in the NMPC.
