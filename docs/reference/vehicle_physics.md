# The Nonlinear Vehicle Plant, in Plain English

Walkthrough of the physics in the `model/vehicle_physics/` package: what each state and parameter means, what changing it does, and where to look when the simulated car misbehaves.

The plant simulates a car the way the world pushes it around (tyres gripping, springs compressing, weight shifting under braking and cornering) so that the controllers, which use far simpler models, have a realistic stand-in to be tested against. It is the ground truth of the offline rollout, not a validated copy of FSDS or the real car ([simulator_fidelity.md](simulator_fidelity.md)).

## Where the code lives

| File | Contents |
|---|---|
| `model/vehicle_physics/state.py` | `IDX_*` state indices, `N_STATES`, `init_plant_state()` |
| `model/vehicle_physics/params.py` | `VehicleParams`, every physical constant, `alat_ceiling_at()` |
| `model/vehicle_physics/tyres.py` | `pacejka_lateral_mf94()`, `pacejka_longitudinal_mf94()` |
| `model/vehicle_physics/plant_step.py` | `step_nonlinear_plant()`, one control tick of the plant |
| `model/vehicle_physics/tracking.py` | `plant_to_tracking_error()` and the reference-point helpers |

The package `__init__.py` re-exports the public names, so callers import from `model.vehicle_physics`. The plant steps at 20 Hz (`dt = 0.05` s) and sub-steps four times internally (0.0125 s each, explicit Euler) to stay stable through the stiff suspension and tyre-relaxation dynamics.

## Three models of the car exist

1. **The LTV-QP's internal model** (`model/bicycle_model.py`): an 8-state linear bicycle model in path-error coordinates. Simple on purpose, because the QP is solved every tick.
2. **The NMPC's internal model** (`controller/nmpc/dynamics.py` offline, `fsae_control/nmpc/dynamics.py` live): an 8-state nonlinear Frenet bicycle model with a lateral-acceleration ceiling.
3. **This plant**: 25 states with soft suspension, tyres that take distance to build grip, weight transfer and aerodynamics.

The gap between what a controller predicts and what the plant does is deliberate. It stops the controller from grading its own homework. The plant does not include the tyre-force cap the real FSDS car shows unless `alat_ceiling_enabled` is on (see the last section).

## The 25 states

The car's situation at any instant is a 25-number vector (`N_STATES = 25`). Indices are the `IDX_*` constants in `state.py`.

| # | Name | What it describes |
|---|---|---|
| 0 | `X` | Where the car is, left to right, on the track map. |
| 1 | `Y` | Where the car is, forward to back, on the track map. |
| 2 | `psi` | Which way the car points (heading), not which way it moves. The two differ when the car slides. |
| 3 | `vx` | Forward speed along the car's own nose-to-tail axis. |
| 4 | `vy` | Sideways speed across the car. Non-zero means it is sliding. |
| 5 | `r` | Yaw rate, how fast the heading rotates. High `r` with little actual cornering means a spin. |
| 6 | `delta_act` | The steering angle the front wheels sit at right now, after the steering-rack lag. |
| 7 | `a_act` | The acceleration or braking delivered right now, after its own lag. |
| 8, 9 | `omega_RL`, `omega_RR` | Rear wheel spin speeds. These are the driven wheels. |
| 10 to 13 | `z_FL`, `z_FR`, `z_RL`, `z_RR` | Spring compression at each corner relative to rest. Positive is squashed, negative is drooping. |
| 14 to 17 | `dz_FL_dt` to `dz_RR_dt` | How fast each corner's suspension is moving. The dampers react to this. |
| 18 to 21 | `Fy_FL_rlx` to `Fy_RR_rlx` | The sideways grip force each tyre is producing now. "Relaxed" because grip chases its ideal value with a lag (see tyre relaxation). |
| 22, 23 | `omega_FL`, `omega_FR` | Front wheel spin speeds. Not driven, so they mostly track ground speed. |
| 24 | `alat_lim` | Build-up of the lateral-acceleration-ceiling restoring term. A memory state, not a directly physical quantity. |

### The plant's first 8 states are not the controllers' 8 states

The plant vector is not laid out like either controller's vector. Only some slots hold the same quantity:

| Slot | Plant | LTV-QP error vector | NMPC Frenet vector |
|---|---|---|---|
| 0 | `X` | `e_y` | `s` |
| 1 | `Y` | `e_y_dot` | `e_y` |
| 2 | `psi` | `e_psi` | `e_psi` |
| 3 | `vx` | yaw rate `r` | `v_x` |
| 4 | `vy` | `e_v` | `v_y` |
| 5 | `r` | unused placeholder | `r` |
| 6 | `delta_act` | `delta_act` | `delta_act` |
| 7 | `a_act` | `a_act` | `a_act` |

- With the LTV-QP only slots 6 and 7 mean the same thing. With the NMPC slots 3 to 7 match by quantity, while slots 0 to 2 are path-relative there and global here.
- A conversion is always needed. `plant_to_tracking_error()` in `tracking.py` returns `(e_y, e_y_dot, e_psi, e_psi_dot, delta_act, a_act, vx)` for the LTV-QP, and the rollout builds `x0` from those values ([error_states.md](error_states.md)).
- Module docstrings in `state.py` and `__init__.py` still claim indices 0 to 7 are identical to the MPC's 8 states and still say 24 states. Both statements are stale. `N_STATES` (25) and this table are correct.

## Vehicle parameters: what each does and what changing it does

Everything lives in `VehicleParams` (`params.py`). The "Default" column is the value in the file today. Three module-level scales multiply groups of values: `GRIP_SCALE = 1.1` (tyre stiffness and peak factor), `INERTIA_SCALE = 0.8` (wheel and drivetrain inertia), `COASTING_SCALE = 3.0` (rolling drag).

### Geometry: the car's basic shape

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `lf`, `lr` | 0.70, 0.85 m | Distance from the centre of mass to the front axle (`lf`) and the rear axle (`lr`). | Raising `lf` puts more weight over the rear, so the car understeers more (front loses grip first). | Less weight over that axle, so that end grips better and the other worse. |
| `m` | 255 kg | Mass of car and driver. | More force needed to accelerate, brake and corner. Tyres are more loaded, which helps peak grip a little and hurts accel and braking much more. | Quicker acceleration, braking and direction change. |
| `Iz` | 150 kg m² | Yaw inertia, how resistant the car is to spinning. | More stable but lazier turn-in. | Sharper turn-in, easier to spin. |
| `tf`, `tr` | 1.25, 1.20 m | Track width, front and rear. | More resistance to side-to-side weight roll. | Rolls and transfers weight more readily. |
| `h_cg` | 0.25 m | Height of the centre of mass. | More weight transfer under braking and cornering, worse balance. | Less transfer, more predictable. |

`lf`, `lr`, `m`, `Iz` must match the live controller constants (parity). The true values are not in the FSDS repo, so they are a deliberate choice: `lf < lr` makes the bicycle model understeer, which is stable at every speed.

### Actuator limits: what the car may do

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `max_steer` | 25 degrees | The furthest the rack can turn the front wheels. | Tighter minimum turning radius. | Cannot turn as sharply whatever is commanded. |
| `max_steer_rate` | 180 degrees/s | How fast the rack can move. Feeds the controllers' `du_max` slew limit ([control_mechanisms.md](control_mechanisms.md#slew-rate-limit-du_max-identical-on-both-sides-at-180-degs)). | The controller may command faster steering changes. | Slower allowed changes, more rate-limit saturation. |
| `max_accel` | 12 m/s² | Hardest forward acceleration. | Faster out of corners, if the tyres can deliver it. | Slower, more conservative. |
| `max_accel_brake` | -7 m/s² | Hardest braking (negative is deceleration). | A more negative value brakes harder and later. | Longer stopping distances. |
| `max_v` | 16.7 m/s | Absolute speed ceiling (the real car tops out near 60 km/h). | Higher top speed. | Capped lower even on a straight. |

### Unsprung mass, tyre geometry and rotational inertia

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `m_us` | 7.5 kg | Mass of each wheel, tyre and upright, the parts below the springs. | Sluggish over bumps, less consistent grip on rough surfaces. | Tracks the road surface more faithfully. |
| `r_eff` | 0.2286 m | Effective rolling radius. | Wheel spins slower for the same ground speed, more ground force per unit of wheel torque, more inertia to overcome. | The opposite. |
| `I_wheel` | 0.72 kg m² (0.9 x scale) | How much a wheel resists changing spin speed. | Slower wheel response, can mask wheelspin or lockup onset. | Twitchier, wheelspin and lockup show sooner. |
| `I_drivetrain` | 0.04 kg m² (0.05 x scale) | Same, for the motor and gearbox, felt at the driven rear wheels only. | As `I_wheel`, rear only. | As `I_wheel`, rear only. |

### Suspension: springs, dampers, anti-roll bars

Springs hold the car up and push back when compressed. Dampers resist how fast the suspension moves. Anti-roll bars link left and right so the car resists leaning.

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `k_susp_f`, `k_susp_r` | 25000, 30000 N/m | Spring stiffness per corner. | Less roll, dive and squat, harsher over bumps. | Smoother ride, more body movement that can upset balance. |
| `c_damp_f`, `c_damp_r` | 1500, 1800 N s/m | Damper rate. | Settles faster after a bump, transmits sharper impacts. | Wallows longer after a disturbance. |
| `k_arb_f`, `k_arb_r` | 8000, 6000 N/m | Anti-roll bar stiffness. | Less roll on that axle, and that axle loses relative grip in hard corners (load shifts to the outside tyre). Stiffening the front bar adds understeer. | More roll on that axle, more even grip. |
| `z_max`, `z_min` | +0.040, -0.040 m | Bump and droop stops. | More travel before hitting a hard stop. | Hits the stops sooner, with a harsh jolt. |

### Kinematic camber: how suspension movement tilts the tyre

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `camber_gain` | 8.73 rad/m | Camber change per unit suspension travel. | More grip on the loaded outside tyre as it compresses, unsettling if overdone. | More predictable, leaves some cornering grip unused. |
| `camber_stiff_f`, `camber_stiff_r` | 0.15, 0.12 | How much extra sideways force a given camber angle produces. | Camber becomes a more powerful and more sensitive lever. | Camber matters less. |

## Tyre model: full MF94 Pacejka

### What a tyre model is

A tyre does not grip like a rigid block. The contact patch deforms before it slides, so force depends in a curved way on how much the tyre is asked to slip:

- **Slip angle** (`alpha`): the angle between where the tyre points and where it travels. It produces sideways (lateral) force, the force that turns the car.
- **Slip ratio** (`kappa`): the mismatch between wheel speed and ground speed. It produces forward and backward (longitudinal) force.

A tyre model is a curve that gives force from slip and load. A real tyre has a grip peak, and past it grip falls off, which is what a slide feels like.

### What "Pacejka" and "MF94" mean

Hans Pacejka's Magic Formula (1994 version) reproduces the S-shaped force curve of a real tyre with one compact equation and a few coefficients. "Full" here means the code keeps the curved shape, offsets and camber effects, unlike the straight-line tyre in the controllers' internal models.

### What the coefficients do

Lateral defaults in `params.py`: front `B_f` 16.5, `C_f` 1.45, `D_f` 1.1, `E_f` -1.5; rear `B_r` 13.2, `C_r` 1.45, `D_r` 1.1, `E_r` -1.8 (`B` and `D` include `GRIP_SCALE`).

| Coefficient | Plain English | Increase it | Decrease it |
|---|---|---|---|
| `B` (stiffness) | How steeply force ramps up for small slip. | Peak grip at a smaller slip angle, sharper and grabbier near centre. | Needs more slip for the same force, vaguer. |
| `C` (shape) | How rounded or peaky the top of the curve is. | Flatter, broader peak, more forgiving near the limit. | Sharper peak, grip falls away more suddenly. |
| `D` (peak) | Scales the maximum force (with `mu` and the load `Fz`). | Higher grip ceiling. | Lower ceiling. |
| `E` (curvature) | Shape near and past the peak. Negative here, typical of a racing slick. | More negative sharpens the peak, a more on/off tyre. | Toward zero rounds it out, a gentler slide. |
| `Sv` (vertical offset) | Small force at zero slip from tyre construction. | A built-in pull to one side when straight. | Perfectly neutral at zero slip. |
| `Sh` (horizontal offset) | Shifts where zero force occurs on the slip axis. | Shifts the neutral point one way. | The other way, or none at zero. |

- `pacejka_lateral_mf94` uses slip angle for cornering force and adds camber thrust.
- `pacejka_longitudinal_mf94` uses slip ratio for accel and braking force. Rear coefficients `Bx_r` 12, `Cx_r` 1.65, `Dx_r` 1.0, `Ex_r` -0.5.

### Friction and load sensitivity

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `mu` | 1.76 (1.6 x scale) | Peak grip of the compound and surface. | More grip everywhere. | Slides, spins and locks up sooner everywhere. |
| `k_sens` | 0.00012 per N | Load sensitivity: grip is not proportional to load, heavier loading gives diminishing returns. | Heavily loaded tyres (outside front in a corner) benefit less from extra load. | Grip stays closer to proportional to load. |
| `road_mu` | 1.0 (argument to `step_nonlinear_plant`) | Multiplier on `mu` for the surface: about 0.6 damp, about 0.3 wet. | Grippier surface. | Slicker surface. |

`mu` was set inside the published FSAE 13-inch slick range (1.4 to 1.8) because FSDS does not publish its tyre model. Scaling `mu` to imitate the FSDS lateral-acceleration cap was tried and failed, see [sim_to_real_investigation.md](../logs/sim_to_real_investigation.md).

### Tyre relaxation: grip does not appear instantly

The rubber and carcass must deform, which takes distance travelled, not time. The plant models it with a relaxation length: `dFy/dt = (vx / sigma) * (Fy_steady_state - Fy)`.

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `sigma_y_f`, `sigma_y_r` | 0.45, 0.40 m | Distance the tyre needs to build to full grip after a slip change. | Laggier grip build-up, a less responsive feel at speed. | Near-instant grip response. |

This is the difference between states 18 to 21 (the lagged force applied now) and the steady-state force computed fresh each sub-step (`Fy_ss`, what the tyre is heading toward). At 10 m/s with `sigma` 0.45 m the time constant is about 45 ms, significant at 20 Hz.

### The friction ellipse: grip is one shared budget

A tyre has one finite amount of grip. Using some for longitudinal force leaves less for lateral. The plant enforces it per wheel by scaling lateral force by `sqrt(1 - (Fx / Fmax)²)`, where `Fmax = mu_eff * Fz`. This is why hard braking while cornering hard loses the car.

## Other physics features

### Aerodynamics

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `Cd_A` | 0.33 m² | Drag area. | More drag, lower top speed. | Less drag. |
| `Cl_A_f`, `Cl_A_r` | 0.645, 0.855 m² | Downforce area, front and rear (a 43/57 split). | More grip on that axle at speed. | Less free grip, especially at high speed. |
| `Cl_pitch_sens` | 0.03 | How much braking pitch shifts downforce toward the front. | Braking loads the front more and can loosen the rear. | Little effect on balance. |

### Rolling resistance and stiction

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `Crr` | 61.5 N (20.5 x `COASTING_SCALE`) | A constant drag force while moving. It lumps rolling resistance with drivetrain drag that is not modelled elsewhere, and is the deliberate coast-down knob. | Coasts to a stop faster. | Coasts further. |
| `F_stiction` | 600 N | Breakaway force to start a stationary car, across all four wheels. | Takes more force to get moving. | Starts more easily. Also sets the roughly 2.3 m/s² needed to break static friction. |

### Actuator lag

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `tau_delta` | 0.08 s | Time for the steering rack to catch up to a command. | Laggier steering. | Near-immediate steering. |
| `tau_a` | 0.02 s | Same, for the throttle and brake actuator. | Accel and braking lag behind commands. | Near-immediate response. |

### Torque vectoring (optional, off)

`tv_gain` is an argument to `step_nonlinear_plant`, not a `VehicleParams` field. Default 0 disables it. When set it splits rear drive force between the wheels in proportion to yaw rate to help rotate the car. Too much makes the car nervous.

### Linear cornering stiffness for the controllers

`VehicleParams.Cf` and `Cr` are computed from the Pacejka coefficients (`mu_eff * Fz_nominal * B * C * D` per axle), not hard-coded, so the linear model's initial slope matches the plant. The live controllers keep a hard-coded copy (`Cf` 29155.5, `Cr` 19512.3 N/rad in `MPCController.__init__` and the NMPC `_Plant`). Changing any tyre coefficient means updating both copies by hand ([offline_live_parity.md](offline_live_parity.md)).

## The FSDS lateral-acceleration ceiling (`alat_ceiling*`)

This is not a real tyre-grip limit. It models a simulator quirk: FSDS caps sustained lateral acceleration well below what the plant's tyres reach unaided (about 14 m/s² measured offline, against about 12.3 m/s² on a real lap), and the measured cap varies with speed. Without the model, the offline sim takes corners the FSDS car cannot, and weights tuned against it assume cornering authority that does not exist in FSDS.

Mechanism: once `|vx * r|` (the plant's lateral-acceleration estimate) exceeds a speed-dependent ceiling, a restoring yaw moment pulls the yaw rate back. In the default `'pi'` mode the excess is a leaky integral held in state 24 (`alat_lim`), so the settled value pins at the ceiling for any gain. The car can briefly exceed the ceiling before being pulled back, which reproduces the roughly 30% overshoot measured on a sudden corner entry and which a hard clip cannot. FSDS's internal cause is unknown. The model reproduces the external symptom only.

The ceiling is `alat_ceiling_at(vx) = max(alat_ceiling, alat_ceiling_intercept + alat_ceiling_slope * vx)`.

| Parameter | Default | Plain English | Increase it | Decrease it |
|---|---|---|---|---|
| `alat_ceiling_enabled` | `True` | Whether the mechanism runs. It models FSDS, not the physical car. | Set `False` to recover the unconstrained plant for real-vehicle work. | n/a |
| `alat_ceiling`, `alat_ceiling_slope`, `alat_ceiling_intercept` | 7.5 m/s², 0.47, 2.46 | Flat low-speed floor and the rising line above it (the line exceeds the floor above about 10.7 m/s). | A higher ceiling, more sustained lateral g allowed. | Pushback engages sooner and more often. |
| `alat_ceiling_mode` | `'pi'` | The control law. `'p'` is a rejected proportional law kept only to reproduce the measurement that rejected it. | n/a | n/a |
| `alat_ceiling_gain` | 450 N m per m/s² | Strength of the restoring moment, fitted to the measured peak. | Firmer pull-back, less overshoot. | Weaker pushback, more overshoot. |
| `alat_ceiling_tau` | 0.40 s | Lag on the restoring moment. Affects only the transient under the integral law. | Slower to engage, larger overshoot on a sudden entry. | Faster, less overshoot. |

Derivation, the open-loop measurements and what remains unresolved: [simulator_fidelity.md](simulator_fidelity.md). Check the plant still reproduces the measurements after any change with `python -m tuner.validation.plant_openloop_validation`.

## Symptom to likely parameter

- **Understeer (pushes wide)**: raise front grip (`mu`, front `B_f`, `D_f`), reduce `k_arb_f`, or reduce front downforce loss under braking.
- **Oversteer (rear steps out)**: the mirror image. Raise rear grip and downforce, reduce `k_arb_r`, check rear `B_r`, `D_r`.
- **Laggy steering response**: check `tau_delta` and `sigma_y_f`, both add delay between input and response.
- **Rear wheelspin under power**: check `mu`, the per-wheel friction ceiling (`mu * Fz`), or whether `max_accel` demands more force than the tyres can give.
- **Harsh ride over bumps**: reduce `k_susp_*` or `c_damp_*`.
- **Too much roll or wallow**: raise `k_arb_*` or `k_susp_*`.
- **Coasts too far off-throttle**: raise `Crr`.
- **Corners harder offline than in FSDS**: check `alat_ceiling_enabled` is `True` and the ceiling parameters match the latest measurement. This is the known modelled gap, not a tyre-grip mismatch.

## Short glossary

Terms specific to this plant. General terms are in [glossary.md](glossary.md).

- **Load transfer**: under braking, accelerating or cornering, weight shifts off some tyres and onto others (braking shifts weight forward).
- **Unsprung mass**: wheels, tyres and uprights, which sit below the springs and move with the road.
- **Contact patch**: the small area where the tyre touches the road, where all grip forces originate.
- **Relaxation length**: the travel distance a tyre needs before its force catches up with a new slip angle.
