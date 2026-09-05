# Understanding `vehicle_physics.py`, A Plain-English Guide

This document explains the physics inside `vehicle_physics.py` without
assuming prior knowledge of vehicle dynamics. It's meant to sit alongside
the code as a lookup for "what does this number actually do?" while tuning
the car.

The short version: this file simulates a car the way the real world would
push it around: tyres gripping, springs compressing, weight shifting under
braking and cornering, so that the MPC controller (which uses a much
simpler model) has something realistic to be tested against.

---

## 1. The Big Picture

There are two "cars" in this project:

1. **The MPC's internal model** (`bicycle_model.py`), a simplified,
   8-state, straight-line-tyre-force version of a car. Simple on purpose,
   because the controller has to solve an optimisation problem with it many
   times per second.
2. **This file's plant model**, a 25-state model with soft suspension,
   tyres that take time to build up grip, weight transfer, aerodynamics,
   and more. This is the "real" car in the simulation, the ground truth
   that the MPC's simplified model is only ever an approximation of.

The gap between what the MPC *thinks* the car will do and what this plant
*actually* makes the car do is deliberate. It's what makes the simulation a
meaningful test of the controller, instead of the controller grading its
own homework.

---

## 2. What Each State Actually Describes

The car's full situation at any instant is stored as 25 numbers (the
"state vector"). The table below lists what each one means physically,
not just its units, but what part of the car each is describing.

| # | Name | What it's describing |
|---|------|-------------------------------|
| 0 | `X` | Where the car is, left-right, on the track map. |
| 1 | `Y` | Where the car is, forward-back, on the track map. |
| 2 | `psi` | Which way the car is *pointing* (its heading), not which way it's *moving*. These can differ, think of a car sliding sideways through a corner. |
| 3 | `vx` | Forward speed, measured along the car's own nose-to-tail axis. This is "how fast the speedometer would read." |
| 4 | `vy` | Sideways speed, measured across the car. Non-zero `vy` means the car is sliding, e.g. drifting or understeering wide. |
| 5 | `r` | Yaw rate, how fast the car's heading is rotating (spinning). High `r` with low actual cornering means the car is spinning out, not turning cleanly. |
| 6 | `delta_act` | The steering angle the front wheels are *actually* sitting at right now, after accounting for the small delay in the steering rack responding to a command. |
| 7 | `a_act` | The acceleration/braking the drivetrain is *actually* delivering right now, after its own small response delay. |
| 8, 9 | `omega_RL`, `omega_RR` | How fast the rear-left and rear-right wheels are spinning. These are the driven wheels. |
| 22, 23 | `omega_FL`, `omega_FR` | How fast the front wheels are spinning. These aren't driven, they free-roll, so they mostly just reflect ground speed. |
| 10–13 | `z_FL`, `z_FR`, `z_RL`, `z_RR` | How compressed or extended each corner's suspension spring currently is, relative to its resting position. Positive = compressed (squashed down); negative = extended (drooping). |
| 14–17 | `dz_FL_dt` … `dz_RR_dt` | How *fast* each corner's suspension is currently moving up or down. This is what the dampers react to. |
| 18–21 | `Fy_FL_rlx` … `Fy_RR_rlx` | The actual sideways grip force each tyre is currently producing. "Relaxed" (`_rlx`) because tyres don't grip instantly, this value chases the ideal target with a short lag, explained in §4. |
| 24 | `alat_lim` | The current buildup of the FSDS lateral-acceleration-ceiling's restoring term (§5), a memory state, not a directly physical quantity, since that mechanism is a lagged pushback rather than an instantaneous clip. |

**Why do positions 0–7 match the MPC's own 8 states?** So that other files
(`gui/simulation.py`, `tuner/offline_tuner.py`) can read "where is the car /
how fast is it going" straight out of this plant's state vector without
needing to convert between two different numbering schemes. States 8–24 are
extra detail the simple MPC model doesn't track at all.

---

## 3. Vehicle Parameters, What Each One Does, and the Effect of Changing It

These live in `VehicleParams`. Grouped by what part of the car they affect.

### Geometry, the car's basic shape

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `lf`, `lr` | Distance from the car's centre of mass to the front axle (`lf`) and rear axle (`lr`). | Moving `lf` up (mass further from front axle) puts more weight over the rear, so the car becomes more prone to understeer (front loses grip first). | Less weight over that axle, so that end grips relatively better, other end worse. |
| `m` | Total mass of car + driver. | Everything gets harder: more force needed to accelerate, brake, and corner at the same rate. Tyres are more loaded (which can help peak grip a little, but hurts accel/braking much more). | Car responds faster to the same forces, with quicker acceleration, braking, and direction changes. |
| `Iz` | Yaw moment of inertia, how resistant the car is to *spinning* (as opposed to just moving). Think of it like the car's "rotational mass." | Car resists spinning more, more stable but slower to react to steering, feels "lazy" turning in. | Car spins more readily for the same yaw moment, sharper turn-in, but also easier to spin out if grip is lost. |
| `tf`, `tr` | Track width, the distance between left and right wheels, front and rear. | Wider track = more resistance to weight rolling from side to side in a corner, generally more cornering stability. | Narrower track = car rolls/transfers weight more readily side-to-side, less stable at the limit. |
| `h_cg` | Height of the centre of mass above the ground. | Higher CoG = more weight thrown onto the outside/front tyres under cornering/braking = less usable grip on the tyres losing load = worse handling balance. | Lower CoG = less weight transfer, generally more predictable, higher-grip handling. |

### Actuator limits, what the car is physically allowed to do

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `max_steer` | The furthest the steering rack can physically turn the front wheels. | Tighter minimum turning radius available. | Car can't turn as sharply regardless of what's commanded. |
| `max_accel` | The hardest the car can accelerate forward. | Faster off the line / out of corners (if the tyres can actually put that much force down). | Slower acceleration, more conservative. |
| `max_accel_brake` | The hardest the car can brake (negative = deceleration). | (More negative = stronger braking) shorter stopping distances, later braking points possible. | Longer stopping distances required. |
| `max_v` | Absolute speed ceiling, the car will not accelerate past this no matter what. | Higher top speed allowed. | Car is artificially capped lower, even on a straight where it has more to give. |

### Unsprung mass, the parts *not* held up by the springs

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `m_us` | Mass of each wheel/tyre/upright assembly, the bits that move with the road surface, below the springs. | Heavier wheels react more sluggishly to bumps, suspension struggles to keep the tyre pressed to the road over rough surfaces, hurting grip consistency. | Lighter wheels track the road surface more faithfully, generally better grip over imperfections. |

### Tyre geometry

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `r_eff` | Effective rolling radius of the tyre, roughly, how far the car travels per wheel revolution. | Wheel needs to spin slower to hit the same ground speed; for the same motor torque, more force reaches the ground (bigger "lever arm"), but also more rotational inertia to overcome. | Opposite: wheel spins faster for the same speed, less torque converted to ground force per unit of wheel torque. |

### Rotational inertia, how hard it is to spin a wheel up or down

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `I_wheel` | How much a wheel "resists" changing its spin speed, its own rotational mass. | Wheel takes longer to spin up under power or spin down under braking, slower throttle/brake response at the wheel, can mask wheelspin/lockup onset (takes longer to develop). | Wheel spins up/down almost instantly with torque, sharper, twitchier response; wheelspin or lockup shows up faster. |
| `I_drivetrain` | Same idea, but for the motor/gearbox's rotating mass, felt through the wheel. | Same effect as `I_wheel` above, but only on the driven (rear) wheels. | Same, opposite direction. |

### Suspension, springs, dampers, anti-roll bars

Think of suspension as: **springs** hold the car up and push back when
compressed, **dampers** resist *how fast* the suspension moves (slowing down
bounce), and **anti-roll bars (ARBs)** link the left and right sides
together so the car resists leaning in corners.

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `k_susp_f`, `k_susp_r` | Spring stiffness, front/rear. How much force it takes to compress the suspension by 1 metre. | Stiffer spring: less body roll/dive/squat, but a harsher ride and less ability to keep the tyre in contact with bumpy ground (grip suffers over rough surfaces). | Softer spring: smoother ride, better bump absorption, but more body movement (roll, dive, squat) which can upset handling balance. |
| `c_damp_f`, `c_damp_r` | Damper (shock absorber) rate, front/rear. Resists the *speed* of suspension movement. | Suspension settles faster after a bump but transmits sharper impacts; too much and the tyre can be forced to skip off bumps instead of following them. | Suspension moves more freely, better at absorbing sharp bumps, but the car "wallows" (takes longer to settle) after a disturbance. |
| `k_arb_f`, `k_arb_r` | Anti-roll bar stiffness, front/rear. Resists the car leaning side-to-side in a corner by linking left and right suspension. | Less body roll in that axle, and (importantly) that axle loses relative grip in hard corners because more load gets pushed onto the *outside* tyre and taken off the inside one, so stiffening the front ARB, for instance, is a classic way to *reduce* front grip and add understeer, or the reverse at the rear to reduce oversteer. | Less coupling between left/right, more body roll for that axle, and that axle keeps relatively more even (and often more total) grip through a corner. |
| `z_max`, `z_min` | The hard mechanical limits of suspension travel (bump stop and droop stop). | Wider range before hitting the hard stop, more room to absorb big hits before the suspension "runs out" and transmits the impact straight to the chassis. | Narrower range, car hits the hard stops sooner under heavy braking/cornering/bumps, causing a sudden harsh jolt. |

### Kinematic camber, how suspension movement tilts the tyre

Camber is the tilt of the tyre when viewed from the front (top-in or
top-out). As the suspension compresses, the geometry of the suspension arms
naturally changes this tilt, that's "kinematic" camber.

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `camber_gain` | How much camber angle changes per unit of suspension travel. | More aggressive camber change with suspension movement, can add useful grip on the loaded (outside) tyre in a corner as the suspension compresses, but if overdone, unsettles the car as it moves through its travel. | Less camber change with movement, more predictable but potentially leaves grip on the table in hard cornering. |
| `camber_stiff_f`, `camber_stiff_r` | How much extra sideways grip force a given camber angle actually produces (per axle). | Camber changes translate into more actual grip change, camber gain becomes a more powerful (and more sensitive) tuning tool. | Camber angle matters less to grip, the car becomes less sensitive to how the suspension geometry tilts the tyres. |

---

## 4. What is "Full MF94 Pacejka", and What Is a Tyre Model At All?

### What a tyre model is, conceptually

A tyre doesn't grip the road like a rigid block of rubber sliding on
sandpaper. It *deforms*, the contact patch stretches and distorts slightly
before it actually slides. Because of this, the force a tyre produces isn't
a simple constant "friction coefficient × weight on it", it depends, in a
curved, non-linear way, on **how much the tyre is being asked to slip**.

There are two kinds of "slip" a tyre experiences:

- **Slip angle** (α, alpha), the angle between where the tyre is *pointed*
  and where it's actually *travelling*. This produces **sideways (lateral)**
  grip force, the force that turns the car.
- **Slip ratio** (κ, kappa), the mismatch between the tyre's rotating speed
  and the car's actual ground speed (spinning faster = wheelspin, slower =
  lockup/braking slip). This produces **forward/backward (longitudinal)**
  grip force, the force that accelerates or brakes the car.

A "tyre model" is just a mathematical curve that says: *given this much
slip angle (or slip ratio) and this much weight on the tyre, how much grip
force comes out?* A bad tyre model (e.g., "grip is just a constant times
weight, always") makes the whole vehicle simulation unrealistic, because
real tyres have a grip *peak*, push past a certain slip angle and grip
starts to *fall off* (this is what a slide or a spin feels like).

### What "Pacejka" and "MF94" mean

**Pacejka** refers to Hans B. Pacejka, the researcher whose tyre force
formula became the standard used across motorsport and vehicle dynamics
research. **MF94** stands for "Magic Formula 1994", a specific, well
established version of his formula. It's called the "Magic Formula" because
a single, fairly compact equation using a handful of coefficients can
closely reproduce the S-shaped force curve real tyres produce on a
test rig.

**"Full"** here means the code isn't using a stripped-down straight-line
approximation (which is what the MPC's simple internal model uses instead).
It includes the full curved shape, offsets, and camber effects, matching
real tyre-test-rig behaviour far more closely.

### The shape of the curve, and what each coefficient controls

Picture a graph: slip angle (or slip ratio) along the bottom, grip force up
the side. As slip starts at zero and increases, force rises steeply, reaches
a peak, and then, for a real tyre, can fall back down slightly (this
falling-off is why sliding a car past its grip peak makes it feel like it
has "let go"). The MF94 formula reproduces exactly this shape using four
main coefficients:

| Coefficient | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `B` (stiffness factor) | How steeply the grip force ramps up for small amounts of slip, the tyre's initial "sharpness." | Tyre reaches its peak grip at a smaller slip angle, feels sharper, more responsive, "grabbier" near centre. | Tyre needs more slip to build up the same force, feels vaguer, more gradual, less immediately responsive. |
| `C` (shape factor) | Controls how rounded vs. peaky the top of the curve is. | Curve peak becomes flatter/broader, grip stays high over a wider slip range, more forgiving near the limit. | Curve peak becomes sharper/narrower, grip falls away more suddenly past the ideal slip angle, less forgiving. |
| `D` (peak factor) | Scales the maximum force the tyre can ever produce (combined with `mu` and the load `Fz`). | Higher maximum grip available overall. | Lower maximum grip ceiling, the tyre can't produce as much force no matter what. |
| `E` (curvature factor) | Fine-tunes the curve's shape near and past the peak, in this file it's negative, which is typical for a racing slick and makes the peak sharper. | Making `E` more negative sharpens the peak further, grip falls off more abruptly once past the ideal slip point (a more "on/off" feeling tyre). | Making `E` less negative (toward 0 or positive) rounds the peak out, a more gradual, forgiving transition into sliding. |
| `Sv` (vertical offset) | A small constant force offset, real tyres aren't perfectly symmetric, so they can produce a tiny bit of force even at zero slip (from tyre construction quirks). | The tyre has a small built-in pull to one side even when going straight. | Removes that built-in pull, the tyre is perfectly neutral at zero slip. |
| `Sh` (horizontal offset) | Shifts *where* zero net force occurs, along the slip-angle axis. | Shifts the "neutral point" of the tyre's response to one side. | Shifts it the other way, or removes the shift entirely at `Sh = 0`. |

### Two flavours in this file: lateral and longitudinal

- **`pacejka_lateral_mf94`**, uses slip *angle* to produce **sideways**
  grip force (cornering). Also adds camber thrust, extra sideways force
  from the tyre being tilted (see §3's camber section).
- **`pacejka_longitudinal_mf94`**, uses slip *ratio* to produce
  **forward/backward** grip force (accelerating/braking). No offsets needed
  here, since accelerating and braking are naturally symmetric.

### Friction and load sensitivity: how `mu` fits in

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `mu` | The tyre's overall peak grip potential, think "how sticky is the rubber compound," combined with track surface quality. | More available grip everywhere, higher cornering speeds, shorter braking distances, more traction out of corners. | Less available grip everywhere, the car will slide/spin/lock up sooner in every situation. |
| `k_sens` | Load sensitivity, real tyres don't produce grip perfectly proportional to the weight on them; heavier loading gives *diminishing returns* on grip. | Grip falls off more sharply as load increases, heavily loaded tyres (e.g. the outside front under hard cornering) give proportionally less benefit from that extra load. | Grip stays closer to proportional with load, heavily loaded tyres keep giving nearly their "fair share" of extra grip. |
| `road_mu` (passed into `step_nonlinear_plant`) | A simple multiplier on top of `mu` representing the *surface*, dry tarmac, damp, wet, etc. | Grippier surface, all tyre forces scale up. | Slicker surface, all tyre forces scale down (this is how "rain" or "low-grip track" scenarios are simulated). |

### Tyre relaxation, why grip doesn't appear instantly

Real tyres don't produce their full grip force the instant a slip angle
appears, the rubber and carcass have to physically deform first, which
takes a small amount of travel distance (not time directly, distance).
This file models that with a **relaxation length**, `sigma_y_f` /
`sigma_y_r`.

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `sigma_y_f`, `sigma_y_r` | Distance (in metres of travel) the tyre needs to build up to its full steady-state grip force after a slip angle change. | Grip response becomes laggier/slower to build, the car feels less immediately responsive to steering input, especially noticeable at higher speed. | Grip responds almost instantly to slip angle changes, sharper, more immediate steering feel. |

This is the difference between states 18–21 (`Fy_*_rlx`, the *actual,
lagged* force being applied to the car right now) and the steady-state
value computed fresh each sub-step inside the function (`Fy_*_ss`, what
the tyre is *heading toward*).

### The friction ellipse: grip is a shared budget, not two separate pools

A tyre has one finite total amount of grip to give at any instant, it
can't produce maximum sideways force *and* maximum forward force
simultaneously; using some grip for one leaves less available for the
other. This is why braking hard *while* cornering hard is a classic way to
lose the car, it asks the tyre for more total grip than it has.

The code enforces this with the *friction ellipse*: whatever fraction of
the tyre's total grip budget is being spent on longitudinal force (`Fx`)
directly reduces how much lateral force (`Fy`) is still available.

---

## 5. Other Physics Features, Explained

### Aerodynamics, drag and downforce

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `Cd_A` | Drag area, how much the car resists moving through the air. | More aerodynamic drag, slower top speed, more energy needed to maintain speed. | Less drag, higher achievable top speed for the same power. |
| `Cl_A_f`, `Cl_A_r` | Downforce area, front/rear, how much the wings/bodywork push the car *down* into the road at speed (which increases tyre grip, since more weight = more available grip, up to the load-sensitivity limits above). | More downforce on that axle, more cornering grip available there at speed, but also (indirectly, alongside `Cd_A`) generally more drag in real cars. | Less downforce, less "free" grip from aerodynamics at that axle, especially noticeable at high speed. |
| `Cl_pitch_sens` | How much braking (nose dipping down, "pitching forward") temporarily shifts downforce toward the front and away from the rear. | Braking creates a bigger front-grip boost / rear-grip reduction, and can make the rear feel looser while braking hard. | Braking has little effect on the front/rear downforce balance. |

### Rolling resistance and stiction

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `Crr` | A steady drag force that's always present while the car is moving, representing tyre rolling resistance plus drivetrain drag that isn't modelled elsewhere. | Car coasts to a stop faster with no throttle/brake input, more "engine braking"-like feel. | Car coasts further and longer with no input, freer rolling. |
| `F_stiction` | The breakaway force needed to get a stationary car moving at all (static friction is higher than moving/rolling friction). | Takes more force to get the car moving from a dead stop. | Car starts moving more easily from rest. |

### Actuator lag, steering and throttle don't respond instantly

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `tau_delta` | How long the steering rack takes to catch up to a commanded steering angle. | Steering feels laggier/slower to respond to inputs. | Steering responds almost immediately to commands. |
| `tau_a` | Same idea, for the throttle/brake actuator. | Acceleration/braking response lags behind commands more. | Acceleration/braking responds almost immediately. |

### Torque vectoring (optional, off by default)

`tv_gain` (default 0, disabled) lets the rear differential push more drive
force to one rear wheel than the other, based on yaw rate, to help rotate
the car through a corner. Increasing it makes the car turn in more eagerly
under power; too much can make it feel unpredictable or nervous.

### The FSDS lateral-acceleration ceiling (`alat_ceiling*`)

This is not a real tyre-grip limit. It's a model of a *simulator quirk*.
FSDS itself caps how much sustained lateral acceleration a car can actually
achieve to roughly 7-9 m/s² depending on speed, well below what this
plant's own tyres are otherwise capable of (measured up to ~14.5 m/s²
unaided, and the real car reaches ~12.3 m/s² on a lap). Without modelling
this, the offline simulator lets the car take corners the real FSDS car
physically cannot, so weights tuned against the unconstrained plant assume
cornering authority that doesn't exist once actually driving in FSDS.

The mechanism works by adding a **restoring yaw moment** once the car's
current lateral acceleration exceeds a speed-dependent ceiling, pulling
the yaw rate back down, the same way FSDS itself apparently does
internally (its exact internal cause is unknown; this only reproduces the
external symptom). It is a soft, lagged pushback, not a hard clip. The car
can briefly exceed the ceiling before being pulled back, which is what
produces the small measured overshoot on a hard, sudden corner entry.

| Parameter | Plain English | Increase it → | Decrease it → |
|---|---|---|---|
| `alat_ceiling_enabled` | Whether this mechanism runs at all. | N/A, set `False` to recover the unconstrained plant (e.g. for real-vehicle work, where this ceiling doesn't apply). | N/A |
| `alat_ceiling`, `alat_ceiling_slope`, `alat_ceiling_intercept` | Together define the ceiling itself as a function of speed: flat at low speed, then rising as `intercept + slope·v` once that line climbs above the flat value. | Higher ceiling, the car is allowed more sustained lateral g before the restoring moment engages. | Lower ceiling, the pushback engages sooner and more often, capping cornering speed harder. |
| `alat_ceiling_mode` | Which control law converts "how far over the ceiling" into restoring moment. Leave on `'pi'` (the validated choice); `'p'` is a legacy mode kept only to reproduce the measurement that rejected it. | N/A | N/A |
| `alat_ceiling_gain` | How strongly the restoring moment reacts to sustained excess above the ceiling. | Pulls the car back to the ceiling more firmly, less overshoot past it. | Weaker pushback, the car can run further over the ceiling before being reined in. |
| `alat_ceiling_tau` | How quickly the restoring moment builds once the ceiling is exceeded (a lag, not an instant clip). | Slower to engage, bigger transient overshoot on a sudden hard corner entry, but no effect on the settled cornering level. | Faster to engage, less overshoot, again with no effect on the settled level. |

Full derivation (the open-loop measurements this was fitted to, why the
integral law replaced an earlier proportional one, and what's still
unresolved) lives in `docs/reference/simulator_fidelity.md`'s "The
sim-to-real gap: a lateral-acceleration ceiling, partly closed" section,
not repeated here.

---

## 6. Quick Reference: Symptom → Likely Parameter

When a specific handling symptom shows up during tuning, these are usually
the first places to look:

- **Car understeers (won't turn in, pushes wide)** → increase front `mu`/grip,
  reduce `k_arb_f` (front anti-roll stiffness), check `Cf`/front `B_f`,`D_f`,
  or reduce front downforce loss under braking.
- **Car oversteers (rear steps out, spins)** → the mirror image: increase
  rear grip/downforce, reduce `k_arb_r`, check `Cr`/rear `B_r`,`D_r`.
- **Car feels laggy/unresponsive to steering** → check `tau_delta` (actuator
  lag) and `sigma_y_f` (tyre relaxation length), both add delay between the
  steering input and the car actually responding.
- **Car spins its rear wheels under acceleration** → check `mu`, `Fmax_RL`/
  `Fmax_RR` friction ceilings (driven by `mu` and rear `Fz`), or whether
  `max_accel`/torque demand is asking for more force than the tyres
  can deliver.
- **Ride feels harsh over bumps** → reduce `k_susp_f`/`k_susp_r` (softer
  springs) or `c_damp_f`/`c_damp_r` (softer dampers).
- **Car rolls/wallows too much in corners** → increase `k_arb_f`/`k_arb_r`
  or `k_susp_f`/`k_susp_r`.
- **Car coasts too far / doesn't slow down off-throttle** → increase `Crr`.
- **Car corners noticeably harder in the offline simulator than it can on
  the real FSDS car** → check `alat_ceiling_enabled` is `True` and the
  ceiling parameters match the latest measurement, this is the known,
  deliberately modelled gap, not a tyre-grip mismatch.

---

## 7. Glossary

- **Slip angle (α)**, angle between where a tyre points and where it
  actually travels; drives sideways (cornering) grip.
- **Slip ratio (κ)**, mismatch between wheel spin speed and ground speed;
  drives forward/backward (accel/braking) grip.
- **Yaw**, rotation of the car about a vertical axis (spinning left/right,
  viewed from above). Yaw *rate* is how fast that rotation is happening.
- **Load transfer / weight transfer**, under braking, accelerating, or
  cornering, weight shifts off some tyres and onto others (e.g. braking
  shifts weight forward, onto the front tyres).
- **Unsprung mass**, the parts of the car (wheels, tyres, uprights) that
  sit *below* the springs and move with the road surface, as opposed to the
  "sprung" chassis the springs are holding up.
- **Contact patch**, the small area where the tyre actually touches the
  road; all grip forces originate here.
- **Friction ellipse/circle**, the idea that a tyre's total grip is a
  shared, finite budget between sideways and forward/backward force.
- **Relaxation length**, the travel distance a tyre needs before its grip
  force catches up to a new slip angle, rather than responding instantly.