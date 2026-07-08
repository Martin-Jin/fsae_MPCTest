"""
vehicle_physics.py — High-Fidelity Nonlinear Vehicle Plant Model

PURPOSE
-------
Implements a 24-state nonlinear vehicle dynamics simulation plant intended to
match the fidelity of Nvidia PhysX (used by FSDS/AirSim). This is the "truth"
model that drives the vehicle in simulation; the MPC controller (optimiser.py)
uses a much simpler 8-state linear model internally, creating a deliberate
plant-model mismatch that mirrors the real-world situation.

The plant is stepped at 20 Hz (dt=0.05 s) but internally sub-steps at 4×
(h=0.0125 s) to maintain numerical stability through the stiff suspension
dynamics and tyre relaxation lag.

STATE VECTOR (24 elements)
--------------------------
Indices 0-7 are identical to the MPC's 8-state linear model so that
simulation.py and offline_tuner.py can read positions/velocities/actuator
states without any index remapping.

  [0]  X            Global position X (m)
  [1]  Y            Global position Y (m)
  [2]  psi          Yaw angle (rad)
  [3]  vx           Longitudinal velocity, body frame (m/s)
  [4]  vy           Lateral velocity, body frame (m/s)
  [5]  r            Yaw rate (rad/s)
  [6]  delta_act    Actual steering angle after first-order lag (rad)
  [7]  a_act        Actual acceleration command after first-order lag (m/s²)
  [8]  omega_RL     Rear-left  wheel angular velocity (rad/s)
  [9]  omega_RR     Rear-right wheel angular velocity (rad/s)
  [10] z_FL         Front-left  suspension deflection from equilibrium (m)
  [11] z_FR         Front-right suspension deflection from equilibrium (m)
  [12] z_RL         Rear-left   suspension deflection from equilibrium (m)
  [13] z_RR         Rear-right  suspension deflection from equilibrium (m)
  [14] dz_FL_dt     Front-left  suspension velocity (m/s)
  [15] dz_FR_dt     Front-right suspension velocity (m/s)
  [16] dz_RL_dt     Rear-left   suspension velocity (m/s)
  [17] dz_RR_dt     Rear-right  suspension velocity (m/s)
  [18] Fy_FL_rlx    Front-left  tyre lateral force after relaxation lag (N)
  [19] Fy_FR_rlx    Front-right tyre lateral force after relaxation lag (N)
  [20] Fy_RL_rlx    Rear-left   tyre lateral force after relaxation lag (N)
  [21] Fy_RR_rlx    Rear-right  tyre lateral force after relaxation lag (N)
  [22] omega_FL     Front-left  wheel angular velocity (rad/s)
  [23] omega_FR     Front-right wheel angular velocity (rad/s)

SUSPENSION DEFLECTION CONVENTION
---------------------------------
z[i] = 0 at the static + aero equilibrium position at a nominal cruise speed.
Spring force on chassis from corner i:  F_spring_i = k_i * (z_eq_i + z_i)
At equilibrium: k_i * z_eq_i = Fz_static_i  so storing only the deviation z[i]
gives a spring-force floor of the static load without double-counting.
The unsprung mass EOM is then:  m_us * ddz_i = −k_i*z_i − c_i*dz_i − F_arb_i

PHYSICS FEATURES
----------------
  1. Full MF94 Pacejka lateral + longitudinal tyre model (B, C, D, E, Sv, Sh, camber)
  2. Per-wheel longitudinal slip ratio and wheel spin dynamics
  3. Per-corner spring / damper / anti-roll-bar suspension with dynamic Fz
  4. Split front/rear aerodynamic downforce with pitch sensitivity
  5. Tyre relaxation length (first-order lag on lateral force)
  6. Optional torque vectoring (tv_gain parameter; default 0 = disabled)
  7. Kinematic camber gain (suspension deflection → camber angle → Pacejka D shift)
  8. Road-surface mu scaling (road_mu parameter; default 1.0 = dry tarmac)

USED BY
-------
  simulation.py   — calls step_nonlinear_plant() at every simulation timestep;
                    calls init_plant_state() to initialise the vehicle.
  offline_tuner.py — calls step_nonlinear_plant() and init_plant_state() in
                    run_headless_rollout() for each CMA-ES candidate evaluation.

DOES NOT USE
------------
  bicycle_model.py, optimiser.py, speed_profile.py, sim_track.py, performance_stats.py
"""

import numpy as np

# ─────────────────────────────────────────────────────────────
# NAMED STATE INDEX CONSTANTS
# Using named constants everywhere prevents silent index bugs
# when the state vector length changes.
# ─────────────────────────────────────────────────────────────
IDX_X        = 0   # Global X position (m)
IDX_Y        = 1   # Global Y position (m)
IDX_PSI      = 2   # Yaw angle (rad)
IDX_VX       = 3   # Longitudinal velocity body frame (m/s)
IDX_VY       = 4   # Lateral velocity body frame (m/s)
IDX_R        = 5   # Yaw rate (rad/s)
IDX_DELTA    = 6   # Actual steering angle after lag (rad)
IDX_A_ACT    = 7   # Actual acceleration after lag (m/s²)
IDX_OMEGA_RL = 8   # Rear-left  wheel spin (rad/s)
IDX_OMEGA_RR = 9   # Rear-right wheel spin (rad/s)
IDX_Z_FL     = 10  # Front-left  suspension deviation from equilibrium (m)
IDX_Z_FR     = 11  # Front-right suspension deviation from equilibrium (m)
IDX_Z_RL     = 12  # Rear-left   suspension deviation from equilibrium (m)
IDX_Z_RR     = 13  # Rear-right  suspension deviation from equilibrium (m)
IDX_DZ_FL    = 14  # Front-left  suspension velocity (m/s)
IDX_DZ_FR    = 15  # Front-right suspension velocity (m/s)
IDX_DZ_RL    = 16  # Rear-left   suspension velocity (m/s)
IDX_DZ_RR    = 17  # Rear-right  suspension velocity (m/s)
IDX_FY_FL    = 18  # Front-left  tyre lateral force after relaxation (N)
IDX_FY_FR    = 19  # Front-right tyre lateral force after relaxation (N)
IDX_FY_RL    = 20  # Rear-left   tyre lateral force after relaxation (N)
IDX_FY_RR    = 21  # Rear-right  tyre lateral force after relaxation (N)
IDX_OMEGA_FL = 22  # Front-left  wheel spin (rad/s)
IDX_OMEGA_FR = 23  # Front-right wheel spin (rad/s)
N_STATES     = 24  # Total state vector length


# ─────────────────────────────────────────────────────────────
# VEHICLE PARAMETERS
# ─────────────────────────────────────────────────────────────
class VehicleParams:
    """
    Physical parameters for a Formula Student electric vehicle (~255 kg wet).

    All values are based on typical FS EV specifications with rationale
    provided inline. This class is instantiated once per simulation run and
    passed by reference to avoid re-allocation overhead during inner loops.

    Used by: step_nonlinear_plant(), init_plant_state(), plant_to_tracking_error()
    Instantiated in: simulation.py, offline_tuner.py (init_worker, run_headless_rollout)
    """

    def __init__(self):
        # ── Tuning Constants (Modify these to change car behavior) ────
        GRIP_SCALE    = 1.1  # Scales tyre stiffness and Pacejka slope
        INERTIA_SCALE = 0.8  # Scales yaw inertia and wheel rotational mass
        COASTING_SCALE = 3.0 # < 1.0 = Rolls further, > 1.0 = Stops faster

        # ── Geometry ────────────────────────────────────────────────────────
        self.lf    = 0.85     # Distance from CoM to front axle (m)
        self.lr    = 0.70     # Distance from CoM to rear  axle (m)
        self.m     = 255.0    # Total vehicle mass including driver (kg)
        self.Iz    = 110.0    # Yaw moment of inertia about CoM (kg·m²)
        self.tf    = 1.25     # Front track width between tyre contact patches (m)
        self.tr    = 1.20     # Rear  track width between tyre contact patches (m)
        self.h_cg  = 0.30     # Centre-of-gravity height (m); drives load transfer
        # Linear cornering stiffness — used only by the MPC's linear model
        # not by this nonlinear plant which uses Pacejka curves directly.
        # Effective linearised cornering stiffness matched to Pacejka initial slope:
        # C_eff ≈ mu * Fz_nominal * B * C * D.  Front: 1.9*600*15*1.45*1.0 ≈ 24,800 N/rad.
        self.Cf = 15000.0     # Front cornering stiffness (N/rad)
        self.Cr = 14000.0     # Rear  cornering stiffness (N/rad)
        # Actuator limits: enforced as hard bounds in optimiser.py's QP constraints.
        self.max_steer       = np.radians(35.0)  # Max rack-limited steering angle (rad)
        # FS EV peak acceleration ~12 m/s² (0→17 m/s in ~2 s); braking ~10 m/s² (~1g).
        self.max_accel       = 12.0              # Max longitudinal acceleration (m/s²)
        self.max_accel_brake = -9.0             # Max longitudinal braking (m/s²)
        self.max_v = 17.0 # Maximum possible speed the vehicle can go

        # ── Unsprung Mass ────────────────────────────────────────────────────
        self.m_us  = 7.5      # Unsprung mass per corner: wheel + upright + hub (kg)

        # ── Tyre Geometry ────────────────────────────────────────────────────
        self.r_eff = 0.2286   # Effective rolling radius for a 13" wheel+tyre (m)

        # ── Rotational Inertia ───────────────────────────────────────────────
        # Each driven wheel's effective inertia includes the motor/gearbox
        # referred through the reduction ratio.
        self.I_wheel      = 0.9 * INERTIA_SCALE  # Per-wheel rotational inertia (kg·m²)
        self.I_drivetrain = 0.05 * INERTIA_SCALE # Motor+gearbox inertia referred to wheel (kg·m²)
        self.I_w_eff_r    = self.I_wheel + self.I_drivetrain  # Rear driven wheels
        self.I_w_eff_f    = self.I_wheel                       # Front free-rolling wheels

        # ── Suspension Spring / Damper / ARB ────────────────────────────────
        # Wheel-rate = spring-rate × motion_ratio²; motion_ratio ≈ 0.65-0.70
        # for a pushrod FS suspension gives ~25 N/mm front, ~30 N/mm rear.
        self.k_susp_f = 25000.0   # Front wheel-rate (N/m)
        self.k_susp_r = 30000.0   # Rear  wheel-rate (N/m)
        # Damping at ~30% of critical damping plus unsprung contribution.
        # c_crit ≈ 2*sqrt(k * m_corner); 30% of that + unsprung ≈ 1500 front.
        self.c_damp_f = 1500.0    # Front corner damper rate (N·s/m)
        self.c_damp_r = 1800.0    # Rear  corner damper rate (N·s/m)
        # Anti-roll bars add an effective wheel-rate that couples left and right
        # suspension: the ARB force = k_arb * (z_left - z_right).
        self.k_arb_f = 8000.0     # Front ARB equivalent wheel-rate (N/m)
        self.k_arb_r = 6000.0     # Rear  ARB equivalent wheel-rate (N/m)
        # Hard bump/droop stops at ±40 mm from equilibrium.
        self.z_max =  0.040       # Maximum compression from equilibrium (m)
        self.z_min = -0.040       # Maximum droop from equilibrium (m)

        # ── Kinematic Camber Gain ────────────────────────────────────────────
        # In a double-wishbone suspension, jounce (compression) induces negative
        # camber on the outer wheel, increasing lateral grip.
        # Gain = 0.5 deg/mm = 0.5 * π/180 / 0.001 ≈ 8.73 rad/m
        self.camber_gain    = 8.73  # Camber change per unit suspension travel (rad/m)
        self.camber_stiff_f = 0.15  # Front camber stiffness: fraction of Fz added as Fy per rad
        self.camber_stiff_r = 0.12  # Rear  camber stiffness (slightly lower, typical slick)

        # ── Pacejka MF94 Lateral Coefficients ───────────────────────────────
        # The MF94 formula: Fy = mu*Fz * sin(C * arctan(B*alpha - E*(B*alpha - arctan(B*alpha))))
        # B: stiffness factor (controls initial slope of the Fy-vs-alpha curve)
        # C: shape factor    (controls the sharpness of the peak)
        # D: peak factor     (scales peak force; combined with mu*Fz gives peak Fy)
        # E: curvature factor (negative = sharper peak, typical for racing slick)
        # Sv, Sh: vertical/horizontal offsets from ply-steer and conicity
        self.B_f  = 15.0 * GRIP_SCALE;  self.C_f  = 1.45;  self.D_f  = 1.0 * GRIP_SCALE
        self.E_f  = -1.5;  self.Sv_f = 0.0;   self.Sh_f = 0.002

        self.B_r  = 12.0 * GRIP_SCALE;  self.C_r  = 1.45;  self.D_r  = 1.0 * GRIP_SCALE
        self.E_r  = -1.8;  self.Sv_r = 0.0;   self.Sh_r = 0.001

        # ── Pacejka MF94 Longitudinal Coefficients ───────────────────────────
        # Same shape function applied to longitudinal slip ratio kappa.
        # Peak at slip ratio ≈ 0.10-0.15 for a slick tyre.
        self.Bx_r = 12.0;  self.Cx_r = 1.65
        self.Dx_r = 1.00;  self.Ex_r = -0.5

        # ── Tyre Relaxation Lengths ───────────────────────────────────────────
        # A tyre does not respond instantaneously to a slip angle change.
        # The lateral force builds up over a "relaxation length" σ (m):
        #   dFy/dt = (vx / σ) * (Fy_steady_state − Fy_actual)
        # At vx=10 m/s with σ=0.45 m: time constant ≈ 40 ms — significant at 20 Hz.
        self.sigma_y_f = 0.45      # Front lateral relaxation length (m)
        self.sigma_y_r = 0.40      # Rear  lateral relaxation length (m)

        # ── Friction and Load Sensitivity ────────────────────────────────────
        # Peak friction coefficient for a dry racing slick.
        self.mu     = 1.75 * GRIP_SCALE # Peak friction coefficient (dimensionless)
        # Reduced load sensitivity: slicks show less degradation than road tyres.
        # At nominal Fz~600 N: mu_eff = 1.9*(1 - 0.00012*600) = 1.76 — still strong.
        self.k_sens = 0.00012      # Load sensitivity (1/N)

        # ── Aerodynamics ─────────────────────────────────────────────────────
        # Total downforce coefficient Cl_A split 43/57 front/rear for
        # neutral balance: more rear downforce than front is typical FS tuning.
        self.rho       = 1.225     # Air density at sea level (kg/m³)
        self.Cd_A      = 0.9 * COASTING_SCALE # Drag area (drag coefficient × frontal area, m²)
        self.Cl_A_f    = 0.645     # Front wing downforce area (m²)
        self.Cl_A_r    = 0.855     # Rear  wing downforce area (m²)
        # Under braking the nose dips (pitch forward), increasing front downforce
        # and reducing rear. Cl_pitch_sens = 0.03 means a 1g deceleration shifts
        # front Cl_f up by 3% and rear Cl_r down by 3%.
        self.Cl_pitch_sens = 0.03  # Fractional Cl change per unit (a/g)

        # ── Rolling Resistance and Stiction ──────────────────────────────────
        self.Crr        = 20.5  * COASTING_SCALE # Constant rolling drag force at speed (N)
        self.F_stiction = 600.0 # Static breakaway force (N); (From the four wheels in total, so / 4 for a single wheel)

        # ── Actuator Lag ─────────────────────────────────────────────────────
        # First-order lag: d(delta_act)/dt = (delta_cmd - delta_act) / tau_delta
        # EV motors respond almost instantly; FS rack-and-pinion steering has
        # a small but non-negligible lag (~80 ms) compared to a hydraulic system.
        self.tau_delta = 0.08      # Steering actuator time constant (s)
        self.tau_a     = 0.02      # Acceleration (torque) time constant (s)

        self.g = 9.81              # Gravitational acceleration (m/s²)

    @property
    def L(self):
        """Total wheelbase: lf + lr (m). Used frequently in weight distribution formulae."""
        return self.lf + self.lr

    def static_fz_per_corner(self):
        """
        Compute the static normal load at each front and rear corner at rest
        (no aerodynamics, no longitudinal acceleration).

        Physics: weight distribution by moment balance about the rear axle:
          Fz_front_total = m * g * (lr / L)
          Fz_rear_total  = m * g * (lf / L)
        Divided by 2 for left/right symmetry.

        Returns
        -------
        (Fz_f, Fz_r) : (float, float)
            Normal load per front corner (N) and per rear corner (N).

        Used by: static_z_equilibrium(), and indirectly by step_nonlinear_plant()
                 to set baseline Fz before load transfer is applied.
        """
        Fz_f = self.m * self.g * (self.lr / self.L) / 2.0
        Fz_r = self.m * self.g * (self.lf / self.L) / 2.0
        return Fz_f, Fz_r

    def static_z_equilibrium(self, v_nominal=7.0):
        """
        Compute the suspension equilibrium deflection z_eq (m) at a nominal
        cruise speed, including aerodynamic downforce.

        Why include aero in the equilibrium?
        If the equilibrium is computed at rest (no aero), then when the vehicle
        reaches speed the aero load pushes the suspension down by z_eq_aero, which
        would appear as a large initial transient. Using a speed-inclusive equilibrium
        centres the suspension in its travel range at typical running speeds.

        Physics: At equilibrium, spring force = total static load:
          k * z_eq = Fz_static + 0.5 * (0.5 * rho * Cl_A * v²)
        Solving: z_eq = (Fz_static + F_aero_per_corner) / k

        Parameters
        ----------
        v_nominal : float
            Representative cruise speed for aero calculation (m/s).
            Defaults to 7.0 m/s (the previous fixed v_ref).

        Returns
        -------
        (z_eq_f, z_eq_r) : (float, float)
            Front and rear equilibrium suspension deflection (m).

        Used by: step_nonlinear_plant() — called once per call to compute the
                 spring force baseline.
        """
        Fz_f_static, Fz_r_static = self.static_fz_per_corner()
        # Aerodynamic downforce per axle at nominal speed, split per corner (÷2)
        F_down_f = 0.5 * self.rho * self.Cl_A_f * v_nominal**2
        F_down_r = 0.5 * self.rho * self.Cl_A_r * v_nominal**2
        Fz_f_total = Fz_f_static + 0.5 * F_down_f   # per corner
        Fz_r_total = Fz_r_static + 0.5 * F_down_r   # per corner
        # Deflection = load / spring-rate
        return Fz_f_total / self.k_susp_f, Fz_r_total / self.k_susp_r


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 TYRE MODEL — LATERAL
# ─────────────────────────────────────────────────────────────────────────────
def _mf94(x_in, B, C, D, E):
    """
    Core Pacejka MF94 (Magic Formula 1994) shape function.

    Computes the normalised tyre force curve used for both lateral (Fy) and
    longitudinal (Fx) tyre force. Does NOT apply friction scaling (mu*Fz)
    or horizontal/vertical offsets — those are applied in the callers.

    The MF94 formula:
        Bx = B * x_in
        y = D * sin(C * arctan(Bx − E * (Bx − arctan(Bx))))

    Behaviour:
      - B controls initial slope: dFy/d(alpha) at alpha=0 ≈ B*C*D (stiffness)
      - C controls shape: C<2 → pronounced peak, C=2 → flat peak (asymptotes)
      - D is the peak value (force normalised; actual peak = mu*Fz*D)
      - E < 0 sharpens the peak and increases the shape near zero slip;
        E → 1 gives a rounder response (typical for bias-ply; slicks: E ≈ −1.5)

    Parameters
    ----------
    x_in : float
        Input slip (slip angle in rad for lateral; slip ratio for longitudinal).
    B, C, D, E : float
        Pacejka shape coefficients.

    Returns
    -------
    float : Normalised force (dimensionless; multiply by mu*Fz to get Newtons).

    Called by: pacejka_lateral_mf94(), pacejka_longitudinal_mf94()
    """
    Bx = B * x_in
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def pacejka_lateral_mf94(alpha, Fz, mu, B, C, D, E, Sv, Sh,
                          gamma=0.0, camber_stiff=0.0):
    """
    Full MF94 lateral tyre force including offsets and camber thrust (N).

    Formula:
        Fy = mu * Fz * MF94(alpha + Sh) + Sv + camber_stiff * Fz * gamma

    The ply-steer offset Sh shifts the zero-force slip angle slightly
    (non-zero Fy at zero slip angle is common in real tyres due to
    construction asymmetry). Sv is a constant vertical offset (conicity).
    Camber thrust adds lateral force proportional to camber angle gamma
    and normal load Fz.

    Parameters
    ----------
    alpha : float
        Tyre slip angle (rad). Positive = tyre pointing left of travel direction.
    Fz : float
        Normal load on the tyre (N). Scales peak lateral force.
    mu : float
        Effective peak friction coefficient (dimensionless). Already includes
        load sensitivity degradation — computed externally.
    B, C, D, E : float
        Pacejka shape coefficients (see _mf94 docstring).
    Sv : float
        Vertical offset (N) — ply-steer / conicity constant force.
    Sh : float
        Horizontal offset (rad) — shifts zero-crossing of the Fy curve.
    gamma : float
        Camber angle (rad). Positive = top of tyre leans outward from car centre.
    camber_stiff : float
        Camber stiffness coefficient: fraction of Fz added as Fy per rad of camber.

    Returns
    -------
    float : Lateral tyre force Fy (N). Positive = leftward in tyre frame.

    Called by: step_nonlinear_plant() — once per wheel per substep.
    """
    Fy0 = mu * Fz * _mf94(alpha + Sh, B, C, D, E) + Sv
    return Fy0 + camber_stiff * Fz * gamma


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 TYRE MODEL — LONGITUDINAL
# ─────────────────────────────────────────────────────────────────────────────
def pacejka_longitudinal_mf94(kappa, Fz, mu, Bx, Cx, Dx, Ex):
    """
    MF94 longitudinal tyre force (N).

    Formula:
        Fx = mu * Fz * MF94(kappa)

    No offsets (Sv, Sh) are needed for the longitudinal direction; the
    symmetry of driving vs. braking is captured by the kappa sign alone.

    Longitudinal slip ratio kappa:
        kappa = (r_eff * omega − vx) / max(r_eff * omega, |vx|)
    Positive kappa = wheel spinning faster than ground speed (drive slip / wheelspin).
    Negative kappa = wheel slower than ground speed (brake slip / lockup).
    Peak Fx typically occurs at |kappa| ≈ 0.10-0.15 for a racing slick.

    Parameters
    ----------
    kappa : float
        Longitudinal slip ratio (dimensionless).
    Fz : float
        Normal load (N).
    mu : float
        Effective peak friction (dimensionless; load-sensitivity applied externally).
    Bx, Cx, Dx, Ex : float
        Pacejka longitudinal shape coefficients.

    Returns
    -------
    float : Longitudinal tyre force Fx (N). Positive = driving/forward force.

    Called by: step_nonlinear_plant() — once per wheel per substep.
    """
    return mu * Fz * _mf94(kappa, Bx, Cx, Dx, Ex)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN PLANT INTEGRATION STEP
# ─────────────────────────────────────────────────────────────────────────────
def step_nonlinear_plant(state, u_cmd, dt, params: VehicleParams,
                         road_mu=1.0, tv_gain=0.0):
    """
    Advance the 24-state nonlinear plant by one control timestep dt.

    This is the core integration function called at every simulation step.
    It sub-steps the ODEs 4× internally (h = dt/4 = 0.0125 s) to remain
    numerically stable through the stiff suspension and tyre relaxation dynamics.

    The 20 computational steps within each sub-step follow this pipeline:
      1.  Actuator lag            — first-order steering and throttle/brake lag
      2.  Aerodynamics            — drag, pitch-sensitive split downforce
      3.  Wheel corner velocities — body-frame velocity at each contact patch
      4.  Lateral slip angles     — angle between tyre heading and travel direction
      5.  Suspension Fz           — spring + ARB + aero gives normal load per corner
      6.  Suspension dynamics     — quasi-static load transfer drives spring deflection
      7.  Kinematic camber        — suspension travel changes camber angle
      8.  Friction coefficient    — degrade mu with normal load (load sensitivity)
      9.  Longitudinal slip ratio — compare wheel speed to ground speed
     10.  Torque vectoring        — optional yaw-rate-based differential torque split
     11.  Pacejka longitudinal Fx — tyre force from slip ratio via MF94
     12.  Wheel spin dynamics     — angular acceleration from torque imbalance
     13.  MF94 lateral Fy_ss     — steady-state lateral force from slip angle
     14.  Tyre relaxation lag     — first-order filter toward Fy_ss
     15.  Friction ellipse        — scale Fy down if Fx is consuming friction budget
     16.  Body-frame transform    — rotate front-wheel forces to body axes
     17.  Rolling resistance      — constant drag opposing longitudinal motion
     18.  Resultant forces + Mz  — sum all forces; compute yaw moment
     19.  Rigid-body EOM         — Newton/Euler: accelerations from forces
     20.  State integration       — explicit Euler on all states (semi-implicit for suspension)

    Parameters
    ----------
    state : np.ndarray, shape (24,)
        Current vehicle state vector (see module-level docstring for layout).
    u_cmd : array-like, shape (2,)
        Control commands: [delta_cmd (rad), a_cmd (m/s²)].
        These are the MPC's computed outputs, passed in as commanded setpoints
        for the first-order actuator lag filters.
    dt : float
        Control timestep (s). Typically 0.05 s (20 Hz).
    params : VehicleParams
        Vehicle parameter struct. Passed by reference — not copied.
    road_mu : float, optional
        Surface grip multiplier. 1.0 = dry tarmac, ~0.6 = damp, ~0.3 = wet.
        Scales the effective mu at every tyre corner.
    tv_gain : float, optional
        Torque vectoring gain (N·m per rad/s of yaw rate).
        ΔT = tv_gain * r ; ΔFx = ΔT / (r_eff * 2) applied between rear wheels.
        Default 0 = disabled (standard open differential behaviour).

    Returns
    -------
    np.ndarray, shape (24,)
        New state vector after dt seconds.

    Called by: simulation.py (simulate_closed_loop),
               offline_tuner.py (run_headless_rollout)
    """
    p         = params
    sub_steps = 4           # Number of Euler sub-steps per control timestep
    h         = dt / sub_steps  # Sub-step size: 0.0125 s

    # Pre-compute equilibrium deflections once per outer step (constant for this vehicle)
    z_eq_f, z_eq_r = p.static_z_equilibrium()

    s = state.copy()  # Work on a copy; never mutate the caller's array

    # ── Suspension integration (semi-implicit Euler) ────────────────────
    def _integrate_susp(z, dz, Fz_road, Fz_spring, F_damp, F_arb_signed, k, c):
        """
        Integrate one suspension corner using semi-implicit Euler.

        Why semi-implicit?
        The FS suspension natural frequency is ~11-12 Hz (high relative to
        the 0.0125 s sub-step). Explicit Euler would require many more
        sub-steps to remain stable because the stability condition for a
        spring-damper is: h < 2/ω_n ≈ 0.018 s (right on the boundary here).
        Semi-implicit Euler treats the damping term implicitly (puts it in
        the denominator) while keeping spring and external forces explicit,
        which gives unconditional stability for any h.

        Scheme:
            dz_new = [dz + (F_net / m_us) * h] / (1 + c/m_us * h)   ← implicit damping
            z_new  = z + dz_new * h                                   ← use updated velocity

        F_net = Fz_road − Fz_spring − F_arb  (external − restoring; damper handled implicitly)

        Hard bump-stops: inelastic contact clamps velocity to zero at the limits.

        Parameters
        ----------
        z : float             Current deflection from equilibrium (m)
        dz : float            Current deflection velocity (m/s)
        Fz_road : float       Road reaction force (N)
        Fz_spring : float     Current spring force k*(z_eq + z) (N)
        F_damp : float        Current damper force c*dz (N) [for reference only]
        F_arb_signed : float  ARB force applied to THIS corner (+ve or -ve)
        k : float             Spring rate (N/m)
        c : float             Damper rate (N·s/m)

        Returns
        -------
        (z_new, dz_new) : (float, float)
        """
        F_net = Fz_road - Fz_spring - F_arb_signed
        # Semi-implicit: damping appears in denominator → unconditional stability
        dz_new = (dz + (F_net / p.m_us) * h) / (1.0 + (c / p.m_us) * h)
        z_new  = float(np.clip(z + dz_new * h, p.z_min, p.z_max))
        # Inelastic bump stop: zero velocity component pressing into the stop
        if z_new >= p.z_max and dz_new > 0.0:
            dz_new = 0.0
        if z_new <= p.z_min and dz_new < 0.0:
            dz_new = 0.0
        return z_new, dz_new

    for _ in range(sub_steps):

        # ── Unpack state vector ──────────────────────────────────────────────
        X         = s[IDX_X]
        Y         = s[IDX_Y]
        psi       = s[IDX_PSI]
        vx        = s[IDX_VX]
        vy        = s[IDX_VY]
        r         = s[IDX_R]
        delta_act = s[IDX_DELTA]
        a_act     = s[IDX_A_ACT]
        omega_RL  = s[IDX_OMEGA_RL]
        omega_RR  = s[IDX_OMEGA_RR]
        omega_FL  = s[IDX_OMEGA_FL]
        omega_FR  = s[IDX_OMEGA_FR]
        z_FL  = s[IDX_Z_FL];   z_FR  = s[IDX_Z_FR]
        z_RL  = s[IDX_Z_RL];   z_RR  = s[IDX_Z_RR]
        dz_FL = s[IDX_DZ_FL];  dz_FR = s[IDX_DZ_FR]
        dz_RL = s[IDX_DZ_RL];  dz_RR = s[IDX_DZ_RR]
        Fy_FL_rlx = s[IDX_FY_FL];  Fy_FR_rlx = s[IDX_FY_FR]
        Fy_RL_rlx = s[IDX_FY_RL];  Fy_RR_rlx = s[IDX_FY_RR]

        # Guard against exact-zero vx to avoid divide-by-zero in slip angle calcs
        vx_safe = max(vx, 0.5)

        # ── 1. Actuator lag (first-order lag on steering and throttle) ────────
        # The rate of change is (target - actual) / time_constant.
        # This is the derivative of a first-order ODE: tau * dx/dt = u - x.
        ddelta = (u_cmd[0] - delta_act) / p.tau_delta  # Steering rate (rad/s)
        da     = (u_cmd[1] - a_act)     / p.tau_a      # Acceleration rate (m/s³)

        # ── 2. Aerodynamics (split front/rear, pitch-sensitive) ──────────────
        v_sq   = vx_safe**2 + vy**2
        # Aerodynamic drag: F_drag = 0.5 * rho * Cd_A * v²
        F_drag = 0.5 * p.rho * p.Cd_A * v_sq
        # Pitch proxy: a_act < 0 (braking) → nose pitches forward → more front downforce.
        # Fraction represents the nose pitch as a fraction of 1g deceleration.
        pitch_frac = -a_act / p.g
        Cl_f_eff  = p.Cl_A_f * (1.0 + p.Cl_pitch_sens * pitch_frac)  # Modified front Cl
        Cl_r_eff  = p.Cl_A_r * (1.0 - p.Cl_pitch_sens * pitch_frac)  # Modified rear  Cl
        # Downforce: F_down = 0.5 * rho * Cl_A * v²  (per axle; ÷2 for per corner below)
        F_down_f  = 0.5 * p.rho * Cl_f_eff * v_sq
        F_down_r  = 0.5 * p.rho * Cl_r_eff * v_sq

        # ── 3. Wheel corner velocities (body frame) ──────────────────────────
        # Velocity at each contact patch due to yaw rotation (r) and lateral offset.
        # vx_corner = vx ± r * (track/2)  ;  vy_corner = vy ± r * axle_distance
        vx_FL = vx_safe - r * (p.tf / 2.0);  vy_FL = vy + r * p.lf
        vx_FR = vx_safe + r * (p.tf / 2.0);  vy_FR = vy + r * p.lf
        vx_RL = vx_safe - r * (p.tr / 2.0);  vy_RL = vy - r * p.lr
        vx_RR = vx_safe + r * (p.tr / 2.0);  vy_RR = vy - r * p.lr

        # ── 4. Lateral slip angles ────────────────────────────────────────────
        # Slip angle alpha = steer_angle - arctan(vy_corner / vx_corner)
        # For rear wheels: no steer, so alpha = -arctan(vy/vx)
        # A positive slip angle generates positive (leftward) lateral force.
        alpha_FL = delta_act - np.arctan2(vy_FL, max(vx_FL, 0.5))
        alpha_FR = delta_act - np.arctan2(vy_FR, max(vx_FR, 0.5))
        alpha_RL =           - np.arctan2(vy_RL, max(vx_RL, 0.5))
        alpha_RR =           - np.arctan2(vy_RR, max(vx_RR, 0.5))

        # ── 5. Suspension spring / damper / ARB → normal loads ───────────────
        # ARB (anti-roll bar) couples left and right corners:
        #   F_arb = k_arb * (z_left - z_right)   [+ve on left, -ve on right]
        # This resists roll by transferring load from the compressing (outside) wheel
        # to the extending (inside) wheel, reducing steady-state roll angle.
        arb_f = p.k_arb_f * (z_FL - z_FR)   # Front ARB force (N)
        arb_r = p.k_arb_r * (z_RL - z_RR)   # Rear  ARB force (N)

        # Spring force = k * (z_eq + z_deviation)  [z_eq accounts for static + aero load]
        Fz_spring_FL = p.k_susp_f * (z_eq_f + z_FL)
        Fz_spring_FR = p.k_susp_f * (z_eq_f + z_FR)
        Fz_spring_RL = p.k_susp_r * (z_eq_r + z_RL)
        Fz_spring_RR = p.k_susp_r * (z_eq_r + z_RR)

        # Total normal load = spring force + ARB contribution + aerodynamic downforce.
        # Floored at 10 N to prevent tyre from going completely unloaded and causing NaN.
        Fz_FL = max(10.0, Fz_spring_FL + arb_f + 0.5 * F_down_f)
        Fz_FR = max(10.0, Fz_spring_FR - arb_f + 0.5 * F_down_f)
        Fz_RL = max(10.0, Fz_spring_RL + arb_r + 0.5 * F_down_r)
        Fz_RR = max(10.0, Fz_spring_RR - arb_r + 0.5 * F_down_r)

        # ── 6. Suspension dynamics (unsprung mass EOM) ───────────────────────
        # Quasi-static load transfer drives the road reaction force at each corner.
        # These are the NET road forces the ground pushes up with, which the
        # suspension spring must resist to find the new equilibrium.
        #
        # Longitudinal transfer: braking shifts load forward by m*ax*h_cg/L per axle.
        # Lateral transfer: cornering shifts load outward by m*ay*h_cg/track per axle.
        # Aero adds directly to road reaction (not to spring force baseline).

        # Use the actual longitudinal acceleration from the previous calculation (or the last sub-step)
        ax_body = a_act
        ay_body = vx_safe * r         # Lateral acceleration proxy: v²/R = vx*r (m/s²)

        # Road reaction at each corner (what the ground pushes up with):
        Fz_road_FL = (p.m * p.g * (p.lr / p.L) / 2.0          # Static weight share
                      - (p.m * ax_body * p.h_cg) / (2.0 * p.L) # Longitudinal transfer (forward under braking)
                      - (p.m * ay_body * p.h_cg) / (2.0 * p.tf) # Lateral transfer (unloads inside front)
                      + 0.5 * F_down_f)                          # Aero pushes down on road
        Fz_road_FR = (p.m * p.g * (p.lr / p.L) / 2.0
                      - (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      + (p.m * ay_body * p.h_cg) / (2.0 * p.tf) # Lateral loads outside front
                      + 0.5 * F_down_f)
        Fz_road_RL = (p.m * p.g * (p.lf / p.L) / 2.0          # Rear has more static share (lf > lr)
                      + (p.m * ax_body * p.h_cg) / (2.0 * p.L)  # Under braking: load transfers off rear
                      - (p.m * ay_body * p.h_cg) / (2.0 * p.tr) # Lateral unloads inside rear
                      + 0.5 * F_down_r)
        Fz_road_RR = (p.m * p.g * (p.lf / p.L) / 2.0
                      + (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      + (p.m * ay_body * p.h_cg) / (2.0 * p.tr) # Lateral loads outside rear
                      + 0.5 * F_down_r)

        # Damper forces (velocity-proportional; oppose suspension motion)
        Fd_FL = p.c_damp_f * dz_FL;  Fd_FR = p.c_damp_f * dz_FR
        Fd_RL = p.c_damp_r * dz_RL;  Fd_RR = p.c_damp_r * dz_RR

        # ── 7. Kinematic camber from suspension deflection ───────────────────
        # Compression (z > 0) on the outside wheel in a corner pulls the tyre
        # into negative camber — the top leans toward the car centre — which
        # increases the contact patch's effective grip angle.
        # gamma sign: positive = top leans away from car centre.
        gamma_FL = -p.camber_gain * (z_eq_f + z_FL)   # Left: negative camber in compression
        gamma_FR =  p.camber_gain * (z_eq_f + z_FR)   # Right: sign flipped (leans other way)
        gamma_RL = -p.camber_gain * (z_eq_r + z_RL)
        gamma_RR =  p.camber_gain * (z_eq_r + z_RR)

        # ── 8. Effective friction coefficient with load sensitivity ──────────
        # Real tyres suffer "friction fade" at high normal loads: the contact
        # patch rubber cannot deform uniformly, reducing available grip per unit load.
        # mu_eff = mu_peak * road_mu * (1 - k_sens * Fz)
        # Floored at 0.1 to prevent instabilities in zero-grip edge cases.
        eff_mu = p.mu * road_mu
        mu_FL  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_FL))
        mu_FR  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_FR))
        mu_RL  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_RL))
        mu_RR  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_RR))

        # ── 9. Longitudinal slip ratios ───────────────────────────────────────
        # Slip ratio kappa = (r_eff * omega - vx_wheel) / max(r_eff*omega, |vx_wheel|)
        # This normalises to [-1, 1] where 0 = free rolling, +1 = full wheelspin.
        def _kappa(omega, vx_w):
            r_om  = p.r_eff * max(omega, 0.0)       # Peripheral tyre speed (m/s)
            v_ref = max(max(r_om, abs(vx_w)), 0.01)  # Denominator: avoid div-by-zero
            return (r_om - abs(vx_w)) / v_ref        # Positive = driving slip

        kappa_FL = _kappa(omega_FL, vx_FL)
        kappa_FR = _kappa(omega_FR, vx_FR)
        kappa_RL = _kappa(omega_RL, vx_RL)
        kappa_RR = _kappa(omega_RR, vx_RR)
        
        # ── 10. Torque vectoring and longitudinal force distribution ──────────
        # TV applies a yaw-stabilising torque by biasing drive torque left/right:
        #   ΔFx_tv = tv_gain * r / tr   [N; added to right, subtracted from left]
        delta_Fx_tv = (tv_gain * r / p.tr) if abs(tv_gain) > 1e-6 else 0.0

        # Velocity Governor: Prevent acceleration if at or above max velocity.
        # We only limit positive acceleration (driving), allowing braking.
        governed_a_act = a_act
        
        # Taper range to prevent high-frequency chattering around max_v
        taper_range = 1.0 
        if a_act > 0.0:
            if vx_safe >= p.max_v:
                governed_a_act = 0.0
            elif vx_safe > (p.max_v - taper_range):
                # Linearly roll off the acceleration request
                governed_a_act = a_act * ((p.max_v - vx_safe) / taper_range)
                
        Fx_req_total = p.m * governed_a_act  # Total required longitudinal force

        if governed_a_act > 0.0:
            # Acceleration: rear-wheel drive only; front wheels are passive.
            Fx_FL_req, Fx_FR_req = 0.0, 0.0
            Fx_RL_req = 0.5 * Fx_req_total - delta_Fx_tv   # TV: reduces left to yaw right
            Fx_RR_req = 0.5 * Fx_req_total + delta_Fx_tv   # TV: increases right
        else:
            Fx_FL_req = 0.5 * Fx_req_total 
            Fx_FR_req = 0.5 * Fx_req_total 
            Fx_RL_req = 0.5 * Fx_req_total - delta_Fx_tv
            Fx_RR_req = 0.5 * Fx_req_total + delta_Fx_tv

        # ── 11. Pacejka longitudinal force ────────────────────────────────────
        # The MF94 formula gives the peak achievable Fx from friction and slip ratio.
        # Commanded force is then clipped to what the tyre's friction circle allows.
        Fmax_FL, Fmax_FR = mu_FL * Fz_FL, mu_FR * Fz_FR  # Friction ceiling per corner
        Fmax_RL, Fmax_RR = mu_RL * Fz_RL, mu_RR * Fz_RR

        # Using rear Bx/Cx/Dx/Ex for fronts; same compound, acceptable approximation.
        Fx_FL_pac = pacejka_longitudinal_mf94(kappa_FL, Fz_FL, mu_FL, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_FR_pac = pacejka_longitudinal_mf94(kappa_FR, Fz_FR, mu_FR, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_RL_pac = pacejka_longitudinal_mf94(kappa_RL, Fz_RL, mu_RL, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_RR_pac = pacejka_longitudinal_mf94(kappa_RR, Fz_RR, mu_RR, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)

        # Clip commanded force to [−min(Fmax, Pac_peak), +min(Fmax, Pac_peak)]
        Fx_FL = float(np.clip(Fx_FL_req, -min(Fmax_FL, abs(Fx_FL_pac) + 1.0), min(Fmax_FL, abs(Fx_FL_pac) + 1.0)))
        Fx_FR = float(np.clip(Fx_FR_req, -min(Fmax_FR, abs(Fx_FR_pac) + 1.0), min(Fmax_FR, abs(Fx_FR_pac) + 1.0)))
        Fx_RL = float(np.clip(Fx_RL_req, -min(Fmax_RL, abs(Fx_RL_pac) + 1.0), min(Fmax_RL, abs(Fx_RL_pac) + 1.0)))
        Fx_RR = float(np.clip(Fx_RR_req, -min(Fmax_RR, abs(Fx_RR_pac) + 1.0), min(Fmax_RR, abs(Fx_RR_pac) + 1.0)))

        # ── 12. Wheel spin dynamics ────────────────────────────────────────────
        # Newton's 2nd for rotation: I * dω/dt = T_drive − T_road_reaction
        # T_drive = Fx_req * r_eff  (commanded torque; what the motor tries to apply)
        # T_road  = Fx_actual * r_eff  (what the road actually reacts with)
        # Net torque = imbalance → wheel accelerates or decelerates.
        domega_FL = (Fx_FL_req * p.r_eff - Fx_FL * p.r_eff) / p.I_w_eff_f
        domega_FR = (Fx_FR_req * p.r_eff - Fx_FR * p.r_eff) / p.I_w_eff_f
        domega_RL = (Fx_RL_req * p.r_eff - Fx_RL * p.r_eff) / p.I_w_eff_r
        domega_RR = (Fx_RR_req * p.r_eff - Fx_RR * p.r_eff) / p.I_w_eff_r

        # ── 13. MF94 lateral steady-state forces ──────────────────────────────
        # These are the forces the tyre WOULD produce if there were no relaxation lag.
        # The relaxation filter (step 14) then delays them toward this target.
        Fy_FL_ss = pacejka_lateral_mf94(
            alpha_FL, Fz_FL, mu_FL, p.B_f, p.C_f, p.D_f, p.E_f, p.Sv_f, p.Sh_f,
            gamma_FL, p.camber_stiff_f)
        Fy_FR_ss = pacejka_lateral_mf94(
            alpha_FR, Fz_FR, mu_FR, p.B_f, p.C_f, p.D_f, p.E_f, p.Sv_f, p.Sh_f,
            gamma_FR, p.camber_stiff_f)
        Fy_RL_ss = pacejka_lateral_mf94(
            alpha_RL, Fz_RL, mu_RL, p.B_r, p.C_r, p.D_r, p.E_r, p.Sv_r, p.Sh_r,
            gamma_RL, p.camber_stiff_r)
        Fy_RR_ss = pacejka_lateral_mf94(
            alpha_RR, Fz_RR, mu_RR, p.B_r, p.C_r, p.D_r, p.E_r, p.Sv_r, p.Sh_r,
            gamma_RR, p.camber_stiff_r)

        # ── 14. Tyre relaxation (first-order lateral force lag) ───────────────
        # Physical origin: when the slip angle changes, the tyre contact patch
        # must travel one "relaxation length" σ before the stress distribution
        # (and thus the lateral force) fully adjusts. The first-order ODE is:
        #   dFy_rlx/dt = (vx / σ) * (Fy_ss − Fy_rlx)
        # Time constant: τ = σ / vx  →  at vx=10 m/s, σ=0.35 m: τ ≈ 35 ms.
        # Gain clamped to 1.0 to avoid overshooting in a single sub-step.
        gain_f = min(1.0, (vx_safe / p.sigma_y_f) * h)  # Front lag gain per sub-step
        gain_r = min(1.0, (vx_safe / p.sigma_y_r) * h)  # Rear  lag gain per sub-step

        Fy_FL_rlx_new = Fy_FL_rlx + gain_f * (Fy_FL_ss - Fy_FL_rlx)
        Fy_FR_rlx_new = Fy_FR_rlx + gain_f * (Fy_FR_ss - Fy_FR_rlx)
        Fy_RL_rlx_new = Fy_RL_rlx + gain_r * (Fy_RL_ss - Fy_RL_rlx)
        Fy_RR_rlx_new = Fy_RR_rlx + gain_r * (Fy_RR_ss - Fy_RR_rlx)

        # ── 15. Friction ellipse coupling (Fx² + Fy² ≤ (mu*Fz)²) ─────────────
        # The friction circle (or ellipse in practice) constrains the combined
        # horizontal force a tyre can produce. When Fx consumes part of the
        # friction budget, the remaining capacity for Fy is reduced:
        #   Fy_available = Fy_rlx * sqrt(1 − (Fx/Fmax)²)
        # This is the "friction ellipse" approximation; the exact shape depends
        # on tyre construction but this is standard for race vehicle simulation.
        ell_FL = max(0.0, 1.0 - (Fx_FL / max(Fmax_FL, 1.0))**2)  # Remaining fraction
        ell_FR = max(0.0, 1.0 - (Fx_FR / max(Fmax_FR, 1.0))**2)
        Fy_FL  = float(np.clip(Fy_FL_rlx_new * np.sqrt(ell_FL), -Fmax_FL, Fmax_FL))
        Fy_FR  = float(np.clip(Fy_FR_rlx_new * np.sqrt(ell_FR), -Fmax_FR, Fmax_FR))

        ell_RL = max(0.0, 1.0 - (Fx_RL / max(Fmax_RL, 1.0))**2)
        ell_RR = max(0.0, 1.0 - (Fx_RR / max(Fmax_RR, 1.0))**2)
        Fy_RL  = float(np.clip(Fy_RL_rlx_new * np.sqrt(ell_RL), -Fmax_RL, Fmax_RL))
        Fy_RR  = float(np.clip(Fy_RR_rlx_new * np.sqrt(ell_RR), -Fmax_RR, Fmax_RR))

        # ── 16. Body-frame coordinate transform (front-steer rotation) ────────
        # Front tyre forces are in the tyre frame (aligned with delta_act).
        # Rotate to body frame using the 2D rotation matrix:
        #   [Fx_b]   [cos(δ)  -sin(δ)] [Fx]
        #   [Fy_b] = [sin(δ)   cos(δ)] [Fy]
        cd = np.cos(delta_act);  sd = np.sin(delta_act)
        Fx_FL_b = Fx_FL * cd - Fy_FL * sd;  Fy_FL_b = Fx_FL * sd + Fy_FL * cd
        Fx_FR_b = Fx_FR * cd - Fy_FR * sd;  Fy_FR_b = Fx_FR * sd + Fy_FR * cd
        Fx_RL_b, Fy_RL_b = Fx_RL, Fy_RL   # Rear tyres aligned with body — no rotation
        Fx_RR_b, Fy_RR_b = Fx_RR, Fy_RR

        # ── 17. Rolling Resistance & Stiction (Static Friction) ────────────────
        # Sum of longitudinal tyre forces acting on the chassis
        Fx_tires = Fx_FL_b + Fx_FR_b + Fx_RL_b + Fx_RR_b

        if abs(vx) > 1e-3:
            # Kinetic regime: Constant hysteresis drag opposing forward motion.
            F_roll = p.Crr * np.sign(vx)
            Fx_total = Fx_tires - F_drag - F_roll
        else:
            # Static regime: Vehicle is effectively at rest (vx < 1 mm/s).
            # Stiction opposes the applied tyre forces exactly, up to the breakaway threshold.
            if abs(Fx_tires) <= p.F_stiction:
                F_roll = 0.0
                Fx_total = 0.0
            else:
                # Breakaway: Tyre forces exceed stiction. Friction drops to kinetic rolling resistance.
                F_roll = p.Crr * np.sign(Fx_tires)
                Fx_total = Fx_tires - F_drag - F_roll

        # ── 18. Resultant forces and yaw moment ───────────────────────────────
        # Sum all corner forces in body frame.
        Fy_total = Fy_FL_b + Fy_FR_b + Fy_RL_b + Fy_RR_b

        # Yaw moment Mz about CoM: front axle forces create positive yaw (turn left),
        # rear axle forces oppose yaw. Track-width terms from asymmetric Fx.
        M_z = (  p.lf * (Fy_FL_b + Fy_FR_b)     # Front lateral → yaw moment
               - p.lr * (Fy_RL_b + Fy_RR_b)      # Rear lateral  → oppose yaw
               + (p.tf / 2.0) * (Fx_FR_b - Fx_FL_b)  # Differential Fx at front
               + (p.tr / 2.0) * (Fx_RR_b - Fx_RL_b)) # Differential Fx at rear (TV)

        # ── 19. Rigid-body equations of motion ────────────────────────────────
        # Newton in body frame; Coriolis terms appear because the frame rotates:
        #   F = m * (a_body + ω × v_body)
        # Longitudinal: ax_rb = Fx/m + vy*r   (centripetal term)
        # Lateral:      ay_rb = Fy/m - vx*r   (centripetal term, opposite sign)
        # Yaw:          r_dot = Mz / Iz
        ax_rb = Fx_total / p.m + vy * r   # Body-frame longitudinal acceleration (m/s²)
        ay_rb = Fy_total / p.m - vx_safe * r  # Body-frame lateral acceleration (m/s²)
        r_dot = M_z / p.Iz                 # Yaw angular acceleration (rad/s²)

        # ── 20. Integrate all states (explicit Euler, h sub-steps) ────────────
        # Velocities and rates: standard Euler forward integration.
        vx_new  = max(0.0, vx + ax_rb * h)   # Clamp to zero: no reversing
        vy_new  = vy + ay_rb * h
        r_new   = r  + r_dot * h

        # Global position: integrate body-frame velocity rotated to world frame.
        # dx_world = vx*cos(ψ) - vy*sin(ψ)  ;  dy_world = vx*sin(ψ) + vy*cos(ψ)
        X_new   = X   + (vx * np.cos(psi) - vy * np.sin(psi)) * h
        Y_new   = Y   + (vx * np.sin(psi) + vy * np.cos(psi)) * h
        psi_new = psi + r * h              # Yaw: integrate yaw rate

        # Actuator states: Euler on lag ODEs, clamped to physical limits.
        # Without clamping, integrator wind-up can push delta_act or a_act
        # outside their hardware limits between MPC solve steps.
        delta_new = float(np.clip(delta_act + ddelta * h, -p.max_steer, p.max_steer))
        a_new     = float(np.clip(a_act     + da     * h, p.max_accel_brake, p.max_accel))
            
        # Wheel speeds: floor at 0 (wheels don't spin backward in normal driving)
        omega_RL_new = max(0.0, omega_RL + domega_RL * h)
        omega_RR_new = max(0.0, omega_RR + domega_RR * h)
        omega_FL_new = max(0.0, omega_FL + domega_FL * h)
        omega_FR_new = max(0.0, omega_FR + domega_FR * h)

        z_FL_new, dz_FL_new = _integrate_susp(
            z_FL, dz_FL, Fz_road_FL, Fz_spring_FL, Fd_FL, arb_f, p.k_susp_f, p.c_damp_f)
        z_FR_new, dz_FR_new = _integrate_susp(
            z_FR, dz_FR, Fz_road_FR, Fz_spring_FR, Fd_FR, -arb_f, p.k_susp_f, p.c_damp_f)
        z_RL_new, dz_RL_new = _integrate_susp(
            z_RL, dz_RL, Fz_road_RL, Fz_spring_RL, Fd_RL, arb_r, p.k_susp_r, p.c_damp_r)
        z_RR_new, dz_RR_new = _integrate_susp(
            z_RR, dz_RR, Fz_road_RR, Fz_spring_RR, Fd_RR, -arb_r, p.k_susp_r, p.c_damp_r)

        # ── Pack new state vector ────────────────────────────────────────────
        s_new = np.empty(N_STATES)
        s_new[IDX_X]        = X_new
        s_new[IDX_Y]        = Y_new
        s_new[IDX_PSI]      = psi_new
        s_new[IDX_VX]       = vx_new
        s_new[IDX_VY]       = vy_new
        s_new[IDX_R]        = r_new
        s_new[IDX_DELTA]    = delta_new
        s_new[IDX_A_ACT]    = a_new
        s_new[IDX_OMEGA_RL] = omega_RL_new
        s_new[IDX_OMEGA_RR] = omega_RR_new
        s_new[IDX_OMEGA_FL] = omega_FL_new
        s_new[IDX_OMEGA_FR] = omega_FR_new
        s_new[IDX_Z_FL]     = z_FL_new
        s_new[IDX_Z_FR]     = z_FR_new
        s_new[IDX_Z_RL]     = z_RL_new
        s_new[IDX_Z_RR]     = z_RR_new
        s_new[IDX_DZ_FL]    = dz_FL_new
        s_new[IDX_DZ_FR]    = dz_FR_new
        s_new[IDX_DZ_RL]    = dz_RL_new
        s_new[IDX_DZ_RR]    = dz_RR_new
        s_new[IDX_FY_FL]    = Fy_FL_rlx_new
        s_new[IDX_FY_FR]    = Fy_FR_rlx_new
        s_new[IDX_FY_RL]    = Fy_RL_rlx_new
        s_new[IDX_FY_RR]    = Fy_RR_rlx_new
        s = s_new

    return s


# ─────────────────────────────────────────────────────────────────────────────
# STATE INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
def init_plant_state(X0, Y0, psi0, vx0=10.0):
    """
    Build a 24-element plant state vector at true static equilibrium.

    "True static equilibrium" means:
      - Wheel speeds set to free-rolling: omega = vx0 / r_eff (no slip)
      - Suspension deviation z_i = 0 at all corners (equilibrium defined
        to include aero at v_nominal, so ddz = 0 exactly at t=0)
      - Tyre relaxation forces = 0 (zero slip angle → zero Fy_ss → no transient)
      - All velocities and rates start at their nominal values with no transients

    Without this initialisation, a simulation started with suspension
    deflection states at zero (the spring-force-only equilibrium ignoring aero)
    would have an initial transient as the suspension compresses to its true
    aero-loaded equilibrium, which pollutes the first ~0.5 s of every rollout.

    Parameters
    ----------
    X0, Y0 : float
        Initial global position (m).
    psi0 : float
        Initial yaw angle (rad).
    vx0 : float, optional
        Initial longitudinal speed (m/s). Defaults to 10.0 m/s.

    Returns
    -------
    np.ndarray, shape (24,)
        Initial state vector, ready to pass to step_nonlinear_plant().

    Called by: simulation.py (simulate_closed_loop),
               offline_tuner.py (run_headless_rollout)
    """
    s = np.zeros(N_STATES)
    p = VehicleParams()

    s[IDX_X]   = X0
    s[IDX_Y]   = Y0
    s[IDX_PSI] = psi0
    s[IDX_VX]  = vx0

    # Set wheel speeds to free-rolling at vx0 (no slip at start → no wheelspin transient)
    omega_init = vx0 / p.r_eff
    s[IDX_OMEGA_RL] = omega_init
    s[IDX_OMEGA_RR] = omega_init
    s[IDX_OMEGA_FL] = omega_init
    s[IDX_OMEGA_FR] = omega_init

    # Suspension deviations z_i = 0 (states 10-17 already zero from np.zeros).
    # z_eq computed at vx0 places the suspension at the correct operating point.

    # Tyre relaxation states = 0 (states 18-21 already zero).
    # At zero slip angle, Fy_ss = 0, so no relaxation transient.

    return s


# ─────────────────────────────────────────────────────────────────────────────
# TRACKING ERROR HELPER
# ─────────────────────────────────────────────────────────────────────────────
def find_closest_reference_bounded(path_X, path_Y, path_Psi, x_g, y_g, last_idx, window=40):
    """
    Find the closest point on the reference path (path_X, path_Y) to the
    given global position, searching within a bounded window around last_idx.

    The windowed search prevents the tracker from jumping backward on paths
    that double back on themselves (e.g. after a hairpin). At the start of a
    simulation (last_idx ≤ 5), a wider initial window prevents the tracker
    from locking onto index 0 if the vehicle has already moved forward.

    This function reads the module-level path_X, path_Y, path_Psi arrays.

    Parameters
    ----------
    x_g, y_g : float   Vehicle global position (m).
    last_idx : int      Previously found closest index (search anchor).
    window : int        Forward search range in path indices. Default 40.

    Returns
    -------
    (global_idx, ref_x, ref_y, ref_psi) : (int, float, float, float)
        Index and coordinates of the nearest path point, plus path heading there.

    Called by: simulate_closed_loop() — fallback path when SimPlanner has no centreline,
               and for path-end detection (idx ≥ len(path_X) - 2)
    """
    if last_idx <= 5:
        start_search = 0
        end_search   = min(len(path_X), 100)   # Wide initial window
    else:
        start_search = max(0, last_idx - 5)
        end_search   = min(len(path_X), last_idx + window)

    distances  = np.hypot(
        path_X[start_search:end_search] - x_g,
        path_Y[start_search:end_search] - y_g,
    )
    local_idx  = np.argmin(distances)
    global_idx = start_search + local_idx

    return global_idx, path_X[global_idx], path_Y[global_idx], path_Psi[global_idx]

def get_interpolated_ref_point(x, y, path_x, path_y, path_psi):
    """
    Computes a smooth, continuous reference point on the path via linear interpolation.

    To eliminate discontinuous "ballooning" errors caused by snapping to discrete path
    nodes, this function projects the vehicle's position onto the line segment between
    the two closest path points. This generates a "virtual" reference point (rx, ry, rpsi)
    that allows for continuous, smooth error estimation, significantly reducing noise
    in the tracking error signal.

    Parameters
    ----------
    x, y : float
        Current vehicle global coordinates (m).
    path_x, path_y : np.ndarray
        Reference path coordinates.
    path_psi : np.ndarray
        Reference path heading at each point (rad).

    Returns
    -------
    ref_x, ref_y : float
        The interpolated global coordinates on the path closest to the vehicle.
    ref_psi : float
        The interpolated heading angle at the projected point (rad).
    """
    # Find squared distance to all points
    dist_sq = (path_x - x)**2 + (path_y - y)**2
    idx = np.argmin(dist_sq)

    # If at the very start or end, return the closest point
    if idx <= 0 or idx >= len(path_x) - 1:
        return path_x[idx], path_y[idx], path_psi[idx]

    # Use the closest point and the one ahead/behind it (whichever is closer)
    # We define vectors along the path
    p_prev = np.array([path_x[idx-1], path_y[idx-1]])
    p_curr = np.array([path_x[idx], path_y[idx]])
    p_next = np.array([path_x[idx+1], path_y[idx+1]])

    # Choose the segment [p_prev, p_curr] or [p_curr, p_next]
    # We pick the one that the vehicle is closer to
    dist_to_prev = np.hypot(x - p_prev[0], y - p_prev[1])
    dist_to_next = np.hypot(x - p_next[0], y - p_next[1])
    
    if dist_to_prev < dist_to_next:
        p1, p2 = p_prev, p_curr
        psi1, psi2 = path_psi[idx-1], path_psi[idx]
    else:
        p1, p2 = p_curr, p_next
        psi1, psi2 = path_psi[idx], path_psi[idx+1]

    # Project vehicle onto line segment p1->p2 to find interpolation factor 't'
    v = p2 - p1
    w = np.array([x, y]) - p1
    t = np.clip(np.dot(w, v) / np.dot(v, v), 0, 1)

    # Linear Interpolate
    ref_x = p1[0] + t * v[0]
    ref_y = p1[1] + t * v[1]
    
    # Angular Interpolation (normalise angle difference)
    d_psi = (psi2 - psi1 + np.pi) % (2 * np.pi) - np.pi
    ref_psi = psi1 + t * d_psi

    return ref_x, ref_y, ref_psi

def plant_to_tracking_error(state, ref_x=None, ref_y=None, ref_psi=None, 
                            path_x=None, path_y=None, path_psi=None):
    """
    Computes tracking error. If path_x/y/psi are provided, it interpolates 
    the reference point for smoother cornering.
    """
    # 1. Resolve Reference Point (Interpolated or Provided)
    if path_x is not None:
        ref_x, ref_y, ref_psi = get_interpolated_ref_point(
            state[IDX_X], state[IDX_Y], path_x, path_y, path_psi
        )
    
    # 2. Extract State
    X, Y, psi  = state[IDX_X], state[IDX_Y], state[IDX_PSI]
    vx, vy     = state[IDX_VX], state[IDX_VY]
    r          = state[IDX_R]
    delta_act  = state[IDX_DELTA]
    a_act      = state[IDX_A_ACT]

    # 3. Calculate Errors
    dx = X - ref_x
    dy = Y - ref_y
    
    # Lateral error (signed distance)
    e_y_proj = dy * np.cos(ref_psi) - dx * np.sin(ref_psi)
    e_y = e_y_proj  # Keep sign intact directly
    
    # Heading error
    e_psi = np.arctan2(np.sin(psi - ref_psi), np.cos(psi - ref_psi))

    # Velocities
    e_y_dot = vx * np.sin(e_psi) + vy * np.cos(e_psi)
    e_psi_dot = r 

    return e_y, e_y_dot, e_psi, e_psi_dot, delta_act, a_act, vx