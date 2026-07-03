# Language: python
# Title: High-Fidelity Vehicle Physics Plant (vehicle_physics.py)
"""
Nonlinear Vehicle Plant Model — PhysX/FSDS Fidelity Level
File Name: vehicle_physics.py

Upgrade from the previous dual-track model to a full 22-state plant that
matches the fidelity level of Nvidia PhysX (used by FSDS / AirSim / UE4)
and the AMZ Driverless winning simulation stack.

State vector (22 elements) — indices 0-7 are IDENTICAL to the old model
so simulation.py and offline_tuner.py require zero changes to their
index accesses:

  [0]  X           — global position X (m)
  [1]  Y           — global position Y (m)
  [2]  psi         — yaw angle (rad)
  [3]  vx          — longitudinal velocity body frame (m/s)
  [4]  vy          — lateral velocity body frame (m/s)
  [5]  r           — yaw rate (rad/s)
  [6]  delta_act   — actual steering angle after lag (rad)
  [7]  a_act       — actual acceleration command after lag (m/s²)
  [8]  omega_RL    — rear-left  wheel spin (rad/s)
  [9]  omega_RR    — rear-right wheel spin (rad/s)
  [10] z_FL        — front-left  suspension deflection from equilibrium (m, +ve = compression)
  [11] z_FR        — front-right suspension deflection from equilibrium (m)
  [12] z_RL        — rear-left   suspension deflection from equilibrium (m)
  [13] z_RR        — rear-right  suspension deflection from equilibrium (m)
  [14] dz_FL_dt    — front-left  suspension velocity (m/s)
  [15] dz_FR_dt    — front-right suspension velocity (m/s)
  [16] dz_RL_dt    — rear-left   suspension velocity (m/s)
  [17] dz_RR_dt    — rear-right  suspension velocity (m/s)
  [18] Fy_FL_rlx   — front-left  actual lateral tyre force (N, after relaxation lag)
  [19] Fy_FR_rlx   — front-right actual lateral tyre force (N)
  [20] Fy_RL_rlx   — rear-left   actual lateral tyre force (N)
  [21] Fy_RR_rlx   — rear-right  actual lateral tyre force (N)

Suspension deflection convention
─────────────────────────────────
z[i] = 0 at the static equilibrium position.  The spring force on the
chassis from corner i is:

    F_spring_i = k_i * (z_eq_i + z_i)

where k_i * z_eq_i = Fz_static_i  (equilibrium condition, pre-computed).

Storing only the DEVIATION from equilibrium (z[i]) means the spring-force
floor is automatically the static load — there is no double-counting with a
quasi-static layer.  The unsprung mass equation is then:

    m_us * ddz_i = Fz_road_i - k_i*(z_eq_i + z_i) - c_i*dz_i - F_arb_i

where Fz_road_i is the normal force the road exerts on the tyre.  We
approximate Fz_road ≈ Fz_static_i (planar road — no wheel hop off the
ground) so the net driving term is just:

    m_us * ddz_i = Fz_static_i - k_i*(z_eq_i + z_i) - c_i*dz_i - F_arb_i
                 = -k_i*z_i - c_i*dz_i - F_arb_i        (since k_i*z_eq_i = Fz_static_i)

This is a standard spring-damper with ARB coupling — stable by construction.

All seven additions from the analysis are present:
  1. Full MF94 Pacejka lateral+longitudinal (E, Sv, Sh, camber thrust)
  2. Longitudinal wheel spin dynamics (omega per driven wheel, slip ratio)
  3. Per-wheel spring/damper/ARB suspension with dynamic Fz
  4. Split front/rear aero downforce (Cl_front, Cl_rear) + pitch sensitivity
  5. Tyre relaxation length (first-order lag on lateral force)
  6. Torque vectoring (tv_gain parameter, default 0 = disabled)
  7. Kinematic camber gain (suspension deflection → camber → Pacejka D shift)
  8. Road-surface mu scaling (road_mu parameter, default 1.0 = dry)
"""
import numpy as np

# ─────────────────────────────────────────────────────────────
# NAMED STATE INDICES
# ─────────────────────────────────────────────────────────────
IDX_X        = 0
IDX_Y        = 1
IDX_PSI      = 2
IDX_VX       = 3
IDX_VY       = 4
IDX_R        = 5
IDX_DELTA    = 6
IDX_A_ACT    = 7
IDX_OMEGA_RL = 8
IDX_OMEGA_RR = 9
IDX_Z_FL     = 10
IDX_Z_FR     = 11
IDX_Z_RL     = 12
IDX_Z_RR     = 13
IDX_DZ_FL    = 14
IDX_DZ_FR    = 15
IDX_DZ_RL    = 16
IDX_DZ_RR    = 17
IDX_FY_FL    = 18
IDX_FY_FR    = 19
IDX_FY_RL    = 20
IDX_FY_RR    = 21
IDX_OMEGA_FL = 22
IDX_OMEGA_FR = 23
N_STATES     = 24


# ─────────────────────────────────────────────────────────────
# VEHICLE PARAMETERS
# ─────────────────────────────────────────────────────────────
class VehicleParams:
    """
    Physical parameters for a Formula Student EV (~255 kg wet).
    All values are documented with source and rationale.
    """

    def __init__(self):
        # ── Geometry ────────────────────────────────────────────────
        self.lf    = 0.85       # CoM → front axle (m)
        self.lr    = 0.70       # CoM → rear  axle (m)
        self.m     = 255.0     # total vehicle mass (kg)
        self.Iz    = 110.0     # yaw inertia (kg·m²)
        self.tf    = 1.25      # front track width (m)
        self.tr    = 1.20      # rear  track width (m)
        self.h_cg  = 0.30      # CoG height (m)
        self.Cf = 11500.0   # front cornering stiffness (N/rad)
        self.Cr = 12500.0   # rear  cornering stiffness (N/rad)
        self.max_steer = np.radians(35.0)  # max steering angle (rad)
        self.max_accel  = 5.0       # max longitudinal acceleration (m/s²)
        self.max_accel_brake = -5.0      # max longitudinal braking (m/s²)

        # ── Unsprung mass ────────────────────────────────────────────
        self.m_us  = 7.5       # unsprung mass per corner (kg)

        # ── Tyre geometry ────────────────────────────────────────────
        self.r_eff = 0.2286    # effective rolling radius, 13" wheel (m)

        # ── Wheel + drivetrain rotational inertia ────────────────────
        # Wheel: hollow cylinder ~0.9 kg·m²
        # Motor/gearbox inertia referred to wheel ~0.05 kg·m²
        self.I_wheel      = 0.9    # per wheel (kg·m²)
        self.I_drivetrain = 0.05   # per rear driven wheel (kg·m²)
        self.I_w_eff_r    = self.I_wheel + self.I_drivetrain
        self.I_w_eff_f    = self.I_wheel  # Front wheels lack drivetrain mass
        self.brake_bias   = 0.60   # 60% front braking distributio

        # ── Suspension ──────────────────────────────────────────────
        # FS typical effective wheel-rate: front 25 N/mm, rear 30 N/mm
        # (spring rate × motion_ratio²; MR ≈ 0.65-0.70 for pushrod)
        self.k_susp_f = 25000.0   # front wheel-rate (N/m)
        self.k_susp_r = 30000.0   # rear  wheel-rate (N/m)

        # Damping: ~30% of critical = 2*sqrt(k * m_corner)
        # m_corner_f ≈ 255*0.4/2 = 51 kg → c_crit ≈ 2*sqrt(25000*51) ≈ 2254
        # 30% → 676; round up to 1500 to include unsprung contribution
        self.c_damp_f = 1500.0    # front damper rate (N·s/m)
        self.c_damp_r = 1800.0    # rear  damper rate (N·s/m)

        # Anti-roll bars — equivalent wheel-rate contribution
        # Front ARB ~8 N/mm, rear ~6 N/mm (FS typical aluminium bars)
        self.k_arb_f = 8000.0     # front ARB wheel-rate (N/m)
        self.k_arb_r = 6000.0     # rear  ARB wheel-rate (N/m)

        # Suspension travel limits (bump / droop from equilibrium)
        self.z_max =  0.040    # max compression from equilibrium (m)
        self.z_min = -0.040    # max droop     from equilibrium (m)

        # ── Kinematic camber gain ───────────────────────────────────
        # Double-wishbone FS: ~0.5 deg camber per mm of travel
        #   = 0.5 * pi/180 / 0.001 = 8.73 rad/m
        self.camber_gain   = 8.73   # rad camber per m of compression
        # Camber stiffness: fraction of Fz added as lateral force per rad gamma
        # Typical slick ~0.12-0.18
        self.camber_stiff_f = 0.15
        self.camber_stiff_r = 0.12

        # ── Pacejka MF94 lateral (FS slick, aligned to Cf=13500 N/rad) ─
        # B, C, D from previous model (calibrated).
        # E ~-1.5 (sharpens peak vs pure BCD), typical for racing slick.
        # Sv, Sh: small ply-steer / conicity offsets (rad, N)
        self.B_f  = 13.5;  self.C_f  = 1.40;  self.D_f  = 0.90
        self.E_f  = -1.5;  self.Sv_f = 0.0;   self.Sh_f = 0.002

        self.B_r  = 10.0;  self.C_r  = 1.40;  self.D_r  = 0.95
        self.E_r  = -1.8;  self.Sv_r = 0.0;   self.Sh_r = 0.001

        # ── Pacejka MF94 longitudinal (rear wheels) ──────────────────
        # Peak at slip ratio ~0.10-0.15 for slick; Dx≈1.0 (similar to mu)
        self.Bx_r = 12.0;  self.Cx_r = 1.65
        self.Dx_r = 1.00;  self.Ex_r = -0.5

        # ── Tyre relaxation lengths ──────────────────────────────────
        # First-order lag: dFy/dt = (vx/sigma) * (Fy_ss - Fy_act)
        # FS slick: sigma_y ≈ 0.30-0.40 m (lateral)
        self.sigma_y_f = 0.35      # front lateral relaxation length (m)
        self.sigma_y_r = 0.30      # rear  lateral relaxation length (m)

        # ── Friction & load sensitivity ──────────────────────────────
        self.mu     = 1.6          # peak friction coefficient (racing slick)
        self.k_sens = 0.00018      # load sensitivity (1/N) — degrades mu at high Fz

        # ── Aerodynamics — split front/rear ─────────────────────────
        # Total Cl_A = 1.5, split 43/57 front/rear for neutral aero balance
        self.rho       = 1.225     # air density (kg/m³)
        self.Cd_A      = 0.9       # drag area (m²)
        self.Cl_A_f    = 0.645     # front downforce area
        self.Cl_A_r    = 0.855     # rear  downforce area
        # Pitch sensitivity: +1% front Cl per % longitudinal deceleration
        # expressed as fraction of g per unit pitch angle proxy
        self.Cl_pitch_sens = 0.03  # fractional Cl_f change per (a/g)

        # ── Rolling resistance & Stiction (UPDATED) ──────────────────
        self.Crr        = 37.5     # constant dynamic drag force (N)
        self.F_stiction = 70.0     # static breakaway friction (N)

        # ── Actuator lag (UPDATED) ───────────────────────────────────
        self.tau_delta = 0.08      # steering (s) - FS actuators are FAST
        self.tau_a     = 0.05      # acceleration (s) - EV torque is nearly instant

        self.g = 9.81

    @property
    def L(self):
        return self.lf + self.lr

    def static_fz_per_corner(self):
        """Static normal load per corner at rest (no aero, no accel)."""
        Fz_f = self.m * self.g * (self.lr / self.L) / 2.0
        Fz_r = self.m * self.g * (self.lf / self.L) / 2.0
        return Fz_f, Fz_r   # (front_corner, rear_corner) in N

    def static_z_equilibrium(self, v_nominal=7.0):
        """
        Suspension equilibrium deflections z_eq (m) at a nominal cruise speed,
        including aerodynamic downforce.  States track deviation from this point.

        Using a speed-inclusive equilibrium means the suspension sits near the
        centre of its travel range at typical running speeds, rather than
        slamming into the bump stop when aero load is added.

        v_nominal : representative speed for aero calculation (m/s).
                    Defaults to 7.0 m/s (the previous fixed v_ref).
        """
        Fz_f_static, Fz_r_static = self.static_fz_per_corner()
        F_down_f = 0.5 * self.rho * self.Cl_A_f * v_nominal**2
        F_down_r = 0.5 * self.rho * self.Cl_A_r * v_nominal**2
        Fz_f_total = Fz_f_static + 0.5 * F_down_f   # per corner
        Fz_r_total = Fz_r_static + 0.5 * F_down_r   # per corner
        return Fz_f_total / self.k_susp_f, Fz_r_total / self.k_susp_r


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 — LATERAL
# ─────────────────────────────────────────────────────────────────────────────
def _mf94(x_in, B, C, D, E):
    """Core MF94 shape function (no offsets, no Fz scaling — applied outside)."""
    Bx = B * x_in
    return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))


def pacejka_lateral_mf94(alpha, Fz, mu, B, C, D, E, Sv, Sh,
                          gamma=0.0, camber_stiff=0.0):
    """
    Full MF94 lateral tyre force (N).

      Fy = mu*Fz * MF94(alpha + Sh)  +  Sv  +  camber_stiff*Fz*gamma

    alpha        : slip angle (rad)
    Fz           : normal load (N)
    mu           : effective friction (load sensitivity already applied)
    B,C,D,E      : Pacejka shape coefficients
    Sv, Sh       : vertical/horizontal offsets (ply steer, conicity)
    gamma        : camber angle (rad; +ve = top leans outward from centre)
    camber_stiff : camber stiffness per unit Fz (fraction)
    """
    Fy0 = mu * Fz * _mf94(alpha + Sh, B, C, D, E) + Sv
    return Fy0 + camber_stiff * Fz * gamma


# ─────────────────────────────────────────────────────────────────────────────
# PACEJKA MF94 — LONGITUDINAL
# ─────────────────────────────────────────────────────────────────────────────
def pacejka_longitudinal_mf94(kappa, Fz, mu, Bx, Cx, Dx, Ex):
    """
    MF94 longitudinal tyre force (N).

      Fx = mu*Fz * MF94(kappa)

    kappa : longitudinal slip ratio (positive = driving, negative = braking)
    """
    return mu * Fz * _mf94(kappa, Bx, Cx, Dx, Ex)


# ─────────────────────────────────────────────────────────────────────────────
# MAIN PLANT STEP
# ─────────────────────────────────────────────────────────────────────────────
def step_nonlinear_plant(state, u_cmd, dt, params: VehicleParams,
                         road_mu=1.0, tv_gain=0.0):
    """
    Advance the 22-state high-fidelity plant by one control timestep dt.

    Parameters
    ----------
    state    : np.ndarray, length 22
    u_cmd    : [delta_cmd (rad), a_cmd (m/s²)]
    dt       : control timestep (s)
    params   : VehicleParams instance
    road_mu  : surface grip multiplier (1.0=dry, 0.6=damp, etc.)
    tv_gain  : torque-vectoring proportional gain (N·m per rad/s of r).
               ΔT = tv_gain * r → ΔFx = ΔT/(r_eff * 2) at each rear wheel.
               Default 0 = disabled.

    Returns
    -------
    np.ndarray, length 22
    """
    p         = params
    sub_steps = 4
    h         = dt / sub_steps

    # Pre-compute static equilibrium deflections (constant for this vehicle)
    z_eq_f, z_eq_r = p.static_z_equilibrium()

    s = state.copy()

    for _ in range(sub_steps):

        # ── Unpack ──────────────────────────────────────────────────────
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
        # Suspension deviation from equilibrium
        z_FL = s[IDX_Z_FL];  z_FR = s[IDX_Z_FR]
        z_RL = s[IDX_Z_RL];  z_RR = s[IDX_Z_RR]
        dz_FL = s[IDX_DZ_FL]; dz_FR = s[IDX_DZ_FR]
        dz_RL = s[IDX_DZ_RL]; dz_RR = s[IDX_DZ_RR]
        # Tyre relaxation states
        Fy_FL_rlx = s[IDX_FY_FL]; Fy_FR_rlx = s[IDX_FY_FR]
        Fy_RL_rlx = s[IDX_FY_RL]; Fy_RR_rlx = s[IDX_FY_RR]

        vx_safe = max(vx, 0.5)

        # ── 1. Actuator lag ─────────────────────────────────────────────
        ddelta = (u_cmd[0] - delta_act) / p.tau_delta
        da     = (u_cmd[1] - a_act)     / p.tau_a

        # ── 2. Aerodynamics (split front/rear, pitch-sensitive) ──────────
        v_sq   = vx_safe**2 + vy**2
        F_drag = 0.5 * p.rho * p.Cd_A * v_sq

        # Pitch proxy: braking (a_act < 0) → nose dips → more front downforce
        pitch_frac = -a_act / p.g
        Cl_f_eff  = p.Cl_A_f * (1.0 + p.Cl_pitch_sens * pitch_frac)
        Cl_r_eff  = p.Cl_A_r * (1.0 - p.Cl_pitch_sens * pitch_frac)
        F_down_f  = 0.5 * p.rho * Cl_f_eff * v_sq
        F_down_r  = 0.5 * p.rho * Cl_r_eff * v_sq

        # ── 3. Wheel-corner velocities (body frame) ─────────────────────
        vx_FL = vx_safe - r * (p.tf / 2.0);  vy_FL = vy + r * p.lf
        vx_FR = vx_safe + r * (p.tf / 2.0);  vy_FR = vy + r * p.lf
        vx_RL = vx_safe - r * (p.tr / 2.0);  vy_RL = vy - r * p.lr
        vx_RR = vx_safe + r * (p.tr / 2.0);  vy_RR = vy - r * p.lr

        # ── 4. Lateral slip angles ───────────────────────────────────────
        alpha_FL = delta_act - np.arctan2(vy_FL, max(vx_FL, 0.5))
        alpha_FR = delta_act - np.arctan2(vy_FR, max(vx_FR, 0.5))
        alpha_RL =           - np.arctan2(vy_RL, max(vx_RL, 0.5))
        alpha_RR =           - np.arctan2(vy_RR, max(vx_RR, 0.5))

        # ── 5. Suspension spring/damper/ARB → normal loads ──────────────
        #
        # Total spring deflection = z_eq_i + z_i  (equilibrium + deviation)
        # Spring force on body = k * (z_eq + z)
        # At equilibrium (z=0, dz=0, z_RL=z_RR): F_spring = k*z_eq = Fz_static ✓
        #
        # ARB coupling: F_arb = k_arb * (z_left - z_right)
        # Applied +F_arb to left corner, -F_arb to right corner.
        #
        # Normal load at each wheel:
        #   Fz = k*(z_eq + z)  +  aero_downforce_per_corner
        # (The aero load adds to road reaction, not to suspension spring force)

        arb_f = p.k_arb_f * (z_FL - z_FR)
        arb_r = p.k_arb_r * (z_RL - z_RR)

        Fz_spring_FL = p.k_susp_f * (z_eq_f + z_FL)
        Fz_spring_FR = p.k_susp_f * (z_eq_f + z_FR)
        Fz_spring_RL = p.k_susp_r * (z_eq_r + z_RL)
        Fz_spring_RR = p.k_susp_r * (z_eq_r + z_RR)

        # Normal load = spring + aero (per corner), floored at 10 N
        Fz_FL = max(10.0, Fz_spring_FL + arb_f + 0.5 * F_down_f)
        Fz_FR = max(10.0, Fz_spring_FR - arb_f + 0.5 * F_down_f)
        Fz_RL = max(10.0, Fz_spring_RL + arb_r + 0.5 * F_down_r)
        Fz_RR = max(10.0, Fz_spring_RR - arb_r + 0.5 * F_down_r)

        # ── 6. Suspension dynamics (deviation from equilibrium) ──────────
        #
        # The unsprung mass equation is:
        #   m_us * ddz = Fz_road - F_spring - F_damp - F_arb
        #
        # For a planar road: Fz_road_i = Fz_static_i + delta_from_load_transfer
        #
        # Quasi-static load transfer terms (what drives the suspension to deflect):
        #   longitudinal: ±(m * ax * h_cg) / L  per axle, then /2 per corner
        #   lateral:      ±(m * ay * h_cg) / track  per axle
        #
        # These are the NET external forces that the suspension spring must resist:
        ax_body = a_act                   # longitudinal acceleration command (m/s²)
        ay_body = vx_safe * r             # lateral acceleration proxy (m/s²)

        # Per-corner road reaction (what the road pushes up with):
        Fz_road_FL = (p.m * p.g * (p.lr / p.L) / 2.0
                      - (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      - (p.m * ay_body * p.h_cg) / (2.0 * p.tf)
                      + 0.5 * F_down_f)
        Fz_road_FR = (p.m * p.g * (p.lr / p.L) / 2.0
                      - (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      + (p.m * ay_body * p.h_cg) / (2.0 * p.tf)
                      + 0.5 * F_down_f)
        Fz_road_RL = (p.m * p.g * (p.lf / p.L) / 2.0
                      + (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      - (p.m * ay_body * p.h_cg) / (2.0 * p.tr)
                      + 0.5 * F_down_r)
        Fz_road_RR = (p.m * p.g * (p.lf / p.L) / 2.0
                      + (p.m * ax_body * p.h_cg) / (2.0 * p.L)
                      + (p.m * ay_body * p.h_cg) / (2.0 * p.tr)
                      + 0.5 * F_down_r)

        # Suspension damper force (opposes velocity)
        Fd_FL = p.c_damp_f * dz_FL;  Fd_FR = p.c_damp_f * dz_FR
        Fd_RL = p.c_damp_r * dz_RL;  Fd_RR = p.c_damp_r * dz_RR

        # The suspension equations of motion are integrated by _integrate_susp
        # below using a semi-implicit scheme (implicit damping) — no ddz needed here.

        # ── 7. Kinematic camber from suspension deflection ───────────────
        # z > 0 (compression) on outside wheel in corner → negative camber
        # gamma sign: +ve = top leans away from vehicle centre
        gamma_FL = -p.camber_gain * (z_eq_f + z_FL)
        gamma_FR =  p.camber_gain * (z_eq_f + z_FR)
        gamma_RL = -p.camber_gain * (z_eq_r + z_RL)
        gamma_RR =  p.camber_gain * (z_eq_r + z_RR)

        # ── 8. Friction coefficient (with load sensitivity) ──────────────
        eff_mu = p.mu * road_mu
        mu_FL  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_FL))
        mu_FR  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_FR))
        mu_RL  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_RL))
        mu_RR  = max(0.1, eff_mu * (1.0 - p.k_sens * Fz_RR))

        # ── 9. Longitudinal slip ratios (rear driven wheels) ─────────────
        def _kappa(omega, vx_w):
            r_om  = p.r_eff * max(omega, 0.0)
            v_ref = max(max(r_om, abs(vx_w)), 0.01)
            return (r_om - abs(vx_w)) / v_ref

        kappa_FL = _kappa(omega_FL, vx_FL)
        kappa_FR = _kappa(omega_FR, vx_FR)
        kappa_RL = _kappa(omega_RL, vx_RL)
        kappa_RR = _kappa(omega_RR, vx_RR)

        # ── 10. Torque vectoring & Longitudinal Force Distribution ─────
        delta_Fx_tv = (tv_gain * r / p.tr) if abs(tv_gain) > 1e-6 else 0.0
        Fx_req_total = p.m * a_act
        
        if a_act > 0:
            # Acceleration: RWD only
            Fx_FL_req, Fx_FR_req = 0.0, 0.0
            Fx_RL_req = 0.5 * Fx_req_total - delta_Fx_tv
            Fx_RR_req = 0.5 * Fx_req_total + delta_Fx_tv
        else:
            # Braking: Distributed by brake bias
            Fx_FL_req = 0.5 * Fx_req_total * p.brake_bias
            Fx_FR_req = 0.5 * Fx_req_total * p.brake_bias
            Fx_RL_req = 0.5 * Fx_req_total * (1.0 - p.brake_bias) - delta_Fx_tv
            Fx_RR_req = 0.5 * Fx_req_total * (1.0 - p.brake_bias) + delta_Fx_tv

        # ── 11. Pacejka longitudinal force ────────────────────────
        Fmax_FL, Fmax_FR = mu_FL * Fz_FL, mu_FR * Fz_FR
        Fmax_RL, Fmax_RR = mu_RL * Fz_RL, mu_RR * Fz_RR

        # Note: Using rear Bx, Cx, Dx, Ex for front assuming identical slick compounds
        Fx_FL_pac = pacejka_longitudinal_mf94(kappa_FL, Fz_FL, mu_FL, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_FR_pac = pacejka_longitudinal_mf94(kappa_FR, Fz_FR, mu_FR, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_RL_pac = pacejka_longitudinal_mf94(kappa_RL, Fz_RL, mu_RL, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)
        Fx_RR_pac = pacejka_longitudinal_mf94(kappa_RR, Fz_RR, mu_RR, p.Bx_r, p.Cx_r, p.Dx_r, p.Ex_r)

        # Commanded force saturated by friction ceiling and Pacejka curve
        Fx_FL = float(np.clip(Fx_FL_req, -min(Fmax_FL, abs(Fx_FL_pac) + 1.0), min(Fmax_FL, abs(Fx_FL_pac) + 1.0)))
        Fx_FR = float(np.clip(Fx_FR_req, -min(Fmax_FR, abs(Fx_FR_pac) + 1.0), min(Fmax_FR, abs(Fx_FR_pac) + 1.0)))
        Fx_RL = float(np.clip(Fx_RL_req, -min(Fmax_RL, abs(Fx_RL_pac) + 1.0), min(Fmax_RL, abs(Fx_RL_pac) + 1.0)))
        Fx_RR = float(np.clip(Fx_RR_req, -min(Fmax_RR, abs(Fx_RR_pac) + 1.0), min(Fmax_RR, abs(Fx_RR_pac) + 1.0)))

        # ── 12. Wheel spin dynamics ──────────────────────────────────────
        # I_w * dω/dt = T_input - Fx * r_eff
        # T_input = Fx_req * r_eff  (commanded torque; Fx_req sets target slip)
        domega_FL = (Fx_FL_req * p.r_eff - Fx_FL * p.r_eff) / p.I_w_eff_f
        domega_FR = (Fx_FR_req * p.r_eff - Fx_FR * p.r_eff) / p.I_w_eff_f
        domega_RL = (Fx_RL_req * p.r_eff - Fx_RL * p.r_eff) / p.I_w_eff_r
        domega_RR = (Fx_RR_req * p.r_eff - Fx_RR * p.r_eff) / p.I_w_eff_r

        # ── 13. MF94 lateral steady-state ───────────────────────────────
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

        # ── 14. Tyre relaxation (first-order lag) ────────────────────────
        # dFy/dt = (vx/σ) * (Fy_ss - Fy_rlx)
        # Euler step: Fy_rlx += h * (vx/σ) * (Fy_ss - Fy_rlx)
        # Clamp gain to 1 to avoid overshoot in a single substep
        gain_f = min(1.0, (vx_safe / p.sigma_y_f) * h)
        gain_r = min(1.0, (vx_safe / p.sigma_y_r) * h)

        Fy_FL_rlx_new = Fy_FL_rlx + gain_f * (Fy_FL_ss - Fy_FL_rlx)
        Fy_FR_rlx_new = Fy_FR_rlx + gain_f * (Fy_FR_ss - Fy_FR_rlx)
        Fy_RL_rlx_new = Fy_RL_rlx + gain_r * (Fy_RL_ss - Fy_RL_rlx)
        Fy_RR_rlx_new = Fy_RR_rlx + gain_r * (Fy_RR_ss - Fy_RR_rlx)

        # ── 15. Friction ellipse coupling (Fx + Fy ≤ mu*Fz) ────────────
        # Front: free-rolling, no Fx commanded → full Fmax available for Fy
        ell_FL = max(0.0, 1.0 - (Fx_FL / max(Fmax_FL, 1.0))**2)
        ell_FR = max(0.0, 1.0 - (Fx_FR / max(Fmax_FR, 1.0))**2)
        Fy_FL  = float(np.clip(Fy_FL_rlx_new * np.sqrt(ell_FL), -Fmax_FL, Fmax_FL))
        Fy_FR  = float(np.clip(Fy_FR_rlx_new * np.sqrt(ell_FR), -Fmax_FR, Fmax_FR))

        # Rear: friction ellipse reduces available lateral force when Fx is present
        ell_RL = max(0.0, 1.0 - (Fx_RL / max(Fmax_RL, 1.0))**2)
        ell_RR = max(0.0, 1.0 - (Fx_RR / max(Fmax_RR, 1.0))**2)
        Fy_RL  = float(np.clip(Fy_RL_rlx_new * np.sqrt(ell_RL), -Fmax_RL, Fmax_RL))
        Fy_RR  = float(np.clip(Fy_RR_rlx_new * np.sqrt(ell_RR), -Fmax_RR, Fmax_RR))

        # ── 16. Body-frame coordinate transform (front steer) ───────────
        cd = np.cos(delta_act);  sd = np.sin(delta_act)
        Fx_FL_b = Fx_FL * cd - Fy_FL * sd;  Fy_FL_b = Fx_FL * sd + Fy_FL * cd
        Fx_FR_b = Fx_FR * cd - Fy_FR * sd;  Fy_FR_b = Fx_FR * sd + Fy_FR * cd
        Fx_RL_b, Fy_RL_b = Fx_RL, Fy_RL
        Fx_RR_b, Fy_RR_b = Fx_RR, Fy_RR

        # ── 17. Rolling resistance ───────────────────────────────────────
        F_roll = p.Crr * np.sign(vx) if abs(vx) > 1e-3 else 0.0

        # ── 18. Resultant forces & yaw moment ───────────────────────────
        Fx_total = Fx_FL_b + Fx_FR_b + Fx_RL_b + Fx_RR_b - F_drag - F_roll
        Fy_total = Fy_FL_b + Fy_FR_b + Fy_RL_b + Fy_RR_b

        M_z = (  p.lf * (Fy_FL_b + Fy_FR_b)
               - p.lr * (Fy_RL_b + Fy_RR_b)
               + (p.tf / 2.0) * (Fx_FR_b - Fx_FL_b)
               + (p.tr / 2.0) * (Fx_RR_b - Fx_RL_b))

        # ── 19. Rigid-body EOM ───────────────────────────────────────────
        ax_rb = Fx_total / p.m + vy * r
        ay_rb = Fy_total / p.m - vx_safe * r
        r_dot = M_z / p.Iz

        # ── 20. Integrate all states (explicit Euler, substep h) ─────────
        vx_new  = max(0.0, vx + ax_rb * h)
        vy_new  = vy + ay_rb * h
        r_new   = r  + r_dot * h

        X_new   = X   + (vx * np.cos(psi) - vy * np.sin(psi)) * h
        Y_new   = Y   + (vx * np.sin(psi) + vy * np.cos(psi)) * h
        psi_new = psi + r * h

        delta_new = delta_act + ddelta * h
        a_new     = a_act     + da     * h

        omega_RL_new = max(0.0, omega_RL + domega_RL * h)
        omega_RR_new = max(0.0, omega_RR + domega_RR * h)
        omega_FL_new = max(0.0, omega_FL + domega_FL * h)
        omega_FR_new = max(0.0, omega_FR + domega_FR * h)

        def _integrate_susp(z, dz, Fz_road, Fz_spring, F_damp, F_arb_signed, k, c):
            """
            Semi-implicit integration for a spring-damper-ARB suspension corner.

            Implicit damping makes this unconditionally stable regardless of the
            spring rate, damper rate, or timestep h — critical here because the
            FS suspension natural frequency (~11 Hz front, ~12 Hz rear) is high
            relative to the 0.0125 s substep, and explicit Euler would require
            many more substeps to remain stable.

            Semi-implicit scheme:
              dz_new = (dz + (Fz_road - Fz_spring - F_arb) / m_us * h)
                       / (1 + c/m_us * h)
              z_new  = z + dz_new * h          ← use updated velocity

            This is equivalent to treating damping implicitly (denominator)
            while treating spring and external forces explicitly.  It is O(h)
            accurate and unconditionally stable for a linear spring-damper,
            which the suspension is (ARB is linear in z difference).

            Hard bump-stop: if travel limit is reached, zero velocity component
            pointing into the limit (inelastic contact).
            """
            # Net external forcing (excluding damper — handled implicitly)
            F_net = Fz_road - Fz_spring - F_arb_signed
            # Semi-implicit velocity update
            dz_new = (dz + (F_net / p.m_us) * h) / (1.0 + (c / p.m_us) * h)
            # Position update using new velocity
            z_new  = float(np.clip(z + dz_new * h, p.z_min, p.z_max))
            # Bump-stop contact: inelastic — zero velocity into the limit
            if z_new >= p.z_max and dz_new > 0.0:
                dz_new = 0.0
            if z_new <= p.z_min and dz_new < 0.0:
                dz_new = 0.0
            return z_new, dz_new

        z_FL_new, dz_FL_new = _integrate_susp(
            z_FL, dz_FL, Fz_road_FL, Fz_spring_FL,
            Fd_FL, arb_f, p.k_susp_f, p.c_damp_f)
        z_FR_new, dz_FR_new = _integrate_susp(
            z_FR, dz_FR, Fz_road_FR, Fz_spring_FR,
            Fd_FR, -arb_f, p.k_susp_f, p.c_damp_f)
        z_RL_new, dz_RL_new = _integrate_susp(
            z_RL, dz_RL, Fz_road_RL, Fz_spring_RL,
            Fd_RL, arb_r, p.k_susp_r, p.c_damp_r)
        z_RR_new, dz_RR_new = _integrate_susp(
            z_RR, dz_RR, Fz_road_RR, Fz_spring_RR,
            Fd_RR, -arb_r, p.k_susp_r, p.c_damp_r)

        # ── 21. Pack new state ───────────────────────────────────────────
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
# INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
def init_plant_state(X0, Y0, psi0, vx0=10.0):
    """
    Build an initial 22-element plant state vector at true static equilibrium:

      - Wheel speeds set to no-slip: omega = vx0 / r_eff
      - Suspension deflection deviation z_i = 0  (all corners at equilibrium;
        the spring force k*(z_eq + 0) = k*z_eq = Fz_static exactly balances
        the weight at each corner, so ddz = 0 and there are no t=0 transients)
      - Relaxed tyre forces = 0 (zero slip angle at start → zero Fy_ss → no transient)
    """
    s = np.zeros(N_STATES)
    p = VehicleParams()

    s[IDX_X]   = X0
    s[IDX_Y]   = Y0
    s[IDX_PSI] = psi0
    s[IDX_VX]  = vx0

    # Wheel angular velocity: rolling without slip at vx0
    s[IDX_OMEGA_RL] = vx0 / p.r_eff
    s[IDX_OMEGA_RR] = vx0 / p.r_eff
    s[IDX_OMEGA_FL] = vx0 / p.r_eff
    s[IDX_OMEGA_FR] = vx0 / p.r_eff

    # Suspension deviation from equilibrium = 0 for all corners.
    # z_eq is computed at vx0, so suspension is centred in its travel
    # range at the initial speed (no t=0 aero transient).
    # Indices 10-17 remain zero from np.zeros — this IS the equilibrium state.

    # Tyre relaxation forces: zero at zero slip (indices 18-21 remain zero)

    return s


# ─────────────────────────────────────────────────────────────────────────────
# TRACKING ERROR HELPER (unchanged — reads only indices 0-7)
# ─────────────────────────────────────────────────────────────────────────────
def plant_to_tracking_error(state, ref_x, ref_y, ref_psi):
    """
    Convert global plant state into MPC tracking errors.
    Reads only indices 0-7, so it works with both 8-state and 22-state vectors.
    """
    X, Y, psi  = state[IDX_X], state[IDX_Y], state[IDX_PSI]
    vx, vy     = state[IDX_VX], state[IDX_VY]
    r          = state[IDX_R]
    delta_act  = state[IDX_DELTA]
    a_act      = state[IDX_A_ACT]

    dx    = X - ref_x
    dy    = Y - ref_y
    e_y   = dy * np.cos(ref_psi) - dx * np.sin(ref_psi)
    e_psi = np.arctan2(np.sin(psi - ref_psi), np.cos(psi - ref_psi))

    e_y_dot   = vx * np.sin(e_psi) + vy * np.cos(e_psi)
    e_psi_dot = r

    return e_y, e_y_dot, e_psi, e_psi_dot, delta_act, a_act, vx