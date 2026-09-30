"""
model/vehicle_physics/plant_step.py — one control-step of the nonlinear plant

PURPOSE
-------
step_nonlinear_plant(): advances the 25-state plant by one control timestep,
sub-stepping the ODEs 4x internally for numerical stability. Combines the
tyre curves (tyres.py), the state layout (state.py) and VehicleParams
(params.py); the per-sub-step pipeline is listed in the function docstring.

USED BY
-------
  gui/simulation.py, gui/manual_drive.py, tuner/offline_tuner.py, the
  offline rollout and tuner/validation/plant_openloop_validation.py.
"""
import numpy as np

from model.vehicle_physics.params import VehicleParams
from model.vehicle_physics.state import (
    IDX_X,
    IDX_Y,
    IDX_PSI,
    IDX_VX,
    IDX_VY,
    IDX_R,
    IDX_DELTA,
    IDX_A_ACT,
    IDX_OMEGA_RL,
    IDX_OMEGA_RR,
    IDX_Z_FL,
    IDX_Z_FR,
    IDX_Z_RL,
    IDX_Z_RR,
    IDX_DZ_FL,
    IDX_DZ_FR,
    IDX_DZ_RL,
    IDX_DZ_RR,
    IDX_FY_FL,
    IDX_FY_FR,
    IDX_FY_RL,
    IDX_FY_RR,
    IDX_OMEGA_FL,
    IDX_OMEGA_FR,
    IDX_ALAT_LIM,
    N_STATES,
)
from model.vehicle_physics.tyres import pacejka_lateral_mf94, pacejka_longitudinal_mf94


# ─────────────────────────────────────────────────────────────────────────────
# MAIN PLANT INTEGRATION STEP
# ─────────────────────────────────────────────────────────────────────────────
def step_nonlinear_plant(state, u_cmd, dt, params: VehicleParams,
                         road_mu=1.0, tv_gain=0.0):
    """
    Advance the 25-state nonlinear plant by one control timestep dt.

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

    Called by: gui/simulation.py (simulate_closed_loop),
               tuner/offline_tuner.py (run_headless_rollout)
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
        alat_lim  = s[IDX_ALAT_LIM]

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

        # ── 18b. FSDS lateral-acceleration ceiling ────────────────────────────
        # Restoring yaw moment for FSDS's lateral-accel ceiling (modelled as
        # lagged, not a hard clip, to reproduce the measured ~30% overshoot a
        # clip cannot). See VehicleParams.alat_ceiling* fields above and
        # `docs/reference/simulator_fidelity.md`'s ceiling section for the
        # measurement and full derivation.
        #
        # alat_lim is a STATE that accumulates over time (not a function of
        # the instantaneous excess) — a memoryless term would engage the
        # instant the ceiling is crossed and could never overshoot it.
        if p.alat_ceiling_enabled:
            ceiling_now = p.alat_ceiling_at(vx_safe)
            if p.alat_ceiling_mode == 'pi':
                # Leaky INTEGRAL of the signed excess, clamped at zero — can
                # only stop growing when the excess is zero, so a_lat settles
                # AT the ceiling for any gain (see alat_ceiling_mode's comment
                # above for why this beats a proportional law).
                err = abs(vx_safe * r) - ceiling_now
                alat_lim = max(0.0, alat_lim + err * (
                    h / max(p.alat_ceiling_tau, 1e-3)))
            else:
                # REJECTED proportional law — see alat_ceiling_mode's comment.
                excess = max(0.0, abs(vx_safe * r) - ceiling_now)
                alat_lim = alat_lim + (excess - alat_lim) * (
                    h / max(p.alat_ceiling_tau, 1e-3))
            M_z = M_z - np.sign(r) * alat_lim * p.alat_ceiling_gain
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
        s_new[IDX_ALAT_LIM] = alat_lim
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
