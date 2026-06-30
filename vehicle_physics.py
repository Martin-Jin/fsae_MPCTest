# Language: python
# Title: Dual-Track Vehicle Physics Simulator (vehicle_physics.py)
"""
Nonlinear Vehicle Plant Model (High-Fidelity Dual-Track Truth Simulator)
File Name: vehicle_physics.py

An advanced 4-wheel dual-track model capturing combined slip friction curves,
2D transient load transfer, and tire load sensitivity to match the physical 
fidelity of advanced driving simulators like FSDS / Unreal Engine 4.
"""
import numpy as np


# Language: python
# Title: Tuned VehicleParams (vehicle_physics.py)
class VehicleParams:
    """Physical parameters tuned to mathematically align with the MPC linear model."""

    def __init__(self):
        self.lf = 0.9          # CoM -> front axle (m)
        self.lr = 0.6          # CoM -> rear  axle (m)
        self.m = 255.0         # mass (kg)
        self.Iz = 110.0        # yaw inertia (kg m^2)

        self.tf = 1.25         # Front track width (m)
        self.tr = 1.20         # Rear track width (m)
        self.h_cg = 0.30       # CoG height (m)
        self.K_rot_f = 0.60    # Front roll stiffness split
        self.k_sens = 0.00018  # Tire load sensitivity coefficient (1/N)

        # TUNED PACEJKA COEFFS: Aligned to Cf=15000, Cr=17000 in model.py
        self.mu = 1.6          # True racing slick peak friction
        self.B_f, self.C_f, self.D_f = 13.5, 1.4, 1.0
        self.B_r, self.C_r, self.D_r = 10.0, 1.4, 1.0

        # Aero configurations
        self.rho = 1.225
        self.Cd_A = 0.9
        self.Cl_A = 1.5

        # Rolling resistance
        self.Crr = 12.0

        # Actuator lag
        self.tau_delta = 0.30
        self.tau_a = 0.20
        self.g = 9.81

    @property
    def L(self):
        return self.lf + self.lr


def pacejka_lateral(alpha, Fz, mu, B, C, D):
    """Simplified Magic Formula lateral force."""
    return mu * Fz * D * np.sin(C * np.arctan(B * alpha))


def step_nonlinear_plant(state, u_cmd, dt, params: VehicleParams):
    """
    Advance a high-fidelity 4-wheel dual-track plant model by one timestep.
    Incorporates 2D load transfer (lateral + longitudinal), tire load sensitivity,
    and fully coupled combined slip friction ellipses per wheel.
    """
    p = params
    sub_steps = 4
    h = dt / sub_steps

    s = state.copy()

    for _ in range(sub_steps):
        X, Y, psi, vx, vy, r, delta_act, a_act = s

        # First-order actuator lag dynamics
        ddelta = (u_cmd[0] - delta_act) / p.tau_delta
        da = (u_cmd[1] - a_act) / p.tau_a

        vx_safe = max(vx, 0.5)  # Guard against division-by-zero anomalies

        # 1. Individual Wheel Center Velocities (Body Frame)
        vx_FL = vx_safe - r * (p.tf / 2.0)
        vy_FL = vy + r * p.lf

        vx_FR = vx_safe + r * (p.tf / 2.0)
        vy_FR = vy + r * p.lf

        vx_RL = vx_safe - r * (p.tr / 2.0)
        vy_RL = vy - r * p.lr

        vx_RR = vx_safe + r * (p.tr / 2.0)
        vy_RR = vy - r * p.lr

        # 2. Wheel Slip Angles (Kinematic slip angles at each patch)
        alpha_FL = delta_act - np.arctan2(vy_FL, max(vx_FL, 0.5))
        alpha_FR = delta_act - np.arctan2(vy_FR, max(vx_FR, 0.5))
        alpha_RL = -np.arctan2(vy_RL, max(vx_RL, 0.5))
        alpha_RR = -np.arctan2(vy_RR, max(vx_RR, 0.5))

        # 3. Aerodynamic Forces
        v_total = np.hypot(vx_safe, vy)
        F_drag = 0.5 * p.rho * p.Cd_A * v_total**2
        F_down = 0.5 * p.rho * p.Cl_A * v_total**2

        # 4. 2D Load Transfer Dynamics (Longitudinal + Lateral)
        W_total = p.m * p.g + F_down
        
        # Static loads + Longitudinal weight shift (using a_act as acceleration proxy)
        Fz_f_static = W_total * (p.lr / p.L) - (p.m * a_act * p.h_cg) / p.L
        Fz_r_static = W_total * (p.lf / p.L) + (p.m * a_act * p.h_cg) / p.L

        # Lateral weight shift (ay proxy = vx * r)
        ay_proxy = vx_safe * r
        dFz_lat_f = (p.m * ay_proxy * p.h_cg * p.K_rot_f) / p.tf
        dFz_lat_r = (p.m * ay_proxy * p.h_cg * (1.0 - p.K_rot_f)) / p.tr

        # Combine distribution to find individual wheel normal forces
        Fz_FL = max(10.0, 0.5 * Fz_f_static - dFz_lat_f)
        Fz_FR = max(10.0, 0.5 * Fz_f_static + dFz_lat_f)
        Fz_RL = max(10.0, 0.5 * Fz_r_static - dFz_lat_r)
        Fz_RR = max(10.0, 0.5 * Fz_r_static + dFz_lat_r)

        # 5. Tire Load Sensitivity Adjustments
        mu_FL = p.mu * (1.0 - p.k_sens * Fz_FL)
        mu_FR = p.mu * (1.0 - p.k_sens * Fz_FR)
        mu_RL = p.mu * (1.0 - p.k_sens * Fz_RL)
        mu_RR = p.mu * (1.0 - p.k_sens * Fz_RR)

        # 6. Pure Lateral Forces (Uncoupled Potential)
        Fy_FL_pure = pacejka_lateral(alpha_FL, Fz_FL, mu_FL, p.B_f, p.C_f, p.D_f)
        Fy_FR_pure = pacejka_lateral(alpha_FR, Fz_FR, mu_FR, p.B_f, p.C_f, p.D_f)
        Fy_RL_pure = pacejka_lateral(alpha_RL, Fz_RL, mu_RL, p.B_r, p.C_r, p.D_r)
        Fy_RR_pure = pacejka_lateral(alpha_RR, Fz_RR, mu_RR, p.B_r, p.C_r, p.D_r)

        # 7. Rear-Wheel Drive Allocation & Combined Slip Friction Ellipses
        Fx_req_total = p.m * a_act
        Fx_RL_req = 0.5 * Fx_req_total
        Fx_RR_req = 0.5 * Fx_req_total

        Fmax_FL = mu_FL * Fz_FL
        Fmax_FR = mu_FR * Fz_FR
        Fmax_RL = mu_RL * Fz_RL
        Fmax_RR = mu_RR * Fz_RR

        # Front wheels are free-rolling longitudinally
        Fx_FL, Fx_FR = 0.0, 0.0
        Fy_FL, Fy_FR = Fy_FL_pure, Fy_FR_pure

        # Rear wheels traction/braking clipping
        Fx_RL = np.clip(Fx_RL_req, -Fmax_RL, Fmax_RL)
        Fx_RR = np.clip(Fx_RR_req, -Fmax_RR, Fmax_RR)

        # Combined slip interaction via friction ellipse coupling
        Fy_RL = Fy_RL_pure * np.sqrt(max(0.0, 1.0 - (Fx_RL / max(Fmax_RL, 1.0))**2))
        Fy_RR = Fy_RR_pure * np.sqrt(max(0.0, 1.0 - (Fx_RR / max(Fmax_RR, 1.0))**2))

        # 8. Coordinate Transformation to Vehicle Body Frame
        Fx_FL_b = Fx_FL * np.cos(delta_act) - Fy_FL * np.sin(delta_act)
        Fy_FL_b = Fx_FL * np.sin(delta_act) + Fy_FL * np.cos(delta_act)

        Fx_FR_b = Fx_FR * np.cos(delta_act) - Fy_FR * np.sin(delta_act)
        Fy_FR_b = Fx_FR * np.sin(delta_act) + Fy_FR * np.cos(delta_act)

        Fx_RL_b, Fy_RL_b = Fx_RL, Fy_RL
        Fx_RR_b, Fy_RR_b = Fx_RR, Fy_RR

        # 9. Total Forces and Yaw Moment
        F_roll = p.Crr * np.sign(vx) if abs(vx) > 1e-3 else 0.0

        Fx_total = Fx_FL_b + Fx_FR_b + Fx_RL_b + Fx_RR_b - F_drag - F_roll
        Fy_total = Fy_FL_b + Fy_FR_b + Fy_RL_b + Fy_RR_b

        # Expanded yaw moment incorporating asymmetric track-width thrust and drag forces
        M_z = (p.lf * (Fy_FL_b + Fy_FR_b) - p.lr * (Fy_RL_b + Fy_RR_b) +
               (p.tf / 2.0) * (Fx_FR_b - Fx_FL_b) +
               (p.tr / 2.0) * (Fx_RR_b - Fx_RL_b))

        # 10. Rigid Body Kinematics Equations of Motion
        ax = Fx_total / p.m + vy * r
        ay = Fy_total / p.m - vx_safe * r
        r_dot = M_z / p.Iz

        # Explicit sub-stepped integration steps
        vx_new = vx + ax * h
        vy_new = vy + ay * h
        r_new = r + r_dot * h

        X_new = X + (vx * np.cos(psi) - vy * np.sin(psi)) * h
        Y_new = Y + (vx * np.sin(psi) + vy * np.cos(psi)) * h
        psi_new = psi + r * h

        delta_new = delta_act + ddelta * h
        a_new = a_act + da * h

        s = np.array([X_new, Y_new, psi_new, vx_new, vy_new, r_new, delta_new, a_new])

    return s


def init_plant_state(X0, Y0, psi0, vx0=10.0):
    """Build an initial nonlinear-plant state vector."""
    return np.array([X0, Y0, psi0, vx0, 0.0, 0.0, 0.0, 0.0])


def plant_to_tracking_error(state, ref_x, ref_y, ref_psi):
    """Convert global plant state into geometric tracking errors for the MPC."""
    X, Y, psi, vx, vy, r, delta_act, a_act = state

    dx = X - ref_x
    dy = Y - ref_y
    e_y = dy * np.cos(ref_psi) - dx * np.sin(ref_psi)
    e_psi = np.arctan2(np.sin(psi - ref_psi), np.cos(psi - ref_psi))

    e_y_dot = vx * np.sin(e_psi) + vy * np.cos(e_psi)
    e_psi_dot = r

    return e_y, e_y_dot, e_psi, e_psi_dot, delta_act, a_act, vx