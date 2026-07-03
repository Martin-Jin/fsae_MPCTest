import numpy as np
from scipy.linalg import expm
import vehicle_physics as vp

# ==========================================
# 1. VEHICLE DYNAMICS MODEL
# ==========================================
def get_8state_discrete_model(v_x, dt):
    """
    Computes 8-state discrete-time tracking matrices.
    Forces consistent sparsity pattern (dense) to avoid OSQP reallocation crashes.
    """
    vehicle_parameters = vp.VehicleParams()
    lf, lr = vehicle_parameters.lf, vehicle_parameters.lr
    L = lf + lr
    m, Iz = vehicle_parameters.m, vehicle_parameters.Iz
    Cf, Cr = vehicle_parameters.Cf, vehicle_parameters.Cr
    tau_delta, tau_a = vehicle_parameters.tau_delta, vehicle_parameters.tau_a

    # Prevent absolute zero
    v_x_safe = max(0.01, v_x)

    # FORCE DENSE: Initialize all entries with epsilon to ensure
    # the solver always sees a fully dense matrix (constant sparsity).
    A_kin = np.ones((8, 8)) * 1e-12
    A_dyn = np.ones((8, 8)) * 1e-12
    B = np.ones((8, 2)) * 1e-12

    # --- 1. KINEMATIC MODEL ---
    A_kin[0, 2] = v_x_safe
    A_kin[2, 6] = v_x_safe / L
    A_kin[4, 5] = 1.0
    A_kin[5, 7] = 1.0
    A_kin[6, 6] = -1.0 / tau_delta
    A_kin[7, 7] = -1.0 / tau_a

    # --- 2. DYNAMIC MODEL ---
    A_dyn[0, 1] = 1.0
    A_dyn[1, 1] = -(2*Cf + 2*Cr) / (m * v_x_safe)
    A_dyn[1, 2] = (2*Cf + 2*Cr) / m
    A_dyn[1, 3] = (-2*Cf*lf + 2*Cr*lr) / (m * v_x_safe)
    A_dyn[1, 6] = (2*Cf) / m
    A_dyn[2, 3] = 1.0
    A_dyn[3, 1] = (-2*Cf*lf + 2*Cr*lr) / (Iz * v_x_safe)
    A_dyn[3, 2] = (2*Cf*lf - 2*Cr*lr) / Iz
    A_dyn[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x_safe)
    A_dyn[3, 6] = (2*Cf * lf) / Iz
    A_dyn[4, 5] = 1.0
    A_dyn[5, 7] = 1.0
    A_dyn[6, 6] = -1.0 / tau_delta
    A_dyn[7, 7] = -1.0 / tau_a
    
    B[6, 0] = 1.0 / tau_delta
    B[7, 1] = 1.0 / tau_a

    # --- 3. BLENDING ---
    alpha = np.clip((v_x - 1.0) / (2.5 - 1.0), 0.0, 1.0)
    A_c = (1.0 - alpha) * A_kin + alpha * A_dyn
    B_c = B # Structure is constant

    # --- 4. ZOH DISCRETIZATION ---
    nx, nu = 8, 2
    M = np.zeros((nx + nu, nx + nu))
    M[:nx, :nx] = A_c
    M[:nx, nx:] = B_c
    
    Md = expm(M * dt)
    return Md[:nx, :nx], Md[:nx, nx:]