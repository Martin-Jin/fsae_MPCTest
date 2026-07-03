import numpy as np
from scipy.linalg import expm
import vehicle_physics as vp

# ==========================================
# 1. VEHICLE DYNAMICS MODEL
# ==========================================
def get_8state_discrete_model(v_x, dt):
    """
    Computes 8-state discrete-time tracking matrices incorporating actuator lag.
    States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]

    Discretization: exact zero-order-hold (ZOH) via the matrix exponential,
    NOT forward-Euler (Ad = I + A_c*dt). This matters a lot here: several
    entries of A_c scale as 1/v_x (e.g. A_c[1,1] = -(2*Cf+2*Cr)/(m*v_x)), so
    at low speed the continuous-time dynamics get very fast. Forward-Euler
    is only stable when |eigenvalue(A_c)| * dt is small; at v_x=0.5 m/s
    that product is large enough that the discretized Ad matrix becomes
    open-loop UNSTABLE (eigenvalue magnitude ~34 instead of <=1), which in
    turn meant the MPC's QP would go infeasible from even a small heading
    error -- the predicted state diverges to astronomical values within the
    horizon and no slack value can satisfy the lateral-error constraint.
    ZOH discretization (via scipy.linalg.expm on the augmented [A_c, B_c; 0,
    0] block matrix) is unconditionally stable for any stable continuous-
    time system regardless of dt, which is what an MPC's internal model
    needs, especially now that the simulator runs at low speed routinely
    (curvature-based speed profiling slows the car for corners).
    """
    v_x = max(0.5, v_x) # Prevent division by zero
    

    vehicle_parameters = vp.VehicleParams()

    lf  = vehicle_parameters.lf    # CoM -> front axle (m)  (wheelbase 1.5 m)
    lr  = vehicle_parameters.lr    # CoM -> rear  axle (m)
    m   = vehicle_parameters.m     # Vehicle mass (kg)  [FSDS spec]
    Iz  = vehicle_parameters.Iz    # Yaw inertia (kg m^2)  [255 kg * 0.43^2 ≈ 110]
    Cf  = vehicle_parameters.Cf    # Front cornering stiffness (N/rad)  [FS slick estimate]
    Cr  = vehicle_parameters.Cr    # Rear  cornering stiffness (N/rad)  [slightly stiffer rear]
    
    tau_delta, tau_a = vehicle_parameters.tau_delta, vehicle_parameters.tau_a  # Actuator lag (s)
    
    A_c = np.zeros((8, 8))
    A_c[0, 1] = 1.0
    A_c[1, 1] = -(2*Cf + 2*Cr) / (m * v_x)
    A_c[1, 2] = (2*Cf + 2*Cr) / m
    A_c[1, 3] = (-2*Cf*lf + 2*Cr*lr) / (m * v_x)
    A_c[1, 6] = (2*Cf) / m                    
    A_c[2, 3] = 1.0
    A_c[3, 1] = (-2*Cf*lf + 2*Cr*lr) / (Iz * v_x)
    A_c[3, 2] = (2*Cf*lf - 2*Cr*lr) / Iz
    A_c[3, 3] = -(2*Cf*lf**2 + 2*Cr*lr**2) / (Iz * v_x)
    A_c[3, 6] = (2*Cf * lf) / Iz              
    A_c[4, 5] = 1.0                           
    A_c[5, 7] = 1.0                           
    A_c[6, 6] = -1.0 / tau_delta
    A_c[7, 7] = -1.0 / tau_a
    
    B_c = np.zeros((8, 2))
    B_c[6, 0] = 1.0 / tau_delta                
    B_c[7, 1] = 1.0 / tau_a                    

    # Exact ZOH discretization via the augmented-system matrix exponential:
    #   expm([[A_c, B_c], [0, 0]] * dt) = [[Ad, Bd], [0, I]]
    nx, nu = 8, 2
    M = np.zeros((nx + nu, nx + nu))
    M[:nx, :nx] = A_c
    M[:nx, nx:] = B_c
    Md = expm(M * dt)
    Ad = Md[:nx, :nx]
    Bd = Md[:nx, nx:]
    return Ad, Bd