import numpy as np

# ==========================================
# 1. VEHICLE DYNAMICS MODEL
# ==========================================
def get_8state_discrete_model(v_x, dt):
    """
    Computes 8-state discrete-time tracking matrices incorporating actuator lag.
    States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
    """
    v_x = max(0.5, v_x) # Prevent division by zero
    
    m, Iz, lf, lr = 1600.0, 2500.0, 1.2, 1.4
    Cf, Cr = 80000.0, 85000.0
    tau_delta, tau_a = 0.15, 0.20     
    
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
    
    Ad = np.eye(8) + A_c * dt
    Bd = B_c * dt
    return Ad, Bd