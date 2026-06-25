from model import get_8state_discrete_model
from optimiser import solve_mpc
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 3. CLOSED-LOOP SIMULATION
# ==========================================
# Simulation Parameters
dt = 0.05        
sim_steps = 150  
v_ref = 15.0     

# MPC Horizon and Constraints
N_horizon = 20
STEER_MAX = np.radians(25.0)
ACCEL_MAX = 2.5
ACCEL_MIN = -4.0
u_bounds_min = np.array([-STEER_MAX, ACCEL_MIN])
u_bounds_max = np.array([STEER_MAX, ACCEL_MAX])

# Cost Matrices (Tuning Weights)
# States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
Q = np.diag([150.0, 10.0, 150.0, 10.0, 80.0, 5.0, 1.0, 1.0]) 
# Inputs: [delta_cmd, a_cmd]
R = np.diag([20.0, 15.0]) 

# Initialization (Start with severe lateral and velocity errors)
x_current = np.array([-3.5, 0.0, 0.2, 0.0, -5.0, 0.0, 0.0, 0.0])
s_path = 0.0

# Data recording arrays
tracked_X = np.zeros(sim_steps)
tracked_Y = np.zeros(sim_steps)
state_history = np.zeros((sim_steps, 8))
u_history = np.zeros((sim_steps, 2))

print("Starting Closed-Loop MPC Simulation...")

for k in range(sim_steps):
    current_v = v_ref + x_current[4]
    
    # 1. Update plant model based on current velocity
    Ad, Bd = get_8state_discrete_model(current_v, dt)
    
    # 2. Optimize future trajectory and extract control input
    u_opt = solve_mpc(x_current, Ad, Bd, N_horizon, Q, R, u_bounds_min, u_bounds_max)
    
    # Record mapping variables for plotting
    tracked_X[k] = s_path - x_current[0] * np.sin(0)
    tracked_Y[k] = 0.0 + x_current[0] * np.cos(0)
    state_history[k, :] = x_current
    u_history[k, :] = u_opt
    
    # 3. Apply control to the dynamic system (x_k+1 = Ad*x_k + Bd*u_k)
    x_current = Ad @ x_current + Bd @ u_opt
    s_path += current_v * dt

print("Simulation Complete. Generating plots.")

# ==========================================
# 4. RESULTS VISUALIZATION
# ==========================================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
time_axis = np.arange(sim_steps) * dt

# Plot 1: X-Y Trajectory Mapping
ax1.plot([0, s_path], [0, 0], 'r--', label='Reference Path', linewidth=2)
ax1.plot(tracked_X, tracked_Y, 'b-', label='MPC Vehicle Track', linewidth=2)
ax1.set_title('Top-Down Vehicle Trajectory')
ax1.set_ylabel('Y Position (m)')
ax1.axis('equal')
ax1.grid(True)
ax1.legend()

# Plot 2: State Tracking Errors
ax2.plot(time_axis, state_history[:, 0], label='Lateral Error (e_y) [m]', color='blue')
ax2.plot(time_axis, state_history[:, 2], label='Heading Error (e_psi) [rad]', color='purple')
ax2.plot(time_axis, state_history[:, 4], label='Velocity Error (e_v) [m/s]', color='orange')
ax2.set_title('Tracking Errors Over Time (Minimizing towards Zero)')
ax2.set_ylabel('Error Magnitude')
ax2.grid(True)
ax2.legend()

# Plot 3: Actuator Commands vs Limits
ax3.plot(time_axis, np.degrees(u_history[:, 0]), label='Steering Command [deg]', color='green')
ax3.plot(time_axis, u_history[:, 1], label='Accel Command [m/s²]', color='red')
ax3.axhline(np.degrees(STEER_MAX), color='green', linestyle=':', alpha=0.5)
ax3.axhline(-np.degrees(STEER_MAX), color='green', linestyle=':', alpha=0.5)
ax3.axhline(ACCEL_MAX, color='red', linestyle=':', alpha=0.5)
ax3.axhline(ACCEL_MIN, color='red', linestyle=':', alpha=0.5)
ax3.set_title('Optimal Control Effort (Actuator Commands)')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Command Input')
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()