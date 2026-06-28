from model import get_8state_discrete_model
from optimiser import solve_mpc
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

# ==========================================
# 3. REAL-TIME SIMULATION SETUP
# ==========================================
dt = 0.05        
v_ref = 20    
N_horizon = 40  # Lowered slightly for real-time framerate stability

u_bounds_min = np.array([-np.radians(25), -4.0])
u_bounds_max = np.array([np.radians(25), 5.0])

Q = np.diag([300.0, 1.0, 100.0, 400.0, 1.0, 1.0, 1.0, 1.0]) 
R = np.diag([100.0, 10.0]) 

# Global state tracking variables
# Starts with car centered on Y=0, but going a bit slow
x_current = np.array([0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0])
s_path = 0.0       # Global X position
Y_car = 0.0        # Global Y position
ref_Y = 0.0        # Current target lane

# Trail history for drawing
history_len = 100
track_X = np.full(history_len, np.nan)
track_Y = np.full(history_len, np.nan)

# ==========================================
# 4. GUI & ANIMATION LAYOUT
# ==========================================
fig = plt.figure(figsize=(14, 7))
gs = fig.add_gridspec(2, 2, width_ratios=[4, 1], height_ratios=[12, 1],
                      left=0.06, right=0.94, top=0.92, bottom=0.05, wspace=0.15, hspace=0.2)

# Main Tracking Map
ax_map = fig.add_subplot(gs[0, 0])
ref_line, = ax_map.plot([], [], 'r--', label='Target Reference Path', linewidth=2)
trail_line, = ax_map.plot([], [], 'b-', label='Vehicle Trail', alpha=0.5, linewidth=2)
vehicle_marker, = ax_map.plot([], [], 'g-', linewidth=2.5, label='Vehicle')

ax_map.set_aspect('equal')
ax_map.grid(True)
ax_map.set_title('Real-Time Live MPC Trajectory (Scrolling Window)', fontweight='bold')
ax_map.set_xlabel('Global X Position (m)')
ax_map.set_ylabel('Global Y Position (m)')
ax_map.legend(loc='upper right')

# Telemetry Panel
ax_info = fig.add_subplot(gs[0, 1])
ax_info.axis('off')
telemetry_text = ax_info.text(0.0, 0.95, '', family='monospace', fontsize=11, verticalalignment='top',
                              bbox=dict(facecolor='#f8f9fa', edgecolor='#ccced1', boxstyle='round,pad=0.8'))

# Interactive Target Lane Slider
ax_slider = fig.add_subplot(gs[1, 0])
lane_slider = Slider(ax_slider, 'Target Lane (Y)', -8.0, 8.0, valinit=0.0, valfmt='%0.1f m', color='orange')

def get_car_triangle(x_pos, y_pos, heading, size=2.5):
    corners = np.array([[size, 0], [-size/2, size/2], [-size/2, -size/2], [size, 0]])
    rot_matrix = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
    rotated = (rot_matrix @ corners.T).T
    return rotated[:, 0] + x_pos, rotated[:, 1] + y_pos

# ==========================================
# 5. LIVE CONTROL LOOP
# ==========================================
def live_update(frame):
    global x_current, s_path, Y_car, ref_Y, track_X, track_Y
    
    # 1. Check for interactive target changes
    target_Y = lane_slider.val
    if target_Y != ref_Y:
        # If the target line moves, the physical car stays in place, 
        # so the relative lateral error (e_y) instantly spikes!
        x_current[0] = (Y_car - target_Y) / np.cos(x_current[2])
        ref_Y = target_Y
        
    current_v = v_ref + x_current[4]
    
    # 2. Formulate dynamics and Solve MPC live
    Ad, Bd = get_8state_discrete_model(current_v, dt)
    u_opt = solve_mpc(x_current, Ad, Bd, N_horizon, Q, R, u_bounds_min, u_bounds_max)
    
    # 3. Apply physics step
    x_current = Ad @ x_current + Bd @ u_opt
    
    # 4. Map back to global coordinates
    s_path += current_v * dt
    Y_car = ref_Y + x_current[0] * np.cos(x_current[2])
    
    # Update trailing arrays
    track_X = np.roll(track_X, -1)
    track_Y = np.roll(track_Y, -1)
    track_X[-1] = s_path
    track_Y[-1] = Y_car
    
    # 5. Visual Updates
    # Draw reference line across the current window
    ref_line.set_data([s_path - 20, s_path + 40], [ref_Y, ref_Y])
    trail_line.set_data(track_X, track_Y)
    
    car_x, car_y = get_car_triangle(s_path, Y_car, x_current[2])
    vehicle_marker.set_data(car_x, car_y)
    
    # SCROLLING WINDOW EFFECT
    ax_map.set_xlim(s_path - 15, s_path + 35)
    ax_map.set_ylim(-12, 12)
    
    # Update Telemetry
    text = (
        f"       LIVE TELEMETRY\n"
        f"=======================\n"
        f"Speed     : {current_v:6.2f} m/s\n"
        f"Target Y  : {ref_Y:6.2f} m\n"
        f"-----------------------\n"
        f"Lat Error : {x_current[0]:6.2f} m\n"
        f"Yaw Error : {np.degrees(x_current[2]):6.2f} deg\n"
        f"-----------------------\n"
        f"Steer Cmd : {np.degrees(u_opt[0]):6.1f} deg\n"
        f"Accel Cmd : {u_opt[1]:6.2f} m/s²\n"
        f"Act Steer : {np.degrees(x_current[6]):6.1f} deg\n"
        f"Act Accel : {x_current[7]:6.2f} m/s²"
    )
    telemetry_text.set_text(text)
    
    return ref_line, trail_line, vehicle_marker, telemetry_text

# Run animation continuously
anim = FuncAnimation(fig, live_update, interval=50, blit=False, cache_frame_data=False)
plt.show()