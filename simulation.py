import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from model import get_8state_discrete_model

# Simulation Constants
dt = 0.05        
sim_steps = 120  
v_ref = 15.0     # Baseline speed profile reference (m/s)

# Initial conditions 
# [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
x = np.array([-1.5, 0.0, 0.1, 0.0, -5.0, 0.0, 0.0, 0.0]) # Starting 5 m/s too slow

# Simulated control input targets [delta_cmd, a_cmd]
u_sim = np.zeros((sim_steps, 2))
u_sim[0:40, 0] = 0.01    
u_sim[0:40, 1] = 3.0     # High acceleration command -> will distinctly stretch out plot dots
u_sim[40:90, 0] = -0.01 
u_sim[40:90, 1] = 5  # Deceleration phase

# Track history matrices
tracked_X = np.zeros(sim_steps)
tracked_Y = np.zeros(sim_steps)
state_history = np.zeros((sim_steps, 8))
actual_speed_history = np.zeros(sim_steps)

# Running distance tracker along the path
s_path = 0.0

# 4. Run Physics Simulation Loop
for k in range(sim_steps):
    current_v = v_ref + x[4] # true longitudinal speed
    actual_speed_history[k] = current_v
    Ad, Bd = get_8state_discrete_model(current_v, dt)
    
    # DYNAMIC MAPPING: Global X position depends directly on integrated distance traversed
    ref_X_k = s_path
    ref_Y_k = 0.0
    ref_psi_k = 0.0
    
    tracked_X[k] = ref_X_k - x[0] * np.sin(ref_psi_k)
    tracked_Y[k] = ref_Y_k + x[0] * np.cos(ref_psi_k)
    state_history[k, :] = x
    
    # Physics evolution
    x = Ad @ x + Bd @ u_sim[k]
    
    # Update integrated path distance using actual velocity
    s_path += current_v * dt

# 5. Maximized Layout Setup using GridSpec
fig = plt.figure(figsize=(14, 7))
gs = fig.add_gridspec(2, 2, width_ratios=[4, 1], height_ratios=[12, 1],
                      left=0.06, right=0.94, top=0.92, bottom=0.05, wspace=0.15, hspace=0.2)

# Main Tracking Plot Area
ax_map = fig.add_subplot(gs[0, 0])
# Generate a static path long enough for full extension
ax_map.plot([0, s_path + 10], [0, 0], 'r--', label='Reference Path Line', linewidth=2)
ax_map.plot(tracked_X, tracked_Y, 'b-', label='Vehicle Track', alpha=0.3)
# Scatter dots over the path to visually inspect density changes caused by speed differences
ax_map.scatter(tracked_X, tracked_Y, c=actual_speed_history, cmap='viridis', s=15, label='Velocity Samples')

ax_map.set_xlim(-5, s_path + 5)
ax_map.set_ylim(-50, 50)
ax_map.set_aspect('equal')
ax_map.grid(True)
ax_map.set_title('Vehicle Trajectory Projection (Dynamic Velocity Mapping)', fontsize=12, fontweight='bold')
ax_map.set_xlabel('Global X Position (m)')
ax_map.set_ylabel('Global Y Position (m)')

vehicle_marker, = ax_map.plot([], [], 'g-', linewidth=2.5, label='Vehicle')
ax_map.legend(loc='lower right')

# Dedicated Telemetry Panel (Right Side)
ax_info = fig.add_subplot(gs[0, 1])
ax_info.axis('off')
telemetry_text = ax_info.text(0.0, 0.85, '', family='monospace', fontsize=11, verticalalignment='top',
                              bbox=dict(facecolor='#f8f9fa', alpha=1.0, edgecolor='#ccced1', boxstyle='round,pad=0.8'))

# Control Widgets Layout (Bottom Row)
ax_slider = fig.add_subplot(gs[1, 0])
time_slider = Slider(ax_slider, 'Time Step', 0, sim_steps - 1, valinit=0, valfmt='%0.0f')

ax_button = fig.add_subplot(gs[1, 1])
play_button = Button(ax_button, 'Play/Pause')

is_playing = True

def get_car_triangle(x_pos, y_pos, heading, size=2.2):
    corners = np.array([
        [size, 0],          
        [-size/2, size/2],  
        [-size/2, -size/2], 
        [size, 0]           
    ])
    rot_matrix = np.array([
        [np.cos(heading), -np.sin(heading)],
        [np.sin(heading),  np.cos(heading)]
    ])
    rotated = (rot_matrix @ corners.T).T
    return rotated[:, 0] + x_pos, rotated[:, 1] + y_pos

def update_plot(frame):
    frame = int(frame)
    x_c, y_c = tracked_X[frame], tracked_Y[frame]
    heading = state_history[frame, 2] # ref_psi is 0
    
    car_x, car_y = get_car_triangle(x_c, y_c, heading)
    vehicle_marker.set_data(car_x, car_y)
    
    s = state_history[frame]
    text = (
        f"       TELEMETRY\n"
        f"=======================\n"
        f"Step      : {frame:02d}\n"
        f"Time      : {frame*dt:.2f} s\n"
        f"-----------------------\n"
        f"True Speed: {actual_speed_history[frame]:6.2f} m/s\n"
        f"e_y       : {s[0]:6.2f} m\n"
        f"e_y_dot   : {s[1]:6.2f} m/s\n"
        f"e_psi     : {s[2]:6.2f} rad\n"
        f"e_psi_dot : {s[3]:6.2f} rad/s\n"
        f"e_v       : {s[4]:6.2f} m/s\n"
        f"e_a       : {s[5]:6.2f} m/s²\n"
        f"-----------------------\n"
        f"Act Steer : {np.degrees(s[6]):6.1f} deg\n"
        f"Act Accel : {s[7]:6.2f} m/s²"
    )
    telemetry_text.set_text(text)
    return vehicle_marker, telemetry_text

def on_slider_change(val):
    if not is_playing:
        update_plot(val)
    fig.canvas.draw_idle()

time_slider.on_changed(on_slider_change)

def toggle_playback(event):
    global is_playing
    if is_playing:
        is_playing = False
        anim.pause()
    else:
        is_playing = True
        anim.resume()

play_button.on_clicked(toggle_playback)

def anim_frame_generator():
    curr = int(time_slider.val)
    while True:
        if is_playing:
            curr = (curr + 1) % sim_steps
            time_slider.set_val(curr)
        yield curr

anim = FuncAnimation(fig, update_plot, frames=anim_frame_generator, interval=50, save_count=sim_steps)
plt.show()