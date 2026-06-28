"""
Micro-Loop Immune Path MPC Simulator (v5)
File Name: simulation.py
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from scipy.interpolate import CubicSpline
from model import get_8state_discrete_model
from optimiser import solve_mpc

# ==========================================
# SETUP AND CONFIGURATION
# ==========================================
dt = 0.05
N_horizon = 25  
v_ref = 10.0    

u_bounds_min = np.array([-np.radians(35), -5.0]) 
u_bounds_max = np.array([np.radians(35), 5.0])

# Highly protective lateral alignment weights
# States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta, a]
Q = np.diag([3500.0, 5.0, 2000.0, 250.0, 10.0, 0.1, 1.0, 1.0]) 
R = np.diag([5.0, 2.0])

is_drawing = False
is_simulated = False
flip_heading_180 = False  
drawn_points = []
path_X, path_Y, path_Psi = [], [], []
sim_history = {}

# ==========================================
# INTERACTIVE GUI LAYOUT
# ==========================================
fig = plt.figure(figsize=(15, 8.5))
gs = fig.add_gridspec(4, 2, width_ratios=[3.8, 1.2], height_ratios=[12, 1, 1, 1],
                      left=0.06, right=0.94, top=0.94, bottom=0.06, wspace=0.15, hspace=0.45)

ax_map = fig.add_subplot(gs[0, 0])
ax_info = fig.add_subplot(gs[0, 1])
ax_info.axis('off')

path_line, = ax_map.plot([], [], 'r--', label='Drawn Target Path', linewidth=2)
trail_line, = ax_map.plot([], [], 'b-', label='Actual Vehicle Trail', alpha=0.6)
pred_line, = ax_map.plot([], [], 'c-o', label='MPC Horizon Prediction', markersize=3, alpha=0.8)
vehicle_marker, = ax_map.plot([], [], 'g-', linewidth=2.5, label='Vehicle')

ax_map.set_xlim(0, 100)
ax_map.set_ylim(0, 60)
ax_map.set_aspect('equal')
ax_map.grid(True)
ax_map.set_title("Robust High-Speed Path MPC Sandbox", fontweight='bold')
ax_map.legend(loc='upper right')

telemetry_text = ax_info.text(0.0, 0.95, '', family='monospace', fontsize=10.5, verticalalignment='top',
                              bbox=dict(facecolor='#f8f9fa', edgecolor='#ccced1', boxstyle='round,pad=0.7'))

ax_ey0 = fig.add_subplot(gs[1, 0])
ax_epsi0 = fig.add_subplot(gs[2, 0])
ax_scrub = fig.add_subplot(gs[3, 0])

slider_ey0 = Slider(ax_ey0, 'Initial Lat Error', -4.0, 4.0, valinit=0.0, valfmt='%0.1f m', color='orange')
slider_epsi0 = Slider(ax_epsi0, 'Initial Yaw Error', -30.0, 30.0, valinit=0.0, valfmt='%0.1f°', color='orange')
slider_scrub = Slider(ax_scrub, 'Timeline Scrub', 0, 1, valinit=0, valfmt='%d', color='teal')
ax_scrub.set_visible(False)

ax_btn_start = fig.add_subplot(gs[1, 1])
ax_btn_flip = fig.add_subplot(gs[2, 1])
ax_btn_reset = fig.add_subplot(gs[3, 1])

btn_start = Button(ax_btn_start, 'Start Sim', color='lightgreen', hovercolor='limegreen')
btn_flip = Button(ax_btn_flip, 'Flip Heading (180°)', color='lightgreen', hovercolor='khaki')
btn_reset = Button(ax_btn_reset, 'Reset Environment', color='tomato', hovercolor='crimson')

# ==========================================
# HELPER MATHEMATICS & WRAPPING FIXES
# ==========================================
def get_car_triangle(x, y, heading, size=2.2):
    corners = np.array([[size, 0], [-size/1.5, size/1.5], [-size/1.5, -size/1.5], [size, 0]])
    rot = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y

def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

def find_closest_reference_bounded(x_g, y_g, last_idx, window=40):
    # Escape loop trap: Force a wider forward verification if stuck at line-start
    if last_idx <= 5:
        start_search = 0
        end_search = min(len(path_X), 100)
    else:
        start_search = max(0, last_idx - 5)
        end_search = min(len(path_X), last_idx + window)
    
    distances = np.hypot(path_X[start_search:end_search] - x_g, path_Y[start_search:end_search] - y_g)
    local_idx = np.argmin(distances)
    global_idx = start_search + local_idx
    
    return global_idx, path_X[global_idx], path_Y[global_idx], path_Psi[global_idx]

# ==========================================
# INTERACTIVE DRAWING HANDLERS
# ==========================================
# ==========================================
# INTERACTIVE EVENT HANDLERS
# ==========================================
def reset_environment(event):
    global is_simulated, flip_heading_180, drawn_points, path_X, path_Y, path_Psi, sim_history
    is_simulated = False
    flip_heading_180 = False
    drawn_points = []
    path_X, path_Y, path_Psi = [], [], []
    sim_history = {}
    
    path_line.set_data([], [])
    trail_line.set_data([], [])
    pred_line.set_data([], [])
    vehicle_marker.set_data([], [])
    telemetry_text.set_text('')
    
    ax_ey0.set_visible(True)
    ax_epsi0.set_visible(True)
    ax_scrub.set_visible(False)
    
    btn_start.set_active(True)
    btn_flip.set_active(True)
    slider_ey0.set_val(0.0)
    slider_epsi0.set_val(0.0)
    ax_map.set_title("Environment Reset. Draw a new path.", fontweight='bold', color='black')
    fig.canvas.draw_idle()


def on_press(event):
    global is_drawing, drawn_points, is_simulated
    if event.inaxes != ax_map or is_simulated: return
    is_drawing = True
    drawn_points = [[event.xdata, event.ydata]]

def on_motion(event):
    global drawn_points
    if not is_drawing or event.inaxes != ax_map: return
    drawn_points.append([event.xdata, event.ydata])
    pts = np.array(drawn_points)
    path_line.set_data(pts[:, 0], pts[:, 1])
    fig.canvas.draw_idle()

def on_release(event):
    global is_drawing, path_X, path_Y, path_Psi, flip_heading_180
    if not is_drawing: return
    is_drawing = False
    if len(drawn_points) < 6: return
    
    # Filter points to remove initial duplicate/jitter data
    raw_pts = np.array(drawn_points)
    filtered_pts = [raw_pts[0]]
    for p in raw_pts[1:]:
        if np.linalg.norm(p - filtered_pts[-1]) > 0.5:
            filtered_pts.append(p)
            
    if len(filtered_pts) < 4: 
        filtered_pts = list(raw_pts) # Fallback if dragged ultra-slow
        
    pts = np.array(filtered_pts)
    t = np.linspace(0, 1, len(pts))
    cs_x = CubicSpline(t, pts[:, 0])
    cs_y = CubicSpline(t, pts[:, 1])
    
    t_fine = np.linspace(0, 1, max(600, len(pts)*6))
    path_X = cs_x(t_fine)
    path_Y = cs_y(t_fine)
    
    dx = cs_x.derivative()(t_fine)
    dy = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)
    
    # Calculate initial chord heading vector
    forward_heading = np.arctan2(path_Y[5] - path_Y[0], path_X[5] - path_X[0])
    path_Psi[0] = forward_heading
    
    flip_heading_180 = False 
    path_line.set_data(path_X, path_Y)
    
    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], forward_heading)
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()

def toggle_heading_flip(event):
    global flip_heading_180
    if len(path_X) == 0: return
    flip_heading_180 = not flip_heading_180
    
    base_heading = path_Psi[0] + (np.pi if flip_heading_180 else 0.0)
    current_psi = normalize_angle(base_heading + np.radians(slider_epsi0.val))
    
    X_g = path_X[0] - slider_ey0.val * np.sin(base_heading)
    Y_g = path_Y[0] + slider_ey0.val * np.cos(base_heading)
    
    car_x, car_y = get_car_triangle(X_g, Y_g, current_psi)
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()

fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('button_release_event', on_release)
btn_flip.on_clicked(toggle_heading_flip)
btn_flip.on_clicked(toggle_heading_flip)
btn_reset.on_clicked(reset_environment)

# ==========================================
# SIMULATION ENGINE
# ==========================================
def run_simulation(event):
    global is_simulated, sim_history
    if len(path_X) == 0:
        ax_map.set_title("ERROR: Draw a path first!", color='red', fontweight='bold')
        fig.canvas.draw_idle()
        return
        
    is_simulated = True
    btn_start.set_active(False)
    btn_flip.set_active(False)
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)
    
    base_path_heading = path_Psi[0] + (np.pi if flip_heading_180 else 0.0)
    
    X_g = path_X[0] - slider_ey0.val * np.sin(base_path_heading)
    Y_g = path_Y[0] + slider_ey0.val * np.cos(base_path_heading)
    psi_g = normalize_angle(base_path_heading + np.radians(slider_epsi0.val))
    
    x_current = np.array([slider_ey0.val, 0.0, np.radians(slider_epsi0.val), 0.0, 0.0, 0.0, 0.0, 0.0])
    
    history = {'X': [], 'Y': [], 'psi': [], 'v': [], 'u_steer': [], 'u_accel': [], 
               'e_y': [], 'e_psi': [], 'pred_X': [], 'pred_Y': []}
    
    idx = 0
    max_steps = 400
    
    for step in range(max_steps):
        current_v = v_ref + x_current[4]
        
        history['X'].append(X_g)
        history['Y'].append(Y_g)
        history['psi'].append(psi_g)
        history['v'].append(current_v)
        history['e_y'].append(x_current[0])
        history['e_psi'].append(x_current[2])
        
        Ad, Bd = get_8state_discrete_model(current_v, dt)
        u_opt = solve_mpc(x_current, Ad, Bd, N_horizon, Q, R, u_bounds_min, u_bounds_max)
        
        history['u_steer'].append(u_opt[0])
        history['u_accel'].append(u_opt[1])
        
        px, py = [], []
        X_p, Y_p, psi_p = X_g, Y_g, psi_g
        x_p_tmp = x_current.copy()
        
        for k in range(N_horizon):
            v_p = v_ref + x_p_tmp[4]
            X_p += v_p * np.cos(psi_p) * dt
            Y_p += v_p * np.sin(psi_p) * dt
            psi_p += x_p_tmp[3] * dt 
            px.append(X_p)
            py.append(Y_p)
            x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt
            
        history['pred_X'].append(px)
        history['pred_Y'].append(py)
        
        x_current = Ad @ x_current + Bd @ u_opt
        
        X_g += current_v * np.cos(psi_g) * dt
        Y_g += current_v * np.sin(psi_g) * dt
        psi_g = normalize_angle(psi_g + (current_v * (np.tan(x_current[6]) / 2.6)) * dt)
        
        idx, rx, ry, rpsi = find_closest_reference_bounded(X_g, Y_g, idx, window=40)
        
        if flip_heading_180:
            rpsi = normalize_angle(rpsi + np.pi)
            
        dx = X_g - rx
        dy = Y_g - ry
        x_current[0] = dy * np.cos(rpsi) - dx * np.sin(rpsi)
        x_current[2] = normalize_angle(psi_g - rpsi) 
        
        if idx >= len(path_X) - 2: break

    sim_history = history
    ax_map.set_title("Simulation Complete! Review tracking via Slider below.", fontweight='bold', color='darkgreen')
    
    ax_scrub.set_visible(True)
    slider_scrub.valmax = len(history['X']) - 1
    slider_scrub.ax.set_xlim(0, len(history['X']) - 1)
    slider_scrub.on_changed(update_scrub_frame)
    update_scrub_frame(0)

btn_start.on_clicked(run_simulation)

# ==========================================
# TIMELINE REVIEW SCRUBBING
# ==========================================
def update_scrub_frame(val):
    frame = int(val)
    h = sim_history
    
    trail_line.set_data(h['X'][:frame+1], h['Y'][:frame+1])
    pred_line.set_data(h['pred_X'][frame], h['pred_Y'][frame])
    
    car_x, car_y = get_car_triangle(h['X'][frame], h['Y'][frame], h['psi'][frame])
    vehicle_marker.set_data(car_x, car_y)
    
    text = (
        f"     HISTORIC FRAME: {frame:03d}\n"
        f"=======================\n"
        f"Speed     : {h['v'][frame]:6.2f} m/s\n"
        f"Pos X     : {h['X'][frame]:6.2f} m\n"
        f"Pos Y     : {h['Y'][frame]:6.2f} m\n"
        f"Heading   : {np.degrees(h['psi'][frame]):6.1f} deg\n"
        f"-----------------------\n"
        f"Lat Error : {h['e_y'][frame]:6.2f} m\n"
        f"Yaw Error : {np.degrees(h['e_psi'][frame]):6.2f} deg\n"
        f"-----------------------\n"
        f"Steer Cmd : {np.degrees(h['u_steer'][frame]):6.1f} deg\n"
        f"Accel Cmd : {h['u_accel'][frame]:6.2f} m/s²"
    )
    telemetry_text.set_text(text)
    fig.canvas.draw_idle()

plt.show()