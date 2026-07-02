"""
Micro-Loop Immune Path MPC Simulator (v6)
File Name: simulation.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from scipy.interpolate import CubicSpline
from model import get_8state_discrete_model
from optimiser import solve_mpc
from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
import performance_stats
import speed_profile
from offline_tuner import curvature_estimate, adaptive_R_rate, adaptive_R_scaling

# ==========================================
# SETUP AND CONFIGURATION
# ==========================================
dt = 0.05
N_horizon = 25
v_ref = 7.0  # fallback constant speed, used only if no path speed profile is available yet

# Speed-profiler planning limits (see speed_profile.py for full rationale).
# Kept here, not buried in speed_profile.py's defaults, so they're easy to
# find/tune alongside the rest of the simulation's physical parameters.
#
# NOTE on these specific values: mu=1.1 (~70% of the tire model's peak
# mu=1.6) initially seemed like a reasonable safety margin in isolation,
# but testing the full closed loop showed it was still too aggressive --
# the MPC's internal linear model (model.py, fixed Cf/Cr cornering
# stiffness) doesn't capture the nonlinear/load-sensitive grip falloff of
# the dual-track plant at speed, so the car would enter corners faster
# than it could actually track, run >3.5m off the soft lateral-error
# corridor, and push the QP into numerically-infeasible territory. mu=0.6
# (~37% of peak) leaves enough margin for that model mismatch in practice.
SPEED_PROFILE_V_MAX = 16.0
SPEED_PROFILE_MU = 0.6
SPEED_PROFILE_A_ACCEL_MAX = 2.5
SPEED_PROFILE_A_BRAKE_MAX = 4.0
SPEED_PROFILE_V_MIN = 2.5

u_bounds_min = np.array([-np.radians(35), -5.0])
u_bounds_max = np.array([np.radians(35), 5.0])

# Cost weight matrices.
# States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
#
# Q[4,4] (e_v) raised from 90 → 150: the old value was too weak on straights
# where lateral/heading errors are small and the speed error term needs to
# matter. 150 gives assertive speed tracking without competing with lateral
# correction in corners (where the speed error is naturally small anyway
# because the profiler has already set a low v_target there).
Q = np.diag(
    [134.95942241300168, 27.327221404865437, 386.4336744636786, 142.63293158273837, 10.812997524426207, 0.0, 0.0, 0.0]
)
R = np.diag(
    [90.99289213029606, 1.476665745034168]
)
R_rate = np.diag(
    [110.57544991998951, 0.24691490065604801]
)

is_drawing = False
is_simulated = False
flip_heading_180 = False
drawn_points = []
path_X, path_Y, path_Psi = [], [], []
path_v_profile = np.array([])  # curvature-based target speed at each path point
sim_history = {}
vehicle_params = VehicleParams()


# ==========================================
# INTERACTIVE GUI LAYOUT
# ==========================================
fig = plt.figure(figsize=(15, 9.2))
gs = fig.add_gridspec(
    5,
    2,
    width_ratios=[3.8, 1.2],
    height_ratios=[12, 1, 1, 1, 1],
    left=0.06,
    right=0.94,
    top=0.94,
    bottom=0.06,
    wspace=0.15,
    hspace=0.45,
)

ax_map = fig.add_subplot(gs[0, 0])
ax_info = fig.add_subplot(gs[0, 1])
ax_info.axis("off")

(path_line,) = ax_map.plot([], [], "r--", label="Drawn Target Path", linewidth=2)
(trail_line,) = ax_map.plot([], [], "b-", label="Actual Vehicle Trail", alpha=0.6)
(pred_line,) = ax_map.plot(
    [], [], "c-o", label="MPC Horizon Prediction", markersize=3, alpha=0.8
)
(vehicle_marker,) = ax_map.plot([], [], "g-", linewidth=2.5, label="Vehicle")

ax_map.set_xlim(0, 100)
ax_map.set_ylim(0, 60)
ax_map.set_aspect("equal")
ax_map.grid(True)
ax_map.set_title("Robust High-Speed Path MPC Sandbox", fontweight="bold")
ax_map.legend(loc="upper right")

telemetry_text = ax_info.text(
    0.0,
    0.95,
    "",
    family="monospace",
    fontsize=10.5,
    verticalalignment="top",
    bbox=dict(facecolor="#f8f9fa", edgecolor="#ccced1", boxstyle="round,pad=0.7"),
)

ax_ey0 = fig.add_subplot(gs[1, 0])
ax_epsi0 = fig.add_subplot(gs[2, 0])
ax_scrub = fig.add_subplot(gs[3:5, 0])

slider_ey0 = Slider(
    ax_ey0,
    "Initial Lat Error",
    -4.0,
    4.0,
    valinit=0.0,
    valfmt="%0.1f m",
    color="orange",
)
slider_epsi0 = Slider(
    ax_epsi0,
    "Initial Yaw Error",
    -30.0,
    30.0,
    valinit=0.0,
    valfmt="%0.1f°",
    color="orange",
)
slider_scrub = Slider(
    ax_scrub, "Time", 0, 1, valinit=0, valfmt="%d", color="teal"
)
ax_scrub.set_visible(False)

ax_btn_start = fig.add_subplot(gs[1, 1])
ax_btn_flip = fig.add_subplot(gs[2, 1])
ax_btn_reset = fig.add_subplot(gs[3, 1])
ax_btn_optimize = fig.add_subplot(gs[4, 1])

btn_start = Button(
    ax_btn_start, "Start Sim", color="lightgreen", hovercolor="limegreen"
)
btn_flip = Button(
    ax_btn_flip, "Flip Heading (180°)", color="lightgreen", hovercolor="khaki"
)
btn_reset = Button(
    ax_btn_reset, "Reset Environment", color="tomato", hovercolor="crimson"
)
btn_optimize = Button(
    ax_btn_optimize, "Show Metrics", color="lightblue", hovercolor="deepskyblue"
)


# ==========================================
# HELPER MATHEMATICS & WRAPPING FIXES
# ==========================================
def get_car_triangle(x, y, heading, size=2.2):
    corners = np.array(
        [[size, 0], [-size / 1.5, size / 1.5], [-size / 1.5, -size / 1.5], [size, 0]]
    )
    rot = np.array(
        [[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]]
    )
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

    distances = np.hypot(
        path_X[start_search:end_search] - x_g, path_Y[start_search:end_search] - y_g
    )
    local_idx = np.argmin(distances)
    global_idx = start_search + local_idx

    return global_idx, path_X[global_idx], path_Y[global_idx], path_Psi[global_idx]


# ==========================================
# INTERACTIVE EVENT HANDLERS
# ==========================================
def reset_environment(event):
    global is_simulated, flip_heading_180, drawn_points, path_X, path_Y, path_Psi, path_v_profile, sim_history
    is_simulated = False
    flip_heading_180 = False
    drawn_points = []
    path_X, path_Y, path_Psi = [], [], []
    path_v_profile = np.array([])
    sim_history = {}

    path_line.set_data([], [])
    trail_line.set_data([], [])
    pred_line.set_data([], [])
    vehicle_marker.set_data([], [])
    telemetry_text.set_text("")

    ax_ey0.set_visible(True)
    ax_epsi0.set_visible(True)
    ax_scrub.set_visible(False)

    btn_start.set_active(True)
    btn_flip.set_active(True)
    btn_optimize.set_active(True)
    slider_ey0.set_val(0.0)
    slider_epsi0.set_val(0.0)
    ax_map.set_title(
        "Environment Reset. Draw a new path.", fontweight="bold", color="black"
    )
    fig.canvas.draw_idle()


def on_press(event):
    global is_drawing, drawn_points, is_simulated
    if event.inaxes != ax_map or is_simulated:
        return
    is_drawing = True
    drawn_points = [[event.xdata, event.ydata]]


def on_motion(event):
    global drawn_points
    if not is_drawing or event.inaxes != ax_map:
        return
    drawn_points.append([event.xdata, event.ydata])
    pts = np.array(drawn_points)
    path_line.set_data(pts[:, 0], pts[:, 1])
    fig.canvas.draw_idle()


def on_release(event):
    global is_drawing, path_X, path_Y, path_Psi, path_v_profile, flip_heading_180
    if not is_drawing:
        return
    is_drawing = False
    if len(drawn_points) < 6:
        return

    # Filter points to remove initial duplicate/jitter data
    raw_pts = np.array(drawn_points)
    filtered_pts = [raw_pts[0]]
    for p in raw_pts[1:]:
        if np.linalg.norm(p - filtered_pts[-1]) > 0.5:
            filtered_pts.append(p)

    if len(filtered_pts) < 4:
        filtered_pts = list(raw_pts)  # Fallback if dragged ultra-slow

    pts = np.array(filtered_pts)
    t = np.linspace(0, 1, len(pts))

    # Clamped boundary conditions: pin the spline's derivative at each end
    # to the direction of the first/last chord, rather than letting scipy's
    # default "not-a-knot" condition choose a free boundary derivative.
    d0 = (pts[1] - pts[0]) / (t[1] - t[0])
    dN = (pts[-1] - pts[-2]) / (t[-1] - t[-2])
    cs_x = CubicSpline(t, pts[:, 0], bc_type=((1, d0[0]), (1, dN[0])))
    cs_y = CubicSpline(t, pts[:, 1], bc_type=((1, d0[1]), (1, dN[1])))

    t_fine = np.linspace(0, 1, max(600, len(pts) * 6))
    path_X = cs_x(t_fine)
    path_Y = cs_y(t_fine)

    dx = cs_x.derivative()(t_fine)
    dy = cs_y.derivative()(t_fine)
    path_Psi = np.arctan2(dy, dx)

    raw_profile = speed_profile.compute_speed_profile(
        path_X, path_Y,
        v_max=SPEED_PROFILE_V_MAX,
        mu=SPEED_PROFILE_MU,
        a_accel_max=SPEED_PROFILE_A_ACCEL_MAX,
        a_brake_max=SPEED_PROFILE_A_BRAKE_MAX,
        v_min=SPEED_PROFILE_V_MIN,
    )
    path_v_profile = speed_profile.smooth_profile(raw_profile, window=9)

    flip_heading_180 = False
    path_line.set_data(path_X, path_Y)

    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()


def toggle_heading_flip(event):
    global flip_heading_180
    if len(path_X) == 0:
        return
    flip_heading_180 = not flip_heading_180

    base_heading = path_Psi[0] + (np.pi if flip_heading_180 else 0.0)
    current_psi = normalize_angle(base_heading + np.radians(slider_epsi0.val))

    X_g = path_X[0] - slider_ey0.val * np.sin(base_heading)
    Y_g = path_Y[0] + slider_ey0.val * np.cos(base_heading)

    car_x, car_y = get_car_triangle(X_g, Y_g, current_psi)
    vehicle_marker.set_data(car_x, car_y)
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)
btn_flip.on_clicked(toggle_heading_flip)
btn_reset.on_clicked(reset_environment)


# ==========================================
# SIMULATION ENGINE
# ==========================================
def simulate_closed_loop(Q_w, R_w, ey0, epsi0, flip, rng_seed=None, max_steps=400, R_rate_w=None):
    """
    Run one closed-loop rollout: nonlinear vehicle plant (vehicle_physics)
    driven by the parameterized MPC (optimiser.solve_mpc), which internally
    predicts using the linear model (model.get_8state_discrete_model).

    The plant (truth) and the controller's internal model are deliberately
    different -- the controller never sees the nonlinear tire/load-transfer
    effects directly, only through the resulting tracking error fed back
    each step, exactly like a real controller working off state estimates.

    rng_seed, if given, adds small random perturbations to initial heading/
    lateral error and a small per-step process-noise jitter on vx, so that
    repeated rollouts (used by the optimizer) aren't all identical -- which
    gives a meaningful average RMSE across "10 runs" rather than just
    running the same deterministic trajectory 10 times.
    """
    if R_rate_w is None:
        R_rate_w = R_rate

    rng = np.random.default_rng(rng_seed)
    jitter_ey = rng.normal(0, 0.05) if rng_seed is not None else 0.0
    jitter_epsi = rng.normal(0, 1.0) if rng_seed is not None else 0.0  # degrees

    base_path_heading = path_Psi[0] + (np.pi if flip else 0.0)
    ey0_eff = ey0 + jitter_ey
    epsi0_eff = epsi0 + jitter_epsi

    X_g = path_X[0] - ey0_eff * np.sin(base_path_heading)
    Y_g = path_Y[0] + ey0_eff * np.cos(base_path_heading)
    psi_g = normalize_angle(base_path_heading + np.radians(epsi0_eff))

    v_start = path_v_profile[0] if len(path_v_profile) > 0 else v_ref

    # Nonlinear plant truth state: [X, Y, psi, vx, vy, r, delta_act, a_act]
    plant_state = init_plant_state(X_g, Y_g, psi_g, vx0=v_start)

    u_prev = np.zeros(2)
    consecutive_solver_failures = 0
    MAX_CONSECUTIVE_FAILURES = 5
    OFFTRACK_LIMIT = 8.0  # meters; well beyond the soft +/-3.5m tracking corridor

    history = {
        "X": [], "Y": [], "psi": [], "v": [], "v_target": [],
        "u_steer": [], "u_accel": [],
        "e_y": [], "e_psi": [],
        "pred_X": [], "pred_Y": [],
        "failed": False,
        "fail_reason": None,
    }

    idx = 0
    for step in range(max_steps):
        # ------------------------------------------------------------------
        # 1. RECORD current plant state BEFORE the solve (for telemetry)
        # ------------------------------------------------------------------
        history["X"].append(plant_state[0])
        history["Y"].append(plant_state[1])
        history["psi"].append(plant_state[2])
        history["v"].append(plant_state[3])

        # ------------------------------------------------------------------
        # 2. FIND reference point and compute tracking errors from CURRENT
        #    plant state — this is the fix for the stale-state bug.
        #    In v5 this block lived at the BOTTOM of the loop, after the
        #    plant step, so the MPC was fed last timestep's errors.
        # ------------------------------------------------------------------
        X_g, Y_g, psi_g = plant_state[0], plant_state[1], plant_state[2]

        idx, rx, ry, rpsi = find_closest_reference_bounded(X_g, Y_g, idx, window=40)
        if flip:
            rpsi = normalize_angle(rpsi + np.pi)

        dx_err = X_g - rx
        dy_err = Y_g - ry
        e_y   = dy_err * np.cos(rpsi) - dx_err * np.sin(rpsi)
        e_psi = normalize_angle(psi_g - rpsi)

        v_target = path_v_profile[idx] if len(path_v_profile) > 0 else v_ref
        history["v_target"].append(v_target)
        history["e_y"].append(e_y)
        history["e_psi"].append(e_psi)

        # Controller's tracking-error state (what the MPC sees):
        # [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
        x_current = np.array([
            e_y,
            plant_state[3] * np.sin(e_psi) + plant_state[4] * np.cos(e_psi),
            e_psi,
            plant_state[5],           # yaw rate r
            plant_state[3] - v_target, # speed error (using current vx, not last)
            0.0,                       # e_a not penalized; R_rate handles smoothness
            plant_state[6],            # delta_act
            plant_state[7],            # a_act
        ])

        # ------------------------------------------------------------------
        # 3. EARLY-EXIT CHECKS (before the solve, using fresh errors)
        # ------------------------------------------------------------------
        if consecutive_solver_failures >= MAX_CONSECUTIVE_FAILURES:
            history["failed"] = True
            history["fail_reason"] = (
                f"solver failed {consecutive_solver_failures} consecutive steps "
                f"at step {step}"
            )
            break

        if abs(e_y) > OFFTRACK_LIMIT:
            history["failed"] = True
            history["fail_reason"] = f"off-track (|e_y|={abs(e_y):.2f} m) at step {step}"
            break

        if idx >= len(path_X) - 2:
            history["reached_end"] = True
            break

        # ------------------------------------------------------------------
        # 4. MPC SOLVE with current state
        # ------------------------------------------------------------------
        current_v = plant_state[3]
        Ad, Bd = get_8state_discrete_model(max(current_v, 0.5), dt)

        kappa = curvature_estimate(plant_state)
        R_rate_scaled = adaptive_R_rate(kappa, R_rate_w)
        R_scaled = adaptive_R_scaling(current_v, R_w)

        u_opt = solve_mpc(
            x_current, Ad, Bd, N_horizon,
            Q_w, R_scaled,
            u_bounds_min, u_bounds_max,
            R_rate=R_rate_scaled,
            u_prev=u_prev,
        )

        solved_ok = u_opt is not None
        if solved_ok:
            consecutive_solver_failures = 0
        else:
            consecutive_solver_failures += 1
            u_opt = u_prev.copy()

        u_prev = u_opt.copy()

        history["u_steer"].append(u_opt[0])
        history["u_accel"].append(u_opt[1])

        # ------------------------------------------------------------------
        # 5. PREDICTED HORIZON for visualization (linear model rollout)
        # ------------------------------------------------------------------
        px, py = [], []
        X_p, Y_p, psi_p = plant_state[0], plant_state[1], plant_state[2]
        x_p_tmp = x_current.copy()
        for k in range(N_horizon):
            v_p = current_v + x_p_tmp[4]
            X_p += v_p * np.cos(psi_p) * dt
            Y_p += v_p * np.sin(psi_p) * dt
            psi_p += x_p_tmp[3] * dt
            px.append(X_p)
            py.append(Y_p)
            x_p_tmp = Ad @ x_p_tmp + Bd @ u_opt
        history["pred_X"].append(px)
        history["pred_Y"].append(py)

        # ------------------------------------------------------------------
        # 6. ADVANCE the nonlinear plant
        # ------------------------------------------------------------------
        plant_state = step_nonlinear_plant(plant_state, u_opt, dt, vehicle_params)

    history.setdefault("reached_end", False)
    if history["reached_end"]:
        history["completion_frac"] = 1.0
    else:
        history["completion_frac"] = len(history["X"]) / max(max_steps, 1)
    return history


def run_simulation(event):
    global is_simulated, sim_history
    if len(path_X) == 0:
        ax_map.set_title("ERROR: Draw a path first!", color="red", fontweight="bold")
        fig.canvas.draw_idle()
        return
    is_simulated = True
    btn_start.set_active(False)
    btn_flip.set_active(False)
    btn_optimize.set_active(False)
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)

    # BUG FIX: pass R_rate_w explicitly so updated tuner weights are used.
    history = simulate_closed_loop(
        Q, R, slider_ey0.val, slider_epsi0.val, flip_heading_180,
        rng_seed=None, R_rate_w=R_rate,
    )

    sim_history = history
    ax_map.set_title(
        "Simulation Complete! Review tracking via Slider below.",
        fontweight="bold",
        color="darkgreen",
    )

    ax_scrub.set_visible(True)
    slider_scrub.valmax = len(history["X"]) - 1
    slider_scrub.ax.set_xlim(0, len(history["X"]) - 1)
    slider_scrub.on_changed(update_scrub_frame)
    update_scrub_frame(0)
    btn_optimize.set_active(True)


btn_start.on_clicked(run_simulation)


# ==========================================
# PERFORMANCE METRICS (SHOW METRICS BUTTON)
# ==========================================
def run_optimize(event):
    """
    Scores the most recently completed simulation and prints a full
    performance breakdown to the console. No rollouts are re-run and no
    weights are modified — weight tuning is done exclusively via
    offline_tuner.py.

    The composite score and all sub-terms are computed with the identical
    formula used by offline_tuner.run_headless_rollout, so live scores are
    directly comparable to offline tuning results.

    Requires a simulation to have been run first ("Start Sim").
    """
    if not is_simulated or not sim_history:
        ax_map.set_title(
            "Run a simulation first (Start Sim), then Show Metrics.",
            fontweight="bold", color="darkorange",
        )
        fig.canvas.draw_idle()
        return

    print("=" * 58)
    metrics = performance_stats.report_performance_metrics(sim_history, log_fn=print)

    ax_map.set_title(
        f"Metrics: composite={metrics['composite_score']:.4f}  "
        f"lat={metrics['lateral_rmse_m']:.3f} m  "
        f"hdg={metrics['heading_rmse_deg']:.2f}°  "
        f"completion={metrics['completion_pct']:.0f}%  "
        f"(see console)",
        fontweight="bold",
        color="darkgreen" if not metrics["failed"] else "crimson",
    )
    fig.canvas.draw_idle()


btn_optimize.on_clicked(run_optimize)


# ==========================================
# TIMELINE REVIEW SCRUBBING
# ==========================================
def update_scrub_frame(val):
    frame = int(val)
    h = sim_history

    trail_line.set_data(h["X"][: frame + 1], h["Y"][: frame + 1])
    pred_line.set_data(h["pred_X"][frame], h["pred_Y"][frame])

    car_x, car_y = get_car_triangle(h["X"][frame], h["Y"][frame], h["psi"][frame])
    vehicle_marker.set_data(car_x, car_y)

    v_target_str = (
        f"{h['v_target'][frame]:6.2f} m/s"
        if "v_target" in h and len(h["v_target"]) > frame
        else "   n/a"
    )
    text = (
        f"     HISTORIC FRAME: {frame:03d}\n"
        f"=======================\n"
        f"Speed     : {h['v'][frame]:6.2f} m/s\n"
        f"Target Spd: {v_target_str}\n"
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