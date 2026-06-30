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
from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
import tuner
import speed_profile

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
SPEED_PROFILE_V_MAX = 10.0
SPEED_PROFILE_MU = 0.6
SPEED_PROFILE_A_ACCEL_MAX = 2.5
SPEED_PROFILE_A_BRAKE_MAX = 4.0
SPEED_PROFILE_V_MIN = 2.5

u_bounds_min = np.array([-np.radians(35), -5.0])
u_bounds_max = np.array([np.radians(35), 5.0])

# Highly protective lateral alignment weights
# States: [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta, a]

Q = np.diag(
    [
        2725.0,  # e_y       — lateral error (closest-point, post-fix)
        215.0,  # e_yd      — lateral velocity
        6590.0,  # e_psi     — heading error (closest-point, post-fix)
        3605.0,  # e_psi_d   — heading rate damping (was 4850, see note)
        90.0,  # e_v       — speed error
        0.0,  # e_a       — acceleration error
        0.0,  # delta_act — actuator steer (regularisation)
        0.0,  # a_act     — actuator accel (regularisation)
    ]
)
R = np.diag(
    [
        1400.0,  # delta_cmd
        108.0,  # a_cmd
    ]
)
R_rate = np.diag(
    [
        1561.0,   # d(delta_cmd)/dt — penalize sharp steering jerk
        4.2,    # d(a_cmd)/dt     — penalize sharp accel/brake jerk
    ]
)

is_drawing = False
is_simulated = False
flip_heading_180 = False
drawn_points = []
path_X, path_Y, path_Psi = [], [], []
path_v_profile = np.array([])  # NEW: curvature-based target speed at each path point
sim_history = {}
vehicle_params = VehicleParams()
is_optimizing = False

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
    ax_btn_optimize, "Optimise Weights (x10)", color="lightblue", hovercolor="deepskyblue"
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
# INTERACTIVE DRAWING HANDLERS
# ==========================================
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
    # With sparse or unevenly-spaced control points (e.g. fewer points
    # surviving the 0.5m de-jitter filter above, which happens more often
    # when the plot is zoomed in -- the same physical mouse drag produces
    # fewer points that clear the filter), not-a-knot splines can badly
    # overshoot right at the start/end, producing a heading there that
    # points nearly the OPPOSITE direction of actual travel. That secretly
    # broke the car's initial heading (path_Psi near t=0) even though only
    # path_Psi[0] itself was being patched afterward -- the overshoot
    # extended several samples into the resampled path, past where the old
    # single-point patch could fix it.
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

    # Compute the curvature-based target speed profile for this path now
    # that path_X/path_Y are finalized -- this replaces the old constant
    # v_ref with a per-point target the car looks up as it tracks along
    # the path (slower through tight corners, faster on straights).
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

    Returns a history dict compatible with the GUI's scrub/telemetry code.

    R_rate_w: optional override for the actuator-rate-of-change weight
    matrix. Defaults to the module-level R_rate global (used by the normal
    "Start Sim" button); the tuner passes a different value explicitly per
    candidate, rather than mutating the global, so that the comparison
    between baseline and candidate weights during one Optimise click can't
    be corrupted by leftover global state from a previous call.
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

    # Initial speed comes from the curvature-based profile at the path's
    # start point (falls back to the constant v_ref if no profile was
    # computed yet, e.g. an extremely short/degenerate path).
    v_start = path_v_profile[0] if len(path_v_profile) > 0 else v_ref

    # Nonlinear plant truth state: [X, Y, psi, vx, vy, r, delta_act, a_act]
    plant_state = init_plant_state(X_g, Y_g, psi_g, vx0=v_start)

    # Controller's tracking-error state (what the MPC sees):
    # [e_y, e_y_dot, e_psi, e_psi_dot, e_v, e_a, delta_act, a_act]
    x_current = np.array([ey0_eff, 0.0, np.radians(epsi0_eff), 0.0, 0.0, 0.0, 0.0, 0.0])

    u_prev = np.zeros(2)  # NEW: last applied [delta_cmd, a_cmd], for rate penalty continuity
    consecutive_solver_failures = 0
    MAX_CONSECUTIVE_FAILURES = 5   # bail out rather than coast on held commands forever
    OFFTRACK_LIMIT = 8.0           # meters; well beyond the soft +/-3.5m tracking corridor

    history = {
        "X": [], "Y": [], "psi": [], "v": [], "v_target": [],
        "u_steer": [], "u_accel": [],
        "e_y": [], "e_psi": [],
        "pred_X": [], "pred_Y": [],
        "failed": False,          # NEW: True if the rollout was aborted early
        "fail_reason": None,      # NEW: human-readable reason, for logging
    }

    idx = 0
    for step in range(max_steps):
        vx_plant = plant_state[3]
        current_v = vx_plant

        history["X"].append(plant_state[0])
        history["Y"].append(plant_state[1])
        history["psi"].append(plant_state[2])
        history["v"].append(current_v)
        history["e_y"].append(x_current[0])
        history["e_psi"].append(x_current[2])

        # MPC plans using its own linear model, linearized at the current speed
        Ad, Bd = get_8state_discrete_model(max(current_v, 0.5), dt)
        u_opt = solve_mpc(
            x_current, Ad, Bd, N_horizon, Q_w, R_w, u_bounds_min, u_bounds_max,
            R_rate=R_rate_w, u_prev=u_prev,
        )

        solved_ok = u_opt is not None

        if solved_ok:
            consecutive_solver_failures = 0
        else:
            consecutive_solver_failures += 1
            u_opt = u_prev.copy()
        
        u_prev = u_opt.copy()
        
        # print("u_opt:", u_opt, "step:", step, "consecutive_solver_failures:", consecutive_solver_failures)

        history["u_steer"].append(u_opt[0])
        history["u_accel"].append(u_opt[1])

        # Predicted horizon for visualization, using the MPC's own linear model
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

        # --- Advance the TRUTH plant nonlinearly ---
        plant_state = step_nonlinear_plant(plant_state, u_opt, dt, vehicle_params)

        X_g, Y_g, psi_g = plant_state[0], plant_state[1], plant_state[2]

        idx, rx, ry, rpsi = find_closest_reference_bounded(X_g, Y_g, idx, window=40)
        if flip:
            rpsi = normalize_angle(rpsi + np.pi)

        dx = X_g - rx
        dy = Y_g - ry
        e_y = dy * np.cos(rpsi) - dx * np.sin(rpsi)
        e_psi = normalize_angle(psi_g - rpsi)

        # Local target speed from the curvature-based profile at the
        # car's current closest-path index -- this is what makes the car
        # slow down for corners and speed up on straights, instead of
        # chasing one constant v_ref everywhere. Falls back to v_ref if
        # no profile is available (shouldn't normally happen once a path
        # has been drawn, but keeps this function safe to call standalone).
        v_target = path_v_profile[idx] if len(path_v_profile) > 0 else v_ref
        history["v_target"].append(v_target)

        # Controller's tracking-error state is rebuilt from the plant's true
        # pose (acting as a stand-in for a state estimator) each step.
        x_current = np.array([
            e_y,
            plant_state[3] * np.sin(e_psi) + plant_state[4] * np.cos(e_psi),
            e_psi,
            plant_state[5],
            current_v - v_target,
            0.0,
            plant_state[6],
            plant_state[7],
        ])

        if consecutive_solver_failures >= MAX_CONSECUTIVE_FAILURES:
            # Holding the last command repeatedly means the controller has
            # lost the ability to meaningfully act -- treat this rollout as
            # failed rather than padding out the remaining steps doing
            # nothing useful (which would otherwise look like a short,
            # deceptively low-error rollout to anything scoring it).
            history["failed"] = True
            history["fail_reason"] = (
                f"solver failed {consecutive_solver_failures} consecutive steps "
                f"at step {step}"
            )
            break

        if abs(e_y) > OFFTRACK_LIMIT:
            # The car has departed the track far enough that continuing the
            # rollout isn't meaningful -- stop here instead of accumulating
            # more (likely still-diverging) error samples.
            history["failed"] = True
            history["fail_reason"] = f"off-track (|e_y|={abs(e_y):.2f} m) at step {step}"
            break

        if idx >= len(path_X) - 2:
            history["reached_end"] = True
            break

    history.setdefault("reached_end", False)
    # completion_frac is only meaningful as a penalty signal when the
    # rollout did NOT reach the end of the path on its own -- a path that's
    # legitimately short and finishes in fewer than max_steps is a full
    # success, not partial completion, so it's reported as 1.0 in that case.
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
    if is_optimizing:
        return

    is_simulated = True
    btn_start.set_active(False)
    btn_flip.set_active(False)
    btn_optimize.set_active(False)
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)

    history = simulate_closed_loop(
        Q, R, slider_ey0.val, slider_epsi0.val, flip_heading_180, rng_seed=None
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
# AUTO-TUNER (OPTIMISE BUTTON)
# ==========================================
def run_optimize(event):
    """
    Runs 10 closed-loop rollouts on the currently drawn path with the
    current Q/R/R_rate weights, then 10 more with a perturbed candidate,
    and keeps whichever set of weights produced lower tracking RMSE.
    Pressing the button again continues the search from whatever weights
    are currently applied.
    """
    global Q, R, R_rate, is_optimizing
    if len(path_X) == 0:
        ax_map.set_title("ERROR: Draw a path first!", color="red", fontweight="bold")
        fig.canvas.draw_idle()
        return
    if is_optimizing:
        return

    is_optimizing = True
    btn_start.set_active(False)
    btn_flip.set_active(False)
    btn_reset.set_active(False)
    btn_optimize.set_active(False)
    ax_map.set_title("Optimising cost weights (10 + 10 rollouts)... please wait.",
                      fontweight="bold", color="darkblue")
    fig.canvas.draw_idle()
    plt.pause(0.01)  # force a redraw before the blocking optimization work

    ey0 = slider_ey0.val if ax_ey0.get_visible() else 0.0
    epsi0 = slider_epsi0.val if ax_epsi0.get_visible() else 0.0

    def run_rollout_fn(Q_w, R_w, R_rate_w, seed):
        return simulate_closed_loop(
            Q_w, R_w, ey0, epsi0, flip_heading_180, rng_seed=seed, R_rate_w=R_rate_w
        )

    print("=" * 60)
    new_Q, new_R, new_R_rate, info = tuner.optimize_weights(
        Q, R, R_rate, run_rollout_fn, num_runs=10, rng_seed=None, log_fn=print
    )
    print("=" * 60)

    Q, R, R_rate = new_Q, new_R, new_R_rate

    # Show the result of the best-weights rollout (seed=0) on screen.
    global sim_history, is_simulated
    sim_history = run_rollout_fn(Q, R, R_rate, seed=0)
    is_simulated = True
    ax_ey0.set_visible(False)
    ax_epsi0.set_visible(False)
    ax_scrub.set_visible(True)
    slider_scrub.valmax = len(sim_history["X"]) - 1
    slider_scrub.ax.set_xlim(0, len(sim_history["X"]) - 1)
    slider_scrub.on_changed(update_scrub_frame)
    update_scrub_frame(0)

    status = "IMPROVED" if info["accepted"] else "UNCHANGED (no improvement found)"
    ax_map.set_title(
        f"Optimise complete: {status}. Best RMSE = {info['best_rmse']:.3f} m "
        f"(see console for full weight log).",
        fontweight="bold",
        color="darkgreen" if info["accepted"] else "darkorange",
    )

    is_optimizing = False
    btn_start.set_active(True)
    btn_flip.set_active(True)
    btn_reset.set_active(True)
    btn_optimize.set_active(True)
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
        f"{h['v_target'][frame]:6.2f} m/s" if "v_target" in h and len(h["v_target"]) > frame
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