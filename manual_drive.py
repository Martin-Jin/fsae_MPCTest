"""
manual_drive.py — Keyboard/Mouse Manual Drive Mode

PURPOSE
-------
Lets a human drive the nonlinear vehicle plant directly with WASD + Space,
instead of the MPC controller. Useful for building intuition for the plant's
handling limits, sanity-checking cone placement/track geometry by feel, and
generating a reference "how would a human drive this" trace to compare
against MPC runs.

This is a companion to simulation.py, not a replacement: simulation.py owns
the MPC/offline-tuner integration; this file owns the human-in-the-loop path.
It reuses the same synthetic path library, cone placement, and 24-state
nonlinear plant so a manually-driven run is physically comparable to an
MPC-driven one, but it does NOT run the MPC solver, adaptive gain scheduling,
or rollout_core/scoring pipeline — driving is open-loop from the human's
perspective (no tracking-error feedback is computed or scored).

CONTROLS
--------
  W        — throttle (accelerate)
  S        — brake / reverse-accelerate
  A        — steer left
  D        — steer right
  SPACE    — full brake (overrides throttle)
  Steering and throttle/brake are rate-limited toward a commanded target so
  key taps feel analog rather than an on/off step input.

ARCHITECTURE WITHIN THIS FILE
------------------------------
  1. Configuration      — dt, plant params, control rate limits
  2. GUI Layout          — matplotlib figure, map axes, telemetry panel
  3. Helper Mathematics  — triangle renderer (mirrors simulation.py)
  4. Path Loading        — cycle synthetic paths, place cones (mirrors simulation.py)
  5. Keyboard State      — key-down/key-up handlers → held-key set
  6. Drive Loop          — FuncAnimation callback: read keys, step plant, redraw

USED BY
-------
  Standalone: run with `python manual_drive.py`
  Imports from: vehicle_physics, offline_tuner (SYNTHETIC_PATHS/PATH_NAMES),
                sim_track (place_cones), settings (DT)

DOES NOT USE
------------
  optimiser.py, bicycle_model.py, rollout_core.py, scoring.py, model_utils.py
  (no MPC solve, no adaptive gains, no scoring — this is open-loop human control)
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation

from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
from offline_tuner import SYNTHETIC_PATHS, PATH_NAMES
from sim_track import place_cones
from settings import DT

# ==========================================
# SETUP AND CONFIGURATION
# ==========================================
TRAIL_MAX_LEN = 4000   # Cap on stored trail points so long free-drives don't leak memory

# ── Control rate limits ─────────────────────────────────────────────────────
# Raw key state is a hard on/off signal; ramping the commanded steering and
# accel/brake toward the key-driven target each tick gives analog-feeling
# control instead of a step input straight into the actuator lag filter.
STEER_RATE = 3.0    # rad/s — how fast delta_cmd ramps toward its target
ACCEL_RATE = 20.0   # m/s³  — how fast a_cmd ramps toward its target

vehicle_params = VehicleParams()
MAX_STEER = vehicle_params.max_steer          # rad
MAX_ACCEL = vehicle_params.max_accel          # m/s² (throttle)
MAX_BRAKE = vehicle_params.max_accel_brake    # m/s² (negative)

# ── Global drive state ──────────────────────────────────────────────────────
is_driving   = False          # True once "Start Driving" has been clicked
path_X, path_Y, path_Psi = [], [], []   # Reference path (drawn for context only)
current_test_path_idx = -1
_blue_cones_all   = np.empty((0, 2))
_yellow_cones_all = np.empty((0, 2))

plant_state   = None          # 24-state nonlinear plant vector (None until driving starts)
delta_cmd     = 0.0           # Current commanded steering angle (rad), ramps toward key target
a_cmd         = 0.0           # Current commanded accel/brake (m/s²), ramps toward key target
held_keys     = set()         # Currently-held keyboard keys
trail_X, trail_Y = [], []     # Vehicle trail history for the blue trail line


# ==========================================
# GUI LAYOUT
# ==========================================
# Simple two-column layout: map on the left, buttons + telemetry stacked
# on the right. No sliders/scrub bar — this mode is live, not scrubbable.
fig = plt.figure(figsize=(12.0, 7.5))
gs  = fig.add_gridspec(
    6, 2,
    width_ratios=[3.6, 1.4],
    left=0.06, right=0.97, top=0.95, bottom=0.06,
    wspace=0.20, hspace=0.45,
)

ax_map = fig.add_subplot(gs[0:6, 0])

(path_line,)         = ax_map.plot([], [], "r--",  label="Reference Path",       linewidth=2)
(trail_line,)        = ax_map.plot([], [], "b-",   label="Driven Trail",         alpha=0.6)
(vehicle_marker,)    = ax_map.plot([], [], "g-",   linewidth=2.0,                label="Vehicle")
(blue_cones_line,)   = ax_map.plot([], [], color="blue", marker="o", linestyle="None", markersize=4, label="Blue Cones")
(yellow_cones_line,) = ax_map.plot([], [], color="gold", marker="o", linestyle="None", markersize=4, label="Yellow Cones")

ax_map.set_xlim(0, 75)
ax_map.set_ylim(0, 45)
ax_map.set_aspect("equal")
ax_map.grid(True)
ax_map.set_title("Manual Drive Mode — WASD + Space", fontweight="bold", fontsize=10)
ax_map.legend(loc="upper right", fontsize=8)

# ── Buttons — right column ───────────────────────────────────────────────────
ax_btn_load  = fig.add_subplot(gs[0, 1])
ax_btn_start = fig.add_subplot(gs[1, 1])
ax_btn_reset = fig.add_subplot(gs[2, 1])

btn_load  = Button(ax_btn_load,  "Load Test Path",  color="thistle",    hovercolor="plum")
btn_start = Button(ax_btn_start, "Start Driving",   color="lightgreen", hovercolor="limegreen")
btn_reset = Button(ax_btn_reset, "Reset",           color="tomato",     hovercolor="crimson")

# ── Telemetry panel — right column, below buttons ────────────────────────────
ax_info = fig.add_subplot(gs[3:6, 1])
ax_info.axis("off")
telemetry_text = ax_info.text(
    0.5, 1.0, "",
    family="monospace", fontsize=9.5, verticalalignment="top",
    horizontalalignment="center",
    transform=ax_info.transAxes,
    bbox=dict(facecolor="#f8f9fa", edgecolor="#ccced1", boxstyle="round,pad=0.8"),
)

# Controls reminder — static text under the telemetry panel
ax_info.text(
    0.5, 0.28,
    "W/S: throttle / brake\nA/D: steer left / right\nSPACE: full brake",
    family="monospace", fontsize=9, verticalalignment="top",
    horizontalalignment="center", transform=ax_info.transAxes, color="dimgray",
)


# ==========================================
# HELPER MATHEMATICS
# ==========================================

def get_car_triangle(x, y, heading, size=2.2):
    """
    Compute the (X, Y) vertices of a triangle representing the vehicle at a
    given position and heading, for rendering on the map axes.

    Mirrors simulation.py's get_car_triangle() exactly so manually-driven
    and MPC-driven runs render identically.

    Called by: update_frame(), load_test_path()
    """
    corners = np.array([
        [ size,          0         ],   # Front apex
        [-size / 1.5,    size / 1.5],   # Rear left
        [-size / 1.5,   -size / 1.5],   # Rear right
        [ size,          0         ],   # Close polygon
    ])
    rot = np.array([
        [np.cos(heading), -np.sin(heading)],
        [np.sin(heading),  np.cos(heading)],
    ])
    rotated = (rot @ corners.T).T
    return rotated[:, 0] + x, rotated[:, 1] + y


# ==========================================
# PATH LOADING
# ==========================================

def load_test_path(event):
    """
    Cycle through the synthetic path library and load the next path.

    The path is drawn only as a visual reference for the human driver — no
    tracking error against it is computed. Cones are placed exactly as in
    simulation.py so track boundaries look and behave the same.

    Does nothing while a drive is in progress (path locked mid-drive).

    Called by: btn_load ("Load Test Path" button)
    """
    global path_X, path_Y, path_Psi, current_test_path_idx
    global _blue_cones_all, _yellow_cones_all

    if is_driving:
        return

    current_test_path_idx = (current_test_path_idx + 1) % len(PATH_NAMES)
    path_name = PATH_NAMES[current_test_path_idx]

    path_X, path_Y, path_Psi, _, _, _ = SYNTHETIC_PATHS[path_name]

    _blue_cones_all, _yellow_cones_all = place_cones(path_X, path_Y)
    if len(_blue_cones_all) > 0:
        blue_cones_line.set_data(_blue_cones_all[:, 0], _blue_cones_all[:, 1])
        yellow_cones_line.set_data(_yellow_cones_all[:, 0], _yellow_cones_all[:, 1])

    path_line.set_data(path_X, path_Y)

    car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
    vehicle_marker.set_data(car_x, car_y)

    ax_map.set_title(f"Loaded: {path_name} | Click 'Start Driving'", fontweight="bold", color="blue")

    margin = 15.0
    ax_map.set_xlim(np.min(path_X) - margin, np.max(path_X) + margin)
    ax_map.set_ylim(np.min(path_Y) - margin, np.max(path_Y) + margin)
    fig.canvas.draw_idle()


btn_load.on_clicked(load_test_path)


# ==========================================
# KEYBOARD STATE
# ==========================================
# Raw key-down/key-up events just toggle membership in held_keys; the actual
# ramping toward a commanded steer/accel target happens once per frame in
# update_frame() so behaviour doesn't depend on OS key-repeat timing.

def on_key_press(event):
    if event.key is not None:
        held_keys.add(event.key.lower())


def on_key_release(event):
    if event.key is not None:
        held_keys.discard(event.key.lower())


fig.canvas.mpl_connect("key_press_event",   on_key_press)
fig.canvas.mpl_connect("key_release_event", on_key_release)


# ==========================================
# START / RESET
# ==========================================

def start_driving(event):
    """
    Initialise the plant at the loaded path's start pose and begin the live
    drive loop. Requires a path to have been loaded first (for a start pose
    and cone context) — driving without ever loading a path has nowhere
    sensible to spawn the car.

    Called by: btn_start ("Start Driving" button)
    """
    global is_driving, plant_state, delta_cmd, a_cmd, trail_X, trail_Y

    if len(path_X) == 0:
        ax_map.set_title("ERROR: Load a path first!", color="red", fontweight="bold")
        fig.canvas.draw_idle()
        return

    plant_state = init_plant_state(path_X[0], path_Y[0], path_Psi[0], vx0=0.0)
    delta_cmd = 0.0
    a_cmd     = 0.0
    trail_X, trail_Y = [], []
    held_keys.clear()

    is_driving = True
    btn_load.set_active(False)
    ax_map.set_title("Driving — WASD + Space", fontweight="bold", color="darkgreen")
    fig.canvas.draw_idle()


def reset_drive(event):
    """
    Stop driving and clear the plant/trail state, returning to the
    pre-drive display (path and cones remain visible for reference).

    Called by: btn_reset ("Reset" button)
    """
    global is_driving, plant_state, trail_X, trail_Y

    is_driving   = False
    plant_state  = None
    trail_X, trail_Y = [], []
    held_keys.clear()

    trail_line.set_data([], [])
    telemetry_text.set_text("")
    btn_load.set_active(True)

    if len(path_X) > 0:
        car_x, car_y = get_car_triangle(path_X[0], path_Y[0], path_Psi[0])
        vehicle_marker.set_data(car_x, car_y)
        ax_map.set_title("Reset. Click 'Start Driving' to go again.", fontweight="bold", color="black")
    else:
        vehicle_marker.set_data([], [])
        ax_map.set_title("Manual Drive Mode — WASD + Space", fontweight="bold", fontsize=10)
    fig.canvas.draw_idle()


btn_start.on_clicked(start_driving)
btn_reset.on_clicked(reset_drive)


# ==========================================
# DRIVE LOOP
# ==========================================

def update_frame(_frame):
    """
    One live-drive tick: read held keys, ramp steering/accel commands toward
    their key-driven targets, step the nonlinear plant, and redraw.

    Rate-limited ramping (STEER_RATE, ACCEL_RATE) is applied here rather than
    snapping straight to ±max on a key press — this keeps the input analog-
    feeling and avoids slamming the actuator lag filter with step inputs,
    matching the smoothness a real driver's hands/feet would produce.

    SPACE overrides W/S entirely (full brake), matching a real e-stop / brake
    pedal always winning over throttle.

    Called by: FuncAnimation(fig, update_frame, ...) every DT seconds
    """
    global plant_state, delta_cmd, a_cmd, trail_X, trail_Y

    if not is_driving or plant_state is None:
        return (trail_line, vehicle_marker)

    # ── Determine commanded targets from held keys ─────────────────────────
    steer_target = 0.0
    if "a" in held_keys:
        steer_target += MAX_STEER    # FSDS ENU: positive delta = left turn
    if "d" in held_keys:
        steer_target -= MAX_STEER

    if " " in held_keys:
        accel_target = MAX_BRAKE     # Space: full brake, overrides W/S
    else:
        accel_target = 0.0
        if "w" in held_keys:
            accel_target += MAX_ACCEL
        if "s" in held_keys:
            accel_target += MAX_BRAKE   # MAX_BRAKE is already negative

    # ── Ramp commands toward target (rate-limited for analog feel) ─────────
    delta_cmd += float(np.clip(steer_target - delta_cmd, -STEER_RATE * DT, STEER_RATE * DT))
    a_cmd     += float(np.clip(accel_target - a_cmd,     -ACCEL_RATE * DT, ACCEL_RATE * DT))
    delta_cmd = float(np.clip(delta_cmd, -MAX_STEER, MAX_STEER))
    a_cmd     = float(np.clip(a_cmd, MAX_BRAKE, MAX_ACCEL))

    # ── Step the nonlinear plant ─────────────────────────────────────────────
    u_cmd = np.array([delta_cmd, a_cmd])
    plant_state = step_nonlinear_plant(plant_state, u_cmd, DT, vehicle_params)

    X_g, Y_g, psi_g, vx = plant_state[0], plant_state[1], plant_state[2], plant_state[3]

    # ── Update trail (capped length) ─────────────────────────────────────────
    trail_X.append(X_g)
    trail_Y.append(Y_g)
    if len(trail_X) > TRAIL_MAX_LEN:
        trail_X = trail_X[-TRAIL_MAX_LEN:]
        trail_Y = trail_Y[-TRAIL_MAX_LEN:]
    trail_line.set_data(trail_X, trail_Y)

    # ── Update vehicle marker ─────────────────────────────────────────────────
    car_x, car_y = get_car_triangle(X_g, Y_g, psi_g)
    vehicle_marker.set_data(car_x, car_y)

    # ── Keep the car in view: re-centre camera if it nears the edge ────────
    xlim = ax_map.get_xlim()
    ylim = ax_map.get_ylim()
    margin = 8.0
    if not (xlim[0] + margin < X_g < xlim[1] - margin
            and ylim[0] + margin < Y_g < ylim[1] - margin):
        half_w = (xlim[1] - xlim[0]) / 2.0
        half_h = (ylim[1] - ylim[0]) / 2.0
        ax_map.set_xlim(X_g - half_w, X_g + half_w)
        ax_map.set_ylim(Y_g - half_h, Y_g + half_h)

    # ── Telemetry panel ───────────────────────────────────────────────────────
    text = (
        f"    LIVE TELEMETRY\n"
        f"========================\n"
        f"Speed     : {vx:6.2f} m/s\n"
        f"Pos X     : {X_g:6.2f} m\n"
        f"Pos Y     : {Y_g:6.2f} m\n"
        f"Heading   : {math.degrees(psi_g):6.1f} deg\n"
        f"-----------------------\n"
        f"Steer Cmd : {math.degrees(delta_cmd):6.1f} deg\n"
        f"Accel Cmd : {a_cmd:6.2f} m/s²"
    )
    telemetry_text.set_text(text)

    return (trail_line, vehicle_marker, blue_cones_line, yellow_cones_line)


# blit=False: the camera re-centring above moves the axes limits, which
# blitting doesn't pick up correctly. Interval matches DT (20 Hz) so plant
# stepping stays in real time regardless of render cost.
anim = FuncAnimation(fig, update_frame, interval=DT * 1000.0, blit=False, cache_frame_data=False)

plt.show()