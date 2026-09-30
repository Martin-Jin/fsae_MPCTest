"""
model/vehicle_physics/state.py — plant state-vector layout and initial state

PURPOSE
-------
Named indices into the 25-element plant state vector (IDX_*, N_STATES) and
init_plant_state(), which builds the vector at static equilibrium. Every
consumer of the plant indexes the state through these names, so a change to
the vector layout starts here.

USED BY
-------
  plant_step.py, tracking.py (indices), gui/simulation.py, gui/manual_drive.py,
  tuner/offline_tuner.py and the rollout code (init_plant_state).
"""
import numpy as np

from model.vehicle_physics.params import VehicleParams


# ─────────────────────────────────────────────────────────────
# NAMED STATE INDEX CONSTANTS
# Using named constants everywhere prevents silent index bugs
# when the state vector length changes.
# ─────────────────────────────────────────────────────────────
IDX_X        = 0   # Global X position (m)
IDX_Y        = 1   # Global Y position (m)
IDX_PSI      = 2   # Yaw angle (rad)
IDX_VX       = 3   # Longitudinal velocity body frame (m/s)
IDX_VY       = 4   # Lateral velocity body frame (m/s)
IDX_R        = 5   # Yaw rate (rad/s)
IDX_DELTA    = 6   # Actual steering angle after lag (rad)
IDX_A_ACT    = 7   # Actual acceleration after lag (m/s²)
IDX_OMEGA_RL = 8   # Rear-left  wheel spin (rad/s)
IDX_OMEGA_RR = 9   # Rear-right wheel spin (rad/s)
IDX_Z_FL     = 10  # Front-left  suspension deviation from equilibrium (m)
IDX_Z_FR     = 11  # Front-right suspension deviation from equilibrium (m)
IDX_Z_RL     = 12  # Rear-left   suspension deviation from equilibrium (m)
IDX_Z_RR     = 13  # Rear-right  suspension deviation from equilibrium (m)
IDX_DZ_FL    = 14  # Front-left  suspension velocity (m/s)
IDX_DZ_FR    = 15  # Front-right suspension velocity (m/s)
IDX_DZ_RL    = 16  # Rear-left   suspension velocity (m/s)
IDX_DZ_RR    = 17  # Rear-right  suspension velocity (m/s)
IDX_FY_FL    = 18  # Front-left  tyre lateral force after relaxation (N)
IDX_FY_FR    = 19  # Front-right tyre lateral force after relaxation (N)
IDX_FY_RL    = 20  # Rear-left   tyre lateral force after relaxation (N)
IDX_FY_RR    = 21  # Rear-right  tyre lateral force after relaxation (N)
IDX_OMEGA_FL = 22  # Front-left  wheel spin (rad/s)
IDX_OMEGA_FR = 23  # Front-right wheel spin (rad/s)
# Lagged state of FSDS's lateral-acceleration ceiling (m/s² of excess).  Held
# as a state because the measured behaviour is a restoring term that BUILDS
# over time: yaw overshoots the ceiling by ~30% before being pulled back.  A
# memoryless function of the current excess engages instantly and cannot
# overshoot — verified: every (gain, soft) combination gave 0.0% overshoot.
IDX_ALAT_LIM = 24  # FSDS lateral-accel ceiling restoring term (m/s²)
N_STATES     = 25  # Total state vector length


# ─────────────────────────────────────────────────────────────────────────────
# STATE INITIALISATION
# ─────────────────────────────────────────────────────────────────────────────
def init_plant_state(X0, Y0, psi0, vx0=10.0):
    """
    Build a 24-element plant state vector at true static equilibrium.

    "True static equilibrium" means:
      - Wheel speeds set to free-rolling: omega = vx0 / r_eff (no slip)
      - Suspension deviation z_i = 0 at all corners (equilibrium defined
        to include aero at v_nominal, so ddz = 0 exactly at t=0)
      - Tyre relaxation forces = 0 (zero slip angle → zero Fy_ss → no transient)
      - All velocities and rates start at their nominal values with no transients

    Without this initialisation, a simulation started with suspension
    deflection states at zero (the spring-force-only equilibrium ignoring aero)
    would have an initial transient as the suspension compresses to its true
    aero-loaded equilibrium, which pollutes the first ~0.5 s of every rollout.

    Parameters
    ----------
    X0, Y0 : float
        Initial global position (m).
    psi0 : float
        Initial yaw angle (rad).
    vx0 : float, optional
        Initial longitudinal speed (m/s). Defaults to 10.0 m/s.

    Returns
    -------
    np.ndarray, shape (24,)
        Initial state vector, ready to pass to step_nonlinear_plant().

    Called by: gui/simulation.py (simulate_closed_loop),
               tuner/offline_tuner.py (run_headless_rollout)
    """
    s = np.zeros(N_STATES)
    p = VehicleParams()

    s[IDX_X]   = X0
    s[IDX_Y]   = Y0
    s[IDX_PSI] = psi0
    s[IDX_VX]  = vx0

    # Set wheel speeds to free-rolling at vx0 (no slip at start → no wheelspin transient)
    omega_init = vx0 / p.r_eff
    s[IDX_OMEGA_RL] = omega_init
    s[IDX_OMEGA_RR] = omega_init
    s[IDX_OMEGA_FL] = omega_init
    s[IDX_OMEGA_FR] = omega_init

    # Suspension deviations z_i = 0 (states 10-17 already zero from np.zeros).
    # z_eq computed at vx0 places the suspension at the correct operating point.

    # Tyre relaxation states = 0 (states 18-21 already zero).
    # At zero slip angle, Fy_ss = 0, so no relaxation transient.

    return s
