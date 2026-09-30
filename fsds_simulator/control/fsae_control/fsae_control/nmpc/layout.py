"""
fsae_control/nmpc/layout.py — NMPC state/input/output layout

IDX_* state indices, NX/NU/NH_* sizes, finite-difference step sizes and the
Frenet denominator guard. Every other nmpc module indexes through these.
"""

import numpy as np


# ── State/input/output layout ────────────────────────────────────────────
IDX_S     = 0   # arc length along the reference path (m)


IDX_EY    = 1   # lateral deviation, + = front axle LEFT of the path (m)


IDX_EPSI  = 2   # heading error, car_yaw - path_yaw, wrapped (rad)


IDX_VX    = 3   # body-frame forward speed (m/s)


IDX_VY    = 4   # body-frame lateral speed (m/s)


IDX_R     = 5   # yaw rate (rad/s)


IDX_DELTA = 6   # actuator-lagged steering angle (rad)


IDX_A     = 7   # actuator-lagged acceleration (m/s^2)


NX = 8


NU = 2


# h() row 4 is EITHER the two-sided speed-error residual (v_x - v_ref, the
# original NH=5 layout) OR, when NMPCController.progress_enabled is True, a
# one-sided speed-CAP hinge (see _outputs' docstring) plus one extra
# progress-reward row -- NH becomes 6 in that case. Both layouts are fixed
# for the lifetime of a controller instance (set once in __init__, not
# per-tick), so w_out/NH never disagree mid-run.
NH_TRACKING = 5   # e_y, e_y_dot, e_psi, e_psi_dot, e_v (original, default)


NH_PROGRESS = 6   # e_y, e_y_dot, e_psi, e_psi_dot, speed-cap hinge, progress


# Extra _outputs()/_output_jacobians() rows appended ONLY when
# NMPCController.friction_circle_enabled (NMPCParams.nmpc_friction_circle_enabled)
# is True: F_yf, F_yr (front/rear axle lateral tyre force, N). Rides along
# through the SAME finite-difference Jacobian pass that produces C for the
# cost rows above, at zero extra rollout cost -- see _outputs()'s docstring.
# NEVER weighted into the cost (w_out has NH entries, not NH+NH_FRICTION);
# used only to build the friction-circle QP constraint rows in _solve_step.
NH_FRICTION = 2


# Finite-difference perturbations, one per state then per input. Sized per
# variable so every column of the Jacobian has a comparable truncation/roundoff
# balance: eps_j ~ 1e-6 * (typical magnitude of that variable). Verified
# against central differences in Part 16 §16.5.
_FD_EPS_X = np.array([1e-6, 1e-6, 1e-7, 1e-6, 1e-6, 1e-7, 1e-7, 1e-6])


_FD_EPS_U = np.array([1e-7, 1e-6])


# Guard on the Frenet denominator (1 - kappa*e_y). It is singular at
# e_y = 1/kappa; on this car that is 4.8 m at the tightest logged corner
# (kappa 0.21) against a 3.5 m track half-width, so this floor is inert in
# normal operation and only prevents a sign flip if the car is somehow far
# off-track on a very tight bend. Not a tunable.
_DENOM_FLOOR = 0.25


def _wrap(a):
    """Wrap an angle (or array of angles) to [-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))
