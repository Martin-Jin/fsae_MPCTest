"""
controller/nmpc/layout.py — NMPC state/input/output index layout and the
numerical constants shared by the model, the cost rows and the solver.
Identical to the live `nmpc/solver.py`'s module-level constants.
"""

import numpy as np

from angles import wrap_angle as _wrap  # noqa: F401 (re-exported, see module list below)


# ── State/input/output layout (identical to the live module) ────────────
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
# NMPCController.friction_circle_enabled is True: F_yf, F_yr (front/rear
# axle lateral tyre force, N). Rides along through the SAME
# finite-difference Jacobian pass that produces C for the cost rows above,
# at zero extra rollout cost -- see _outputs()'s docstring. NEVER weighted
# into the cost (w_out has NH entries, not NH+NH_FRICTION); used only to
# build the friction-circle QP constraint rows in _solve_step.
NH_FRICTION = 2

_FD_EPS_X = np.array([1e-6, 1e-6, 1e-7, 1e-6, 1e-6, 1e-7, 1e-7, 1e-6])
_FD_EPS_U = np.array([1e-7, 1e-6])

# Guard on the Frenet denominator (1 - kappa*e_y): singular at e_y = 1/kappa.
# Inert on any real track line (see the live module's identical comment).
_DENOM_FLOOR = 0.25
