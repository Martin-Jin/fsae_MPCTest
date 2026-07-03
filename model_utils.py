import numpy as np

def curvature_estimate(state):
    """Simple yaw-rate / speed curvature proxy from the plant state vector."""
    vx = max(state[3], 0.5)
    r = state[5]
    return abs(r / vx)

def adaptive_R_rate(kappa, R_rate_base):
    """Curvature-dependent steering jerk softening."""
    R = np.array(R_rate_base, copy=True)
    scale = max(0.35, 1 / (1 + 3 * kappa))
    R[0, 0] *= scale
    return R


def adaptive_R_scaling(vx, R_base):
    """Speed-dependent steering cost shaping with a saturating scale."""
    vx = max(vx, 0.5)
    A = 1.5
    vx_half = 6.0
    steer_scale = 1.0 + (A * vx) / (vx_half + vx)
    accel_scale = 1.0 + 0.05 * vx
    R_scaled = np.array(R_base, copy=True)
    R_scaled[0, 0] *= steer_scale
    R_scaled[1, 1] *= accel_scale
    return R_scaled
