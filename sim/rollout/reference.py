"""
sim/rollout/reference.py — the reference/tracking-error phase of
run_core_rollout(), lifted out of its loop so the loop itself reads as a
sequence of named steps.

Stateless: any value the loop carries from one tick to the next
(ref_psi_prev, ...) is passed in and returned explicitly rather than
stored, so the loop in sim/rollout/core.py remains the one place that owns
rollout state.

settings constants are read here as settings.X at call time, so a runtime
override is honoured. Only default arguments (in sim/rollout/core.py) bind at
import. Callers that override settings
(tuner/investigations/steering_chatter_check.py) should still do so before the
first import of sim.rollout.core, which imports this module.
"""

import numpy as np

from angles import wrap_angle as _normalize_angle  # noqa: F401 (re-exported, see sim/rollout/core.py)
from model.vehicle_physics import plant_to_tracking_error

import settings


def _rate_limit_ref_psi(ref_psi_raw, ref_psi_prev, max_rate_rad_per_s, dt):
    """
    Cap how fast the reference heading (ref_psi) is allowed to change per
    tick, same shape as SPEED_TARGET_RISE_RATE's cap on v_target.

    Why this exists: most of the planner's reference-heading swing is real
    track geometry, but a tail-concentrated excess — the reference correctly
    anticipating a sharp corner earlier than the car has actually yawed — is
    strongly linked to steering saturation. Limiting only the RATE (never
    the final direction — once the car catches up, the raw reference is
    reached again) trades slightly later turn-in commitment for not asking
    the controller to snap onto a heading the car has no chance of reaching
    yet.

    Symmetric (limits swings in either direction) — unlike
    SPEED_TARGET_RISE_RATE, which only limits increases because slowing down
    is always safe. There is no equivalent "always safe" direction for a
    heading reference: swinging the target toward straight ahead just as
    hard as toward the apex can be equally premature relative to where the
    car has actually turned.

    Parameters
    ----------
    ref_psi_raw : float
        This tick's actual reference heading (rad), unwrapped-compatible with
        ref_psi_prev (i.e. already continuous, not wrapped to [-pi, pi]).
    ref_psi_prev : float or None
        Previous tick's LIMITED reference heading. None on the first tick
        after start/reset, in which case the raw value passes through
        unlimited (mirrors v_des_prev's None handling).
    max_rate_rad_per_s : float
        Maximum |d(ref_psi)/dt|, rad/s.
    dt : float
        Tick period, s.

    Returns
    -------
    float — the limited reference heading (rad, unwrapped-compatible).
    """
    if ref_psi_prev is None:
        return ref_psi_raw
    max_step = max_rate_rad_per_s * dt
    delta = _normalize_angle(ref_psi_raw - ref_psi_prev)
    delta = np.clip(delta, -max_step, max_step)
    return ref_psi_prev + delta


def compute_reference(
    use_planner, perception, planner, cone_noise, pose_age_ticks,
    state, state_est, X_est, Y_est, psi_est, car_pos_np,
    path_X, path_Y, path_Psi, ref_psi_prev, history,
):
    """
    The CONTROLLER's view of tracking error, from the live planner's
    centreline when one is ready, otherwise the oracle path.

    Returns (e_y, e_psi, rpsi, planner_cl, ref_psi_prev, cl_idx). `rpsi` is
    None unless the planner branch produced one. `planner_cl` is the planner
    centreline this tick's error was measured against, or None when the
    oracle path was used (the NMPC must track the same source). `cl_idx` is
    the nearest-point index into `planner_cl` (None when `planner_cl` is
    None), reused by compute_speed_target() so it isn't recomputed twice per
    tick. Appends the planner-centreline snapshot to `history` when it is
    not None.
    """
    rpsi = None
    planner_cl = None
    cl_idx = None
    if use_planner:
        # Skip perception/planning entirely while the pose is held. On the
        # car, a stalled pose feed stalls everything downstream of it: the
        # planner is triggered by car_position, so no new pose means no new
        # centreline AND no new tracking error. Re-planning here from a
        # frozen pose would still hand the controller a subtly different
        # centreline each tick (the fit is not a pure function of pose), so
        # e_y would keep changing and the controller would never actually
        # be blind — which is exactly what the first version of this model
        # got wrong (measured: e_y repeated on 0.0% of ticks instead of the
        # intended ~5%).
        if pose_age_ticks == 0:
            b_vis, y_vis = perception.visible_cones(X_est, Y_est, psi_est)
            if cone_noise is not None:
                b_vis, y_vis = cone_noise.corrupt(b_vis), cone_noise.corrupt(y_vis)
            planner.update(b_vis, y_vis, car_pos_np, psi_est)

        cl = planner.centreline
        if cl is not None and len(cl) >= 2:
            planner_cl = cl
            cl_x, cl_y = cl[:, 0], cl[:, 1]
            cl_psi = np.zeros_like(cl_x)
            cl_psi[:-1] = np.arctan2(np.diff(cl_y), np.diff(cl_x))
            cl_psi[-1] = cl_psi[-2] if len(cl_psi) > 1 else state[2]

            e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                state_est, path_x=cl_x, path_y=cl_y, path_psi=cl_psi
            )
            rpsi = psi_est - e_psi

            # ── Reference-heading rate limit (settings.REF_HEADING_RATE_LIMIT_ENABLED) ──
            # See _rate_limit_ref_psi's own docstring for the mechanism.
            # Only applied here (the live planner branch) — the
            # fallback/oracle branches below reference path_X/path_Y/
            # path_Psi, the fixed geometric path that does NOT carry this
            # excess, so there is nothing to limit there.
            if settings.REF_HEADING_RATE_LIMIT_ENABLED:
                rpsi_limited = _rate_limit_ref_psi(
                    rpsi, ref_psi_prev, np.radians(settings.REF_HEADING_RISE_RATE), settings.DT
                )
                ref_psi_prev = rpsi_limited
                e_psi = _normalize_angle(psi_est - rpsi_limited)
                rpsi = rpsi_limited
            else:
                ref_psi_prev = rpsi

            dists = np.linalg.norm(cl - car_pos_np, axis=1)
            cl_idx = int(np.argmin(dists))

            if history is not None:
                history["planner_X"].append(cl_x)
                history["planner_Y"].append(cl_y)
        else:
            # Planner not yet ready — fall back to the global reference path.
            # Without this fallback, e_y/e_psi would silently reuse stale
            # values from the previous step whenever the planner isn't ready.
            e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
                state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
            )

            if history is not None:
                # No planner centreline yet this step — record an empty
                # snapshot rather than skipping the index, so history["planner_X"]
                # stays aligned index-for-index with history["X"]/pred_X.
                history["planner_X"].append(np.empty(0))
                history["planner_Y"].append(np.empty(0))
    else:
        e_y, _, e_psi, _, _, _, _ = plant_to_tracking_error(
            state_est, path_x=path_X, path_y=path_Y, path_psi=path_Psi
        )

    return e_y, e_psi, rpsi, planner_cl, ref_psi_prev, cl_idx
