"""
sim/rollout/delay.py — delay-compensation and tracking-error phases of
run_core_rollout(), lifted out of its loop so the loop itself reads as a
sequence of named steps.

Stateless: any value the loop carries from one tick to the next
(command_queue, ...) is passed in and returned explicitly rather than
stored, so the loop in sim/rollout/core.py remains the one place that owns
rollout state.

settings.py constants are bound here by name at import time, the same as
elsewhere in this package. Callers that override settings
(tuner/investigations/steering_chatter_check.py) must do so before the
first import of sim.rollout.core, which imports this module.
"""

import numpy as np

from model.vehicle_physics import plant_to_tracking_error

import settings

_PREDICT_EPSI_CLIP = 0.5   # rad (~28.6°) — small-angle bound, see predict_ahead below



def predict_ahead(x0, Ad, Bd, pending_cmds):
    """
    Roll the linear error-state model forward through commands already
    committed but not yet applied to the plant, so the MPC solves against
    the state it will actually face when its new output takes effect
    instead of the stale current state (delay compensation).

    pending_cmds must be ordered oldest-first (the order they will be
    applied to the plant). Same x_p = Ad @ x_p + Bd @ u mechanics as the
    horizon-prediction preview below.

    Ad's e_psi -> e_y_dot coupling is the kinematic relation e_y_dot ~= vx *
    sin(e_psi), linearised to vx * e_psi (bicycle_model.py). That's only
    valid for small e_psi (sin(x) ~= x). Unlike the closed-loop MPC horizon
    (which re-measures every real step), this rollforward is open-loop over
    several steps with no ground-truth correction in between, so a large
    e_psi here (sharp corner + a perturbed initial heading) compounds every
    step instead of getting corrected — observed to blow up e_y_dot and
    saturate steering on PATH_SUDDEN_TURN. Clip e_psi to a small-angle range
    before each step's matrix multiply so the rollforward can't leave the
    regime the linear model is actually valid in; the real (unclipped) e_psi
    is still what the QP solves against afterwards via x0_mpc.
    """
    x_p = x0.copy()
    for u in pending_cmds:
        x_p[2] = np.clip(x_p[2], -_PREDICT_EPSI_CLIP, _PREDICT_EPSI_CLIP)
        x_p = Ad @ x_p + Bd @ u
    return x_p


def true_tracking_error(state, e_y, e_psi, path_X, path_Y, path_Psi, diverged):
    """
    Ground-truth tracking error, for scoring and the off-track check only.

    The controller's e_y/e_psi are not where the car actually is whenever its
    reference differs from the true path. Scoring must use ground truth,
    otherwise a car could score well by tracking its own wrong belief, the
    exact asymmetry real perception/localisation error has. Always measured
    against the true reference path, never the planner's centreline.

    `diverged` must be True whenever EITHER source of divergence is active:
      1. SLAM noise:  the pose fed to the tracking-error helper is corrupted.
      2. use_planner: the REFERENCE is the planner's cone-derived, FOV-limited,
                      EMA-blended centreline, so e_y is a distance to an
                      estimated line even with a perfect pose.
    Case 2 must count even with SLAM noise off (the default): otherwise
    e_y_true would alias the planner-relative error, so most of the score
    (rmse + peak_lateral_error) would measure controller-vs-planner agreement
    with no ground-truth anchor, and a drifting planner would read as good
    tracking while also suppressing the off-track trigger.
    """
    if diverged:
        e_y_true, _, e_psi_true, _, _, _, _ = plant_to_tracking_error(
            state, path_x=path_X, path_y=path_Y, path_psi=path_Psi
        )
        return e_y_true, e_psi_true
    return e_y, e_psi


def believed_pending_cmds(command_queue, delay_rng, u_prev):
    """
    Commands the CONTROLLER believes are still in transit to the plant.

    command_queue[0] is applied to the plant THIS step; everything after it
    (DELAY_STEPS commands) is already committed and will land before a new
    solve's output ever reaches the plant, so the controller rolls its state
    forward through them (see settings.py DELAY_STEPS note).

    With DELAY_JITTER_STEPS > 0, only the controller's BELIEF about how many
    commands are in flight is perturbed; the queue itself (and so the plant's
    real lag) is untouched. Rounding a Gaussian gives the live failure mode:
    the estimate is usually right, occasionally off by a step, which is what
    makes x0 jump between rollforward depths on the real car. Draws exactly
    one sample from `delay_rng` per call, so call it once per tick.
    """
    pending_cmds = list(command_queue)[1:]
    if settings.DELAY_JITTER_STEPS > 0.0:
        n_true = len(pending_cmds)
        n_believed = int(round(n_true + delay_rng.normal(0.0, settings.DELAY_JITTER_STEPS)))
        # Cap over-estimates at the live MAX_DELAY_COMPENSATION_STEPS
        # equivalent so a tail draw can't roll forward absurdly far.
        n_believed = int(np.clip(n_believed, 0, max(n_true, 0) + 2))
        if n_believed <= n_true:
            pending_cmds = pending_cmds[n_true - n_believed:] if n_believed else []
        else:
            # Over-estimating: the controller thinks more commands are in
            # flight than there are, so it rolls forward through the
            # oldest one extra times — the same over-compensation a
            # too-large pose_age_s produces live.
            pad = n_believed - n_true
            oldest = pending_cmds[0] if pending_cmds else u_prev
            pending_cmds = [oldest] * pad + pending_cmds
    return pending_cmds
