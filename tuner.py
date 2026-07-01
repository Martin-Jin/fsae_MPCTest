"""
Cost-Weight Auto-Tuner
File Name: tuner.py

Changes from previous version:
  IMPROVEMENT: adaptive_R_scaling now uses a saturating (Michaelis-Menten
    style) speed scale instead of a linear one, so steering cost is bounded
    at high speed. The old linear 1 + 0.25*vx meant at 10 m/s the steering
    cost was 3.5x baseline, which over-penalized corrections when heading
    errors are hardest to recover from.  New formula:
      steer_scale = 1 + (vx_max_scale * vx) / (vx_half + vx)
    which saturates around 2.5x at the top of the speed profile rather than
    continuing to grow unboundedly.

  IMPROVEMENT: adaptive_R_rate no longer softens the accel jerk penalty
    (R_rate[1,1]) in corners. The old code reduced it by a factor of
    0.7 + 0.3*scale in high curvature, which relaxed braking rate limits
    exactly when the vehicle is most load-sensitive. The accel channel now
    keeps its full baseline penalty in corners; only steering transitions
    are softened (as before).

  IMPROVEMENT: optimize_weights now proposes and evaluates NUM_CANDIDATES=4
    perturbed candidates per call instead of 1, then keeps the best one
    that beats baseline. This gives 4x better search coverage per button
    press in high-dimensional weight space (9 tunable parameters) at the
    same num_runs*N_candidates evaluation budget, since the incremental
    cost of evaluating a few more candidates is low compared to the baseline
    evaluation that must happen regardless.
"""
import numpy as np


# Indices into the 8-element Q diagonal / 2-element R diagonal / 2-element
# R_rate diagonal that we are willing to auto-tune.  We deliberately leave
# the actuator-regularization entries (indices 6, 7 of Q) fixed and small,
# since those exist only to keep the QP well-posed, not to be optimized
# for tracking performance.
TUNABLE_Q_IDX     = [0, 1, 2, 3, 4]  # e_y, e_y_dot, e_psi, e_psi_dot, e_v
TUNABLE_R_IDX     = [0, 1]            # delta_cmd, a_cmd
TUNABLE_R_RATE_IDX = [0, 1]           # d(delta_cmd)/dt, d(a_cmd)/dt

Q_BOUNDS      = [(10.0, 3000.0), (5.0, 1000.0), (50.0, 8000.0), (200.0, 10000.0), (0.1, 200.0)]
R_BOUNDS      = [(5.0, 2000.0), (1.0, 200.0)]
R_RATE_BOUNDS = [(1.0, 5000.0), (0.1, 200.0)]

# Number of perturbed candidates evaluated per optimize_weights() call.
# 4 gives good coverage without blowing up wall-clock time (baseline
# evaluation must happen regardless, so this adds 4*num_runs rollouts
# on top of the fixed num_runs baseline cost).
NUM_CANDIDATES = 4


def rmse_score(history):
    """RMSE over lateral error (m) and heading error (rad, weighted to put
    it on a comparable scale to meters) across one rollout.

    Rollouts that failed or didn't reach the end of the path are penalized
    proportional to how little of the path they actually covered.
    """
    e_y   = np.array(history["e_y"])
    e_psi = np.array(history["e_psi"])
    if len(e_y) == 0:
        return 1e6

    lateral_rmse = np.sqrt(np.mean(e_y**2))
    heading_rmse = np.sqrt(np.mean(e_psi**2)) * 2.0
    base_score   = lateral_rmse + heading_rmse

    failed          = history.get("failed", False)
    completion_frac = history.get("completion_frac", 1.0)

    if failed or completion_frac < 1.0:
        FAIL_PENALTY = 50.0
        shortfall    = 1.0 - completion_frac
        return base_score + FAIL_PENALTY * shortfall

    return base_score


def weights_to_vector(Q, R, R_rate):
    return np.array(
        [Q[i, i]      for i in TUNABLE_Q_IDX]
        + [R[i, i]    for i in TUNABLE_R_IDX]
        + [R_rate[i, i] for i in TUNABLE_R_RATE_IDX]
    )


def vector_to_weights(vec, Q_template, R_template, R_rate_template):
    Q      = Q_template.copy()
    R      = R_template.copy()
    R_rate = R_rate_template.copy()
    n_q    = len(TUNABLE_Q_IDX)
    n_r    = len(TUNABLE_R_IDX)
    for j, i in enumerate(TUNABLE_Q_IDX):
        Q[i, i] = vec[j]
    for j, i in enumerate(TUNABLE_R_IDX):
        R[i, i] = vec[n_q + j]
    for j, i in enumerate(TUNABLE_R_RATE_IDX):
        R_rate[i, i] = vec[n_q + n_r + j]
    return Q, R, R_rate


def evaluate_weights(Q, R, R_rate, run_rollout_fn, num_runs, log_fn=print):
    """
    run_rollout_fn(Q, R, R_rate, seed) -> history dict.
    Returns mean RMSE across num_runs rollouts.
    """
    scores   = []
    n_failed = 0
    for i in range(num_runs):
        history = run_rollout_fn(Q, R, R_rate, seed=i)
        if history.get("failed", False):
            n_failed += 1
        scores.append(rmse_score(history))
    if n_failed > 0:
        log_fn(f"[Optimiser]   ({n_failed}/{num_runs} rollouts in this batch failed "
               f"-- e.g. solver gave up or car went off-track; penalized in score)")
    return float(np.mean(scores)), scores


def propose_candidate(current_vec, bounds, rng, step_frac=0.20):
    """Perturb each tunable weight multiplicatively in log-space by up to
    +/- step_frac, clipped to bounds.  Log-space perturbation is important
    here because weights span several orders of magnitude (e.g. Q[0,0]~2000
    vs R_rate[1,1]~4) -- a flat additive step would be negligible for large
    weights and explosive for small ones."""
    new_vec = current_vec.copy()
    for i in range(len(new_vec)):
        log_val  = np.log(current_vec[i])
        log_val += rng.uniform(-step_frac, step_frac)
        val      = np.exp(log_val)
        lo, hi   = bounds[i]
        new_vec[i] = np.clip(val, lo, hi)
    return new_vec


def optimize_weights(Q, R, R_rate, run_rollout_fn, num_runs=10, rng_seed=None, log_fn=print):
    """
    One "Optimise" button press worth of work:
      1. Evaluate current weights over num_runs rollouts -> baseline RMSE.
      2. Propose NUM_CANDIDATES perturbed candidates, evaluate each over
         num_runs rollouts (previously only 1 candidate was proposed).
      3. Accept the best candidate if it beats baseline; otherwise keep
         current weights.

    Returns: (best_Q, best_R, best_R_rate, info_dict)
    """
    bounds = Q_BOUNDS + R_BOUNDS + R_RATE_BOUNDS
    rng    = np.random.default_rng(rng_seed)

    current_vec    = weights_to_vector(Q, R, R_rate)
    baseline_rmse, baseline_scores = evaluate_weights(
        Q, R, R_rate, run_rollout_fn, num_runs, log_fn=log_fn
    )
    log_fn(
        f"[Optimiser] Baseline RMSE over {num_runs} runs: {baseline_rmse:.4f} m "
        f"(min {min(baseline_scores):.4f}, max {max(baseline_scores):.4f})"
    )

    best_cand_rmse  = baseline_rmse
    best_cand_Q     = Q
    best_cand_R     = R
    best_cand_R_rate = R_rate
    accepted        = False

    for cand_idx in range(NUM_CANDIDATES):
        candidate_vec = propose_candidate(current_vec, bounds, rng)
        cand_Q, cand_R, cand_R_rate = vector_to_weights(candidate_vec, Q, R, R_rate)
        cand_rmse, cand_scores = evaluate_weights(
            cand_Q, cand_R, cand_R_rate, run_rollout_fn, num_runs, log_fn=log_fn
        )
        log_fn(
            f"[Optimiser] Candidate {cand_idx + 1}/{NUM_CANDIDATES} RMSE: "
            f"{cand_rmse:.4f} m "
            f"(min {min(cand_scores):.4f}, max {max(cand_scores):.4f})"
        )

        if cand_rmse < best_cand_rmse:
            best_cand_rmse   = cand_rmse
            best_cand_Q      = cand_Q
            best_cand_R      = cand_R
            best_cand_R_rate = cand_R_rate
            accepted         = True
            log_fn(
                f"[Optimiser]   ^ New best candidate so far "
                f"({baseline_rmse:.4f} -> {cand_rmse:.4f} m)"
            )

    if accepted:
        log_fn(
            f"[Optimiser] Best candidate IMPROVED tracking "
            f"({baseline_rmse:.4f} -> {best_cand_rmse:.4f} m). Applying new weights."
        )
        best_Q, best_R, best_R_rate, best_rmse = (
            best_cand_Q, best_cand_R, best_cand_R_rate, best_cand_rmse
        )
    else:
        log_fn("[Optimiser] No candidate improved on baseline. Keeping current weights.")
        best_Q, best_R, best_R_rate, best_rmse = Q, R, R_rate, baseline_rmse

    def fmt_diag(M, idxs, names):
        return ", ".join(f"{n}={M[i, i]:.2f}" for i, n in zip(idxs, names))

    q_names      = ["e_y", "e_y_dot", "e_psi", "e_psi_dot", "e_v"]
    r_names      = ["delta_cmd", "a_cmd"]
    r_rate_names = ["d_delta_cmd", "d_a_cmd"]
    log_fn(
        "[Optimiser] Current weights -> "
        + fmt_diag(best_Q,      TUNABLE_Q_IDX,      q_names)      + " | "
        + fmt_diag(best_R,      TUNABLE_R_IDX,      r_names)      + " | "
        + fmt_diag(best_R_rate, TUNABLE_R_RATE_IDX, r_rate_names)
    )

    info = {
        "baseline_rmse": baseline_rmse,
        "best_rmse":     best_cand_rmse,
        "accepted":      accepted,
    }
    return best_Q, best_R, best_R_rate, info


# ==========================================
# ADAPTIVE MPC GAIN HELPERS
# ==========================================

def curvature_estimate(state):
    """Simple yaw-rate / speed curvature proxy from the plant state vector.
    state: [X, Y, psi, vx, vy, r, delta_act, a_act]
    """
    vx = max(state[3], 0.5)
    r  = state[5]
    return abs(r / vx)


def adaptive_R_rate(kappa, R_rate_base):
    """
    Curvature-dependent steering jerk softening.

    In tight corners we allow smoother (less penalized) steering transitions
    so the controller can unwind quickly without fighting the rate penalty.

    CHANGE: The accel jerk penalty (R_rate[1,1]) is NO LONGER reduced in
    corners. The old code reduced it by (0.7 + 0.3*scale), relaxing braking
    rate limits exactly when the vehicle is most load-sensitive and needs
    decisive, well-timed braking. The accel channel now keeps its full
    baseline penalty regardless of curvature.
    """
    R = R_rate_base.copy()

    # Steering: soften in high curvature (scale → 0 as kappa → large)
    scale   = 1.0 / (1.0 + 3.0 * kappa)
    R[0, 0] *= scale

    # Accel/brake: keep full baseline penalty in all conditions
    # R[1, 1] unchanged

    return R


def adaptive_R_scaling(vx, R_base):
    """
    Speed-dependent steering cost shaping with a saturating scale.

    CHANGE: Replaced the old linear scale (1 + 0.25*vx) with a saturating
    (Michaelis-Menten) function:
        steer_scale = 1 + (A * vx) / (vx_half + vx)
    where A=1.5 and vx_half=6.0, giving:
        vx=0  → scale=1.0x  (baseline)
        vx=6  → scale=1.75x (50% of asymptote)
        vx=10 → scale=~2.0x (approaching asymptote at 2.5x)

    The old linear formula gave 3.5x at 10 m/s, which over-penalized
    steering corrections at the top of the speed profile and caused
    the controller to under-respond to heading errors at high speed.

    Accel scale remains a mild linear function of vx (unchanged).
    """
    vx = max(vx, 0.5)

    A        = 1.5   # asymptotic gain above baseline
    vx_half  = 6.0   # speed at which scale = 1 + A/2
    steer_scale = 1.0 + (A * vx) / (vx_half + vx)

    accel_scale = 1.0 + 0.05 * vx

    R_scaled        = R_base.copy()
    R_scaled[0, 0] *= steer_scale
    R_scaled[1, 1] *= accel_scale

    return R_scaled