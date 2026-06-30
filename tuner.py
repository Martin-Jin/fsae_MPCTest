"""
Cost-Weight Auto-Tuner
File Name: tuner.py

Runs the closed-loop (nonlinear plant + MPC) simulation N times against a
fixed reference path and scores tracking performance via RMSE of lateral
and heading error. Performs a simple randomized coordinate-search local
optimization over the diagonal Q, R, and R_rate weights, since the
closed-loop RMSE is not differentiable / not cheaply gradient-accessible
(it requires solving an MPC QP at every timestep of every rollout).

Each call to `optimize_weights` runs `num_runs` rollouts to evaluate the
*current* weights (to get a robust score, since initial conditions are
randomized slightly) , then proposes and evaluates a perturbed candidate,
accepting it if it improves the RMSE. This is a simple stochastic
hill-climber -- appropriate here since each evaluation is expensive
(a full MPC closed-loop rollout), so we cannot afford many candidates per
call. Calling "Optimise" repeatedly continues the search from the last
accepted weights, so results compound across button presses.
"""
import numpy as np


# Indices into the 8-element Q diagonal / 2-element R diagonal / 2-element
# R_rate diagonal that we are willing to auto-tune. We deliberately leave
# the actuator-regularization entries (indices 6, 7 of Q) fixed and small,
# since those exist only to keep the QP well-posed, not to be optimized
# for tracking performance.
TUNABLE_Q_IDX = [0, 1, 2, 3, 4]   # e_y, e_y_dot, e_psi, e_psi_dot, e_v
TUNABLE_R_IDX = [0, 1]           # delta_cmd, a_cmd
TUNABLE_R_RATE_IDX = [0, 1]      # d(delta_cmd)/dt, d(a_cmd)/dt

Q_BOUNDS = [(10.0, 3000.0), (5.0, 1000.0), (50.0, 8000.0), (200.0, 10000.0), (0.1, 200.0)]
R_BOUNDS = [(5.0, 2000.0), (1.0, 200.0)]
R_RATE_BOUNDS = [(1.0, 5000.0), (0.1, 200.0)]


def rmse_score(history):
    """RMSE over lateral error (m) and heading error (rad, weighted to put
    it on a comparable scale to meters) across one rollout.

    Rollouts that failed (solver gave up repeatedly, or the car went
    off-track) or that otherwise didn't reach the end of the path are
    penalized on top of their raw tracking RMSE, scaled by how little of
    the path they actually covered. Without this, a rollout that crashes
    out early can look BETTER than one that runs the full path honestly,
    simply because it accumulated fewer error samples before quitting --
    which would make the auto-tuner actively favor unstable weight
    combinations. FAIL_PENALTY is large enough that even a rollout which
    fails almost immediately (completion_frac near 0) scores far worse
    than the worst realistic honest-completion RMSE.
    """
    e_y = np.array(history["e_y"])
    e_psi = np.array(history["e_psi"])
    if len(e_y) == 0:
        return 1e6

    lateral_rmse = np.sqrt(np.mean(e_y**2))
    # Heading error in radians is numerically small; scale by a typical
    # lever-arm-like factor (~2m) so heading tracking actually matters in
    # the combined score instead of being swamped by lateral error.
    heading_rmse = np.sqrt(np.mean(e_psi**2)) * 2.0
    base_score = lateral_rmse + heading_rmse

    failed = history.get("failed", False)
    completion_frac = history.get("completion_frac", 1.0)

    if failed or completion_frac < 1.0:
        FAIL_PENALTY = 50.0  # meters-equivalent; far above any realistic honest RMSE
        shortfall = 1.0 - completion_frac  # 0 = ran full length, 1 = failed immediately
        return base_score + FAIL_PENALTY * shortfall

    return base_score


def weights_to_vector(Q, R, R_rate):
    return np.array(
        [Q[i, i] for i in TUNABLE_Q_IDX]
        + [R[i, i] for i in TUNABLE_R_IDX]
        + [R_rate[i, i] for i in TUNABLE_R_RATE_IDX]
    )


def vector_to_weights(vec, Q_template, R_template, R_rate_template):
    Q = Q_template.copy()
    R = R_template.copy()
    R_rate = R_rate_template.copy()
    n_q = len(TUNABLE_Q_IDX)
    n_r = len(TUNABLE_R_IDX)
    for j, i in enumerate(TUNABLE_Q_IDX):
        Q[i, i] = vec[j]
    for j, i in enumerate(TUNABLE_R_IDX):
        R[i, i] = vec[n_q + j]
    for j, i in enumerate(TUNABLE_R_RATE_IDX):
        R_rate[i, i] = vec[n_q + n_r + j]
    return Q, R, R_rate


def evaluate_weights(Q, R, R_rate, run_rollout_fn, num_runs, log_fn=print):
    """
    run_rollout_fn(Q, R, R_rate, seed) -> history dict (as produced by the
    sim engine). Returns mean RMSE across num_runs rollouts (with slightly
    randomized initial conditions per run, handled by run_rollout_fn via
    seed).
    """
    scores = []
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
    """Perturb each tunable weight by up to +/- step_frac (multiplicatively,
    in log-space so weights spanning orders of magnitude are searched
    sensibly), clipped to bounds."""
    new_vec = current_vec.copy()
    for i in range(len(new_vec)):
        log_val = np.log(current_vec[i])
        log_val += rng.uniform(-step_frac, step_frac)
        val = np.exp(log_val)
        lo, hi = bounds[i]
        new_vec[i] = np.clip(val, lo, hi)
    return new_vec


def optimize_weights(Q, R, R_rate, run_rollout_fn, num_runs=10, rng_seed=None, log_fn=print):
    """
    One "Optimise" button press worth of work:
      1. Evaluate current weights over num_runs rollouts -> baseline RMSE.
      2. Propose one perturbed candidate (now spanning Q, R, and R_rate),
         evaluate it over num_runs rollouts.
      3. Keep whichever is better.

    Returns: (best_Q, best_R, best_R_rate, info_dict)
    """
    bounds = Q_BOUNDS + R_BOUNDS + R_RATE_BOUNDS
    rng = np.random.default_rng(rng_seed)

    current_vec = weights_to_vector(Q, R, R_rate)
    baseline_rmse, baseline_scores = evaluate_weights(Q, R, R_rate, run_rollout_fn, num_runs, log_fn=log_fn)
    log_fn(f"[Optimiser] Baseline RMSE over {num_runs} runs: {baseline_rmse:.4f} m "
           f"(min {min(baseline_scores):.4f}, max {max(baseline_scores):.4f})")

    candidate_vec = propose_candidate(current_vec, bounds, rng)
    cand_Q, cand_R, cand_R_rate = vector_to_weights(candidate_vec, Q, R, R_rate)
    cand_rmse, cand_scores = evaluate_weights(cand_Q, cand_R, cand_R_rate, run_rollout_fn, num_runs, log_fn=log_fn)
    log_fn(f"[Optimiser] Candidate RMSE over {num_runs} runs: {cand_rmse:.4f} m "
           f"(min {min(cand_scores):.4f}, max {max(cand_scores):.4f})")

    if cand_rmse < baseline_rmse:
        log_fn(f"[Optimiser] Candidate IMPROVED tracking ({baseline_rmse:.4f} -> "
               f"{cand_rmse:.4f} m). Applying new weights.")
        best_Q, best_R, best_R_rate, best_rmse = cand_Q, cand_R, cand_R_rate, cand_rmse
        accepted = True
    else:
        log_fn(f"[Optimiser] Candidate did not improve on baseline. Keeping current weights.")
        best_Q, best_R, best_R_rate, best_rmse = Q, R, R_rate, baseline_rmse
        accepted = False

    def fmt_diag(M, idxs, names):
        return ", ".join(f"{n}={M[i, i]:.2f}" for i, n in zip(idxs, names))

    q_names = ["e_y", "e_y_dot", "e_psi", "e_psi_dot", "e_v"]
    r_names = ["delta_cmd", "a_cmd"]
    r_rate_names = ["d_delta_cmd", "d_a_cmd"]
    log_fn("[Optimiser] Current weights -> "
           + fmt_diag(best_Q, TUNABLE_Q_IDX, q_names) + " | "
           + fmt_diag(best_R, TUNABLE_R_IDX, r_names) + " | "
           + fmt_diag(best_R_rate, TUNABLE_R_RATE_IDX, r_rate_names))

    info = {
        "baseline_rmse": baseline_rmse,
        "candidate_rmse": cand_rmse,
        "best_rmse": best_rmse,
        "accepted": accepted,
    }
    return best_Q, best_R, best_R_rate, info

# def optimize_weights(Q, R, R_rate, run_rollout_fn, num_runs=5, rng_seed=None, log_fn=print):
    """
    Replaces the local hill-climber with a global Differential Evolution algorithm.
    It actively explores the parameter space rather than guessing randomly.
    """
    bounds = Q_BOUNDS + R_BOUNDS + R_RATE_BOUNDS
    
    # Track the best baseline to ensure we don't regress
    baseline_rmse, _ = evaluate_weights(Q, R, R_rate, run_rollout_fn, num_runs, log_fn=lambda x: None)
    log_fn(f"[Optimiser] Baseline RMSE over {num_runs} runs: {baseline_rmse:.4f} m")
    
    def objective_fn(vec):
        # Decode vector back into Q, R, R_rate matrices
        cand_Q, cand_R, cand_R_rate = vector_to_weights(vec, Q, R, R_rate)
        
        # Evaluate candidate (passing a dummy log_fn to avoid console spam)
        rmse, _ = evaluate_weights(cand_Q, cand_R, cand_R_rate, run_rollout_fn, num_runs, log_fn=lambda x: None)
        return rmse

    log_fn("[Optimiser] Running Differential Evolution (this may take a moment)...")
    
    # Run evolutionary search. Keeps population low for interactive GUI speeds.
    result = differential_evolution(
        objective_fn, 
        bounds, 
        maxiter=3,       
        popsize=4,       
        seed=rng_seed,
        workers=1        
    )
    
    best_vec = result.x
    cand_rmse = result.fun
    
    if cand_rmse < baseline_rmse:
        log_fn(f"[Optimiser] DE found IMPROVED tracking ({baseline_rmse:.4f} -> {cand_rmse:.4f} m).")
        best_Q, best_R, best_R_rate = vector_to_weights(best_vec, Q, R, R_rate)
        accepted = True
        best_rmse = cand_rmse
    else:
        log_fn(f"[Optimiser] DE did not improve on baseline. Keeping current weights.")
        best_Q, best_R, best_R_rate = Q, R, R_rate
        accepted = False
        best_rmse = baseline_rmse

    info = {
        "baseline_rmse": baseline_rmse,
        "candidate_rmse": cand_rmse,
        "best_rmse": best_rmse,
        "accepted": accepted,
    }
    
    return best_Q, best_R, best_R_rate, info