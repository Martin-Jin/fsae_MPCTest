# Language: python
# Title: Complete Windows-Safe Headless Auto-Tuner (offline_tuner.py)
import numpy as np
import multiprocessing as mp
import time
from scipy.optimize import differential_evolution

# Import bounds and decoding logic from your existing tuner module
from tuner import Q_BOUNDS, R_BOUNDS, R_RATE_BOUNDS, vector_to_weights

# Import physics, model, and optimiser for the simulation loop
from vehicle_physics import VehicleParams, step_nonlinear_plant, init_plant_state
from model import get_8state_discrete_model
from optimiser import solve_mpc

# Module-level dictionary to share initial parameters safely across processes
_init_context = {}

def init_worker(Q_init, R_init, R_rate_init):
    """
    Runs immediately when a new worker process is spawned.
    Explicitly populates the global memory context for that child process.
    """
    global _init_context
    _init_context['Q'] = Q_init
    _init_context['R'] = R_init
    _init_context['R_rate'] = R_rate_init

# ==========================================
# HEADLESS SIMULATION ENVIRONMENT
# ==========================================
def run_headless_rollout(weights_vector, num_steps=300):
    """
    Runs a purely mathematical, GUI-free simulation rollout for a given weight candidate.
    Returns the RMSE of the tracking error.
    """
    # These will now safely resolve inside the child workers
    Q_init = _init_context['Q']
    R_init = _init_context['R']
    R_rate_init = _init_context['R_rate']
    
    Q, R, R_rate = vector_to_weights(weights_vector, Q_init, R_init, R_rate_init)
    
    p = VehicleParams()
    dt = 0.05
    N = 25
    u_min, u_max = [-0.4, -10.0], [0.4, 4.0]
    
    # Re-seed random generation state per process to diversify rollouts
    np.random.seed()
    
    # Corrected: Match the (X0, Y0, psi0, vx0) signature from vehicle_physics.py
    X0_rand = np.random.uniform(-0.1, 0.1)
    Y0_rand = np.random.uniform(-0.1, 0.1)
    state = init_plant_state(X0_rand, Y0_rand, 0.0, vx0=7.0)
    
    error_sq_sum = 0.0
    u_prev = np.array([0.0, 0.0]) # Initialized to zero actuator states
    
    for step in range(num_steps):
        v_x = max(state[3], 0.5)
        Ad, Bd = get_8state_discrete_model(v_x, dt)
        
        # Tracking error calculation (assuming straight-line trajectory tracking)
        e_y = state[1]
        e_y_dot = state[4]
        e_psi = state[2]
        e_psi_dot = state[5]
        e_v = state[3] - 7.0
        
        x0_mpc = np.array([e_y, e_y_dot, e_psi, e_psi_dot, e_v, state[7], state[6], state[7]])
        
        u_opt = solve_mpc(x0_mpc, Ad, Bd, N, Q, R, u_min, u_max, R_rate=R_rate, u_prev=u_prev)
        
        if u_opt is None:
            return 999.0  # Heavy penalty for infeasible/failed solves
            
        u_prev = u_opt
        
        # Corrected: Pass u_opt as a unified command array, followed by dt then params
        state = step_nonlinear_plant(state, u_opt, dt, p)
        
        error_sq_sum += (state[1]**2) + (state[2]**2)

    return np.sqrt(error_sq_sum / num_steps)

# ==========================================
# PICKLEABLE OBJECTIVE WRAPPERS
# ==========================================
def evaluate_candidate(vec):
    """
    Evaluates a candidate vector over a few iterations to get an average fitness score.
    Sitting cleanly at the root module level so Windows workers can map it.
    """
    num_runs = 3
    rmses = [run_headless_rollout(vec) for _ in range(num_runs)]
    return np.mean(rmses)

# ==========================================
# MAIN EXECUTION
# ==========================================
if __name__ == '__main__':
    # Define Baseline Matrices
    Q_init = np.diag([10.0, 1.0, 5.0, 0.5, 1.0, 0.0, 0.0, 0.0])
    R_init = np.diag([1.0, 0.1])
    R_rate_init = np.diag([10.0, 1.0])
    
    # Save parameters locally to the host parent context
    _init_context['Q'] = Q_init
    _init_context['R'] = R_init
    _init_context['R_rate'] = R_rate_init
    
    bounds = Q_BOUNDS + R_BOUNDS + R_RATE_BOUNDS
    
    print("[Offline Tuner] Initializing Differential Evolution (Headless)...")
    start_time = time.time()
    
    num_cores = max(1, mp.cpu_count() - 1)
    print(f"[Offline Tuner] Firing up {num_cores} parallel workers.")
    
    # Use initializer and initargs to seed the context parameters directly into child memory blocks
    with mp.Pool(processes=num_cores, initializer=init_worker, initargs=(Q_init, R_init, R_rate_init)) as pool:
        result = differential_evolution(
            evaluate_candidate, 
            bounds, 
            strategy='best1bin',
            maxiter=20,       
            popsize=15,       
            mutation=(0.5, 1.0),
            recombination=0.7,
            disp=True,           
            updating='deferred', 
            workers=pool.map     
        )
    
    end_time = time.time()
    
    # Process final optimal weights
    best_vec = result.x
    best_Q, best_R, best_R_rate = vector_to_weights(best_vec, Q_init, R_init, R_rate_init)
    
    print("\n" + "="*40)
    print(f"OPTIMIZATION COMPLETE in {(end_time - start_time)/60:.2f} minutes.")
    print(f"Best RMSE Achieved: {result.fun:.4f} m")
    print("="*40)
    print("Replace your simulation weights with the following:")
    print("Q_diag =", np.diag(best_Q).tolist())
    print("R_diag =", np.diag(best_R).tolist())
    print("R_rate_diag =", np.diag(best_R_rate).tolist())
    print("="*40)