"""
model/vehicle_physics/ — High-Fidelity Nonlinear Vehicle Plant Model

PURPOSE
-------
Implements a 25-state nonlinear vehicle dynamics simulation plant intended to
approach the fidelity of Nvidia PhysX (used by FSDS/AirSim); this intent is not
the same as a confirmed match, see docs/logs/sim_to_real_investigation.md for
the open, only-partially-closed gap to the real car. This is the "truth" model
relative to the MPC's own internal model: the MPC controller
(controller/lmpc/solve.py) uses a much simpler 8-state linear model internally,
creating a deliberate plant-model mismatch that mirrors the real-world
situation.

The plant is stepped at 20 Hz (dt=0.05 s) but internally sub-steps at 4×
(h=0.0125 s) to maintain numerical stability through the stiff suspension
dynamics and tyre relaxation lag.

STATE VECTOR (24 elements)
--------------------------
Indices 0-7 are identical to the MPC's 8-state linear model so that
gui/simulation.py and tuner/offline_tuner.py can read positions/velocities/actuator
states without any index remapping.

  [0]  X            Global position X (m)
  [1]  Y            Global position Y (m)
  [2]  psi          Yaw angle (rad)
  [3]  vx           Longitudinal velocity, body frame (m/s)
  [4]  vy           Lateral velocity, body frame (m/s)
  [5]  r            Yaw rate (rad/s)
  [6]  delta_act    Actual steering angle after first-order lag (rad)
  [7]  a_act        Actual acceleration command after first-order lag (m/s²)
  [8]  omega_RL     Rear-left  wheel angular velocity (rad/s)
  [9]  omega_RR     Rear-right wheel angular velocity (rad/s)
  [10] z_FL         Front-left  suspension deflection from equilibrium (m)
  [11] z_FR         Front-right suspension deflection from equilibrium (m)
  [12] z_RL         Rear-left   suspension deflection from equilibrium (m)
  [13] z_RR         Rear-right  suspension deflection from equilibrium (m)
  [14] dz_FL_dt     Front-left  suspension velocity (m/s)
  [15] dz_FR_dt     Front-right suspension velocity (m/s)
  [16] dz_RL_dt     Rear-left   suspension velocity (m/s)
  [17] dz_RR_dt     Rear-right  suspension velocity (m/s)
  [18] Fy_FL_rlx    Front-left  tyre lateral force after relaxation lag (N)
  [19] Fy_FR_rlx    Front-right tyre lateral force after relaxation lag (N)
  [20] Fy_RL_rlx    Rear-left   tyre lateral force after relaxation lag (N)
  [21] Fy_RR_rlx    Rear-right  tyre lateral force after relaxation lag (N)
  [22] omega_FL     Front-left  wheel angular velocity (rad/s)
  [23] omega_FR     Front-right wheel angular velocity (rad/s)

SUSPENSION DEFLECTION CONVENTION
---------------------------------
z[i] = 0 at the static + aero equilibrium position at a nominal cruise speed.
Spring force on chassis from corner i:  F_spring_i = k_i * (z_eq_i + z_i)
At equilibrium: k_i * z_eq_i = Fz_static_i  so storing only the deviation z[i]
gives a spring-force floor of the static load without double-counting.
The unsprung mass EOM is then:  m_us * ddz_i = −k_i*z_i − c_i*dz_i − F_arb_i

PHYSICS FEATURES
----------------
  1. Full MF94 Pacejka lateral + longitudinal tyre model (B, C, D, E, Sv, Sh, camber)
  2. Per-wheel longitudinal slip ratio and wheel spin dynamics
  3. Per-corner spring / damper / anti-roll-bar suspension with dynamic Fz
  4. Split front/rear aerodynamic downforce with pitch sensitivity
  5. Tyre relaxation length (first-order lag on lateral force)
  6. Optional torque vectoring (tv_gain parameter; default 0 = disabled)
  7. Kinematic camber gain (suspension deflection → camber angle → Pacejka D shift)
  8. Road-surface mu scaling (road_mu parameter; default 1.0 = dry tarmac)

LAYOUT
------
  state.py       IDX_* / N_STATES and init_plant_state()
  params.py      VehicleParams
  tyres.py       Pacejka MF94 lateral and longitudinal force
  plant_step.py  step_nonlinear_plant()
  tracking.py    reference lookup and plant_to_tracking_error()

USED BY
-------
  gui/simulation.py   — calls step_nonlinear_plant() at every simulation timestep;
                        calls init_plant_state() to initialise the vehicle.
  tuner/offline_tuner.py — calls step_nonlinear_plant() and init_plant_state() in
                           run_headless_rollout() for each CMA-ES candidate evaluation.

DOES NOT USE
------------
  model/bicycle_model.py, controller/lmpc/solve.py, sim/speed_profile.py, sim/perception.py, sim/planner.py, tuner/performance_stats.py
"""
from model.vehicle_physics.state import (
    IDX_X,
    IDX_Y,
    IDX_PSI,
    IDX_VX,
    IDX_VY,
    IDX_R,
    IDX_DELTA,
    IDX_A_ACT,
    IDX_OMEGA_RL,
    IDX_OMEGA_RR,
    IDX_Z_FL,
    IDX_Z_FR,
    IDX_Z_RL,
    IDX_Z_RR,
    IDX_DZ_FL,
    IDX_DZ_FR,
    IDX_DZ_RL,
    IDX_DZ_RR,
    IDX_FY_FL,
    IDX_FY_FR,
    IDX_FY_RL,
    IDX_FY_RR,
    IDX_OMEGA_FL,
    IDX_OMEGA_FR,
    IDX_ALAT_LIM,
    N_STATES,
    init_plant_state,
)
from model.vehicle_physics.params import VehicleParams
from model.vehicle_physics.tyres import pacejka_lateral_mf94, pacejka_longitudinal_mf94
from model.vehicle_physics.plant_step import step_nonlinear_plant
from model.vehicle_physics.tracking import (
    find_closest_reference_bounded,
    get_interpolated_ref_point,
    plant_to_tracking_error,
)

__all__ = [
    "IDX_X",
    "IDX_Y",
    "IDX_PSI",
    "IDX_VX",
    "IDX_VY",
    "IDX_R",
    "IDX_DELTA",
    "IDX_A_ACT",
    "IDX_OMEGA_RL",
    "IDX_OMEGA_RR",
    "IDX_Z_FL",
    "IDX_Z_FR",
    "IDX_Z_RL",
    "IDX_Z_RR",
    "IDX_DZ_FL",
    "IDX_DZ_FR",
    "IDX_DZ_RL",
    "IDX_DZ_RR",
    "IDX_FY_FL",
    "IDX_FY_FR",
    "IDX_FY_RL",
    "IDX_FY_RR",
    "IDX_OMEGA_FL",
    "IDX_OMEGA_FR",
    "IDX_ALAT_LIM",
    "N_STATES",
    "VehicleParams",
    "init_plant_state",
    "step_nonlinear_plant",
    "pacejka_lateral_mf94",
    "pacejka_longitudinal_mf94",
    "find_closest_reference_bounded",
    "get_interpolated_ref_point",
    "plant_to_tracking_error",
]
