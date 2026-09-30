# Glossary

One-line definitions of the terms the other docs rely on, in alphabetical order, each with where to read more. The first three entries (FSDS, offline rollout, 2D GUI) are the "three simulators" people conflate, and the trust ordering under the table says how far each can be believed.

Symbols are as in the code (`e_y`, `e_psi`, `kappa`). Names in `code font` are identifiers or files.

| Term | Meaning | More |
|---|---|---|
| 2D GUI | The interactive matplotlib tool (`gui/simulation.py`) for drawing or loading a path and watching one run frame by frame. It calls the same rollout as the offline rollout, so its physics is identical, not lower fidelity. Not a validated model of the car. | [offline_guide.md](../guides/offline_guide.md) |
| a_lat | Lateral (sideways) acceleration in m/s². The FSDS sustained ceiling is modelled as `alat_ceiling*`. | [vehicle_physics.md](vehicle_physics.md) |
| Actuator lag | The steering rack and throttle reach a command through a first-order delay (`tau_delta` 0.08 s, `tau_a` 0.02 s). The controllers carry the lagged values as states `delta_act`, `a_act`. | [vehicle_physics.md](vehicle_physics.md) |
| Adaptive gains | Per-tick reweighting of `Q`, `R` and `R_rate` from the current state (corner factor, anti-hunt, adaptive Q-scaling, reversal penalty). LTV-QP only, except the two NMPC rate schedules. | [control_mechanisms.md](control_mechanisms.md) |
| Anti-hunt | Extra `R_rate[0,0]` cost when the car is straight, centred and aligned, to suppress small steering wobble. | [control_mechanisms.md](control_mechanisms.md#steering-rate-anti-hunt-costlier-steering-changes-when-nothing-needs-doing) |
| Arc length `s` | Distance travelled along the reference path. A state in the NMPC, so curvature can be looked up at the predicted position. | [error_states.md](error_states.md) |
| Blend (kinematic/dynamic) | The bicycle model interpolates from a kinematic model below about 1 m/s to a dynamic tyre-force model above about 2.5 m/s (`clip((v_x - 1.0) / 1.5, 0, 1)`), because tyre slip angles are undefined at standstill. | [lmpc.md](../controllers/lmpc.md) |
| Bridge (`fsds_bridge`) | The ROS 2 node that carries FSDS state and commands to and from the AirSim RPC interface. | [ros_integration.md](../fsds/ros_integration.md) |
| Centreline | The planner's estimate of the middle of the track. The default reference line. It beat the optimised raceline on score and steering reversals. | [reference_path_and_speed.md](reference_path_and_speed.md) |
| Chatter (hunting) | Rapid small back-and-forth steering motion, distinct from a sign-flip reversal. | [tuning.md](../guides/tuning.md) |
| CMA-ES | The evolutionary optimiser the offline auto-tuner (`tuner/offline_tuner.py`) uses to search weights. | [offline_guide.md](../guides/offline_guide.md) |
| Composite score | The single number that ranks a rollout, from 13 normalised metrics (`sim/scoring.py`). Lower is better. Runs that fail land above `CONSTRAINT_FLOOR` (10.0). | [tuning.md](../guides/tuning.md) |
| Condensing | Eliminating the predicted states from the NMPC's QP so only the inputs remain as variables (a dense QP). | [nmpc.md](../controllers/nmpc.md) |
| Corner factor | The 0 (straight) to 1 (full corner) fraction from the current `|kappa|`, `1 - 1 / (1 + k |kappa|)`, that blends the LTV-QP weights. | [control_mechanisms.md](control_mechanisms.md#corner-factor-scheduler-ltv-qp-weights-follow-the-current-curvature) |
| Delay compensation | Rolling the measured state forward through commands already issued but not yet reflected in the pose, by `n_delay` steps. | [lmpc.md](../controllers/lmpc.md) |
| DNF | "Did not finish": a rollout that leaves the track, stalls, hits a run of solver failures, or does not cover `COMPLETION_THRESHOLD` (0.98) of the path. Scored above `CONSTRAINT_FLOOR`. | [tuning.md](../guides/tuning.md) |
| `du_max` | The hard per-tick slew limit on steering and acceleration commands: 180 degrees/s times `dt`, and 0.6 m/s² per tick. | [control_mechanisms.md](control_mechanisms.md#slew-rate-limit-du_max-identical-on-both-sides-at-180-degs) |
| `dt` | Control tick period, 0.05 s (20 Hz). | [lmpc.md](../controllers/lmpc.md) |
| `e_psi` | Heading error: car yaw minus reference path direction, wrapped to `(-pi, pi]`. | [error_states.md](error_states.md) |
| `e_v` | Speed error: current speed minus target speed. | [error_states.md](error_states.md) |
| `e_y` | Lateral error: signed perpendicular distance from the path, positive to the left. | [error_states.md](error_states.md) |
| FSDS | The AirSim/Unreal 3D simulator (the outer repo): a real physics engine driving a simulated car. The live ROS 2 stack (`fsae_planning`) drives in it. Closer to the real car than the offline rollout, and itself not confirmed accurate. | [integration_guide.md](../fsds/integration_guide.md) |
| `fsae_autonomous` | The production repo that goes on the real car. It lags the simulation repos by design and receives only deliberate, tested changes. | [offline_live_parity.md](offline_live_parity.md) |
| `fsae_planning` | The ROS 2 workspace (`ros2/src/fsae_planning/`) holding the live planning and control code and the track data. The simulation-side development tree. | [offline_live_parity.md](offline_live_parity.md) |
| Frenet frame | Describing the car by distance along the path plus a perpendicular offset, instead of global X, Y. Both controllers measure error this way. | [error_states.md](error_states.md) |
| Friction ellipse (circle) | A tyre has one shared grip budget: longitudinal force used reduces lateral force available. The plant applies it per wheel. The NMPC hard-constraint version is disabled. | [vehicle_physics.md](vehicle_physics.md) |
| `fsds_simulator/` | The staging mirror of the `fsae_planning` workspace inside this repo. Files present in both must match. It is not a live copy. | [offline_live_parity.md](offline_live_parity.md) |
| Gauss-Newton SQP | The NMPC's solve method: repeatedly linearise the nonlinear model and solve a QP for a step direction. | [nmpc.md](../controllers/nmpc.md) |
| Heading-lead profile | A precomputed shaped reference heading that starts the turn early (`psi_target`). LTV-QP only, off by default. | [control_mechanisms.md](control_mechanisms.md#precomputed-shaped-heading-lead-profile-ltv-qp-only-off) |
| Horizon (`N`) | The number of future steps a controller predicts: 35 for the LTV-QP (1.75 s), 20 for the NMPC (1.0 s). | [lmpc.md](../controllers/lmpc.md), [nmpc.md](../controllers/nmpc.md) |
| Inherit sentinel (`-1.0`) | An `nmpc_*` override left at `-1.0` inherits the LTV-QP field of the same name. | [control_mechanisms.md](control_mechanisms.md#which-settings-affect-which-controller) |
| Jerk cost | A penalty on the second difference of the inputs (`nmpc_rjerk_delta`), which is small for a steady turn and large for a wobble. | [tuning.md](../guides/tuning.md) |
| `kappa` | Path curvature in 1/m (1 / radius). Positive for a left bend. | [error_states.md](error_states.md) |
| `launch_all.sh` | The shell script (`ros2/launch_all.sh`) that starts FSDS, the bridge and the ROS 2 nodes. Its shortlist can override dataclass and YAML defaults. | [integration_guide.md](../fsds/integration_guide.md) |
| LTV-QP (LMPC) | The linear time-varying MPC (`MPCController`): one convex QP per tick against a linear bicycle model in path-error coordinates. | [lmpc.md](../controllers/lmpc.md) |
| Mechanism | An extra layer on a controller's core solve, enabled and tuned independently, targeting one failure mode. | [control_mechanisms.md](control_mechanisms.md) |
| `MPCParams`, `NMPCParams` | The dataclasses (`mpc_params.py`, `nmpc_params.py`) that hold every live tuning weight and flag. `settings/` mirrors them offline. | [offline_live_parity.md](offline_live_parity.md) |
| NMPC | The nonlinear Frenet-frame MPC (`NMPCController`), selected by `use_nmpc`. | [nmpc.md](../controllers/nmpc.md) |
| Offline rollout | The headless closed-loop simulation (`sim/rollout/core.py`) driving the 25-state plant. What the auto-tuner and `python -m tuner.validation.recorded_map_rollout` run. It is what other docs mean by "the offline sim". | [offline_guide.md](../guides/offline_guide.md) |
| Oracle profile | The precomputed path and speed files for an already-recorded track, used in place of the live planner. | [reference_path_and_speed.md](reference_path_and_speed.md) |
| OSQP | The QP solver both controllers use. | [lmpc.md](../controllers/lmpc.md) |
| Pacejka (MF94) | The Magic Formula tyre force model, an S-shaped curve of force against slip. | [vehicle_physics.md](vehicle_physics.md) |
| Parity | The rule that the live and offline copies of any planning or control logic, weight, plant constant or scoring constant stay numerically identical, kept by hand because the two trees cannot import each other. | [offline_live_parity.md](offline_live_parity.md) |
| Plant | The simulated car being controlled (`model/vehicle_physics/`), the ground truth against which controllers are tested. | [vehicle_physics.md](vehicle_physics.md) |
| Pose age | How long ago the pose the controller is solving against was measured. Sets `n_delay`. | [simulator_fidelity.md](simulator_fidelity.md) |
| Precomputed path / speed profile | `raceline.csv`, `centerline.csv` and `speed_profile.csv` for a track, loaded through `map_path` and `USE_PRECOMPUTED_*`. | [reference_path_and_speed.md](reference_path_and_speed.md) |
| `Q`, `R`, `R_rate` | The cost weights: `Q` on tracking error, `R` on control effort, `R_rate` on tick-to-tick change of the control. Raw units, not normalised. | [lmpc.md](../controllers/lmpc.md) |
| QP | Quadratic program: minimise a quadratic cost under linear constraints. Convex, so one global optimum. | [lmpc.md](../controllers/lmpc.md) |
| Raceline | A speed-optimised reference path from `tuner/tools/raceline_optimizer.py`, as opposed to the centreline. | [reference_path_and_speed.md](reference_path_and_speed.md) |
| Real-time iteration (RTI) | One SQP iteration per tick, warm-started from the previous tick, so convergence carries across ticks. | [nmpc.md](../controllers/nmpc.md) |
| Recorded map | A track captured from a real lap (cone map plus derived files), stored in `ros2/src/fsae_planning/tracks/<name>/`. The standard test track is `comp_test_map_3`. | [reference_path_and_speed.md](reference_path_and_speed.md) |
| Reversal | A steering command that changes sign tick to tick. The reversal penalty targets it. | [control_mechanisms.md](control_mechanisms.md#soft-steering-reversal-penalty-costlier-steering-changes-when-steering-is-near-zero) |
| Saturation (steering) | The share of ticks the steering command sits at the mechanical limit (25 degrees). A key sim-to-real gap metric: about 4.8% offline against 21.1% live. | [simulator_fidelity.md](simulator_fidelity.md) |
| `settings/` | The offline package of constants (`general`, `noise`, `planner`, `lmpc`, `nmpc`, `solver`, `scoring`). Consumers use `import settings; settings.X` so runtime overrides reach them. | [offline_live_parity.md](offline_live_parity.md) |
| Sim-to-real gap | The measured difference between offline and live behaviour. Partly explained by FSDS's lateral-acceleration ceiling, partly open. | [simulator_fidelity.md](simulator_fidelity.md) |
| Slack | A penalised relaxation variable that lets a soft constraint (track boundary) be violated at a cost instead of going infeasible. | [nmpc.md](../controllers/nmpc.md) |
| Slew rate | How fast a signal may change. The steering slew limit is `du_max`. | [control_mechanisms.md](control_mechanisms.md#slew-rate-limit-du_max-identical-on-both-sides-at-180-degs) |
| Slip angle, slip ratio | Slip angle is the angle between where a tyre points and where it travels (sideways force). Slip ratio is wheel speed against ground speed (forward and backward force). | [vehicle_physics.md](vehicle_physics.md) |
| Standstill | Near-zero speed, where tyre slip is ill-defined. The NMPC fades tyre force with speed and adds steering damping (`nmpc_standstill_*`). | [nmpc.md](../controllers/nmpc.md) |
| Stanley | The geometric path-tracking controller, used as a reference and in FSDS tests. Has no cost function or prediction. | [stanley.md](../controllers/stanley.md) |
| Telemetry | The per-tick CSV log the live controller writes, one column per signal (`fsae_control/telemetry/`). | [debugging_tools.md](../guides/debugging_tools.md) |
| Three-zone schedule | The NMPC rate-cost multiplier that boosts on straights, eases on approach and drops mid-corner (`nmpc_rrate_zone_*`). | [control_mechanisms.md](control_mechanisms.md#three-zone-rate-schedule-nmpc_rrate_zone_enabled) |
| Trust region | A per-iteration cap on how far the SQP may move the inputs (`nmpc_trust_delta_rad`, `nmpc_trust_a`). | [nmpc.md](../controllers/nmpc.md) |
| Turn-in | The moment steering begins toward a corner. "Late turn-in" is the failure the NMPC was built to fix. | [error_states.md](error_states.md) |
| Understeer, oversteer | Understeer: the front loses grip first and the car pushes wide. Oversteer: the rear steps out. | [vehicle_physics.md](vehicle_physics.md) |
| Unsprung mass | Wheels, tyres and uprights, which sit below the springs. | [vehicle_physics.md](vehicle_physics.md) |
| Warm start | Starting the solver from the previous tick's solution shifted one step. | [nmpc.md](../controllers/nmpc.md) |
| Yaw, yaw rate | Rotation of the car about a vertical axis. Yaw rate `r` is how fast that rotation is happening, in rad/s. | [vehicle_physics.md](vehicle_physics.md) |
| ZOH | Zero-order hold: the exact discretisation of the continuous linear model over one tick, holding the input constant. The LTV-QP uses it. The NMPC uses RK4. | [lmpc.md](../controllers/lmpc.md) |

## How far each simulator can be trusted

- **The offline rollout and the 2D GUI** are rough validation only. They check that the control maths behaves sensibly and get a weight set into the right ballpark. They have never been matched against reality and carry no measured accuracy figure.
- **FSDS** is a materially closer approximation, since it is a real physics engine, and it is mainly used to test and validate new features. It is also not confirmed accurate against the real car.

From least to most representative of the real car, none confirmed accurate:

```
2D GUI = offline rollout   <   FSDS   <   real car
(same physics as offline)      (real physics engine, still not validated)
```

Concrete measured gaps, on the same map and gains (steering saturation 4.8% offline against 21.1% live, and a partly modelled lateral-acceleration ceiling) are in [simulator_fidelity.md](simulator_fidelity.md). Validate any planning or control change in FSDS at minimum, and on the car ideally, before trusting an offline-only result.
