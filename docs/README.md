# Documentation index

Find the doc for a task, then read that doc.
Files under `docs/logs/` are a frozen investigation record, see [logs/README.md](logs/README.md).

## To do X, read Y

| To | Read |
|---|---|
| Get set up and understand the project from scratch | [guides/getting_started.md](guides/getting_started.md) |
| Run the offline simulator, the tuner or the GUI | [guides/offline_guide.md](guides/offline_guide.md) |
| Retune MPC weights or gains | [guides/tuning.md](guides/tuning.md) |
| Diagnose a bad run or pick a debugging tool | [guides/debugging_tools.md](guides/debugging_tools.md) |
| Understand the LTV-QP MPC | [controllers/lmpc.md](controllers/lmpc.md) |
| Understand the nonlinear MPC | [controllers/nmpc.md](controllers/nmpc.md) |
| Understand the Stanley controller | [controllers/stanley.md](controllers/stanley.md) |
| See how the whole offline system fits together | [reference/architecture.md](reference/architecture.md) |
| Keep offline and live MPC numerically identical | [reference/offline_live_parity.md](reference/offline_live_parity.md) |
| Look up a control mechanism and its tuning knobs | [reference/control_mechanisms.md](reference/control_mechanisms.md) |
| Check a mechanism that was tried and retired | [reference/retired_mechanisms.md](reference/retired_mechanisms.md) |
| Understand the tracking error definitions | [reference/error_states.md](reference/error_states.md) |
| Understand the reference path and speed profile | [reference/reference_path_and_speed.md](reference/reference_path_and_speed.md) |
| Judge how far to trust the offline sim against the car | [reference/simulator_fidelity.md](reference/simulator_fidelity.md) |
| Understand the vehicle plant model | [reference/vehicle_physics.md](reference/vehicle_physics.md) |
| Look up a term | [reference/glossary.md](reference/glossary.md) |
| Set up FSDS and connect the ROS 2 bridge | [fsds/integration_guide.md](fsds/integration_guide.md), [fsds/ros_integration.md](fsds/ros_integration.md), [fsds/settings.md](fsds/settings.md) |
| Find which file does what in the offline repo | [modules/offline_sim.md](modules/offline_sim.md) |
| Find which file does what in the ROS 2 workspace, and how the mirror workflow works | [modules/fsds_ros2.md](modules/fsds_ros2.md) |
| Read the history behind a decision | [logs/README.md](logs/README.md) |

## Layout

- `guides/`: task-oriented how-tos.
- `controllers/`: one doc per controller.
- `reference/`: how the system works and why.
- `fsds/`: simulator and bridge setup.
- `modules/`: file-by-file references.
- `logs/`: frozen investigation record.
