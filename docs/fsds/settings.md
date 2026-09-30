# FSDS and Live Settings: Where a Value Comes From

This doc covers the live (ROS 2) settings surface: how MPC and NMPC weights and flags are declared, and which of three places decides the value a running controller uses. It is not a tuning guide. For what each weight does see [tuning.md](../guides/tuning.md). For the offline equivalent, the `settings/` package, see [architecture.md](../reference/architecture.md). For the meaning of "FSDS" and "offline" see [glossary.md](../reference/glossary.md).

## Summary

- Every tuning field is declared once, as a dataclass field with metadata. YAML defaults, launch arguments and ROS parameters are generated from or checked against it.
- Under `ros2 launch` the value that runs is the launch argument. `ros2/launch_all.sh` sets a few of them. The dataclass default alone does not tell what is live.
- The live and offline sides are kept numerically identical by hand. See [offline_live_parity.md](../reference/offline_live_parity.md) for the rule and the field-by-field table.

## Declaring a field once

`MPCParams` (69 fields) and `NMPCParams` (35 fields, a separate sibling dataclass, not a subclass) hold 104 fields in total. They live in `mpc/mpc_params.py` and `mpc/nmpc_params.py` under `ros2/src/fsae_planning/control/fsae_control/fsae_control/`. Each field carries a default and metadata:

```python
q_e_y: float = field(default=6.4, metadata={
    "unit": "1/m^2",
    "desc": "lateral deviation from path centreline",
    "controller": "both",
})
```

Counts drift as fields are added. Recount with `dataclasses.fields()` before quoting them.

Three consumers read the declaration:

1. **ROS parameters on the node.** `declare_mpc_params(node)` and `declare_nmpc_params(node)` declare every field as a parameter. `mpc_params_from_node(node)` and `nmpc_params_from_node(node)` read them back into fresh `MPCParams` and `NMPCParams` instances at node construction. `MPCController` (`lmpc/controller.py`) and `NMPCController` (`nmpc/solver.py`) receive those instances.
2. **`fsae_params.yaml`.** The `controller:` block in `common/fsae_bringup/config/fsae_params.yaml` lists every field. All 104 defaults match the dataclasses now. The same block also holds `v_max`, `v_min`, `stanley_gain` and the dynamic-speed-cap fields, which are not `MPCParams` fields.
3. **Launch arguments.** `control.launch.py` and `sim.launch.py` build one `DeclareLaunchArgument` per field from `MPC_PARAM_FIELDS` and `NMPC_PARAM_FIELDS`. Adding a field to a dataclass is enough to get a launch argument. Names cannot drift apart. Values can, by design, which is what the next section covers.

## Which value runs

Order of precedence, lowest to highest:

1. The dataclass default.
2. `fsae_params.yaml`.
3. A launch argument, either typed on the `ros2 launch` line or forwarded by `launch_all.sh`.

Under `ros2 launch` in this workspace, layer 3 always applies. `control.launch.py` forwards every field to the node as a launch value whose default is the dataclass default, and lists those values after the YAML file in the node's parameters. A ROS 2 launch parameter dictionary listed later overrides an earlier source. An edit made only to the YAML block therefore has no effect on a launch-started node. Change the dataclass default as well, or pass a launch argument. This follows from the code order and was not run live.

`tuner/tools/sync_mpc_params.py` copies the three live parameter files (`mpc_params.py`, `nmpc_params.py`, `fsae_params.yaml`) one way from `fsae_planning` to `fsae_autonomous` and the `fsds_simulator/` mirror. See [debugging_tools.md](../guides/debugging_tools.md).

## What `launch_all.sh` overrides

`ros2/launch_all.sh` is the day-to-day launcher. A helper `_append_mpc_arg field value` adds `field:=value` to the launch command only when the shell variable is non-empty. A variable left unset changes nothing.

The active (uncommented) lines differ from the dataclass or launch defaults as follows:

| Variable | Value in script | Default it overrides |
|---|---|---|
| `USE_NMPC` | `true` | `use_nmpc` dataclass default `false` |
| `REVERSAL_PENALTY_ENABLED` | `true` | `reversal_penalty_enabled` default `false` |
| `MPC_ADAPTIVE_Q_SCALING_ENABLED` | `false` | `adaptive_q_scaling_enabled` default `true` |
| `ENABLE_DYNAMIC_SPEED_CAP` | `false` | launch and YAML default `true` |
| `V_MAX` | `20.0` | launch and YAML default `15.0` |

These active lines set the same value as the dataclass default: `V_MIN=1.5`, `NMPC_SLACK_LINEAR_WEIGHT=500.0`, `NMPC_CORNER_RRATE_BLEND_ENABLED=false`, `NMPC_CORNER_FACTOR_K=27.0`, `NMPC_RRATE_ZONE_ENABLED=true` with its three endpoints (`2.0`, `0.80`, `0.15`), `NMPC_RJERK_DELTA=150.0` and `NMPC_REVERSAL_PENALTY_ENABLED=false`.

- **Top of the script**: `CONTROLLER=mpc`, `STANDALONE_OUTPUT=true`, `V_MAX`, `V_MIN` and `USE_NMPC`, plus the track and precomputed-data toggles.
- **The tuning shortlist**: a longer block further down, mostly commented out, covering the commonly retuned weights and gains, the adaptive-gain and corner-factor fields, the dynamic speed cap and a large NMPC set. The shortlist changes as tuning proceeds. Read the script for the current state, not this table.
- **`V_MAX` does not reach the precomputed-speed branch.** With `use_precomputed_speed` on, the CSV's own top speed is the car's top speed. A speed cap for a test means swapping the CSV. See [reference_path_and_speed.md](../reference/reference_path_and_speed.md).

Why three layers exist (dataclass, YAML and launch argument, instead of one) is not recorded. The practical roles are: the dataclass is the authoritative default, the YAML is a deployment file, and a launch argument is a per-run override.

## Perception feeding the live planner

`sim_perception` (`fsae_sim_perception`) filters FSDS's cone ground truth by field of view, box and radius. It publishes `left_track` and `right_track`, `cone_detection` and `car_position`/`car_odom` on separate timers. `pose_rate` defaults to 20 Hz and `cone_rate` to 10 Hz. One planner (`centerline_planner` or `skidpad_planner`, chosen by the `planner` launch argument) turns them into the centreline the controller tracks. Its behaviour is set by the node parameters in `fsae_params.yaml`, not by anything in this doc.

The offline side has its own reimplementation, `SimPerception` in `sim/perception.py` and `SimPlanner` in `sim/planner.py`, gated by `USE_PLANNER`. It exists so a bug reproduced offline is a perception or planning bug and not a simulator artefact. See [architecture.md](../reference/architecture.md).
