import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node

# fsae_control is an installed package by the time `ros2 launch` generates
# this file (same as any node import), so this resolves the same way
# mpc_core.py's own `from fsae_control.mpc_params import ...` does -- no
# relative path back to src/ needed, unlike map_path's hardcoded absolute
# default below (which points at data, not code on the Python path).
from fsae_control.mpc_params import MPC_PARAM_FIELDS


# Control subsystem: a path-tracking controller + the FSDS command bridge.
# The controller is selectable with `controller:=stanley|mpc|mpc_standalone`.
#   stanley, mpc       — publish the shared cmd_vel interface; fsds_bridge
#                         converts it (speed/steering -> ControlCommand) and
#                         owns GO-gating + cone e-braking, identically for both.
#   mpc_standalone     — publishes fs_msgs/ControlCommand directly, using the
#                         MPC's own throttle/brake instead of fsds_bridge's
#                         speed-error P-loop (preserves the offline-tuned
#                         longitudinal behaviour from the fsae_MPCTest repo).
#                         Owns GO-gating + cone e-braking itself, so
#                         fsds_bridge is skipped for this mode — see
#                         mpc_controller_standalone.py's module docstring.
# The skidpad planner drives the car itself (it publishes cmd_vel directly),
# so the controller is skipped in skidpad mode; fsds_bridge still runs then
# (skidpad also uses the shared cmd_vel interface).
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('fsae_bringup'), 'config', 'fsae_params.yaml'
    )
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    log_csv = LaunchConfiguration('log_csv')
    log_dir = LaunchConfiguration('log_dir')
    map_path = LaunchConfiguration('map_path')
    use_precomputed_speed = LaunchConfiguration('use_precomputed_speed')
    path_map_path = LaunchConfiguration('path_map_path')
    use_precomputed_path = LaunchConfiguration('use_precomputed_path')
    v_max = LaunchConfiguration('v_max')
    v_min = LaunchConfiguration('v_min')
    stanley_gain = LaunchConfiguration('stanley_gain')
    enable_dynamic_speed_cap = LaunchConfiguration('enable_dynamic_speed_cap')
    dynamic_cap_a_lat_max = LaunchConfiguration('dynamic_cap_a_lat_max')
    dynamic_cap_safety = LaunchConfiguration('dynamic_cap_safety')
    # Every MPCController tuning field (Q/R/R_rate weights, adaptive-gain
    # shape constants, feature flags) -- see mpc_params.py's MPCParams for
    # the authoritative field list/defaults/units. Generated from
    # MPC_PARAM_FIELDS rather than hand-written so this launch file, the
    # dataclass, and fsae_params.yaml's defaults can't silently drift against
    # each other (56 near-identical hand-written args is itself a drift
    # risk -- see the plan this was built from).
    mpc_param_configs = {
        name: LaunchConfiguration(name) for name, _default, _meta in MPC_PARAM_FIELDS
    }
    # Effective map_path handed to the node: '' whenever the feature is
    # switched off, regardless of what map_path itself is set to -- so
    # use_precomputed_speed:=false is a reliable one-flag disable without
    # having to also clear map_path (map_path alone doubles as "where's the
    # file" and, implicitly, "is this on"; this makes "is this on" explicit).
    # IfElseSubstitution (not PythonExpression) deliberately: map_path is an
    # arbitrary filesystem path that could contain backslashes (Windows) or
    # quotes, which would corrupt/break a PythonExpression string built by
    # concatenating it into Python source text -- IfElseSubstitution passes
    # it through as data instead of evaluating it.
    effective_map_path = IfElseSubstitution(
        EqualsSubstitution(use_precomputed_speed, 'true'),
        map_path,
        '',
    )
    # Same pattern as effective_map_path, for the path-import toggle.
    effective_path_map_path = IfElseSubstitution(
        EqualsSubstitution(use_precomputed_path, 'true'),
        path_map_path,
        '',
    )
    # Map the friendly name to the package entry point; node name stays
    # 'controller' either way so all three read the `controller:` params block.
    controller_exec = PythonExpression([
        "'mpc_controller_standalone' if '", controller, "' == 'mpc_standalone' "
        "else 'mpc_controller' if '", controller, "' == 'mpc' "
        "else 'controller'"
    ])
    # fsds_bridge is redundant (and would race mpc_standalone for
    # /fsds/control_command) when the standalone MPC node is selected.
    run_bridge = IfCondition(
        PythonExpression(["'", controller, "' != 'mpc_standalone'"])
    )
    # path_map_path is an MPC-only param (mpc_controller.py / mpc_controller_standalone.py
    # both declare_parameters it; stanley_controller.py does not track a static
    # path override) -- passing it unconditionally would raise
    # ParameterNotDeclaredException on the default controller:=stanley. map_path
    # (the speed profile) IS declared by stanley_controller.py too -- see its
    # own map_path param -- so both Node() entries below receive it, letting a
    # Stanley run and an MPC run on the same track share the identical speed
    # target for a directly comparable telemetry CSV. Kept as two Node()
    # entries (rather than branching one entry's params) because path_map_path
    # still can't go to Stanley.
    run_controller_mpc = IfCondition(
        PythonExpression([
            "'", planner, "' != 'skidpad_planner' and '", controller,
            "' in ('mpc', 'mpc_standalone')"
        ])
    )
    run_controller_non_mpc = IfCondition(
        PythonExpression([
            "'", planner, "' != 'skidpad_planner' and '", controller,
            "' not in ('mpc', 'mpc_standalone')"
        ])
    )

    # One DeclareLaunchArgument per MPCParams field, default matching
    # fsae_params.yaml's controller block (and MPCParams' own default)
    # exactly -- leaving all of these unset on the command line changes
    # nothing, same guarantee v_max/v_min/stanley_gain already give.
    def _mpc_launch_arg(name, default, meta):
        unit = meta.get('unit', '')
        desc = meta.get('desc', '')
        suffix = f' ({unit})' if unit and unit != 'unitless' else ''
        if isinstance(default, bool):
            default_str = 'true' if default else 'false'
        else:
            default_str = str(default)
        return DeclareLaunchArgument(
            name, default_value=default_str,
            description=f'{desc}{suffix} (overrides fsae_params.yaml controller.{name})',
        )

    mpc_launch_args = [
        _mpc_launch_arg(name, default, meta) for name, default, meta in MPC_PARAM_FIELDS
    ]

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='centerline_planner'),
        DeclareLaunchArgument(
            'controller', default_value='stanley',
            description='stanley | mpc | mpc_standalone — path-tracking controller to run'),
        DeclareLaunchArgument(
            'log_csv', default_value='false',
            description='Write controller CSV telemetry (e_y/e_psi/steer/...) to log_dir'),
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description="Controller CSV telemetry output dir ('' -> ~/fsae_logs)"),
        DeclareLaunchArgument(
            # Default points into THIS repo's own tracks/<name>/ -- committed
            # data, not runtime-generated output, so a fresh clone of FSDS +
            # fsae_planning alone (no fsae_MPCTest checkout) can drive
            # comp_test_map_3 immediately with zero setup. fsae_MPCTest is
            # only where NEW tracks get produced (recording + the two
            # exporters); its output is meant to be copied into this
            # directory afterwards, mirroring the workflow the other
            # direction (see fsae_MPCTest/docs/developer_guide.md).
            #
            # To drive a DIFFERENT (already-committed) track, don't edit this
            # line: set TRACK= in ros2/launch_all.sh, which expands to
            # map_path/path_map_path for both args at once. This default is
            # only the fallback for a bare
            # `ros2 launch fsae_bringup control.launch.py`.
            #
            # Hardcoded absolute path, not derived from __file__ or
            # get_package_share_directory(): both resolve to the INSTALLED
            # copy under ros2/install/... at runtime (confirmed: this launch
            # file is itself copied there by colcon build), which has no
            # relationship to this file's location in src/ -- there is no
            # ROS-visible path back to a source-tree sibling directory. This
            # matches WHERE launch_all.sh runs `ros2 launch` FROM (inside
            # WSL/the Docker container, not Windows) -- update this line if
            # the repo root ever moves.
            'map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/ros2/src/fsae_planning/tracks/comp_test_map_3/speed_profile.csv',
            description=(
                "Path to a CSV exported from a recorded cone map, committed "
                "under this repo's own tracks/<name>/ so a fresh FSDS + "
                "fsae_planning clone can use it immediately -- no "
                "fsae_MPCTest checkout required to READ it (only to produce "
                "a NEW one; see tracks/README or fsae_MPCTest's "
                "tuner/tools/export_speed_profile.py). Has no effect unless "
                "use_precomputed_speed:=true. Applies to `mpc`, "
                "`mpc_standalone`, AND `stanley` (all three declare this "
                "param) -- letting Stanley and MPC runs on the same track "
                "share the identical speed target. If the file doesn't "
                "exist, the node logs an error at startup and falls back to "
                "live curvature_speed() -- it does not crash."
            )),
        DeclareLaunchArgument(
            'use_precomputed_speed', default_value='true',
            description=(
                "true (default) -> look up the target speed from map_path's "
                "precomputed oracle profile instead of live curvature_speed() "
                "every tick. Only valid for a track that's already been fully "
                "mapped -- see mpc_controller_standalone.py's map_path param. "
                "Set to false here (or on the command line) to go back to "
                "unchanged live curvature_speed() behaviour regardless of "
                "map_path."
            )),
        DeclareLaunchArgument(
            # Same x,y,psi,v_target file FORMAT as map_path, but a different
            # default FILE: the track's raceline.csv (tuner/tools/raceline_optimizer.py's
            # minimum-time line) rather than its speed_profile.csv (the
            # centreline). The MPC's optimum is always e_y=0 on whatever path
            # it is given, so it can never invent a racing line from a
            # centreline reference no matter how Q/R are tuned -- the tracked
            # geometry has to contain the widen-entry/clip-apex shape itself.
            # Kept as a separate launch arg from map_path so the path and speed
            # bypasses can be toggled independently.
            'path_map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/ros2/src/fsae_planning/tracks/comp_test_map_3/raceline.csv',
            description=(
                "Path to a CSV exported from a recorded cone map (same "
                "this-repo tracks/<name>/ location as map_path), used as "
                "the tracked PATH (not just speed) -- see "
                "mpc_controller_standalone.py's path_map_path param. Has no "
                "effect unless use_precomputed_path:=true. Applies to both "
                "`mpc` and `mpc_standalone`; ignored by `stanley`."
            )),
        DeclareLaunchArgument(
            'use_precomputed_path', default_value='true',
            description=(
                "true -> track path_map_path's precomputed path instead of "
                "subscribing to the live planner's "
                "/fsae/planning/selected_trajectory -- removes "
                "centerline_planner.py from the control loop entirely, to "
                "isolate controller/plant tracking error from planner-induced "
                "path error. Only valid for a track that's already been fully "
                "mapped. On by default, matching "
                "use_precomputed_speed -- set false for the planner-in-loop "
                "diagnostic/experiment mode instead."
            )),
        # v_max/v_min/stanley_gain: overrides the shared controller.ros__parameters
        # block in fsae_params.yaml (applies to stanley, mpc, and mpc_standalone
        # alike -- the node name is 'controller' for all three). Defaults match
        # that file's current values exactly, so leaving these unset on the
        # command line changes nothing; pass e.g. v_max:=3.0 for a one-off slow
        # lap (cone-map recording, characterisation runs) without editing the
        # shared config file.
        DeclareLaunchArgument(
            'v_max', default_value='15.0',
            description='m/s -- top speed on straights (overrides fsae_params.yaml controller.v_max)'),
        DeclareLaunchArgument(
            'v_min', default_value='1.5',
            description='m/s -- minimum speed through tight corners (overrides fsae_params.yaml controller.v_min)'),
        DeclareLaunchArgument(
            'stanley_gain', default_value='1.0',
            description='cross-track gain k_cte (overrides fsae_params.yaml controller.stanley_gain)'),
        # MPC-only, like path_map_path -- neither stanley_controller.py nor
        # fsds_bridge.py declares these. See control_utils.dynamic_speed_cap()'s
        # docstring for the mechanism: a real-time curvature-lookahead speed
        # cap layered under map_path's precomputed profile (min of the two),
        # so a corner reached faster than the oracle profile planned still
        # gets braked for in time. No effect when map_path is unset.
        DeclareLaunchArgument(
            'enable_dynamic_speed_cap', default_value='true',
            description='Layer a live curvature-lookahead speed cap under map_path\'s '
                        'precomputed profile (overrides fsae_params.yaml '
                        'controller.enable_dynamic_speed_cap)'),
        DeclareLaunchArgument(
            'dynamic_cap_a_lat_max', default_value='3.2',
            description='m/s^2 -- lateral-accel budget for the dynamic speed cap only '
                        '(overrides fsae_params.yaml controller.dynamic_cap_a_lat_max)'),
        DeclareLaunchArgument(
            'dynamic_cap_safety', default_value='0.9',
            description='safety margin for the dynamic speed cap only '
                        '(overrides fsae_params.yaml controller.dynamic_cap_safety)'),
        *mpc_launch_args,
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {
                'log_csv': log_csv, 'log_dir': log_dir,
                'map_path': effective_map_path,
                'path_map_path': effective_path_map_path,
                'v_max': v_max, 'v_min': v_min,
                'enable_dynamic_speed_cap': enable_dynamic_speed_cap,
                'dynamic_cap_a_lat_max': dynamic_cap_a_lat_max,
                'dynamic_cap_safety': dynamic_cap_safety,
                # stanley_gain deliberately omitted here: neither
                # mpc_controller.py nor mpc_controller_standalone.py declares
                # it, so passing it would raise ParameterNotDeclaredException.
                # Every MPCParams field IS declared by both MPC nodes (see
                # mpc_params.py's declare_mpc_params()), so unlike
                # stanley_gain these are safe to pass unconditionally here.
                **mpc_param_configs,
            }],
            condition=run_controller_mpc,
        ),
        Node(
            package='fsae_control',
            executable=controller_exec,
            name='controller',
            output='screen',
            parameters=[config, {
                'log_csv': log_csv, 'log_dir': log_dir,
                'map_path': effective_map_path,
                'v_max': v_max, 'v_min': v_min, 'stanley_gain': stanley_gain,
            }],
            condition=run_controller_non_mpc,
        ),
        Node(
            package='fsae_control',
            executable='fsds_bridge',
            name='fsds_bridge',
            output='screen',
            parameters=[config],
            condition=run_bridge,
        ),
    ])
