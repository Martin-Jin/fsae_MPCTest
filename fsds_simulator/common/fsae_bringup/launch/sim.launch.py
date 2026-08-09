import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


# Top-level simulator bring-up: composes perception + planning + control.
# Pick the planner with `planner:=…`; the mode wires the rest automatically.
#
#   ros2 launch fsae_bringup sim.launch.py                              # mpc_standalone (default)
#   ros2 launch fsae_bringup sim.launch.py planner:=skidpad_planner
#   ros2 launch fsae_bringup sim.launch.py controller:=stanley          # use the Stanley controller
#   ros2 launch fsae_bringup sim.launch.py record_cones:=false          # skip cone_recorder
#   ros2 launch fsae_bringup sim.launch.py use_precomputed_speed:=false # live curvature_speed()
#                                                                        # instead of the mapped-track
#                                                                        # speed profile.
#                                                                        # On (mapped-track profile) by
#                                                                        # default -- edit this file's
#                                                                        # use_precomputed_speed default
#                                                                        # below to change the default
#                                                                        # instead of passing the flag
#                                                                        # every launch.
#   ros2 launch fsae_bringup sim.launch.py use_precomputed_path:=false  # live planner's centreline
#                                                                        # (centerline_planner.py) instead
#                                                                        # of the precomputed oracle path
#                                                                        # -- planner-vs-controller
#                                                                        # isolation / live-planner-in-loop
#                                                                        # experiment mode. Precomputed
#                                                                        # path is on by default -- see
#                                                                        # this file's use_precomputed_path
#                                                                        # default below to change it.
def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('fsae_bringup'), 'launch')
    planner = LaunchConfiguration('planner')
    controller = LaunchConfiguration('controller')
    record_cones = LaunchConfiguration('record_cones')
    cone_out_path = LaunchConfiguration('cone_out_path')
    log_csv = LaunchConfiguration('log_csv')
    log_dir = LaunchConfiguration('log_dir')
    map_path = LaunchConfiguration('map_path')
    use_precomputed_speed = LaunchConfiguration('use_precomputed_speed')
    path_map_path = LaunchConfiguration('path_map_path')
    use_precomputed_path = LaunchConfiguration('use_precomputed_path')
    v_max = LaunchConfiguration('v_max')
    v_min = LaunchConfiguration('v_min')
    stanley_gain = LaunchConfiguration('stanley_gain')

    # Skidpad needs the whole cone map up front to reconstruct the figure-8.
    full_track = PythonExpression(
        ["'true' if '", planner, "' == 'skidpad_planner' else 'false'"]
    )

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner',
            default_value='centerline_planner',
            description='centerline_planner | skidpad_planner'),
        DeclareLaunchArgument(
            'controller',
            default_value='mpc_standalone',
            description='stanley | mpc | mpc_standalone — path-tracking controller to run'),
        DeclareLaunchArgument(
            'record_cones',
            default_value='true',
            description='Also launch cone_recorder to log one lap of boundary cones'),
        DeclareLaunchArgument(
            'cone_out_path',
            default_value='',
            description="cone_recorder output path ('' -> ~/fsae_logs/cone_map_<timestamp>.json)"),
        DeclareLaunchArgument(
            'log_csv',
            default_value='true',
            description='Write controller CSV telemetry (e_y/e_psi/steer/...) to log_dir'),
        DeclareLaunchArgument(
            'log_dir',
            default_value='',
            description="Controller CSV telemetry output dir ('' -> ~/fsae_logs). "
                        "launch_all.sh passes the repo root here so logs land in "
                        "<repo>/fsae_logs instead."),
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tuner/speed_profile_export.csv',
            description="Passed through to control.launch.py — see that file's "
                        "map_path description for the full explanation."),
        DeclareLaunchArgument(
            'use_precomputed_speed',
            default_value='true',
            description="Passed through to control.launch.py — see that file's "
                        "use_precomputed_speed description. Toggle here (or "
                        "override with use_precomputed_speed:=false on the "
                        "command line) to switch every mpc/mpc_standalone run "
                        "back to live curvature_speed()."),
        DeclareLaunchArgument(
            'path_map_path',
            default_value='/home/Formula-Student-Driverless-Simulator/fsae_MPCTest/tuner/raceline_export.csv',
            description="Passed through to control.launch.py — see that file's "
                        "path_map_path description. Switched 2026-08-10 from "
                        "speed_profile_export.csv (the centreline) to "
                        "raceline_export.csv (tuner/raceline_optimizer.py's "
                        "minimum-time line) so the tracked geometry actually "
                        "contains the widen-entry/clip-apex shape a corner "
                        "needs -- the MPC's optimum is always e_y=0 on "
                        "whatever path it is given, so it can never invent a "
                        "racing line from a centreline reference no matter how "
                        "Q/R are tuned. Same x,y,psi,v_target format, so this "
                        "is a drop-in swap. NOTE: only the GEOMETRY is used "
                        "while use_precomputed_speed:=false (launch_all.sh's "
                        "default) -- the raceline's own v_target (min 5.89 m/s "
                        "vs the centreline's 2.13) is NOT applied, which keeps "
                        "this a clean line-only experiment rather than also "
                        "raising corner entry speed."),
        DeclareLaunchArgument(
            'use_precomputed_path',
            default_value='true',
            description="Passed through to control.launch.py — see that file's "
                        "use_precomputed_path description. On by default: "
                        "matches use_precomputed_speed's default so "
                        "mpc_standalone tracks the precomputed "
                        "oracle path/speed pair by default, planner out of "
                        "the loop. Override with use_precomputed_path:=false "
                        "on the command line for the planner-vs-controller "
                        "isolation / live-planner-in-loop experiment mode."),
        DeclareLaunchArgument(
            'v_max', default_value='15.0',
            description='m/s -- top speed on straights (overrides fsae_params.yaml controller.v_max)'),
        DeclareLaunchArgument(
            'v_min', default_value='1.5',
            description='m/s -- minimum speed through tight corners (overrides fsae_params.yaml controller.v_min)'),
        DeclareLaunchArgument(
            'stanley_gain', default_value='1.0',
            description='cross-track gain k_cte, stanley only (overrides fsae_params.yaml controller.stanley_gain)'),
        include('perception.launch.py', {'full_track': full_track}),
        include('planning.launch.py',   {'planner': planner}),
        include('control.launch.py',    {
            'planner': planner, 'controller': controller,
            'log_csv': log_csv, 'log_dir': log_dir,
            'map_path': map_path, 'use_precomputed_speed': use_precomputed_speed,
            'path_map_path': path_map_path, 'use_precomputed_path': use_precomputed_path,
            'v_max': v_max, 'v_min': v_min, 'stanley_gain': stanley_gain,
        }),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cone_recorder.launch.py')),
            launch_arguments={'out_path': cone_out_path}.items(),
            condition=IfCondition(record_cones),
        ),
    ])
