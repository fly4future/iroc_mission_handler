#!/usr/bin/env python3

import os
import yaml
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def expand_to_pwd_if_relative(arg_name: str):
    """
    ""          -> ""
    "/abs/..."  -> "/abs/..."
    "rel/..."   -> "$PWD/rel/..."
    """
    cfg = LaunchConfiguration(arg_name)
    return IfElseSubstitution(
        condition=PythonExpression(['"', cfg, '" != "" and not "', cfg, '".startswith("/")'
        ]),
        if_value=PathJoinSubstitution([EnvironmentVariable("PWD"), cfg]),
        else_value=cfg,
    )


def launch_setup(context, *args, **kwargs):

    pkg_share = get_package_share_directory("iroc_mission_handler")

    mrs_traj_gen_share = get_package_share_directory("mrs_uav_trajectory_generation")

    # These were expanded via SetLaunchConfiguration before OpaqueFunction
    custom_config = LaunchConfiguration("custom_config").perform(context)

    executor_config = LaunchConfiguration("executor_config").perform(context)

    trajectory_generation_config = LaunchConfiguration(
        "trajectory_generation_config").perform(context)

    robot_name = LaunchConfiguration("robot_name").perform(context)

    use_sim_time = LaunchConfiguration("use_sim_time").perform(
        context).lower() in ("true", "1", "yes")

    static_remappings = [
        ("~/uav_state_in", "state_monitor/uav_state"),
        ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),

        # Service clients
        ("~/svc_takeoff_in", "uav_manager/takeoff"),
        ("~/svc_land_in", "uav_manager/land"),
        ("~/svc_land_home_in", "uav_manager/land_home"),
        ("~/svc_path_in", "trajectory_generation/path"),
        ("~/svc_get_path_in", "trajectory_generation/get_path"),
        ("~/svc_hover_in", "control_manager/hover"),
        ("~/svc_mission_pause_in", "control_manager/stop_trajectory_tracking"),
        ("~/svc_mission_flying_to_start_in", "control_manager/goto_trajectory_start"),
        ("~/svc_mission_start_in", "control_manager/start_trajectory_tracking"),
        ("~/svc_mission_validation_in", "control_manager/validate_reference_array"),
        ("~/svc_trajectory_reference_in", "control_manager/trajectory_reference"),
        ("~/svc_transform_reference_in", "control_manager/transform_reference"),
        ("~/svc_transform_reference_array_in", "control_manager/transform_reference_array"),
        ("~/svc_servo_camera_set_orientation_in", "servo_camera/set_orientation"),
        ("~/servo_camera_orientation_in", "servo_camera/orientation"),

        # Service servers
        ("~/svs_mission_activation_out", "~/mission_activation"),
        ("~/svs_mission_pausing_out", "~/mission_pausing"),

    ]

    default_config = os.path.join(pkg_share, "config", "config.yaml")
    default_executor_config = os.path.join(pkg_share, "config", "basic_subtask_executor_plugins.yaml")
    default_trajectory_config = os.path.join(mrs_traj_gen_share, "config", "public", "trajectory_generation.yaml")

    final_config = custom_config if custom_config else default_config
    final_executor_config = executor_config if executor_config else default_executor_config
    final_trajectory_config = trajectory_generation_config if trajectory_generation_config else default_trajectory_config

    # Container + component
    container = ComposableNodeContainer(
        name="iroc_mission_handler_container",
        namespace=robot_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[
            ComposableNode(
                package="iroc_mission_handler",
                plugin="iroc_mission_handler::MissionHandler",
                name="iroc_mission_handler",
                namespace=robot_name,
                parameters=[
                    {"robot_name": robot_name},
                    {"config": default_config},
                    {"custom_config": final_config}, 
                    {"executor_config": final_executor_config}, 
                    {"trajectory_generation_config": final_trajectory_config}, 
                    {"use_sim_time": use_sim_time},
                ],
                remappings=static_remappings,
            )
        ],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_name",
            default_value=os.getenv("UAV_NAME", "uav1"),
            description="Namespace of the UAV (e.g., uav1, uav2, ...)."
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=os.getenv("USE_SIM_TIME", "false"),
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "custom_config",
            default_value="",
            description="Custom config path (abs or relative to PWD).",
        ),
        DeclareLaunchArgument(
            "trajectory_generation_config",
            default_value="",
            description="Trajectory generation config path (abs or relative to PWD).",
        ),
        DeclareLaunchArgument(
            "executor_config",
            default_value="",
            description="Executor config path (abs or relative to PWD).",
        ),

        DeclareLaunchArgument(
            "network_config",
            default_value="",
            description="Network config YAML path (abs or relative to PWD).",
        ),

        # Expand relative paths before launch_setup runs
        SetLaunchConfiguration("custom_config", expand_to_pwd_if_relative("custom_config")),

        OpaqueFunction(function=launch_setup),
    ])



