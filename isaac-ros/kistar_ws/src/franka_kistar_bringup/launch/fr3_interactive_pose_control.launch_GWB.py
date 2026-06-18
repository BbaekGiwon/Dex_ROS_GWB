"""
Interactive Pose Control Launch File

PC1 (Planning Computer)에서 실행:
- MoveIt planning
- pose_commander (CUI pose input + user confirmation)
- trajectory_forwarder (trajectory → /trajectory_commands topic)
- RViz (조건부 - gui:=true일 때)

Usage:
  ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py
  ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py gui:=true
  ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py gui:=false

Author: Chanyoung Ahn
Date: 2025
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    gui = LaunchConfiguration("gui")
    planning_time = LaunchConfiguration("planning_time")
    end_effector_link = LaunchConfiguration("end_effector_link")
    reference_frame = LaunchConfiguration("reference_frame")
    execute_mode = LaunchConfiguration("execute_mode")
    franka_speed_factor = LaunchConfiguration("franka_speed_factor")
    use_fake_joint_states = LaunchConfiguration("use_fake_joint_states")

    # Include fr3_kistar_moveit_planning_pc.launch.py
    # (이미 MoveIt + trajectory_forwarder + RViz + franka_joint_position_bridge 포함)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_kistar_bringup"),
                "launch",
                "fr3_kistar_moveit_planning_pc.launch.py"
            ])
        ]),
        launch_arguments={
            "use_rviz": gui,                               # gui 인자에 따라 RViz on/off
            "use_fake_joint_states": use_fake_joint_states,  # false → franka_joint_position_bridge 실행
        }.items()
    )

    # NOTE: pose_commander_GWB.py is NOT launched here.
    # It requires interactive stdin input, which ros2 launch cannot forward.
    # Use start_robot_gwb.sh instead — it starts this launch file in the
    # background and runs pose_commander_GWB.py in the foreground (same terminal).

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Enable RViz GUI (true/false)"
        ),
        DeclareLaunchArgument(
            "planning_time",
            default_value="5.0",
            description="MoveIt planning timeout (seconds)"
        ),
        DeclareLaunchArgument(
            "end_effector_link",
            default_value="fr3_link8",
            description="End-effector link name"
        ),
        DeclareLaunchArgument(
            "reference_frame",
            default_value="world",
            description="Reference frame for target pose (e.g. world, base)"
        ),
        DeclareLaunchArgument(
            "execute_mode",
            default_value="trajectory_forwarder",
            description="Execution mode: trajectory_forwarder | direct_franka_topic"
        ),
        DeclareLaunchArgument(
            "franka_speed_factor",
            default_value="0.1",
            description="Speed factor for direct_franka_topic mode (0.0 ~ 1.0)"
        ),
        DeclareLaunchArgument(
            "use_fake_joint_states",
            default_value="false",
            description="Use fake joint states (false = use franka_joint_position_bridge)"
        ),

        # Nodes
        moveit_launch,
    ])
