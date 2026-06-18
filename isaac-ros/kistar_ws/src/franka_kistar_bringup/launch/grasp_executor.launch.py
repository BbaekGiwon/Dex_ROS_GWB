"""
Grasp Executor Launch File

PC1 (Planning Computer)에서 실행:
- MoveIt planning (fr3_kistar_moveit_planning_pc 포함)
- grasp_executor — 공유 디렉터리의 grasp_command.json 감시 후 실행

사용법:
  # 기본 (계획만, 실행은 touch grasp_confirm 으로 승인)
  ros2 launch franka_kistar_bringup grasp_executor.launch.py

  # 자동 실행 모드
  ros2 launch franka_kistar_bringup grasp_executor.launch.py auto_execute:=true

  # 공유 디렉터리 지정
  ros2 launch franka_kistar_bringup grasp_executor.launch.py \\
      watch_dir:=/home/kist/shared/grasp_ros2

  # direct_franka_topic 모드 (trajectory 없이 최종 관절값만 전송)
  ros2 launch franka_kistar_bringup grasp_executor.launch.py \\
      execute_mode:=direct_franka_topic auto_execute:=true

호스트에서 grasp 전송:
  python scripts/send_to_ros2.py \\
      --grasp_json data/outputs/<stem>_summary.json \\
      --shared_dir /home/kist/shared/grasp_ros2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch Arguments ──────────────────────────────────────────────
    watch_dir_arg = DeclareLaunchArgument(
        "watch_dir",
        default_value="/shared/grasp_ros2",
        description="호스트↔Docker 공유 디렉터리 경로",
    )
    auto_execute_arg = DeclareLaunchArgument(
        "auto_execute",
        default_value="false",
        description="true: 계획 즉시 실행 / false: grasp_confirm 파일 생성 후 실행",
    )
    execute_mode_arg = DeclareLaunchArgument(
        "execute_mode",
        default_value="trajectory_forwarder",
        description="trajectory_forwarder | direct_franka_topic",
    )
    speed_factor_arg = DeclareLaunchArgument(
        "speed_factor",
        default_value="0.1",
        description="속도 스케일링 (0.0 ~ 1.0)",
    )
    execute_hand_arg = DeclareLaunchArgument(
        "execute_hand",
        default_value="true",
        description="true: 핸드 관절 목표도 전송",
    )
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="true: RViz 궤적 시각화 활성화",
    )

    # ── MoveIt + trajectory_forwarder + RViz ──────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_kistar_bringup"),
                "launch",
                "fr3_kistar_moveit_planning_pc.launch.py",
            ])
        ]),
        launch_arguments={"use_rviz": LaunchConfiguration("gui")}.items(),
    )

    # ── Grasp Executor Node ───────────────────────────────────────────
    grasp_executor = Node(
        package="franka_kistar_bringup",
        executable="grasp_executor.py",
        name="grasp_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "watch_dir":          LaunchConfiguration("watch_dir"),
            "auto_execute":       LaunchConfiguration("auto_execute"),
            "execute_mode":       LaunchConfiguration("execute_mode"),
            "speed_factor":       LaunchConfiguration("speed_factor"),
            "execute_hand":       LaunchConfiguration("execute_hand"),
            "planning_group":     "fr3_arm",
            "end_effector_link":  "fr3_hand_tcp",
            "reference_frame":    "world",
            "planning_time":      5.0,
            "hand_target_topic":  "/hand/target_joint",
            "hand_move_duration": 2.0,
            "poll_interval":      0.5,
        }],
    )

    return LaunchDescription([
        watch_dir_arg,
        auto_execute_arg,
        execute_mode_arg,
        speed_factor_arg,
        execute_hand_arg,
        gui_arg,
        moveit_launch,
        grasp_executor,
    ])
