import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_robot_nodes(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    # planning_context
    urdf_path = os.path.join(
        get_package_share_directory("franka_kistar_description"),
        "urdf",
        "fr3_kistar.urdf.xacro",
    )

    tf_remaps = [
        ('tf', '/tf'),
        ('tf_static', '/tf_static'),
        ('robot_description', '/robot_description'),  # rsp가 topic publish할 때
    ]

    # --- xacro -> robot_description ---
    xacro_cmd = Command([
        FindExecutable(name='xacro'),
        ' ',
        urdf_path,
        ' ros2_control:=', LaunchConfiguration('enable_ros2_control'),
        ' arm_prefix:=', LaunchConfiguration('arm_prefix'),
        ' robot_ip:=', LaunchConfiguration('robot_ip'),
        ' use_fake_hardware:=', LaunchConfiguration('use_fake_hardware'),
        ' fake_sensor_commands:=', LaunchConfiguration('fake_sensor_commands'),
        ' gazebo:=false',
    ])

    robot_description = {
        'robot_description': ParameterValue(xacro_cmd, value_type=str)
    }
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)

    # ---- Static TF args ----
    world_frame = LaunchConfiguration('world_frame').perform(context)
    robot_base_frame = LaunchConfiguration('robot_base_frame').perform(context)
    table_frame = LaunchConfiguration('table_frame').perform(context)

    table_xyz = LaunchConfiguration('table_xyz').perform(context).split()
    table_rpy = LaunchConfiguration('table_rpy').perform(context).split()

    if len(table_xyz) != 3 or len(table_rpy) != 3:
        raise RuntimeError("table_xyz / table_rpy must be 'x y z' / 'r p y' (3 values each)")

    world_to_robot_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot_base_tf',
        arguments=[
            '--x', LaunchConfiguration('robot_base_x'),
            '--y', LaunchConfiguration('robot_base_y'),
            '--z', LaunchConfiguration('robot_base_z'),
            '--roll',  LaunchConfiguration('robot_base_roll'),
            '--pitch', LaunchConfiguration('robot_base_pitch'),
            '--yaw',   LaunchConfiguration('robot_base_yaw'),
            '--frame-id', LaunchConfiguration('world_frame'),
            '--child-frame-id', LaunchConfiguration('robot_base_frame'),
        ],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        output='screen',
    )

    world_to_table_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        arguments=[
            '--x', table_xyz[0], '--y', table_xyz[1], '--z', table_xyz[2],
            '--roll', table_rpy[0], '--pitch', table_rpy[1], '--yaw', table_rpy[2],
            '--frame-id', world_frame,
            '--child-frame-id', table_frame,
        ],
        output='screen',
    )

    # --- robot_state_publisher (TF의 핵심) ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[robot_description, {'publish_robot_description': True}],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('robot_description', '/robot_description'),
        ],
        output='screen',
    )

    # --- ros2_control (옵션) ---
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            controllers_yaml,
            robot_description
        ],
        output='screen',
        on_exit=Shutdown(),
        condition=IfCondition(LaunchConfiguration('enable_ros2_control')),
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ros2_control')),
    )

    franka_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['franka_robot_state_broadcaster'],
        parameters=[{'arm_id': LaunchConfiguration('arm_id').perform(context)}],
        output='screen',
        # fake hardware면 굳이 안 띄워도 됨
        condition=IfCondition(LaunchConfiguration('enable_ros2_control')),
    )

    # --- ros2_control OFF일 때: joint_state_publisher_gui (URDF를 파라미터로 직접 줘야 함) ---
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        parameters=[robot_description, {'use_robot_description': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joint_state_gui')),
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[robot_description, {'use_robot_description': True}],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_joint_state_gui')),
    )

    # --- RViz: robot_description을 반드시 parameters로 넘겨라 (MoveIt에서 되던 핵심 이유) ---
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('franka_kistar_bringup'),
        'rviz',
        LaunchConfiguration('rviz_config')
    ]).perform(context)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',          # /rviz 고정
        namespace=namespace,
        arguments=['-d', rviz_config_path],
        parameters=[robot_description],   # <= 이게 핵심
        output='screen',
    )

    nodes = [
        world_to_robot_base_tf,
        world_to_table_tf,
        rsp,
        rviz_node,
        ros2_control_node,
        jsb_spawner,
        franka_state_spawner,
        # (enable_ros2_control=false이면 GUI로 조인트 조작 가능)
        jsp_gui if LaunchConfiguration('enable_ros2_control').perform(context) == 'false' else None,
        jsp if LaunchConfiguration('enable_ros2_control').perform(context) == 'false' else None,
    ]

    return [n for n in nodes if n is not None]


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('arm_id', default_value='fr3'),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),

        DeclareLaunchArgument('urdf_file', default_value='fr3_kistar.urdf.xacro'),
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.3'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false'),

        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'config',
                'controllers.yaml'
            ])
        ),

        # --- TF ---
        DeclareLaunchArgument('world_frame', default_value='world'),
        DeclareLaunchArgument('robot_base_frame', default_value='fr3_link0'),
        DeclareLaunchArgument('table_frame', default_value='table_link'),
        DeclareLaunchArgument('robot_base_x', default_value='0.0'),
        DeclareLaunchArgument('robot_base_y', default_value='0.0'),
        DeclareLaunchArgument('robot_base_z', default_value='0.0'),
        DeclareLaunchArgument('robot_base_roll',  default_value='0.7689'),
        DeclareLaunchArgument('robot_base_pitch', default_value='0.0'),
        DeclareLaunchArgument('robot_base_yaw',   default_value='0.0'),
        DeclareLaunchArgument('table_xyz', default_value='0.6 0.0 0.0'),
        DeclareLaunchArgument('table_rpy', default_value='0 0 0'),

        # --- switches ---
        DeclareLaunchArgument('enable_ros2_control', default_value='true'),
        DeclareLaunchArgument('use_joint_state_gui', default_value='true'),

        # RViz config
        DeclareLaunchArgument(
            'rviz_config',
            default_value='fr3_kistar.rviz',
            description='RViz config filename inside franka_kistar_bringup/rviz'
        ),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
