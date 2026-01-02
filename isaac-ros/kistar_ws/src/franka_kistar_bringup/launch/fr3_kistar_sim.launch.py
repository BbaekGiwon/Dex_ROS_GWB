import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import UnlessCondition


def generate_robot_nodes(context):
    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_kistar_description'),
        'urdf',
        LaunchConfiguration('urdf_file')
    ]).perform(context)

    # 2) fr3_kistar.urdf.xacro가 받는 arg만 mappings로 넘김
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'arm_prefix': LaunchConfiguration('arm_prefix').perform(context),
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
        }
    ).toprettyxml(indent='  ')

    namespace = LaunchConfiguration('namespace').perform(context)
    controllers_yaml = LaunchConfiguration('controllers_yaml').perform(context)

    # 3) joint_state_publisher는 일단 팔 joint만 합친다
    joint_state_publisher_sources = ['franka/joint_states']
    joint_state_rate = int(LaunchConfiguration('joint_state_rate').perform(context))

    nodes = [
        # robot_state_publisher: TF / RobotModel 퍼블리시
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ros2_control_node: fr3_arm_controller, joint_state_broadcaster 등 관리
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                controllers_yaml,
                {'robot_description': robot_description},
            ],
            # 컨트롤러 쪽 joint_states → franka/joint_states 로 remap
            remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
            on_exit=Shutdown(),
        ),

        # joint_state_publisher: 여러 joint_states 소스를 합쳐서 publish
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': joint_state_rate,
                'use_robot_description': False,
            }],
            output='screen',
        ),

        # joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        # franka_robot_state_broadcaster (Franka 전용 상태 토픽)
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['franka_robot_state_broadcaster'],
            parameters=[{'arm_id': LaunchConfiguration('arm_id').perform(context)}],
            condition=UnlessCondition(LaunchConfiguration('use_fake_hardware')),
            output='screen',
        ),

        # KISTAR Hand는 아직 ros2_control 안 붙였으니
        #     franka_gripper 관련 launch는 일단 제거
    ]

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'arm_id',
            default_value='fr3',
            description='ID of the type of arm used'
        ),
        DeclareLaunchArgument(
            'arm_prefix',
            default_value='',
            description='Prefix for arm topics'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'
        ),
        # franka_kistar_description/urdf 안의 파일 이름
        DeclareLaunchArgument(
            'urdf_file',
            default_value='fr3_kistar.urdf.xacro',
            description='Path to URDF file (relative to franka_kistar_description/urdf)'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='172.16.0.3',
            description='Hostname or IP address of the robot'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Fake sensor commands'
        ),
        DeclareLaunchArgument(
            'joint_state_rate',
            default_value='30',
            description='Rate for joint state publishing (Hz)'
        ),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('franka_bringup'),
                    'config',
                    "controllers.yaml"
                ]),
            description='Override the default controllers.yaml file.'
        ),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_robot_nodes)])
