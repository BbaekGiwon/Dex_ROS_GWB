# pseudo
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare("franka_kistar_moveit_config"),
                "launch",
                "moveit.launch.py",
            ]
        )
    ),
    launch_arguments={
        "robot_ip": "0.0.0.0",
        "use_fake_hardware": "true",
        "load_gripper": "false",
        "ee_id": "none",
        "namespace": "",
    }.items(),
)
