rs
ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  bridge:=real arm_side:=right robot_ip:=10.10.0.7

ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  bridge:=real arm_side:=right robot_ip:=10.10.0.7 command_rate_hz:=100

  colcon build --symlink-install
  colcon build --symlink-install --packages-select franka_kistar_isaac_moveit franka_kistar_isaac_moveit_config

  ros2 pkg executables franka_kistar_isaac_moveit



ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  bridge:=real arm_side:=right \
  resample_dt:=0.005 \
  robot_ip:=10.10.0.7 use_fake_hardware:=false

  ros2 run franka_kistar_isaac_moveit real_moveit_bridge --ros-args \
  -p command_rate_hz:=100.0 \
  -p publish_dummy_hand_joints:=true