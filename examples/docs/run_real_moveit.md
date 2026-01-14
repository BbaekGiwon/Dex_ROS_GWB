rs
ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  bridge:=real arm_side:=right robot_ip:=10.10.0.7

ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  bridge:=real arm_side:=right robot_ip:=10.10.0.7 command_rate_hz:=300