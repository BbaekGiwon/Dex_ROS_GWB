# Real Robot - MoveIt Example (Robot ↔ MoveIt)
![](../../fig/real_moveit.gif)
```shell
rs
ros2 launch franka_kistar_moveit_config moveit.launch.py bridge:=real arm_side:=right robot_ip:=10.10.0.7 use_fake_hardware:=false
# ros2 launch franka_kistar_bringup fr3_kistar.launch.py

# With TF
ros2 launch franka_kistar_bringup fr3_kistar_moveit_bringup.launch.py   bridge:=real robot_ip:=172.16.0.3 use_rviz:=true rviz_source:=kistar
```

## Troubleshooting
```shell
ros2 pkg executables franka_kistar_isaac_moveit

# Rebuild
colcon build --symlink-install
bash install/local_setup.sh
```
