# Real Robot - MoveIt Example (Robot ↔ MoveIt)
![](../../fig/real_moveit.gif)
```shell
rs
ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py bridge:=real arm_side:=right robot_ip:=10.10.0.7 use_fake_hardware:=false
# ros2 launch franka_kistar_bringup fr3_kistar.launch.py
```

## Troubleshooting
```shell
ros2 pkg executables franka_kistar_isaac_moveit

# Rebuild
colcon build --symlink-install
bash install/local_setup.sh
```
