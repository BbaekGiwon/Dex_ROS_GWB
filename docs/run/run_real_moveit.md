# Real Robot - MoveIt Example (Robot ↔ MoveIt)
<table>
  <tr>
    <td align="center">
      <img src="../../fig/sim_moveit.gif" width="400"/>
    </td>
    <td align="center">
      <img src="../../fig/real_moveit.gif" width="400"/>
    </td>
  </tr>
</table>

```shell
rs

# Ongoing.. (previous)
ros2 launch franka_kistar_moveit_config moveit.launch.py bridge:=real arm_side:=right robot_ip:=10.10.0.7 use_fake_hardware:=false

# Updated Command (2025-01-30)
ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py robot_ip:=172.16.0.1 use_rviz:=true 

# Run w/o real robot
ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py use_fake_hardware:=true use_rviz:=true 
```

# Activate Realsense Camera
```shell
 ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```


## Troubleshooting
```shell
ros2 pkg executables franka_kistar_isaac_moveit

# Rebuild
colcon build --symlink-install
bash install/local_setup.sh
```
