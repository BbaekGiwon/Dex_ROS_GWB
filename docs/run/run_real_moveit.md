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

## MoveIt w/ EE pose

### Test Fake Robot with single PC
```shell
# Terminal 1
rs
ros2 launch franka_kistar_bringup fr3_kistar_moveit_planning_pc.launch.py \
  use_fake_joint_states:=true

# Terminal 2
rs
ros2 run franka_kistar_bringup pose_commander.py \
  --ros-args \
  -p gui:=true \
  -p planning_group:=fr3_arm \
  -p end_effector_link:=fr3_link8 \
  -p planning_time:=5.0 \
  -p reference_frame:=world


0.307 0.0 0.487 0 1.0 0 0
# Check MoveIt planning with GUI 
```
### Test Real Robot with dual PC
```shell
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0

# PC2 with real robot + controller
ros2 launch franka_kistar_bringup robot_execution_pc.launch.py \
  robot_ip:=[ROBOT_IP]

# PC1 
```

## Troubleshooting
```shell
ros2 pkg executables franka_kistar_isaac_moveit

# Rebuild
colcon build --symlink-install
bash install/local_setup.sh
```
