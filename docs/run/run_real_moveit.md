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

ros2 launch franka_kistar_bringup fr3_kistar_moveit_planning_pc.launch.py \
  use_fake_joint_states:=false

# Terminal 2
rs
ros2 run franka_kistar_bringup pose_commander.py \
  --ros-args \
  -p gui:=true \
  -p planning_group:=fr3_arm \
  -p end_effector_link:=fr3_link8 \
  -p planning_time:=5.0 \
  -p reference_frame:=fr3_link0

  # -p reference_frame:=world


0.307 0.0 0.487 0 1.0 0 0
# Check MoveIt planning with GUI 
```
### Test Real Robot with dual PC
```shell
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0

# PC2 with real robot + controller
ros2 launch franka_kistar_bringup robot_execution_pc.launch.py \
  robot_ip:=192.168.1.250

# PC1 
```

---

## Setting up the `ros2_humble` Docker Container

The recommended base image is **`osrf/ros:humble-desktop`** (OSRF official, includes ROS2 Humble + RViz2).

### Create the container

```bash
docker run -it \
  --name ros2_humble \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/kist/HARILAB:/root/HARILAB \
  -v /home/kist/ros2_ws:/root/ros2_ws \
  osrf/ros:humble-desktop \
  bash
```

> Adjust the `-v` mount paths to match your machine (see `paths.yaml`).  
> `--network host` is required for ROS2 DDS communication between the host and container.

After creating the container, build kistar_ws inside it:

```bash
# Inside the container
cd /root/HARILAB/dex_ros/isaac-ros/kistar_ws
apt install libnlopt-dev libnlopt-cxx-dev -y
colcon build --symlink-install
```

From then on, start the existing container with `docker start ros2_humble` — no need to re-create it.

---

## Running with Topdown_Grasp (Docker — `ros2_humble` container)

The [Topdown_Grasp](https://github.com/KIST-HARILAB/Topdown_Grasp) pipeline (`run_pipeline_interactive.py`) sends robot commands via `docker exec` into the `ros2_humble` container at Stage 3 (robot execution).  
MoveIt2 must be running inside the container before starting the pipeline.

### 1. Allow X11 forwarding

```bash
xhost +local:docker
```

### 2. Start the container

```bash
docker start ros2_humble
```

### 3. Launch MoveIt2 (`fr3_interactive_pose_control.launch_GWB.py`)

```bash
docker exec -it -e DISPLAY=$DISPLAY ros2_humble bash -c "
  unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
  export PATH=/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/humble/bin
  export ROS_DOMAIN_ID=9
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_LOCALHOST_ONLY=0
  source /opt/ros/humble/setup.bash
  source /root/HARILAB/dex_ros/isaac-ros/kistar_ws/install/setup.bash
  ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch_GWB.py \
    gui:=true \
    use_fake_joint_states:=false \
    execute_mode:=direct_franka_topic \
    reference_frame:=base
"
```

**Launch parameters:**

| Parameter | Value | Description |
|---|---|---|
| `gui` | `true` | Enable RViz |
| `use_fake_joint_states` | `false` | Use real robot joint states |
| `execute_mode` | `direct_franka_topic` | Send trajectory directly via Franka topic |
| `reference_frame` | `base` | MoveIt planning reference frame |

> The `unset PYTHONPATH ...` lines prevent the host Conda environment from leaking into the container.  
> Omitting them can corrupt the container's Python/ROS2 packages and cause import errors.

### 4. Verify readiness

Once the following action server is active, Topdown_Grasp Stage 3 can communicate with the robot.

```bash
# Check from the host
ROS_DOMAIN_ID=9 ros2 action list
# /move_action  ← must be listed
```

### Notes

- If the container is not restarted after a previous session, two MoveGroup nodes may be running simultaneously.  
  If you see `[WARN] Ignoring unexpected result response`, restart the container:

  ```bash
  docker restart ros2_humble
  # then re-run the docker exec launch command above
  ```

- Launch file location:
  ```
  isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/
      fr3_interactive_pose_control.launch_GWB.py
  ```

---

## Troubleshooting
```shell
ros2 pkg executables franka_kistar_isaac_moveit

# Rebuild
colcon build --symlink-install
bash install/local_setup.sh
```
