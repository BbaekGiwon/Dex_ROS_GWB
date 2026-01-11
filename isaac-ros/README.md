# Isaac Sim ROS & ROS2 Workspaces

This repository contains two workspaces: `humble_ws` (ROS2 Humble) and `jazzy_ws` (ROS2 Jazzy). 

[Click here for usage and installation instructions with Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/index.html)

When cloning this repository, both workspaces are downloaded. Depending on which ROS distro you are using, follow the [setup instructions](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html#setting-up-workspaces) for building your specific workspace.


## Installation 


1. Build Docker image
```shell
cd ~/isaac-ros
./build_ros.sh -d humble -v 22.04
```

2. when you want to activate ROS2 Python 3.11, input these commands each terminal.


```shell
source build_ws/humble/humble_ws/install/local_setup.bash
source build_ws/humble/isaac_sim_ros_ws/install/local_setup.bash
```

### Recommand 
- Just input this commands in your bash 
  ```shell
    dex() {
        cd "$HOME/isaac_ws/dex_soldering" || return
        conda activate dexsdr
        export PATH=/usr/local/go/bin:$PATH
        export isaac_sim_package_path=$HOME/isaacsim/isaacsim-5.1.0
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

    }

    # If you want to source ROS2 .. python 3.10
    rs(){
    source /opt/ros/humble/setup.bash
    }
    
  ```

## Just use ROS2 Humble with Python 3.10
```shell
cd ~/isaac-ros/humble_ws
git submodule update --init --recursive # If using docker, perform this step outside the container and relaunch the container
rosdep install -i --from-path src --rosdistro humble -y
colcon build

# for developer
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# for inference
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


# 1) 항상 ROS 기본
source /opt/ros/humble/setup.bash

# 2) Franka 패키지 (underlay)
source ~/fr_ws/install/setup.bash

# 3) Isaac 예제 + 네 코드 (overlay)
source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/humble_ws/install/setup.bash
source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws/install/setup.bash

# Build overlay
source install/setup.bash
```

## Example
```shell
ros2 run joint_state_publisher_gui joint_state_publisher_gui /tmp/fr3_kistar.urdf

```

## MoveIt! Example 
![Demonstration](../fig/moveit.gif)
```shell
rs 

ros2 launch moveit_setup_assistant setup_assistant.launch.py

# 터미널 1 – 실로봇 bringup (공식)
ros2 launch franka_bringup franka.launch.py \
  robot_ip:=172.16.0.3 \
  use_fake_hardware:=false

# 터미널 2 – MoveIt + RViz (kistar URDF)
ros2 launch franka_kistar_moveit_config moveit.launch.py

```

