# Network Setting for KIST L8 4F 

## This Computer 
- For learning using IsaacSim / Inference.. 
    ```shell
    # PRIME 산업부 computer
    Static IP: 10.10.0.8
    Netmask: 255.255.255.0 
    Gateway: X 

    # Robot Computer (Shared)
    # enp1s0f1
    Static IP: 10.10.0.7
    Netmask: 255.255.255.0
    Gateway: X 
    ```

- Check Network Connection 
    ```shell
    ping -c 4 10.10.0.7 # from primecomputer to robot computer
    ```

## Subscribe other ROS Topic
- If you want to subscribe other ROS topic, you should set ROS_DOMAIN_ID same value. 
    ```shell
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   
    export ROS_DOMAIN_ID=9
    export ROS_LOCALHOST_ONLY=0        
    ```


## Recommand: Useful Commands (in bashrc)
- Just put this command to your BASH 
  ```shell
    dex() {
        cd "$HOME/isaacsim/isaacsim-5.1.0" || return
        conda activate dexsdr
        export isaac_sim_package_path=$HOME/isaacsim/isaacsim-5.1.0
        # 1) 기존 ROS 흔적 싹 지우기
        unset ROS_VERSION ROS_PYTHON_VERSION
        unset ROS_PACKAGE_PATH
        clean_var() {
        local name="$1"
        local val="${!name}"
        if [ -n "$val" ]; then
            export "$name"="$(echo "$val" | tr ':' '\n' | grep -v '/opt/ros/humble' | paste -sd: -)"
        fi
        }
        clean_var PYTHONPATH
        clean_var LD_LIBRARY_PATH
        clean_var PATH
        # 2) IsaacSim ROS bridge용 최소 세팅만 다시 넣기
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DOMAIN_ID=9
        export ROS_LOCALHOST_ONLY=0
        export LD_LIBRARY_PATH="$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH"

        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH=$PWD/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH

    }

    # Ros2 Isaac Sim Python 3.10
    rs(){
    conda activate ros
    source /opt/ros/humble/setup.bash
    source ~/fr_ws/install/setup.bash
    source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/humble_ws/install/setup.bash
    source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=9                  
    export ROS_LOCALHOST_ONLY=0     
    }

    # Ros2 Isaac Sim Python 3.11
    rsi(){
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=9                    
    export ROS_LOCALHOST_ONLY=0     
    cd "$HOME/isaac_ws/dex_soldering/dex_ros/isaac-ros/humble_ws" || return
    source install/setup.bash

    }
  ```

## Quick Test
- Turn on and connect PRIME / Robot computer! 
- Publish robot ROS2 topic of Robot computer
- Test on PRIME computer
```shell
    rs  # or rsi
    ros2 topic list
    ros2 topic pub --once /franka/arm_target/right kistar_hand_ros2/msg/FrankaArmTarget "{joint_targets: [0.5, -0.6, 0.7, -2.4, -0.02, 1.2, 1.], arm_id: 0}"
```ㄱ