# Network Setting for KIST L8 1F 

## This Computer 
- For learning using IsaacSim / Inference.. 
    ```shell
    Static IP: 10.10.0.38
    Netmask: 255.255.255.0 
    Gateway: X 

    # Robot Computer
    # enp1s0f1
    Static IP: 10.10.0.7
    Netmask: 255.255.255.0
    Gateway: X 
    ```

- Check Network Connection 
    ```shell
    ping 10.10.0.7 # from primecomputer to robot computer
    ```

## Subscribe other ROS Topic
- If you want to subscribe other ROS topic, you should set ROS_DOMAIN_ID same value. 
    ```shell
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # 둘 다 동일
    export ROS_DOMAIN_ID=10                      # 둘 다 동일한 숫자
    export ROS_LOCALHOST_ONLY=0        
    ```


## Recommand: Useful Commands 
- Just put this command to your BASH 
  ```shell
    dex() {
        cd "$HOME/isaac_ws/dex_soldering" || return
        conda activate dexsdr
        export PATH=/usr/local/go/bin:$PATH
        export isaac_sim_package_path=$HOME/isaacsim/isaacsim-5.1.0
        export ROS_DISTRO=humble
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
        export ROS_DOMAIN_ID=10                    
        export ROS_LOCALHOST_ONLY=0     
    }

    # If you want to source ROS2 .. python 3.10
    rs(){
    source /opt/ros/humble/setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_DOMAIN_ID=10                    
    export ROS_LOCALHOST_ONLY=0     
    }
  ```