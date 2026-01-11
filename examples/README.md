# DEX-ROS Example

Standalone example to run isaacsim and ROS2.

```shell
cd ~/isaacsim/isaacsim-5.1.0

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$PWD/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH

./python.sh /home/cy/isaac_ws/dex_soldering/dex_ros/examples/run_soldering_env.py
```