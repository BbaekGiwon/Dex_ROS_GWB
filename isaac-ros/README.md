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

