
# Dex ROS
- Original code by **[Chanyoung Ahn](https://github.com/cold-young)** (PRIME LAB) — [dex_ros](https://github.com/KIST-PRIME-Lab/dex_ros)
- Modified by **[Giwon Baek](https://github.com/BbaekGiwon)** (HARI LAB), 2026.06 — change scripts for docker usage

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![IsaacLab](https://img.shields.io/badge/IsaacLab-v2.2.1-green.svg)](https://github.com/isaac-sim/IsaacLab/)


### Submodule for dex_soldering project! 
<table>
  <tr>
    <td align="center">
      <img src="./fig/sim_moveit.gif" width="400"/>
    </td>
    <td align="center">
      <img src="./fig/real_moveit.gif" width="400"/>
    </td>
  </tr>
</table>


**Dex ROS** is an ROS2 framework for robotic hand learning using multimodal haptic sensing to accelerate dexterous manipulation with intrinsic properties. Particulary, this work specialized at tool oriented manipulation such as soldering. It builds on [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) and [NVIDIA IsaacLab](https://github.com/isaac-sim/IsaacLab/) to take advantage of a variety of learning approaches (such as RL, learning from demonstration) and fast and accurate simulation.

* **Contributor:** Chanyoung Ahn

* **Caution:** I always have been developing this repostiory in `cy` branch, It may be experimental setup. <br> Thus, I **strongly** recommand that use `main` branch. 




## IsaacSim Installation (Local)
Install `IsaacSim-5.1.0` [[Link]](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

## Docs 
- Initial Settings (H/W, Network, ROS2 ..etc): [`\docs`](./docs) 
- MoveIt and Specific Packages for MOTIE Projects: [`\examples`](./examples/)
- **(Real Robot + [Topdown_Grasp](https://github.com/KIST-HARILAB/Topdown_Grasp) 연계)** Docker `ros2_humble` 컨테이너로 MoveIt2 실행: [`docs/run/run_real_moveit.md`](./docs/run/run_real_moveit.md#grasp_fruit-연계-실행-docker--ros2_humble-컨테이너)

## Package Build

### 1. 시스템 의존성 설치

```shell
# ROS2 Humble (필수)
# https://docs.ros.org/en/humble/Installation.html

# RealSense 카메라
sudo apt install ros-humble-realsense2*

# TRAC-IK 빌드 의존성 (src/trac_ik 소스가 repo에 포함되어 있으나 libnlopt는 별도 설치 필요)
sudo apt install libnlopt-dev libnlopt-cxx-dev
```

또는 `rosdep`으로 한 번에:

```shell
cd isaac-ros/kistar_ws
rosdep install -i --from-path src --rosdistro humble -y
```

### 2. 빌드

```shell
cd isaac-ros/kistar_ws
colcon build --symlink-install
bash ./install/local_setup.sh
```

> **TRAC-IK**: `src/trac_ik/`가 repo에 직접 포함되어 있으므로 `colcon build` 시 함께 빌드된다.  
> 단, `libnlopt`가 시스템에 없으면 빌드 중 `NLopt not found` 에러가 발생하므로 위 의존성 설치를 먼저 한다.


## TODO! 
I will irregularly update the development progress in [Link](https://cold-young.github.io/projects/kist-soldering/). (only devleopment status! not detail information)

- [x] Initial Setting

  - [x] Our own docker/singluarity image

- [ ] ROS2 Interface 
  - [x] Planning (Moveit) - w/ ros topic
  - [ ] Perception 
  - [x] RViz in ROS2 + IsaacSim 5.1
  - [x] TF and Vision Calibration
  - [ ] Setup dual camera ~ w/ define name space.
    - [ ] Dynamical update TF depend on AprilTag detection
  - [x] Obstacle Decection + Planning 
  - [ ] Hand ros2 topic interface
    - [ ] controller?
    - [ ] grasp planner w/ MoveIt?

- [x] Connect with Real Environment 
  - [ ] RL policy to MoveIt and Robot

- [ ] Check CUI version launch 

___
## Citation
```
@disc{ahn2025Dexros,
   author={Ahn, Chanyoung},
   title={DexROS: ROS2 Platform for Tool-usage with Multimodal Haptic Sensing},
   year={2025},
}
```