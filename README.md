
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
- **(HARI LAB)** kistar_ws build guide (submodules, TRAC-IK, dependencies): [`docs/setup/kistar_ws_build.md`](./docs/setup/kistar_ws_build.md)
- **(HARI LAB)** Running MoveIt2 via Docker `ros2_humble` for [Topdown_Grasp](https://github.com/KIST-HARILAB/Topdown_Grasp): [`docs/run/run_real_moveit.md`](./docs/run/run_real_moveit.md#running-with-topdown_grasp-docker--ros2_humble-container)

## Package Build

**[[kistar_ws Build Guide]](./docs/setup/kistar_ws_build.md)** — clone (with submodules), install dependencies, colcon build


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