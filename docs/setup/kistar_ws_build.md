# kistar_ws Build Guide

> Written by Giwon Baek (HARI LAB), 2026.06

---

## 1. Clone

```shell
git clone --recurse-submodules <repo_url>
```

If already cloned without submodules:

```shell
git submodule update --init
```

> **TRAC-IK** (`src/trac_ik`) is registered as a submodule pointing to the `rolling` branch of [traclabs/trac_ik](https://github.com/traclabs/trac_ik) (ROS2 Humble compatible — no dedicated `humble` branch exists).  
> Cloning without `--recurse-submodules` will cause a build failure since the package source won't be present.

---

## 2. System Dependencies

```shell
# RealSense camera
sudo apt install ros-humble-realsense2*

# TRAC-IK build dependency (build fails with "NLopt not found" if missing)
sudo apt install libnlopt-dev libnlopt-cxx-dev
```

---

## 3. Build

```shell
cd isaac-ros/kistar_ws
colcon build --symlink-install
bash ./install/local_setup.sh
```
