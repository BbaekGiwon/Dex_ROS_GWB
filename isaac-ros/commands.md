# DEX-ROS 실행 순서 (Isaac Sim ↔ MoveIt)

아래 순서는 `isaac-ros/README.md`와 `examples/README.md` 내용을 기준으로 정리했습니다.

## 0) 1회 준비 (빌드/설치가 안 돼 있다면)

```shell
cd ~/isaac-ros
./build_ros.sh -d humble -v 22.04
```

## 1) 터미널 A: Isaac Sim 실행 (ROS2 Bridge 포함)

가능하면 새 셸에서 시작하세요(ROS 환경이 섞이면 충돌 가능성이 큼).

```shell
cd ~/isaacsim/isaacsim-5.1.0

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
export LD_LIBRARY_PATH=$PWD/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH

./python.sh /home/cy/isaac_ws/dex_soldering/dex_ros/examples/run_soldering_env.py
```

## 2) 터미널 B: ROS2 + MoveIt 실행

`isaac-ros/README.md`의 `rs()` 함수를 등록해 뒀다면 `rs`만 실행해도 됩니다.  
직접 설정할 경우 아래처럼 소스 순서를 맞춰 주세요.

```shell
source /opt/ros/humble/setup.bash
source ~/fr_ws/install/setup.bash
source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/humble_ws/install/setup.bash
source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
```

MoveIt 실행:

```shell
ros2 launch franka_kistar_isaac_moveit_config moveit.launch.py \
  robot_ip:=dummy use_fake_hardware:=true launch_rviz:=true
```

필요 시(URDF 확인용) 별도 터미널에서:

```shell
ros2 launch franka_kistar_moveit_config moveit.launch.py
```
