# IsaacSim - MoveIt Example (Isaac Sim ↔ MoveIt)

## 0) First time, Build ROS2 packages

```shell
cd ~/isaac-ros
./build_ros.sh -d humble -v 22.04
```

## 1) Terminal A: Run Isaac Sim Robot Env

ROS 환경(특히 `/opt/ros`나 conda `ros`)은 섞지 않는 것을 권장합니다.

```shell
conda deactivate  

export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
export __GLX_VENDOR_LIBRARY_NAME=nvidia

cd ~/isaacsim/isaacsim-5.1.0

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0

source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/humble_ws/install/setup.bash
source ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws/install/setup.bash

./python.sh /home/cy/isaac_ws/dex_soldering/dex_ros/examples/run_soldering_env.py  --enable isaacsim.ros2.bridge 
```

## 2) Terminal B: ROS2 + MoveIt

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

* Run MoveIt

```shell
rs
ros2 launch franka_kistar_moveit_config moveit.launch.py \
  robot_ip:=dummy use_fake_hardware:=true launch_rviz:=true
```

## 3) Toubleshooting

MoveIt에서 보낸 명령이 IsaacSim으로 안 가면 **ROS_DOMAIN_ID 불일치**가 가장 흔합니다.
터미널 A/B 모두 같은 값(예: 9)인지 확인하세요.

```shell
rs
ros2 topic list | rg "isaac_joint"
```

`/isaac_joint_states`, `/isaac_joint_commands`가 보이면 브리지 통신은 살아 있습니다.

- `libtbb`/`libstdc++`가 `/lib/x86_64-linux-gnu`에서 로딩되면
  OmniGraph 초기화 중 세그폴트가 발생할 수 있습니다.
- 이 경우 터미널 A에서 `LD_LIBRARY_PATH`를 Kit 경로로 강제하고,
  ROS/conda 환경을 섞지 않는 것이 가장 안정적입니다.
