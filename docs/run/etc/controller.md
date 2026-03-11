# MoveIt Controller Configuration

이 문서는 MoveIt으로 로봇 움직임 trajectory를 생성하는 시스템의 controller 구성과 trajectory output 형태를 설명합니다.

## 실행 명령어

### Isaac Sim 환경
```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
source install/setup.bash
ros2 launch franka_kistar_bringup fr3_kistar_moveit_bringup.launch.py bridge:=isaac
```

### 실제 로봇 환경
```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
source install/setup.bash
ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py robot_ip:=172.16.0.1 use_rviz:=true 
```

## Controller 설정

### 1. Controller Manager
- **Type**: `MoveItSimpleControllerManager`
- **Package**: `moveit_simple_controller_manager`
- **Config File**: [isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_controllers.yaml](../../isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_controllers.yaml)

### 2. Arm Controller
```yaml
controller_names:
  - fr3_arm_controller
  - fr3_gripper

fr3_arm_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - fr3_joint1
    - fr3_joint2
    - fr3_joint3
    - fr3_joint4
    - fr3_joint5
    - fr3_joint6
    - fr3_joint7
```

- **Controller Name**: `fr3_arm_controller`
- **Controller Type**: `FollowJointTrajectory`
- **Action Interface**: `/fr3_arm_controller/follow_joint_trajectory`
- **Action Type**: `control_msgs/action/FollowJointTrajectory`
- **Controlled Joints**: 7-DOF (fr3_joint1 ~ fr3_joint7)

### 3. Planning & Trajectory Generation
- **Planner**: OMPL (Open Motion Planning Library)
- **Trajectory Parameterization**: Time Optimal Trajectory Generation (TOTG)
  - `resample_dt`: 0.01s (default) → ~100Hz waypoint 생성
  - `path_tolerance`: 0.1
  - `min_angle_change`: 0.001

**참조 파일:**
- Launch: [isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/fr3_kistar_moveit_bringup.launch.py](../../isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/fr3_kistar_moveit_bringup.launch.py)
- MoveIt Config: [isaac-ros/kistar_ws/src/franka_kistar_moveit_config/launch/moveit.launch.py](../../isaac-ros/kistar_ws/src/franka_kistar_moveit_config/launch/moveit.launch.py)

## Trajectory Output 형태

### Bridge Architecture

MoveIt에서 생성된 trajectory는 bridge 노드를 통해 실제 로봇으로 전달됩니다. Bridge 타입에 따라 두 가지 방식이 있습니다:

#### 1. Isaac Bridge (`bridge:=isaac`)

**Node**: `isaac_moveit_bridge`
**Source**: [isaac-ros/kistar_ws/src/franka_kistar_isaac_moveit/scripts/isaac_moveit_bridge.py](../../isaac-ros/kistar_ws/src/franka_kistar_isaac_moveit/scripts/isaac_moveit_bridge.py)

**동작 방식:**
- **Input**: FollowJointTrajectory 액션 요청 (전체 trajectory를 한번에 수신)
- **Output**: `/isaac_joint_commands` topic (JointState 메시지)

**Trajectory 처리:**
```python
# MoveIt으로부터 전체 trajectory 수신
# trajectory.points = [point_0, point_1, ..., point_N]

for i, point in enumerate(trajectory.points):
    # 각 waypoint를 순차적으로 publish
    cmd = JointState()
    cmd.name = joint_names
    cmd.position = point.positions

    publish(cmd)

    # time_from_start 기준으로 sleep
    dt = point.time_from_start - previous_time
    sleep(dt)
```

**특징:**
- ✅ 순차적 waypoint 전송 (하나씩)
- ✅ Trajectory의 타이밍 그대로 재현
- ⚠️ Real-time tracking 없음 (open-loop)

**Topic Interface:**
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/isaac_joint_states` | `sensor_msgs/JointState` | Input | Isaac Sim → Bridge |
| `/joint_states` | `sensor_msgs/JointState` | Output | Bridge → MoveIt |
| `/isaac_joint_commands` | `sensor_msgs/JointState` | Output | Bridge → Isaac Sim |
| `/fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Input | MoveIt → Bridge |

---
## Trajectory Execution 흐름

```
┌─────────────┐
│   MoveIt    │
│ move_group  │
└──────┬──────┘
       │ Action Goal
       │ (전체 trajectory 한번에 전달)
       │
       ▼
┌─────────────────────────────────────┐
│  FollowJointTrajectory Action       │
│  /fr3_arm_controller/               │
│  follow_joint_trajectory            │
└──────┬──────────────────────────────┘
       │
       ▼
┌─────────────────┬───────────────────┐
│  Isaac Bridge   │   Real Bridge     │
│  (순차 전송)      │   (연속 스트리밍)   │
└────────┬────────┴────────┬──────────┘
         │                 │
         │ JointState      │ FrankaArmTarget
         │ (waypoint별)     │ (100Hz)
         │                 │
         ▼                 ▼
   ┌──────────┐      ┌──────────┐
   │ Isaac    │      │  Real    │
   │   Sim    │      │  Robot   │
   └──────────┘      └──────────┘
```

## 핵심 차이점 요약

| 항목 | Isaac Bridge | Real Bridge |
|------|-------------|-------------|
| **Output Topic** | `/isaac_joint_commands` | `/franka/arm_target/{left\|right}` |
| **Message Type** | `sensor_msgs/JointState` | `kistar_hand_ros2/FrankaArmTarget` |
| **전송 방식** | 순차적 (waypoint별) | 연속적 (100Hz) |
| **제어 방식** | Open-loop | Closed-loop (PD + feedback) |
| **Interpolation** | ❌ (MoveIt waypoint 그대로) | ✅ (Cubic Hermite) |
| **Tracking** | ❌ | ✅ (Jerk-limited tracker) |
| **Feedback** | ❌ | ✅ (실시간 관절 상태) |
| **적용 환경** | Simulation | Real Robot |

## 참고 자료

- MoveIt2 Documentation: [FollowJointTrajectory Controller](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- ROS2 Control: [JointTrajectoryController](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
