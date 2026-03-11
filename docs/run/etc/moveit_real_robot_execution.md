# MoveIt Real Robot Execution (fr3_kistar_moveit_real.launch.py)

`ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py robot_ip:=172.16.0.1 use_rviz:=true` 실행 시 동작하는 시스템의 Input/Output 정보를 설명합니다.

## 실행 명령어

### 실제 로봇과 연결
```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
source install/setup.bash
ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py \
    robot_ip:=172.16.0.1 \
    use_rviz:=true
```

### Fake Hardware (로봇 없이 테스트)
```bash
ros2 launch franka_kistar_bringup fr3_kistar_moveit_real.launch.py \
    use_fake_hardware:=true \
    use_rviz:=true
```

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│  MoveIt move_group                                              │
│  - OMPL Planning                                                │
│  - TOTG (Time Optimal Trajectory Generation)                    │
└──────┬──────────────────────────────────────────────────────────┘
       │
       │ FollowJointTrajectory Action
       │ /fr3_arm_controller/follow_joint_trajectory
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│  JointTrajectoryController (ros2_controllers)                   │
│  - Type: joint_trajectory_controller/JointTrajectoryController  │
│  - Update Rate: 1000Hz                                          │
│  - PID Control (effort-based)                                   │
│                                                                  │
│  Input:                                                          │
│    - trajectory (JointTrajectory)                               │
│      positions[7], velocities[7], accelerations[7],            │
│      time_from_start                                            │
│                                                                  │
│  Processing:                                                     │
│    1. Cubic spline interpolation (1000Hz)                       │
│    2. PID control:                                              │
│       effort_cmd = Kp*(pos_ref - pos_actual) +                  │
│                    Kd*(vel_ref - vel_actual) +                  │
│                    Ki*integral(error)                           │
│                                                                  │
│  Output:                                                         │
│    - effort_cmd[7] → Hardware Interface                         │
└──────┬──────────────────────────────────────────────────────────┘
       │
       │ Command Interface (effort)
       │ effort_cmd[7]: [Nm, Nm, Nm, Nm, Nm, Nm, Nm]
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│  Hardware Interface (ros2_control)                              │
│  - libfranka (Franka Robot C++ library)                         │
│                                                                  │
│  write():                                                        │
│    robot_->control([effort_cmd](RobotState, Duration) {         │
│      return Torques(effort_cmd);                                │
│    });                                                           │
│                                                                  │
│  read():                                                         │
│    RobotState state = robot_->readOnce();                       │
│    joint_positions = state.q;                                   │
│    joint_velocities = state.dq;                                 │
│    joint_efforts = state.tau_J;                                 │
└──────┬──────────────────────────────────────────────────────────┘
       │
       │ FCI (Franka Control Interface)
       │ - EtherCAT protocol
       │ - 1000Hz real-time communication
       │
       ▼
┌─────────────────────────────────────────────────────────────────┐
│  Real Robot (Franka FR3)                                        │
│  - IP: 172.16.0.1 (or 172.16.0.3)                              │
│  - 7-DOF manipulator                                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## 주요 컴포넌트

### 1. Controller Manager

**Package**: `controller_manager`
**Executable**: `ros2_control_node`

**역할:**
- 모든 ros2_controllers 관리
- 1000Hz 업데이트 주기로 실행
- Hardware Interface와 Controller 연결

**Configuration** ([fr3_ros_controllers.yaml](../../isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_ros_controllers.yaml)):
```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
```

### 2. JointTrajectoryController

**Type**: `joint_trajectory_controller/JointTrajectoryController`
**Name**: `fr3_arm_controller`

**Configuration:**
```yaml
fr3_arm_controller:
  ros__parameters:
    command_interfaces:
      - effort           # Effort(torque) control mode
    state_interfaces:
      - position
      - velocity
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7
    gains:
      fr3_joint1: { p: 600., d: 30., i: 0., i_clamp: 1. }
      fr3_joint2: { p: 600., d: 30., i: 0., i_clamp: 1. }
      fr3_joint3: { p: 600., d: 30., i: 0., i_clamp: 1. }
      fr3_joint4: { p: 600., d: 30., i: 0., i_clamp: 1. }
      fr3_joint5: { p: 250., d: 10., i: 0., i_clamp: 1. }
      fr3_joint6: { p: 150., d: 10., i: 0., i_clamp: 1. }
      fr3_joint7: { p: 50., d: 5., i: 0., i_clamp: 1. }
```

**PID 제어 특성:**
- Joint 1-4: 높은 강성 (Kp=600, Kd=30)
- Joint 5-7: 낮은 강성 (Kp=250/150/50, Kd=10/10/5)
- Integral term: 0 (정상상태 오차 보정 없음)
- i_clamp: 1.0 (적분 windup 방지)

### 3. Joint State Broadcaster

**Type**: `joint_state_broadcaster/JointStateBroadcaster`
**Name**: `joint_state_broadcaster`

**역할:**
- Hardware Interface에서 읽은 joint states를 `/joint_states` topic으로 publish
- MoveIt과 RViz에 현재 로봇 상태 전달

### 4. Franka Robot State Broadcaster

**Type**: `franka_robot_state_broadcaster/FrankaRobotStateBroadcaster`
**Name**: `franka_robot_state_broadcaster`

**역할:**
- Franka 로봇의 확장 상태 정보 broadcast
- `/franka_robot_state` topic으로 publish
- Lock 메커니즘으로 동시 접근 제어

**Configuration:**
```yaml
franka_robot_state_broadcaster:
  ros__parameters:
    lock_try_count: 5
    lock_sleep_interval: 5  # microseconds
    lock_log_error: false
    lock_update_success: true
```

---

## Input / Output 정보

### Input (MoveIt → Controller)

#### 1. FollowJointTrajectory Action Goal

**Action Interface:**
- Name: `/fr3_arm_controller/follow_joint_trajectory`
- Type: `control_msgs/action/FollowJointTrajectory`

**Message Structure:**
```python
# Goal
trajectory_msgs/JointTrajectory trajectory
  std_msgs/Header header
  string[] joint_names: ["fr3_joint1", ..., "fr3_joint7"]
  JointTrajectoryPoint[] points:
    - float64[] positions       # 7개 관절 각도 (rad)
    - float64[] velocities      # 7개 관절 속도 (rad/s)
    - float64[] accelerations   # 7개 관절 가속도 (rad/s²)
    - Duration time_from_start  # 시작부터 경과 시간

# Example
Goal:
  trajectory:
    joint_names: ["fr3_joint1", "fr3_joint2", ..., "fr3_joint7"]
    points: [
      {
        positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 0, nanosec: 0}
      },
      {
        positions: [0.01, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78],
        velocities: [0.1, 0.05, 0.0, 0.06, 0.0, 0.01, 0.05],
        accelerations: [0.5, 0.2, 0.0, 0.3, 0.0, 0.1, 0.2],
        time_from_start: {sec: 0, nanosec: 10000000}  # 0.01s
      },
      # ... 100-200개 waypoints (resample_dt = 0.01s 기준)
    ]
```

**특징:**
- MoveIt은 **전체 trajectory를 한번에** 전송 (100-200개 waypoints)
- TOTG (Time Optimal Trajectory Generation)에 의해 생성
- positions, velocities, accelerations 모두 포함
- time_from_start: 각 waypoint 도달 시간

### Output (Controller → Robot)

#### 1. Effort Commands (Hardware Interface)

**Command Interface:**
- Type: `effort` (토크 제어)
- Update Rate: 1000Hz
- Units: Nm (Newton-meter)

**Command Flow:**
```
1000Hz 주기로:
  t = current_time
  ref = interpolate(trajectory, t)  # Cubic spline

  # PID control per joint
  for i in range(7):
    error_pos = ref.positions[i] - actual.positions[i]
    error_vel = ref.velocities[i] - actual.velocities[i]

    effort_cmd[i] = gains[i].p * error_pos +
                    gains[i].d * error_vel +
                    gains[i].i * integral_error[i]

  # Send to robot via libfranka
  robot.send_torque_command(effort_cmd)
```

**Example Command:**
```cpp
// Hardware Interface write() 함수에서
Torques effort_cmd = {
  15.2,   // fr3_joint1 torque (Nm)
  -8.5,   // fr3_joint2
  3.1,    // fr3_joint3
  -12.3,  // fr3_joint4
  2.5,    // fr3_joint5
  -1.8,   // fr3_joint6
  0.5     // fr3_joint7
};
robot_->control([&effort_cmd](const RobotState&, Duration) {
  return effort_cmd;
});
```

#### 2. Joint States (Feedback)

**Topic Interface:**

| Topic | Type | Rate | Direction | Description |
|-------|------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 1000Hz | Controller → MoveIt/RViz | 현재 관절 상태 |
| `/franka/joint_states` | `sensor_msgs/JointState` | 1000Hz | Hardware → Controller | 로봇 raw 상태 |

**Message Structure:**
```python
# /joint_states
sensor_msgs/JointState:
  header:
    stamp: current_time
    frame_id: ""
  name: ["fr3_joint1", "fr3_joint2", ..., "fr3_joint7"]
  position: [0.01, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78]  # rad
  velocity: [0.1, 0.05, 0.0, 0.06, 0.0, 0.01, 0.05]     # rad/s
  effort: [15.2, -8.5, 3.1, -12.3, 2.5, -1.8, 0.5]      # Nm
```

#### 3. Action Feedback / Result

**Feedback (실행 중 주기적):**
```python
# control_msgs/action/FollowJointTrajectory.Feedback
Feedback:
  header: {stamp: current_time}
  joint_names: ["fr3_joint1", ..., "fr3_joint7"]
  desired:    # 현재 목표값 (interpolated)
    positions: [...]
    velocities: [...]
  actual:     # 현재 실제값
    positions: [...]
    velocities: [...]
  error:      # 오차
    positions: [...]
    velocities: [...]
```

**Result (완료 시):**
```python
# control_msgs/action/FollowJointTrajectory.Result
Result:
  error_code: 0  # SUCCESSFUL
  error_string: ""
```

**Error Codes:**
- `0` (SUCCESSFUL): 정상 완료
- `-1` (INVALID_GOAL): 잘못된 목표
- `-2` (INVALID_JOINTS): 관절 이름 불일치
- `-4` (PATH_TOLERANCE_VIOLATED): 경로 허용 오차 초과
- `-5` (GOAL_TOLERANCE_VIOLATED): 목표 허용 오차 초과

---

## Topic/Action/Service 전체 리스트

### Actions

| Name | Type | Direction | Description |
|------|------|-----------|-------------|
| `/fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt → Controller | Trajectory 실행 |

### Topics (Published)

| Name | Type | Rate | Description |
|------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 1000Hz | 현재 관절 상태 (MoveIt/RViz용) |
| `/franka/joint_states` | `sensor_msgs/JointState` | 1000Hz | 로봇 raw 관절 상태 |
| `/franka_robot_state` | `franka_msgs/FrankaState` | 1000Hz | Franka 확장 상태 정보 |
| `/tf` | `tf2_msgs/TFMessage` | 실시간 | 동적 TF |
| `/tf_static` | `tf2_msgs/TFMessage` | Latch | 정적 TF (world, base, table 등) |
| `/planning_scene` | `moveit_msgs/PlanningScene` | 비주기 | MoveIt planning scene |
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | 비주기 | RViz trajectory 시각화 |

### Topics (Subscribed)

| Name | Type | Source | Description |
|------|------|--------|-------------|
| `/franka/joint_states` | `sensor_msgs/JointState` | Hardware Interface | 로봇 상태 읽기 |

### Services

| Name | Type | Description |
|------|------|-------------|
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Controller 목록 조회 |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | Controller 전환 |
| `/plan_kinematic_path` | `moveit_msgs/srv/GetMotionPlan` | MoveIt planning 요청 |

---

## Execution 타이밍 다이어그램

```
Time │ MoveIt                │ Controller (1000Hz)    │ Robot (Franka)
─────┼───────────────────────┼────────────────────────┼─────────────────
0.0s │ Planning...           │                        │
     │ (0.5-2s)              │                        │
─────┼───────────────────────┼────────────────────────┼─────────────────
1.5s │ TOTG...               │                        │
     │ (0.1-0.3s)            │                        │
─────┼───────────────────────┼────────────────────────┼─────────────────
1.8s │ Send Action Goal      │ Receive trajectory     │
     │ (전체 trajectory)      │ (100-200 waypoints)    │
─────┼───────────────────────┼────────────────────────┼─────────────────
1.8s │                       │ t=0.000: interp→cmd[0] │ Execute effort[0]
     │                       │   effort = PID(...)    │   (1ms)
─────┼───────────────────────┼────────────────────────┼─────────────────
1.801│                       │ t=0.001: interp→cmd[1] │ Execute effort[1]
     │                       │   effort = PID(...)    │   (1ms)
─────┼───────────────────────┼────────────────────────┼─────────────────
1.802│                       │ t=0.002: interp→cmd[2] │ Execute effort[2]
...  │                       │   ...                  │   ...
─────┼───────────────────────┼────────────────────────┼─────────────────
4.8s │                       │ t=3.000: interp→cmd[N] │ Execute effort[N]
     │                       │ Trajectory done        │ Reached goal
─────┼───────────────────────┼────────────────────────┼─────────────────
4.8s │ Receive Result        │ Send Result            │
     │   SUCCESSFUL          │   error_code = 0       │
─────┴───────────────────────┴────────────────────────┴─────────────────
```

**핵심:**
- MoveIt: Planning (0.5-2s) + TOTG (0.1-0.3s) → 전체 trajectory 전송
- Controller: 1000Hz 주기로 interpolation + PID → effort 명령 생성
- Robot: 1ms(1000Hz)마다 effort 명령 실행 → 관절 토크 제어

---

## 실행 흐름 요약

### 1. Planning Phase (MoveIt)

```
User (RViz) → Plan 버튼 클릭
  ↓
MoveIt move_group:
  1. 현재 상태 읽기 (/joint_states)
  2. OMPL planning (장애물 회피 경로 생성)
  3. TOTG (시간 최적 궤적 생성)
     - resample_dt = 0.01s
     - positions, velocities, accelerations 계산
  4. JointTrajectory 생성 (100-200 waypoints)
```

### 2. Execution Phase (Controller)

```
User (RViz) → Execute 버튼 클릭
  ↓
MoveIt → FollowJointTrajectory Action Goal 전송
  ↓
JointTrajectoryController:
  1. Trajectory 수신 (전체 points)
  2. 1000Hz loop 시작:
     While t < trajectory_duration:
       a. t에 해당하는 reference 계산 (cubic spline)
       b. 현재 상태 읽기 (read from Hardware Interface)
       c. PID control:
          effort = Kp*(ref.pos - actual.pos) +
                   Kd*(ref.vel - actual.vel)
       d. effort 명령 전송 (write to Hardware Interface)
       e. Feedback publish
       f. sleep(1ms)
  3. Result 전송 (SUCCESSFUL)
```

### 3. Hardware Interface (libfranka)

```
write(effort_cmd):
  robot_->control([effort_cmd] {
    return Torques(effort_cmd);  # EtherCAT → Robot
  });

read():
  state = robot_->readOnce();    # EtherCAT ← Robot
  joint_positions = state.q;
  joint_velocities = state.dq;
  joint_efforts = state.tau_J;
```

---

## 주요 파일

| 파일 | 경로 | 설명 |
|------|------|------|
| Launch 파일 | [isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/fr3_kistar_moveit_real.launch.py](../../isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/fr3_kistar_moveit_real.launch.py) | 전체 시스템 실행 |
| Controller 설정 | [isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_ros_controllers.yaml](../../isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_ros_controllers.yaml) | JointTrajectoryController 설정 |
| MoveIt Controller | [isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_controllers.yaml](../../isaac-ros/kistar_ws/src/franka_kistar_moveit_config/config/fr3_controllers.yaml) | MoveIt SimpleControllerManager 설정 |
| URDF/xacro | isaac-ros/kistar_ws/src/franka_kistar_description/urdf/fr3_kistar.urdf.xacro | Robot description (ros2_control 포함) |

---

## 디버깅 명령어

### Controller 상태 확인
```bash
# Controller 목록
ros2 control list_controllers

# 출력:
# fr3_arm_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# franka_robot_state_broadcaster[franka_robot_state_broadcaster/FrankaRobotStateBroadcaster] active

# Controller 정보
ros2 control list_hardware_interfaces

# 출력:
# Command interfaces:
#   fr3_joint1/effort [available] [claimed]
#   fr3_joint2/effort [available] [claimed]
#   ...
# State interfaces:
#   fr3_joint1/position [available]
#   fr3_joint1/velocity [available]
#   ...
```

### Topic 확인
```bash
# Joint states 확인
ros2 topic echo /joint_states

# Action 확인
ros2 action list
# /fr3_arm_controller/follow_joint_trajectory

# Action 정보
ros2 action info /fr3_arm_controller/follow_joint_trajectory
```

### 수동 Trajectory 전송 (테스트)
```bash
# 간단한 trajectory 전송
ros2 action send_goal /fr3_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
      points: [
        {
          positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
          velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 0, nanosec: 0}
        },
        {
          positions: [0.1, -0.7, 0.0, -2.3, 0.0, 1.6, 0.8],
          velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2, nanosec: 0}
        }
      ]
    }
  }"
```

---

## 참고 자료

- [ROS2 Control Documentation](https://control.ros.org/)
- [JointTrajectoryController](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [Franka ROS2 Documentation](https://frankaemika.github.io/docs/)
- [libfranka Documentation](https://frankaemika.github.io/libfranka/)
- [MoveIt2 Controller Configuration](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
