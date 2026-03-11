# MoveIt Trajectory Execution Guide

MoveIt에서 생성된 trajectory의 구조와 표준 ROS2 Control을 사용한 실행 방법, 그리고 분산 시스템 구성 방법을 설명합니다.

## MoveIt Trajectory 데이터 구조

### 1. JointTrajectory Message Type

MoveIt에서 생성되는 trajectory는 `trajectory_msgs/JointTrajectory` 타입입니다.

```python
# trajectory_msgs/JointTrajectory
std_msgs/Header header          # timestamp와 frame_id
string[] joint_names            # 관절 이름 리스트 (예: ["fr3_joint1", "fr3_joint2", ...])
JointTrajectoryPoint[] points   # Trajectory waypoints 배열
```

### 2. JointTrajectoryPoint 구조

각 waypoint는 다음 정보를 포함합니다:

```python
# trajectory_msgs/JointTrajectoryPoint
float64[] positions         # 필수: 각 관절의 목표 위치 (rad or m)
float64[] velocities        # 선택: 각 관절의 목표 속도 (rad/s or m/s)
float64[] accelerations     # 선택: 각 관절의 목표 가속도 (rad/s² or m/s²)
float64[] effort            # 선택: 각 관절의 힘/토크 (N or Nm)
Duration time_from_start    # 시작점으로부터 경과 시간 (sec + nanosec)
```

### 3. MoveIt이 생성하는 Trajectory 특성

**TOTG (Time Optimal Trajectory Generation) 기준:**
- **positions**: ✅ 항상 포함 (모든 waypoint의 관절 각도)
- **velocities**: ✅ 포함 (TOTG가 계산한 속도 프로파일)
- **accelerations**: ✅ 포함 (TOTG가 계산한 가속도 프로파일)
- **effort**: ❌ 일반적으로 비어있음
- **time_from_start**: ✅ 항상 포함 (각 waypoint 도달 시간)

**Dimension:**
- `len(joint_names)`: 7 (Franka FR3의 경우)
- `len(points)`: 가변적 (resample_dt에 따라 결정)
  - `resample_dt = 0.01s` → 1초 trajectory = 약 100개 waypoints
  - `resample_dt = 0.02s` → 1초 trajectory = 약 50개 waypoints

**Example Trajectory:**
```python
trajectory = JointTrajectory(
    header=Header(stamp=..., frame_id="world"),
    joint_names=["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
                 "fr3_joint5", "fr3_joint6", "fr3_joint7"],
    points=[
        JointTrajectoryPoint(
            positions=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            time_from_start=Duration(sec=0, nanosec=0)
        ),
        JointTrajectoryPoint(
            positions=[0.01, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78],
            velocities=[0.1, 0.05, 0.0, 0.06, 0.0, 0.01, 0.05],
            accelerations=[0.5, 0.2, 0.0, 0.3, 0.0, 0.1, 0.2],
            time_from_start=Duration(sec=0, nanosec=10000000)  # 0.01s
        ),
        # ... 더 많은 waypoints (보통 100-200개)
    ]
)
```

### 4. FollowJointTrajectory Action Interface

MoveIt은 trajectory를 **FollowJointTrajectory Action**을 통해 전달합니다.

```python
# control_msgs/action/FollowJointTrajectory

# Goal
trajectory_msgs/JointTrajectory trajectory
trajectory_msgs/MultiDOFJointTrajectory multi_dof_trajectory
JointTolerance[] path_tolerance
JointTolerance[] goal_tolerance
builtin_interfaces/Duration goal_time_tolerance

---
# Result
int32 error_code
  SUCCESSFUL = 0
  INVALID_GOAL = -1
  INVALID_JOINTS = -2
  PATH_TOLERANCE_VIOLATED = -4
  GOAL_TOLERANCE_VIOLATED = -5
string error_string

---
# Feedback (실행 중 주기적으로 전송)
std_msgs/Header header
string[] joint_names
JointTrajectoryPoint desired   # 현재 목표값
JointTrajectoryPoint actual    # 현재 실제값
JointTrajectoryPoint error     # 오차
```

**핵심:**
- MoveIt은 **전체 trajectory를 한번에** Action Goal로 전송
- 개별 waypoint가 아닌 **points 배열 전체**를 전달
- Action Server는 trajectory를 받아 실시간으로 실행하며 Feedback 전송
- 완료 시 Result 반환

---

## 표준 ROS2 Control Execution 흐름

### 1. 전체 아키텍처

```
┌─────────────────────────────────────────┐
│  MoveIt (move_group)                    │
│  - OMPL Planning                        │
│  - TOTG (Time Optimal Traj. Gen.)      │
└──────┬──────────────────────────────────┘
       │
       │ FollowJointTrajectory Action Goal
       │ trajectory = JointTrajectory(
       │    joint_names=[j1, j2, ..., j7],
       │    points=[
       │       {pos, vel, acc, time_from_start},
       │       {pos, vel, acc, time_from_start},
       │       ...  # 100-200개 waypoints
       │    ]
       │ )
       │
       ▼
┌──────────────────────────────────────────┐
│  JointTrajectoryController               │
│  (ros2_controllers)                      │
│                                          │
│  1. Trajectory 수신 (전체 points)        │
│  2. 내부 interpolation (cubic spline)   │
│  3. 실시간 setpoint 계산                 │
│     - Update rate: 100-1000Hz           │
│     - t=0.00s: interpolate(t) → cmd     │
│     - t=0.01s: interpolate(t) → cmd     │
│     - t=0.02s: interpolate(t) → cmd     │
│  4. PID control (선택)                   │
│  5. Feedback publish                    │
└──────┬───────────────────────────────────┘
       │
       │ Command Interface
       │ - position_command[7]  or
       │ - velocity_command[7]  or
       │ - effort_command[7]
       │ (controller update cycle마다 갱신)
       │
       ▼
┌──────────────────────────────────────────┐
│  Hardware Interface (ros2_control)       │
│                                          │
│  write():                                │
│    - 명령을 하드웨어 프로토콜로 변환      │
│    - EtherCAT, CAN, Ethernet, etc.      │
│    - robot.send_command(positions)      │
│                                          │
│  read():                                 │
│    - 센서 데이터 읽기                     │
│    - robot.get_state() → joint states   │
└──────┬───────────────────────────────────┘
       │
       │ Hardware-specific protocol
       │ (EtherCAT, libfranka, CAN, etc.)
       │
       ▼
┌──────────────────────────────────────────┐
│  Real Robot (Franka FR3)                 │
└──────────────────────────────────────────┘
```

### 2. 핵심 컴포넌트

#### JointTrajectoryController (ros2_controllers)

**역할:**
- MoveIt의 trajectory를 받아 실시간으로 실행
- 내부적으로 cubic spline interpolation 수행
- Controller update cycle마다 새로운 setpoint 계산 (100-1000Hz)
- 선택적으로 PID feedback control 적용

**동작 방식:**
```python
class JointTrajectoryController:
    def __init__(self):
        self.update_rate = 100  # Hz
        self.current_trajectory = None
        self.start_time = None

    def on_action_goal(self, goal):
        """MoveIt으로부터 trajectory 수신"""
        self.current_trajectory = goal.trajectory  # 전체 points
        self.start_time = now()
        return ACCEPT

    def update(self):
        """100Hz로 호출됨 (controller update cycle)"""
        if not self.current_trajectory:
            return

        # 현재 시간에 해당하는 setpoint 계산 (interpolation)
        t = now() - self.start_time
        setpoint = self.interpolate(self.current_trajectory, t)

        # PID control (선택)
        current_state = self.read_joint_states()
        error = setpoint.position - current_state.position
        command = setpoint.position + self.kp * error

        # 하드웨어에 명령 전송
        self.write_command(command)

        # Feedback publish
        self.publish_feedback(setpoint, current_state)

    def interpolate(self, trajectory, t):
        """Cubic spline interpolation"""
        # trajectory.points에서 t에 해당하는 위치 계산
        # positions, velocities, accelerations 모두 interpolation
        return interpolated_setpoint
```

**장점:**
- ✅ 표준 ROS2 Control 사용
- ✅ 실시간 interpolation (부드러운 움직임)
- ✅ Feedback control (정확도 향상)
- ✅ 다양한 로봇에 재사용 가능

#### Hardware Interface (ros2_control)

**역할:**
- 로봇 하드웨어와의 실제 통신
- `read()`: 센서 데이터 읽기
- `write()`: 모터 명령 전송

**Example (Franka):**
```cpp
class FrankaHardwareInterface : public hardware_interface::SystemInterface {
public:
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // 로봇으로부터 현재 상태 읽기
    franka::RobotState state = robot_->readOnce();
    for (size_t i = 0; i < 7; ++i) {
      joint_positions_[i] = state.q[i];      // position
      joint_velocities_[i] = state.dq[i];    // velocity
      joint_efforts_[i] = state.tau_J[i];    // torque
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // 로봇에 명령 전송 (libfranka 사용)
    robot_->control([this](const franka::RobotState&, franka::Duration) {
      return franka::JointPositions({
        joint_commands_[0], joint_commands_[1], joint_commands_[2],
        joint_commands_[3], joint_commands_[4], joint_commands_[5],
        joint_commands_[6]
      });
    });
    return return_type::OK;
  }

private:
  std::unique_ptr<franka::Robot> robot_;
  std::array<double, 7> joint_positions_;
  std::array<double, 7> joint_velocities_;
  std::array<double, 7> joint_efforts_;
  std::array<double, 7> joint_commands_;
};
```

### 3. Execution 타이밍

```
MoveIt Planning: 0.5-2s
   ↓
Trajectory Generation (TOTG): 0.1-0.3s
   ↓ (전체 trajectory 한번에 전송)
JointTrajectoryController:
   t=0.000s: interpolate() → cmd[0]  → write() → robot
   t=0.010s: interpolate() → cmd[1]  → write() → robot
   t=0.020s: interpolate() → cmd[2]  → write() → robot
   t=0.030s: interpolate() → cmd[3]  → write() → robot
   ...
   t=3.000s: interpolate() → cmd[N]  → write() → robot
   ↓
Action Result: SUCCESSFUL
```

**핵심:**
- MoveIt은 planning 후 **전체 trajectory를 한번에** 전송
- Controller는 **실시간으로 interpolation**하며 명령 생성
- 로봇에는 **controller update rate (100-1000Hz)** 주기로 명령 전송

---

## 분산 시스템 구성 (Multi-Computer Setup)

### 시나리오: MoveIt PC → Robot Control PC

MoveIt을 실행하는 컴퓨터와 실제 로봇을 제어하는 컴퓨터가 분리된 경우의 구성 방법입니다.

### 1. 시스템 아키텍처

```
┌────────────────────────────────┐         ┌────────────────────────────────┐
│  PC 1: MoveIt Planning         │         │  PC 2: Robot Control           │
│  (192.168.1.100)               │         │  (192.168.1.200)               │
│                                │         │                                │
│  ┌──────────────────┐          │         │  ┌──────────────────────┐      │
│  │   MoveIt         │          │         │  │  JointTrajectory     │      │
│  │   move_group     │          │         │  │  Controller          │      │
│  └────────┬─────────┘          │         │  │  (ros2_controllers)  │      │
│           │                    │         │  └──────┬───────────────┘      │
│           │ Action Client      │  ROS2   │         │                      │
│           │                    │  DDS    │         │ Action Server        │
│           │                    ◄────────►│         │ (Network를 통해       │
│  ┌────────▼─────────┐          │         │  ┌──────▼───────────────┐      │
│  │ FollowJoint      │          │         │  │  Hardware Interface  │      │
│  │ Trajectory       │          │         │  │  (ros2_control)      │      │
│  │ Action Client    │          │         │  │                      │      │
│  └──────────────────┘          │         │  │  - read(): 센서      │      │
│                                │         │  │  - write(): 명령     │      │
│  ┌──────────────────┐          │         │  └──────┬───────────────┘      │
│  │  RViz2           │          │         │         │                      │
│  │  (Visualization) │          │         │  ┌──────▼───────────────┐      │
│  └──────────────────┘          │         │  │  Real Robot          │      │
│                                │         │  │  (Franka FR3)        │      │
└────────────────────────────────┘         │  │  - libfranka         │      │
                                           │  │  - EtherCAT          │      │
                                           │  └──────────────────────┘      │
                                           │                                │
                                           └────────────────────────────────┘
```

**핵심:**
- PC1: MoveIt planning만 수행, trajectory 생성
- PC2: Trajectory 실행 + 로봇 제어
- **ROS2 DDS**: 네트워크를 통해 Action 통신 (투명하게 동작)

### 2. ROS2 DDS 네트워크 설정

ROS2는 DDS (Data Distribution Service)를 사용하여 네트워크 통신을 자동으로 처리합니다.

#### PC 1 (MoveIt Planning PC) 설정

```bash
# ~/.bashrc 또는 launch 전 설정
export ROS_DOMAIN_ID=42          # 같은 도메인 ID 사용
export ROS_LOCALHOST_ONLY=0      # 네트워크 통신 활성화 (중요!)

# (선택) 특정 네트워크 인터페이스 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # or rmw_fastrtps_cpp

# (선택) Cyclone DDS 설정 파일
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

**cyclonedds.xml (선택):**
```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain>
    <General>
      <NetworkInterfaceAddress>192.168.1.0</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

#### PC 2 (Robot Control PC) 설정

```bash
# ~/.bashrc 또는 launch 전 설정
export ROS_DOMAIN_ID=42          # PC1과 동일 (필수!)
export ROS_LOCALHOST_ONLY=0      # 네트워크 통신 활성화 (필수!)

# DDS 구현 선택 (PC1과 동일하게)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**중요:**
- `ROS_DOMAIN_ID`: 양쪽 PC에서 **동일해야 함**
- `ROS_LOCALHOST_ONLY=0`: 네트워크 통신을 위해 **반드시 0으로 설정**
- 방화벽: UDP ports 7400-7500 허용 필요

### 3. PC 2 (Robot Control PC) 구현

#### 방법 1: 표준 ros2_control 사용 (권장)

**구성 파일 (`robot_controller.yaml`):**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz (로봇 제어 주기)

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - fr3_joint1
      - fr3_joint2
      - fr3_joint3
      - fr3_joint4
      - fr3_joint5
      - fr3_joint6
      - fr3_joint7

    command_interfaces:
      - position  # or velocity, effort

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 100.0       # joint_states publish rate
    action_monitor_rate: 20.0       # action feedback rate

    allow_partial_joints_goal: false
    allow_integration_in_goal_trajectories: true

    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5  # trajectory 완료 후 대기 시간

    # 각 관절별 PID gains (선택)
    gains:
      fr3_joint1: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint2: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint3: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint4: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint5: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint6: {p: 100.0, i: 0.0, d: 10.0}
      fr3_joint7: {p: 100.0, i: 0.0, d: 10.0}
```

**Launch 파일 (`robot_control.launch.py`):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # URDF 로드
    robot_description = ...  # URDF/xacro 파일에서 로드

    return LaunchDescription([
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                os.path.join(pkg_share, 'config', 'robot_controller.yaml'),
            ],
            output='screen',
        ),

        # Joint State Broadcaster 실행
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_state_broadcaster'],
            output='screen',
        ),

        # Joint Trajectory Controller 실행
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                 'joint_trajectory_controller'],
            output='screen',
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
    ])
```

**Hardware Interface 구현 (`franka_hardware.cpp`):**
```cpp
#include <hardware_interface/system_interface.hpp>
#include <franka/robot.h>

namespace franka_hardware
{

class FrankaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    // URDF에서 정보 읽기
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Joint 정보 초기화
    joint_positions_.resize(7, 0.0);
    joint_velocities_.resize(7, 0.0);
    joint_efforts_.resize(7, 0.0);
    joint_commands_.resize(7, 0.0);

    // Franka 로봇 연결
    std::string robot_ip = info_.hardware_parameters["robot_ip"];
    robot_ = std::make_unique<franka::Robot>(robot_ip);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 로봇 활성화
    robot_->setCollisionBehavior(...);
    robot_->setJointImpedance(...);
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < 7; ++i) {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < 7; ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_commands_[i]);
    }
    return command_interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // 로봇 상태 읽기
    franka::RobotState state = robot_->readOnce();
    for (size_t i = 0; i < 7; ++i) {
      joint_positions_[i] = state.q[i];
      joint_velocities_[i] = state.dq[i];
      joint_efforts_[i] = state.tau_J[i];
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // 로봇에 명령 전송
    // Franka는 real-time control loop에서만 명령 전송 가능
    // libfranka motion generator 사용
    robot_->control([this](const franka::RobotState&, franka::Duration) {
      return franka::JointPositions({
        joint_commands_[0], joint_commands_[1], joint_commands_[2],
        joint_commands_[3], joint_commands_[4], joint_commands_[5],
        joint_commands_[6]
      });
    });
    return return_type::OK;
  }

private:
  std::unique_ptr<franka::Robot> robot_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_commands_;
};

}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface, hardware_interface::SystemInterface)
```

#### 방법 2: Custom Action Server 구현 (간단한 경우)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from control_msgs.action import FollowJointTrajectory
import time

class SimpleTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # Action Server 생성 (네트워크를 통해 MoveIt과 통신)
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
        )

        # 로봇 연결 (하드웨어별 구현)
        self.robot = self.connect_robot()  # libfranka, ROS topic 등

        self.get_logger().info('Trajectory Executor started (waiting for MoveIt trajectory)')

    def goal_callback(self, goal_request):
        """MoveIt으로부터 trajectory 수신"""
        trajectory = goal_request.trajectory
        self.get_logger().info(
            f'[Goal] Received trajectory: {len(trajectory.points)} points, '
            f'duration: {trajectory.points[-1].time_from_start.sec}s'
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """전체 trajectory 실행"""
        trajectory = goal_handle.request.trajectory
        n_points = len(trajectory.points)

        if n_points == 0:
            goal_handle.succeed()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL
            )

        self.get_logger().info(f'Executing trajectory ({n_points} waypoints)')

        # 옵션 1: 전체 trajectory를 로봇에 한번에 전송 (로봇이 지원하는 경우)
        if self.robot.supports_trajectory_mode():
            self.robot.execute_trajectory(trajectory)
            goal_handle.succeed()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL
            )

        # 옵션 2: Waypoint별로 순차 실행 (간단한 방법)
        start_time = time.time()
        for i, point in enumerate(trajectory.points):
            # 취소 요청 확인
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            # 시간 동기화
            target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            while (time.time() - start_time) < target_time:
                time.sleep(0.001)

            # 로봇에 명령 전송
            self.robot.move_to(
                positions=point.positions,
                velocities=point.velocities if point.velocities else None
            )

            # Feedback publish
            feedback = FollowJointTrajectory.Feedback()
            feedback.desired.positions = point.positions
            feedback.actual.positions = self.robot.get_joint_positions()
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f'[{i+1}/{n_points}] Waypoint executed')

        # 성공 반환
        goal_handle.succeed()
        return FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL
        )

    def connect_robot(self):
        """로봇 연결 (하드웨어별 구현)"""
        # Franka libfranka 사용 예시
        # from franka import Robot
        # return Robot("172.16.0.3")
        pass

def main():
    rclpy.init()
    executor = SimpleTrajectoryExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4. 실행 순서

#### PC 2 (Robot Control PC) - 먼저 실행

```bash
# Terminal 1: Controller Manager & Hardware Interface 실행
cd ~/robot_ws
source install/setup.bash

# 환경 변수 설정
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# 로봇 제어 노드 실행
ros2 launch franka_hardware robot_control.launch.py

# 확인: Action server가 동작하는지 체크
ros2 action list
# 출력: /fr3_arm_controller/follow_joint_trajectory

ros2 node list
# 출력:
#   /controller_manager
#   /joint_state_broadcaster
#   /joint_trajectory_controller
#   /robot_state_publisher
```

#### PC 1 (MoveIt Planning PC)

```bash
# Terminal 1: MoveIt 실행
cd ~/moveit_ws
source install/setup.bash

# 환경 변수 설정 (PC2와 동일!)
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# MoveIt 실행
ros2 launch franka_moveit_config demo.launch.py

# 확인: PC2의 action server가 보이는지 체크
ros2 action list
# 출력: /fr3_arm_controller/follow_joint_trajectory (PC2에서 제공)

ros2 topic echo /joint_states
# PC2에서 publish하는 joint_states가 보여야 함
```

**RViz에서 trajectory planning & execution:**
1. RViz2에서 Planning 탭 선택
2. Goal State 설정
3. Plan 버튼 클릭 → MoveIt이 trajectory 생성
4. Execute 버튼 클릭 → **네트워크를 통해 PC2로 전송 → 로봇 실행**

### 5. 네트워크 디버깅

```bash
# PC1과 PC2가 서로 보이는지 확인
ros2 node list
# 양쪽 PC의 노드가 모두 보여야 함

ros2 topic list
# 양쪽 PC의 topic이 모두 보여야 함

ros2 action list
# PC2의 action server가 PC1에서 보여야 함

# DDS discovery 확인
ros2 doctor --report

# Topic 통신 테스트
# PC1에서 실행
ros2 topic echo /joint_states
# PC2의 joint_states가 실시간으로 보여야 함

# Action 통신 테스트
# PC1에서 실행
ros2 action send_goal /fr3_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{...}"
# PC2의 로봇이 움직여야 함
```

**문제 해결:**

| 증상 | 원인 | 해결 |
|------|------|------|
| `ros2 node list` 에서 상대 PC 노드 안보임 | `ROS_LOCALHOST_ONLY=1` | `export ROS_LOCALHOST_ONLY=0` |
| Topic은 보이지만 데이터 안옴 | 방화벽 | UDP 7400-7500 포트 허용 |
| `ROS_DOMAIN_ID` 불일치 | 다른 도메인 ID | 양쪽 PC 동일하게 설정 |
| DDS 구현 불일치 | PC1: FastRTPS, PC2: Cyclone | `RMW_IMPLEMENTATION` 통일 |

### 6. 장점

**분산 시스템의 이점:**
- ✅ **Computing 분산**: Planning은 강력한 PC, 제어는 Real-time PC
- ✅ **안전성**: 로봇 PC 문제 시에도 Planning PC는 정상 동작
- ✅ **유지보수**: 각 PC 독립적 업데이트 가능
- ✅ **확장성**: 여러 로봇을 한 Planning PC에서 제어 가능
- ✅ **개발 편의성**: Planning 개발 시 로봇 없이 시뮬레이션 가능

---

## 요약

### MoveIt Trajectory 특성
- **타입**: `trajectory_msgs/JointTrajectory`
- **포함 데이터**: positions (필수), velocities, accelerations, time_from_start
- **Dimension**: 7개 관절 × 100-200개 waypoints (resample_dt에 따라)
- **전송 방식**: 전체 trajectory를 **한번에** Action Goal로 전송

### Execution 방식

| 항목 | 표준 ROS2 Control | Custom Bridge |
|------|-------------------|---------------|
| **사용 패키지** | ros2_controllers | 직접 구현 |
| **Controller** | JointTrajectoryController | Action Server (직접 구현) |
| **Interpolation** | ✅ Cubic spline (내부) | ❌ or 직접 구현 |
| **Update Rate** | 100-1000Hz | 가변적 |
| **Feedback** | ✅ 실시간 | 선택적 |
| **재사용성** | ✅ 높음 | ❌ 로봇별 구현 |
| **권장 용도** | 실제 로봇 | Simulation or 간단한 경우 |

### 분산 시스템
- **ROS2 DDS**: 자동 네트워크 통신 (Action, Topic, Service)
- **설정**: `ROS_DOMAIN_ID` (동일), `ROS_LOCALHOST_ONLY=0`
- **PC1**: MoveIt planning
- **PC2**: Trajectory execution + 로봇 제어
- **통신**: Transparent (코드 변경 불필요)

---

## 참고 자료

- [ROS2 Control Documentation](https://control.ros.org/)
- [JointTrajectoryController](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
- [MoveIt2 Controller Configuration](https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html)
- [ROS2 DDS Configuration](https://docs.ros.org/en/humble/Guides/DDS-Configuration.html)
- [Franka libfranka Documentation](https://frankaemika.github.io/libfranka/)
