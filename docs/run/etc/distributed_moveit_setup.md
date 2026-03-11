# Distributed MoveIt Setup Guide

MoveIt Planning을 한 컴퓨터에서 수행하고, 생성된 trajectory를 topic으로 publish하여 다른 컴퓨터로 전송하는 시스템입니다.

---

## 빠른 시작 가이드

### PC1 (Planning 컴퓨터 - 192.168.1.38)

```bash
# 1. 네트워크 설정 (~/.bashrc에 추가 후 source ~/.bashrc)
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0

# 2. 빌드 및 실행
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch franka_kistar_bringup moveit_planning_pc.launch.py
```

**RViz 사용법:**
1. Planning 탭 → Goal State 설정 (Interactive Marker 드래그 또는 Joint 값 직접 입력)
2. **"Plan"** 버튼 클릭 → 경로 계산
3. **"Execute"** 버튼 클릭 → `/trajectory_commands` 토픽으로 trajectory publish

### PC2 (로봇 제어 컴퓨터 - 192.168.1.250)

```bash
# 1. 네트워크 설정 (~/.bashrc에 추가 후 source ~/.bashrc)
export ROS_DOMAIN_ID=9          # PC1과 반드시 동일!
export ROS_LOCALHOST_ONLY=0     # 필수!

# 2. Trajectory subscriber 실행 (아래 예시 코드 참고하여 직접 구현)
python3 your_trajectory_executor.py
```

**PC2 구현 내용:**
- `/trajectory_commands` 토픽 subscribe
- 받은 `trajectory_msgs/msg/JointTrajectory` 메시지를 로봇 제어 코드로 전달
- 로봇이 부드럽게 움직이도록 trajectory 실행

### Topic 확인

```bash
# PC1 또는 PC2 어디서나
ros2 topic list | grep trajectory
# 출력: /trajectory_commands

# Trajectory 데이터 확인 (PC1에서 Execute 버튼 클릭 시)
ros2 topic echo /trajectory_commands
```

---

## 시스템 개요

```
┌────────────────────────────────────┐         ┌────────────────────────────────────┐
│  PC1: Planning Computer            │         │  PC2: Robot Execution Computer     │
│  (192.168.1.38)                    │         │  (192.168.1.250)                   │
│                                    │         │                                    │
│  ┌──────────────────────┐          │         │  ┌──────────────────────────┐      │
│  │  MoveIt move_group   │          │         │  │  Trajectory Subscriber   │      │
│  │  - GUI (RViz)        │          │         │  │  (사용자 구현)            │      │
│  │  - OMPL Planning     │          │         │  │                          │      │
│  │  - TOTG              │          │         │  │  ↓                       │      │
│  └─────────┬────────────┘          │         │  │  로봇 제어 코드           │      │
│            │ Action                │         │  └──────────────────────────┘      │
│            ▼                       │  ROS2   │                                    │
│  ┌──────────────────────┐          │  DDS    │                                    │
│  │  TrajectoryForwarder │          │  Topic  │                                    │
│  │  (Action → Topic)    ├──────────┼────────►│  /trajectory_commands              │
│  └──────────────────────┘          │         │                                    │
│                                    │         │                                    │
│  ┌──────────────────────┐          │         │                                    │
│  │  RViz2               │          │         │                                    │
│  │  (Visualization)     │          │         │                                    │
│  └──────────────────────┘          │         │                                    │
│                                    │         │                                    │
└────────────────────────────────────┘         └────────────────────────────────────┘
```

### PC1 역할 (Planning Computer)
- MoveIt으로 trajectory planning
- RViz GUI로 목표 위치 설정
- 생성된 trajectory를 `/trajectory_commands` topic으로 publish

### PC2 역할 (Robot Execution Computer)
- `/trajectory_commands` topic subscribe
- 수신한 trajectory를 로봇에 전달 (사용자가 직접 구현)

---

## PC1 (Planning Computer) 설정 및 실행

### 1. 패키지 빌드

```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws

# 전체 빌드
colcon build --symlink-install

# 또는 최소 패키지만
colcon build --symlink-install \
  --packages-select \
    franka_kistar_bringup \
    franka_kistar_moveit_config \
    franka_kistar_description

source install/setup.bash
```

### 2. 네트워크 설정

`~/.bashrc` 파일 끝에 추가:
```bash
# ROS2 DDS 네트워크 설정
export ROS_DOMAIN_ID=9                      # PC2와 동일하게!
export ROS_LOCALHOST_ONLY=0                 # 네트워크 통신 활성화 (필수!)

# (선택) Cyclone DDS 사용 (권장)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

**적용:**
```bash
source ~/.bashrc
```

### 3. 실행

```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
source install/setup.bash

# 환경 변수 확인
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"           # 출력: 9
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY" # 출력: 0

# PC1 실행
ros2 launch franka_kistar_bringup moveit_planning_pc.launch.py
```

**실행되는 노드:**
```
/move_group               # MoveIt planning
/rviz                     # 시각화 GUI
/trajectory_forwarder     # Action → Topic 변환
/robot_state_publisher    # 로봇 모델
/joint_state_publisher    # Fake joint states (planning용)
```

### 4. MoveIt으로 Trajectory 생성 및 전송

**RViz2 GUI에서:**

1. **Planning 탭 선택**
   - 왼쪽 패널에서 "MotionPlanning" 선택

2. **Goal State 설정**
   - **방법 1**: Interactive Marker로 드래그
     - RViz 화면에서 로봇 end-effector의 파란색 구를 마우스로 드래그
   - **방법 2**: 숫자로 직접 입력
     - Planning 탭 → "Goal State" 섹션
     - 각 joint 값 입력

3. **Plan 버튼 클릭**
   - MoveIt이 trajectory 계산
   - Planning 성공 시 경로가 RViz에 표시됨
   - Planning 실패 시 다른 Goal State로 재시도

4. **Execute 버튼 클릭**
   - **이 단계에서 trajectory가 topic으로 publish됩니다!**
   - `/trajectory_commands` topic으로 전송
   - PC2가 이 topic을 subscribe

### 5. Topic 확인

**다른 터미널에서:**
```bash
# Topic 존재 확인
ros2 topic list | grep trajectory
# 출력: /trajectory_commands

# Topic 정보 확인
ros2 topic info /trajectory_commands
# 출력:
#   Type: trajectory_msgs/msg/JointTrajectory
#   Publisher count: 1
#   Subscription count: 0 (PC2 실행 전) or 1 (PC2 실행 후)

# Trajectory 데이터 실시간 확인 (Execute 버튼 클릭 시)
ros2 topic echo /trajectory_commands
```

**출력 예시:**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'world'
joint_names:
- fr3_joint1
- fr3_joint2
- fr3_joint3
- fr3_joint4
- fr3_joint5
- fr3_joint6
- fr3_joint7
points:
- positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  time_from_start:
    sec: 0
    nanosec: 0
- positions: [0.01, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78]
  velocities: [0.1, 0.05, 0.0, 0.06, 0.0, 0.01, 0.05]
  accelerations: [0.5, 0.2, 0.0, 0.3, 0.0, 0.1, 0.2]
  time_from_start:
    sec: 0
    nanosec: 10000000
# ... 100-200개 waypoints
```

### 6. 로그 확인

**TrajectoryForwarder 로그:**
```
[trajectory_forwarder]: ======================================================================
[trajectory_forwarder]: Trajectory Forwarder Node Started
[trajectory_forwarder]:   Action Server: /fr3_arm_controller/follow_joint_trajectory
[trajectory_forwarder]:   Topic Publisher: /trajectory_commands
[trajectory_forwarder]:   Waiting for MoveIt trajectories...
[trajectory_forwarder]: ======================================================================

(Execute 버튼 클릭 시)
[trajectory_forwarder]:
[trajectory_forwarder]: ======================================================================
[trajectory_forwarder]: [GOAL RECEIVED]
[trajectory_forwarder]:   Joints: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
[trajectory_forwarder]:   Waypoints: 150
[trajectory_forwarder]:   Duration: 3.245s
[trajectory_forwarder]: ======================================================================

[trajectory_forwarder]:
[trajectory_forwarder]: [FORWARDING] Publishing trajectory to network...
[trajectory_forwarder]: [SUCCESS] Trajectory #1 published
[trajectory_forwarder]:   Topic: /trajectory_commands
[trajectory_forwarder]:   Waypoints: 150
```

---

## PC2 (Robot Execution Computer) 설정

PC2는 사용자가 직접 구현합니다. 아래는 간단한 템플릿입니다.

### 1. 네트워크 설정

`~/.bashrc`:
```bash
export ROS_DOMAIN_ID=9                      # PC1과 동일!
export ROS_LOCALHOST_ONLY=0                 # 필수!
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 2. Trajectory Subscriber 예시

**`simple_executor.py` (PC2):**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import time

class SimpleTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # PC1의 topic subscribe
        self.create_subscription(
            JointTrajectory,
            '/trajectory_commands',
            self.trajectory_callback,
            10
        )

        # TODO: 로봇 초기화
        # self.robot = Robot("172.16.0.1")

        self.get_logger().info('Waiting for trajectories from PC1...')

    def trajectory_callback(self, trajectory):
        """PC1에서 받은 trajectory 실행"""
        n_points = len(trajectory.points)
        self.get_logger().info(f'Received trajectory: {n_points} waypoints')

        # TODO: 로봇 제어 구현
        # 예시:
        # start_time = time.time()
        # for point in trajectory.points:
        #     target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        #
        #     while (time.time() - start_time) < target_time:
        #         time.sleep(0.001)
        #
        #     self.robot.move_to(
        #         positions=point.positions,
        #         velocities=point.velocities
        #     )

        self.get_logger().info('Trajectory execution completed')

def main():
    rclpy.init()
    node = SimpleTrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**실행:**
```bash
# PC2
python3 simple_executor.py
```

---

## Trajectory 메시지 구조

### `trajectory_msgs/msg/JointTrajectory`

```python
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id

string[] joint_names                    # 7개 관절 이름
JointTrajectoryPoint[] points           # 100-200개 waypoints

# 각 JointTrajectoryPoint:
  float64[] positions                   # 필수: 관절 각도 (rad)
  float64[] velocities                  # 선택: 관절 속도 (rad/s)
  float64[] accelerations               # 선택: 관절 가속도 (rad/s²)
  float64[] effort                      # 선택: 토크 (Nm) - 보통 비어있음
  builtin_interfaces/Duration time_from_start  # 시작부터 경과 시간
```

### 데이터 특성

- **joint_names**: `["fr3_joint1", "fr3_joint2", ..., "fr3_joint7"]`
- **points 개수**: 약 100-200개 (resample_dt=0.01s 기준)
- **positions**: ✅ 항상 포함
- **velocities**: ✅ 포함 (TOTG가 계산)
- **accelerations**: ✅ 포함 (TOTG가 계산)
- **effort**: ❌ 비어있음
- **time_from_start**: ✅ 각 waypoint 도달 시간

**예시 (3초 trajectory, 100Hz):**
- points[0]: time_from_start = 0.00s
- points[1]: time_from_start = 0.01s
- points[2]: time_from_start = 0.02s
- ...
- points[300]: time_from_start = 3.00s

---

## 네트워크 디버깅

### 1. 노드 통신 확인

```bash
# PC1에서
ros2 node list
# 출력:
#   /move_group
#   /rviz
#   /trajectory_forwarder
#   /trajectory_executor (PC2 실행 시)

# PC2에서
ros2 node list
# 출력:
#   /trajectory_executor
#   /move_group (PC1의 노드)
#   /trajectory_forwarder (PC1의 노드)
```

**PC2의 노드가 PC1에서 안보이면:**
- `ROS_DOMAIN_ID` 확인
- `ROS_LOCALHOST_ONLY=0` 확인
- 네트워크 연결 확인 (`ping 192.168.1.250`)

### 2. Topic 통신 확인

```bash
# PC1에서
ros2 topic list | grep trajectory
# 출력: /trajectory_commands

ros2 topic info /trajectory_commands
# Subscription count: 1 (PC2가 subscribe 중)

# PC2에서
ros2 topic echo /trajectory_commands
# PC1에서 Execute 시 데이터가 실시간으로 보여야 함
```

### 3. 수동 Trajectory 전송 테스트

**PC1에서 수동으로 trajectory publish:**
```bash
ros2 topic pub --once /trajectory_commands trajectory_msgs/msg/JointTrajectory \
  "{
    header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base'},
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
  }"
```

**PC2에서 수신 확인:**
```bash
ros2 topic echo /trajectory_commands
# 데이터가 보여야 함
```

### 4. 일반적인 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| PC2 노드가 안보임 | `ROS_LOCALHOST_ONLY=1` | `export ROS_LOCALHOST_ONLY=0` |
| Topic은 보이지만 데이터 안옴 | 방화벽 | `sudo ufw allow 7400:7500/udp` |
| `ROS_DOMAIN_ID` 불일치 | 다른 도메인 | 양쪽 PC 동일하게 설정 (예: 9) |
| DDS 구현 불일치 | PC1: FastRTPS, PC2: Cyclone | `RMW_IMPLEMENTATION` 통일 |

---

## 시스템 종료

### 안전한 종료 순서

1. **RViz에서 실행 중인 trajectory 완료 대기**
2. **PC1 종료**
   ```bash
   # Ctrl+C
   ```
3. **PC2 종료**
   ```bash
   # Ctrl+C
   ```

---

## 파일 위치

| 파일 | 경로 | 설명 |
|------|------|------|
| PC1 Launch | [isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/moveit_planning_pc.launch.py](../../isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/moveit_planning_pc.launch.py) | PC1 실행 파일 |
| TrajectoryForwarder | [isaac-ros/kistar_ws/src/franka_kistar_bringup/scripts/trajectory_forwarder.py](../../isaac-ros/kistar_ws/src/franka_kistar_bringup/scripts/trajectory_forwarder.py) | Action → Topic 변환 |

---

## 요약

### PC1 (Planning Computer)

1. **빌드:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **네트워크 설정:**
   ```bash
   export ROS_DOMAIN_ID=9
   export ROS_LOCALHOST_ONLY=0
   ```

3. **실행:**
   ```bash
   ros2 launch franka_kistar_bringup moveit_planning_pc.launch.py
   ```

4. **사용:**
   - RViz에서 Goal State 설정
   - Plan 버튼 클릭
   - Execute 버튼 클릭 → `/trajectory_commands` topic으로 publish

### PC2 (Robot Execution Computer)

1. **Topic subscribe:**
   - `/trajectory_commands` (type: `trajectory_msgs/msg/JointTrajectory`)

2. **구현:**
   - 사용자가 직접 로봇 제어 코드 작성
   - 위 예시 코드 참고

---

## 참고 자료

- [ROS2 DDS Configuration](https://docs.ros.org/en/humble/Guides/DDS-Configuration.html)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [trajectory_msgs/JointTrajectory](https://docs.ros2.org/latest/api/trajectory_msgs/msg/JointTrajectory.html)
