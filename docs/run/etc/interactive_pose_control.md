# Interactive End-Effector Pose Control Guide

CUI로 target end-effector pose를 입력받아 MoveIt planning을 수행하고, 사용자 확인 후 `/trajectory_commands` topic으로 publish하는 시스템입니다.

---

## 시스템 구조

```
┌─────────────────────────────────────────────────────────────────┐
│  PC1: Planning Computer (192.168.1.38)                          │
│                                                                  │
│  ┌──────────────────┐                                           │
│  │  Pose Commander  │  CUI Input: x y z qx qy qz qw             │
│  │  (사용자 입력)    │  ──────────────────┐                      │
│  └──────────────────┘                     │                      │
│           │                               ▼                      │
│           │ Target Pose         ┌──────────────────┐            │
│           ▼                     │  MoveIt Planning │            │
│  ┌──────────────────┐           │  (move_group)    │            │
│  │  MoveGroup       │◄──────────┤  - OMPL          │            │
│  │  Action Client   │  Planning │  - TOTG          │            │
│  └──────────────────┘  Request  └──────────────────┘            │
│           │                               │                      │
│           │ Planned Trajectory            │ Planning Result      │
│           ▼                               ▼                      │
│  ┌──────────────────┐           ┌──────────────────┐            │
│  │ DisplayTrajectory│           │ User Confirmation│            │
│  │ Publisher        │           │ (CUI: y/n)       │            │
│  │ /display_        │           └──────────────────┘            │
│  │  planned_path    │                    │                      │
│  └──────────────────┘                    │ Confirmed            │
│           │                              ▼                      │
│           │                    ┌──────────────────┐             │
│           │                    │ Trajectory       │             │
│           │                    │ Forwarder        │             │
│           │                    │ (Action→Topic)   │             │
│           │                    └──────────────────┘             │
│           │                              │                      │
│           ▼                              ▼                      │
│  ┌──────────────────┐           ┌──────────────────┐           │
│  │      RViz2       │           │ /trajectory_     │  ROS2 DDS │
│  │  (gui:=true)     │           │  commands        ├───────────┼──►
│  │  Trajectory      │           │  Topic           │  Network  │
│  │  Visualization   │           └──────────────────┘           │
│  └──────────────────┘                                           │
│           ▲                                                     │
│           │ /joint_states (PC2 → PC1 via DDS)                  │
│           │                                                     │
└───────────┼─────────────────────────────────────────────────────┘
            │
            │ ROS2 DDS Network (ROS_DOMAIN_ID=9)
            │
┌───────────┼─────────────────────────────────────────────────────┐
│  PC2: Robot Execution Computer (192.168.1.250)   │             │
│           │                                       │             │
│           │                              ┌────────┴──────────┐  │
│           │                              │ Trajectory        │  │
│           │                              │ Subscriber        │  │
│           │                              │ (topic → action)  │  │
│           │                              └────────┬──────────┘  │
│           │                                       │             │
│           │                                       ▼             │
│           │                              ┌───────────────────┐  │
│           │                              │ ros2_control      │  │
│           │                              │ JointTrajectory   │  │
│           │                              │ Controller        │  │
│           │                              └────────┬──────────┘  │
│           │                                       │             │
│           │                                       ▼             │
│           │                              ┌───────────────────┐  │
│           │                              │ Real Robot        │  │
│           │                              │ (Franka FR3)      │  │
│           │                              └────────┬──────────┘  │
│           │                                       │             │
│           │                              ┌────────┴──────────┐  │
│           └──────────────────────────────┤ joint_state_      │  │
│                                          │ broadcaster       │  │
│                                          │ /joint_states     │  │
│                                          └───────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 주요 기능

1. **CUI Pose 입력**: Quaternion 형식 (x y z qx qy qz qw)
2. **MoveIt Planning**: Cartesian goal constraint로 trajectory 생성
3. **RViz 시각화** (gui:=true): Planned trajectory 표시
4. **사용자 확인**: Terminal에서 "Execute? (y/n)" 입력
5. **Trajectory 전송**: `/trajectory_commands` topic으로 publish
6. **RViz Tracking**: PC2의 실시간 joint states 자동 표시

---

## 파일 구조

### 새로 생성된 파일

| 파일 | 경로 | 설명 |
|------|------|------|
| **pose_commander.py** | `isaac-ros/kistar_ws/src/franka_kistar_bringup/scripts/` | Pose 입력 + MoveIt planning + 사용자 확인 노드 |
| **fr3_interactive_pose_control.launch.py** | `isaac-ros/kistar_ws/src/franka_kistar_bringup/launch/` | Interactive pose control 통합 launch 파일 |

### 수정된 파일

| 파일 | 변경사항 |
|------|----------|
| **CMakeLists.txt** | `pose_commander.py` 설치 추가 |

### 재사용된 파일

| 파일 | 역할 |
|------|------|
| **fr3_kistar_moveit_planning_pc.launch.py** | MoveIt + trajectory_forwarder 실행 |
| **trajectory_forwarder.py** | FollowJointTrajectory Action → /trajectory_commands Topic |

---

## 빠른 시작 가이드

### 1. 빌드

```bash
cd ~/isaac_ws/dex_soldering/dex_ros/isaac-ros/kistar_ws
colcon build --symlink-install --packages-select franka_kistar_bringup
source install/setup.bash
```

### 2. PC1 실행 (Planning Computer)

#### GUI Mode (RViz 포함 - 권장)
```bash
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py gui:=true
```

**실행되는 노드:**
- `move_group` - MoveIt planning
- `rviz2` - Trajectory 시각화
- `trajectory_forwarder` - Action → Topic 변환
- `pose_commander` - CUI 입력 + 사용자 확인

#### CUI Mode (Terminal만)
```bash
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py gui:=false
```

**차이점:**
- RViz 창 없음
- Terminal만으로 planning + execution
- 동일한 기능 동작

### 3. Pose 입력 및 실행

**Terminal에서:**
```
======================================================================
Enter target pose (Quaternion):
  Format: x y z qx qy qz qw
  Example: 0.5 0.0 0.4 0 0.707 0 0.707
  (or 'quit' to exit)
======================================================================
>> 0.5 0.0 0.4 0 0.707 0 0.707
0.307 0.0 0.487 0 1.0 0 0
[PLANNING] Target pose:
  Position: (0.500, 0.000, 0.400)
  Orientation: (0.000, 0.707, 0.000, 0.707)
[PLANNING] Requesting trajectory from MoveGroup...
[SUCCESS] Planning succeeded!
  Waypoints: 120
  Duration: 2.345s
[RVIZ] Trajectory published to /display_planned_path

Execute trajectory? (y/n): y
[EXECUTING] Sending trajectory to trajectory_forwarder...
[SUCCESS] Trajectory sent to /trajectory_commands
          (PC2 will execute the trajectory)
```

### 4. PC2 확인 (Robot Computer)

**Trajectory 수신 확인:**
```bash
# PC2
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /trajectory_commands
```

**PC2 Robot 실행 (실제 로봇 제어):**
```bash
# PC2
ros2 launch franka_kistar_bringup robot_execution_pc.launch.py robot_ip:=172.16.0.1
```

---

## 사용 예시

### 예시 1: Home Position으로 이동

```bash
>> 0.307 0.0 0.487 0 1.0 0 0
```
- Position: (0.307, 0.0, 0.487) meters
- Orientation: Quaternion (0, 1.0, 0, 0) - 90도 회전

### 예시 2: 특정 작업 위치

```bash
>> 0.5 0.2 0.3 0 0.707 0 0.707
```
- Position: (0.5, 0.2, 0.3) meters
- Orientation: 복합 회전

### 예시 3: Planning 실패 시

```bash
>> 2.0 2.0 2.0 0 0 0 1

[PLANNING] Target pose:
  Position: (2.000, 2.000, 2.000)
[PLANNING] Requesting trajectory from MoveGroup...
[ERROR] Planning failed with error code: -31
  Error: NO_IK_SOLUTION
```
→ Workspace 밖의 pose는 IK 솔루션이 없음

---

## RViz Tracking (PC2 → PC1)

### 동작 원리

**네트워크 설정 (양쪽 PC 모두):**
```bash
# ~/.bashrc에 추가
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
```

**Tracking 흐름:**
1. **Planning 시**: PC1의 `joint_state_publisher` → Fake states → RViz
2. **Execution 시**: PC2의 실제 로봇 → `/joint_states` → DDS network → PC1 RViz
3. **RViz**: `/joint_states` 자동 subscribe → RobotModel 실시간 업데이트

### 추가 개발 필요 여부

**✅ 추가 개발 불필요!**

PC2에서 로봇이 실행되면:
- `joint_state_broadcaster` (ros2_control)가 `/joint_states` publish
- ROS2 DDS가 자동으로 PC1으로 전송
- PC1 RViz가 자동으로 subscribe하여 표시

**조건:**
- 양쪽 PC의 `ROS_DOMAIN_ID` 동일 (9)
- 양쪽 PC의 `ROS_LOCALHOST_ONLY=0` 설정
- 네트워크 연결 정상 (ping 확인)

---

## Launch 파라미터

### fr3_interactive_pose_control.launch.py

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `gui` | `true` | RViz 활성화 여부 (true/false) |
| `planning_time` | `5.0` | MoveIt planning timeout (초) |
| `end_effector_link` | `fr3_hand_tcp` | End-effector link 이름 |

**사용 예시:**
```bash
# Planning timeout 늘리기
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py planning_time:=10.0

# 다른 end-effector link 사용
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py end_effector_link:=fr3_link8
```

---

## 문제 해결

### 문제 1: Planning 실패 (NO_IK_SOLUTION)

**증상:**
```
[ERROR] Planning failed with error code: -31
  Error: NO_IK_SOLUTION
```

**원인:** Workspace 밖의 pose 또는 도달 불가능한 자세

**해결:**
- Reachable한 pose 입력 (로봇 workspace 범위 내)
- FR3 workspace: 반경 ~0.855m, 높이 0.0~1.19m
- Home position 근처부터 시작: `0.307 0.0 0.487 0 1.0 0 0`

### 문제 2: Planning 시간 초과

**증상:**
```
[ERROR] Planning request timed out
```

**해결:**
```bash
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py planning_time:=10.0
```

### 문제 3: RViz에 trajectory 안 보임

**증상:** Planning 성공했지만 RViz에 경로가 표시되지 않음

**확인사항:**
1. RViz MotionPlanning display 활성화 확인
2. Topic 확인:
   ```bash
   ros2 topic info /display_planned_path
   # Publisher count: 1 (pose_commander)
   # Subscription count: 1 (rviz)
   ```

### 문제 4: PC2에서 trajectory 수신 안 됨

**증상:** `/trajectory_commands` topic이 PC2에서 안 보임

**해결:**
```bash
# PC1과 PC2 모두 확인
echo $ROS_DOMAIN_ID       # 출력: 9 (동일해야 함)
echo $ROS_LOCALHOST_ONLY  # 출력: 0 (반드시 0)

# 네트워크 연결 확인
ping 192.168.1.250  # PC1 → PC2
ping 192.168.1.38   # PC2 → PC1

# Topic 확인
ros2 topic list | grep trajectory
```

### 문제 5: RViz Tracking 안 됨 (PC2 로봇 움직임 안 보임)

**증상:** PC2 로봇 실행 중인데 PC1 RViz에 반영 안 됨

**확인:**
```bash
# PC1에서
ros2 topic hz /joint_states
# PC2의 joint_states가 수신되는지 확인 (30Hz 정도)

# PC2에서
ros2 topic info /joint_states
# Publisher count: 1 (joint_state_broadcaster)
```

**해결:**
- PC2에서 `robot_execution_pc.launch.py` 정상 실행 확인
- `joint_state_broadcaster` 실행 확인
- 방화벽 확인: `sudo ufw allow 7400:7500/udp`

---

## 고급 사용법

### 1. Cartesian Path Planning (직선 경로)

현재는 자유 공간 planning (OMPL) 사용. 직선 경로가 필요하면:

**향후 추가 가능:**
- `/compute_cartesian_path` service 사용
- Waypoint 기반 직선 이동

### 2. Interactive Marker로 Pose 입력

현재는 CUI 입력. 향후 개선:

**향후 추가 가능:**
- RViz InteractiveMarker로 드래그/회전
- 더 직관적인 pose 설정

### 3. Trajectory 저장/재생

**향후 추가 가능:**
- 계획된 trajectory를 파일로 저장
- 저장된 trajectory 재생 기능

---

## 네트워크 구성

### PC1 (Planning Computer - 192.168.1.38)

**역할:**
- MoveIt planning
- Pose 입력 및 사용자 확인
- Trajectory visualization
- Trajectory forwarding

**필수 설정:**
```bash
# ~/.bashrc
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
```

### PC2 (Robot Computer - 192.168.1.250)

**역할:**
- Trajectory 수신
- 로봇 제어 (ros2_control)
- Joint states publishing

**필수 설정:**
```bash
# ~/.bashrc
export ROS_DOMAIN_ID=9
export ROS_LOCALHOST_ONLY=0
```

---

## 시스템 흐름 요약

```
사용자 Pose 입력 (CUI)
    │
    ▼
MoveIt Planning (move_group)
    │
    ▼
DisplayTrajectory publish (RViz 시각화)
    │
    ▼
사용자 확인 (y/n)
    │
    ▼
FollowJointTrajectory Action (trajectory_forwarder)
    │
    ▼
/trajectory_commands Topic publish
    │
    ├─── PC1 확인 (ros2 topic echo)
    │
    └─── ROS2 DDS Network
         │
         ▼
         PC2 수신 (trajectory_subscriber)
         │
         ▼
         로봇 실행 (ros2_control)
         │
         ▼
         /joint_states publish
         │
         └─── ROS2 DDS Network
              │
              ▼
              PC1 RViz Tracking
```

---

## 참고 자료

- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [ROS2 DDS Configuration](https://docs.ros.org/en/humble/Guides/DDS-Configuration.html)
- [control_msgs/FollowJointTrajectory](https://github.com/ros-controls/control_msgs/blob/master/control_msgs/action/FollowJointTrajectory.action)
- [geometry_msgs/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)

---

## 요약

### PC1 실행
```bash
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch.py
```

### Pose 입력
```
>> x y z qx qy qz qw
```

### 확인 및 실행
```
Execute? (y/n): y
```

### PC2 확인
```bash
ros2 topic echo /trajectory_commands
```

**RViz Tracking: 추가 개발 불필요** - ROS2 DDS가 자동으로 joint_states 전송
