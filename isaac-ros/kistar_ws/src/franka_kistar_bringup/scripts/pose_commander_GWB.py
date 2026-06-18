#!/usr/bin/env python3
"""
Pose Commander Node (GWB)

Interactive end-effector + hand pose control with MoveIt planning

Features:
- CUI input for EE target pose (7D: x y z qx qy qz qw)
- CUI input for hand pose (16D joint angles in degrees)
- Input validation with re-prompt on failure
- MoveIt planning via MoveGroup action
- RViz trajectory visualization (DisplayTrajectory)
- User confirmation (CUI)
- Arm execution via trajectory_forwarder or direct_franka_topic
- Hand execution via /hand/target/right (HandTarget, int16 raw encoder)

Author: Chanyoung Ahn
Date: 2025
"""

# /franka/target_joint 보내는 걸로 변경 2026.04.09 by GWB
# hand pose 입력 추가 2026.04.16 by GWB

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    DisplayTrajectory,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from moveit_msgs.srv import GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32MultiArray, Float64, Float64MultiArray, Int16MultiArray
import tf2_ros
import threading
import math
import time


# ------------------------------------------------------------------
# Hand joint constants
# ------------------------------------------------------------------
#
# Input convention  : run_anydexgrasp_only.py 출력 그대로
#   - 단위  : 라디안 (rad)
#   - 순서  : HW 순서 [thumb(0-3), index(4-7), middle(8-11), ring(12-15)]
#             (run_anydexgrasp_only.py 가 _KISTAR_JSON_TO_ROS2_IDX 로 이미 변환)
#
# HW 토픽  : /hand/target/right  (HandTarget, int16)
# 변환식   : raw = round(rad * 8192 / π)   ← anydexgrasp 와 동일
# ------------------------------------------------------------------

# HW 순서: thumb(0-3), index(4-7), middle(8-11), ring(12-15)
HAND_JOINT_NAMES_HW = [
    "thumb_joint_0",  "thumb_joint_1",  "thumb_joint_2",  "thumb_joint_3",
    "index_joint_0",  "index_joint_1",  "index_joint_2",  "index_joint_3",
    "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
    "ring_joint_0",   "ring_joint_1",   "ring_joint_2",   "ring_joint_3",
]

# Joint limits in radians (from kistar_hand.xacro URDF), HW order
HAND_JOINT_LIMITS_RAD = [
    ( 0.0,      1.5708),   # thumb_joint_0  (flexion)
    (-1.5708,   1.5708),   # thumb_joint_1  (rotation)
    ( 0.0,      1.5708),   # thumb_joint_2
    ( 0.0,      1.5708),   # thumb_joint_3
    (-0.2618,   0.2618),   # index_joint_0  (spread)
    ( 0.0,      1.5708),   # index_joint_1  (flexion)
    ( 0.0,      1.5708),   # index_joint_2
    ( 0.0,      1.5708),   # index_joint_3
    (-0.2618,   0.2618),   # middle_joint_0 (spread)
    ( 0.0,      1.5708),   # middle_joint_1
    ( 0.0,      1.5708),   # middle_joint_2
    ( 0.0,      1.5708),   # middle_joint_3
    (-0.2618,   0.2618),   # ring_joint_0   (spread)
    ( 0.0,      1.5708),   # ring_joint_1
    ( 0.0,      1.5708),   # ring_joint_2
    ( 0.0,      1.5708),   # ring_joint_3
]

# Unit conversion: rad → int16 raw encoder  (8192 / π, same as anydexgrasp)
RAD_TO_RAW = 8192.0 / math.pi


def rad_to_raw(rad: float) -> int:
    """Convert radians to int16 raw encoder value (8192/π per radian)."""
    return int(round(rad * RAD_TO_RAW))


# ------------------------------------------------------------------
# Ready pose constants
# ------------------------------------------------------------------
READY_ARM_JOINTS = [1.5481, 0.7276, -1.7492, -1.8678, 1.4551, 1.4033, -0.4819]
READY_HAND_JOINTS_RAW = [0] * 16   # all zeros (open hand)

# fr3_arm planning group joint names (MoveIt joint-space planning)
FR3_ARM_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


class PoseCommander(Node):
    """
    Interactive pose commander for MoveIt planning + hand control
    """

    def __init__(self):
        super().__init__('pose_commander')

        # Parameters
        self.declare_parameter('gui', True)
        self.declare_parameter('planning_group', 'fr3_arm')
        self.declare_parameter('end_effector_link', 'fr3_hand_tcp')
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('execute_mode', 'trajectory_forwarder')
        self.declare_parameter('franka_target_topic', '/franka/target_joint')
        self.declare_parameter('franka_speed_topic', '/franka/target_speed_factor')
        self.declare_parameter('franka_speed_factor', 0.1)
        self.declare_parameter('hand_target_topic', '/hand/target_joint')
        # step1(approach) 전용 플래너
        # 옵션: RRTConnect / RRTstar / BiTRRT / PRMstar / "" (MoveIt 기본값)
        self.declare_parameter('approach_planner_id', '')
        self.declare_parameter('approach_num_attempts', 20)
        self.declare_parameter('hand_traj_steps', 10)   # 중간 waypoint 개수
        self.declare_parameter('hand_traj_period', 0.2) # waypoint 간격 (초)

        self.gui = self.get_parameter('gui').value
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.planning_time = self.get_parameter('planning_time').value
        self.ref_frame = self.get_parameter('reference_frame').value
        self.execute_mode = self.get_parameter('execute_mode').value
        self.franka_target_topic = self.get_parameter('franka_target_topic').value
        self.franka_speed_topic = self.get_parameter('franka_speed_topic').value
        self.franka_speed_factor = self.get_parameter('franka_speed_factor').value
        self.hand_target_topic = self.get_parameter('hand_target_topic').value
        self.approach_planner_id = self.get_parameter('approach_planner_id').value
        self.approach_num_attempts = self.get_parameter('approach_num_attempts').value
        self.hand_traj_steps = self.get_parameter('hand_traj_steps').value
        self.hand_traj_period = self.get_parameter('hand_traj_period').value
        self._current_hand_deg: list | None = None  # /hand/joint_position (degrees)

        # Callback group for threading
        self.cb_group = ReentrantCallbackGroup()

        # Clients and Publishers
        self._setup_clients()

        # Start input thread
        self.input_thread = threading.Thread(
            target=self._input_loop,
            daemon=True
        )
        self.input_thread.start()

        self.get_logger().info('=' * 70)
        self.get_logger().info('Pose Commander (GWB) Started')
        self.get_logger().info(f'  Mode: {"GUI" if self.gui else "CUI"}')
        self.get_logger().info(f'  Planning group: {self.planning_group}')
        self.get_logger().info(f'  End-effector: {self.ee_link}')
        self.get_logger().info(f'  Reference frame: {self.ref_frame}')
        self.get_logger().info(f'  Planning timeout: {self.planning_time}s')
        self.get_logger().info(f'  Execute mode: {self.execute_mode}')
        self.get_logger().info(f'  Hand target topic: {self.hand_target_topic}')
        planner_tag = self.approach_planner_id if self.approach_planner_id else 'default(RRTConnect)'
        self.get_logger().info(f'  Approach planner: {planner_tag}  attempts: {self.approach_num_attempts}')
        self.get_logger().info(
            f'  Hand trajectory: {self.hand_traj_steps} steps x {self.hand_traj_period:.2f}s'
            f' = {self.hand_traj_steps * self.hand_traj_period:.1f}s total'
        )
        self.get_logger().info('=' * 70)

    def _setup_clients(self):
        """Setup action clients and publishers"""

        # MoveGroup action client (planning)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.cb_group
        )

        # DisplayTrajectory publisher (RViz visualization)
        self.display_pub = self.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10
        )

        # Arm publishers
        self.franka_target_pub = self.create_publisher(
            Float64MultiArray,
            self.franka_target_topic,
            10
        )
        self.franka_speed_pub = self.create_publisher(
            Float64,
            self.franka_speed_topic,
            10
        )

        # Hand publisher (Int16MultiArray: raw encoder, hw order)
        self.hand_target_pub = self.create_publisher(
            Int16MultiArray,
            self.hand_target_topic,
            10
        )

        # Hand state subscriber (degrees) — 현재 핸드 위치를 보간 시작점으로 사용
        self.hand_state_sub = self.create_subscription(
            Float32MultiArray,
            '/hand/joint_position',
            self._hand_state_callback,
            10,
        )

        # Cartesian path service (step 2: approach → target, straight-line EE motion)
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self.cb_group
        )

        # TF2 (현재 EE 위치 조회용 - lift 기능)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Waiting for MoveGroup action server...')
        self.move_group_client.wait_for_server()

        if self.execute_mode == 'trajectory_forwarder':
            self.traj_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/fr3_arm_controller/follow_joint_trajectory',
                callback_group=self.cb_group
            )
            self.traj_client.wait_for_server()
            self.get_logger().info('MoveGroup and trajectory action servers connected')
        elif self.execute_mode == 'direct_franka_topic':
            self.traj_client = None
            self.get_logger().info('MoveGroup action server connected')
            self.get_logger().info(
                f'Franka direct topics ready: {self.franka_target_topic}, {self.franka_speed_topic}'
            )
        else:
            raise ValueError(
                "execute_mode must be 'trajectory_forwarder' or 'direct_franka_topic'"
            )

    # ------------------------------------------------------------------
    # Input loop
    # ------------------------------------------------------------------

    def _input_loop(self):
        """Input thread - blocking CUI input for EE pose then hand pose"""
        import traceback
        while rclpy.ok():
            # Step 1: EE pose (or 'r' for ready, or 'HAND_ONLY' sentinel)
            target_pose = self._get_ee_pose_input()
            if target_pose is None:
                break  # quit requested

            # 'r' → ready pose shortcut (skip planning)
            if target_pose == 'READY':
                try:
                    self._execute_ready_pose()
                except Exception as e:
                    self.get_logger().error(f"Error during ready pose: {e}")
                    traceback.print_exc()
                continue

            # 'LIFT' / 'LIFT20' → 'l'/'ll': move EE +10cm / +20cm in world Z
            if target_pose in ('LIFT', 'LIFT20'):
                lift_m = 0.20 if target_pose == 'LIFT20' else 0.10
                try:
                    self._execute_lift(lift_m=lift_m)
                except Exception as e:
                    self.get_logger().error(f"Error during lift: {e}")
                    traceback.print_exc()
                continue

            # 'HAND_ONLY' → 's' entered at EE prompt: skip arm, go straight to hand
            if target_pose == 'HAND_ONLY':
                hand_joints = self._get_hand_pose_input()
                if hand_joints is None:
                    break
                if hand_joints == 'SKIP':
                    print("[SKIP] Hand skipped.")
                    continue
                try:
                    print("\n[HAND] Arm skipped. Ready to execute hand.")
                    if self._confirm_execution("Execute hand? (y/n): "):
                        self._execute_hand(hand_joints)
                    else:
                        print("[CANCELED] Hand execution canceled")
                except Exception as e:
                    self.get_logger().error(f"Error during hand execute: {e}")
                    traceback.print_exc()
                continue

            # Step 2: plan + execute arm
            # Returns True if arm executed (or confirmed), False if failed/canceled
            try:
                arm_ok = self._plan_and_execute_arm(target_pose)
            except Exception as e:
                self.get_logger().error(f"Error during arm plan/execute: {e}")
                traceback.print_exc()
                arm_ok = False

            if not arm_ok:
                # Planning failed or user canceled → back to arm input
                continue

            # Step 3: hand pose (only reached after arm success)
            hand_joints = self._get_hand_pose_input()
            if hand_joints is None:
                break  # quit requested
            if hand_joints == 'SKIP':
                print("[SKIP] Hand skipped.")
                continue

            # Step 4: execute hand
            try:
                print("\n[HAND] Arm command sent. Press y when arm finishes to execute hand.")
                if self._confirm_execution("Execute hand? (y/n): "):
                    self._execute_hand(hand_joints)
                else:
                    print("[CANCELED] Hand execution canceled")
            except Exception as e:
                self.get_logger().error(f"Error during hand execute: {e}")
                traceback.print_exc()

    def _get_ee_pose_input(self):
        """
        Prompt for EE pose until valid input is given.
        Returns PoseStamped, 'READY' sentinel, or None if user wants to quit.
        """
        while rclpy.ok():
            print("\n" + "=" * 70)
            print("  [1/2] Enter EE target pose (Quaternion):")
            print("  Format : x y z qx qy qz qw")
            print("  Example: 0.5 0.0 0.4 0 0.707 0 0.707")
            print("  (or 'r' = ready pose, 's' = skip arm → hand only, 'l' = lift +10cm, 'll' = lift +20cm, 'quit' = exit)")
            print("=" * 70)

            user_input = input("ee: ").strip()

            if user_input.lower() in ['quit', 'exit', 'q']:
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
                return None

            if user_input.lower() == 'r':
                return 'READY'

            if user_input.lower() == 's':
                return 'HAND_ONLY'

            if user_input.lower() == 'll':
                return 'LIFT20'

            if user_input.lower() == 'l':
                return 'LIFT'

            try:
                values = [float(x) for x in user_input.split()]
            except ValueError:
                print("Error: Non-numeric value detected. Please re-enter.")
                continue

            if len(values) != 7:
                print(f"Error: Expected 7 values (x y z qx qy qz qw), got {len(values)}. Please re-enter.")
                continue

            x, y, z, qx, qy, qz, qw = values

            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            if norm < 0.01:
                print("Error: Quaternion norm is near-zero (invalid rotation). Please re-enter.")
                continue

            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
            return self._create_pose(x, y, z, qx, qy, qz, qw)

        return None

    def _get_hand_pose_input(self):
        """
        Prompt for hand joint angles (16D, radians, HW order) until valid input.

        Input format matches run_anydexgrasp_only.py output:
          "KISTAR 16D joint angles (rad): v0 v1 ... v15"
          → 숫자 부분만 붙여넣으면 됩니다.

        Order  : thumb(0-3) index(4-7) middle(8-11) ring(12-15)   ← HW order
        Unit   : radians
        Returns: list of 16 floats (rad), or None if user wants to quit.
        """
        limit_str = "  Limits per joint (rad):\n"
        for i, (name, (lo, hi)) in enumerate(zip(HAND_JOINT_NAMES_HW, HAND_JOINT_LIMITS_RAD)):
            limit_str += f"    [{i:2d}] {name:<20s}: [{lo:6.4f}, {hi:6.4f}]\n"

        while rclpy.ok():
            print("\n" + "=" * 70)
            print("  [2/2] Enter hand pose (16 joint angles in radians):")
            print("  Order  : thumb(0-3) index(4-7) middle(8-11) ring(12-15)")
            print("  Source : copy from  'KISTAR 16D joint angles (rad): ...'")
            print("  Example (open hand): 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0")
            print(limit_str.rstrip())
            print("  (or 's' = skip hand, 'quit' = exit)")
            print("=" * 70)

            user_input = input("hand pose (rad): ").strip()

            if user_input.lower() in ['quit', 'exit', 'q']:
                self.get_logger().info('Shutting down...')
                rclpy.shutdown()
                return None

            if user_input.lower() == 's':
                return 'SKIP'

            try:
                values = [float(x) for x in user_input.split()]
            except ValueError:
                print("Error: Non-numeric value detected. Please re-enter.")
                continue

            if len(values) != 16:
                print(f"Error: Expected 16 values, got {len(values)}. Please re-enter.")
                continue

            # Joint limit check (radians)
            violations = []
            for i, (v, (lo, hi)) in enumerate(zip(values, HAND_JOINT_LIMITS_RAD)):
                if v < lo or v > hi:
                    violations.append(
                        f"  [{i:2d}] {HAND_JOINT_NAMES_HW[i]}: {v:.4f} rad  (limit: [{lo:.4f}, {hi:.4f}])"
                    )

            if violations:
                print("Error: The following joints exceed their limits. Please re-enter.")
                for v in violations:
                    print(v)
                continue

            return values

        return None

    # ------------------------------------------------------------------
    # Pose creation
    # ------------------------------------------------------------------

    def _create_pose(self, x, y, z, qx, qy, qz, qw):
        """Create PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = self.ref_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return pose

    # ------------------------------------------------------------------
    # Planning & execution
    # ------------------------------------------------------------------

    def _compute_approach_pose(self, target_pose: PoseStamped, offset_m: float = 0.1) -> PoseStamped:
        """
        world Z축 위쪽으로 offset_m 만큼 올린 approach pose 반환.
        x/y는 타겟과 동일, orientation도 동일하게 유지.
        """
        approach = PoseStamped()
        approach.header = target_pose.header
        approach.pose.orientation = target_pose.pose.orientation
        approach.pose.position.x = target_pose.pose.position.x
        approach.pose.position.y = target_pose.pose.position.y
        approach.pose.position.z = target_pose.pose.position.z + offset_m
        return approach

    def _plan_to_ee_pose(self, target_pose: PoseStamped, label: str = "",
                         planner_id: str = "", num_attempts: int = 20):
        """
        EE 목표 pose로 MoveGroup 플래닝 요청.
        성공 시 result 반환, 실패 시 None 반환.
        """
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.planner_id = planner_id          # "" = MoveIt 기본(RRTConnect)
        goal.request.num_planning_attempts = num_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # workspace bounds (IK sampler가 zero bounds이면 FAILURE 99999 반환)
        # FR3 최대 도달 거리 ~855mm → ±5m로 여유 있게 설정
        goal.request.workspace_parameters.header.frame_id = self.ref_frame
        goal.request.workspace_parameters.min_corner.x = -5.0
        goal.request.workspace_parameters.min_corner.y = -5.0
        goal.request.workspace_parameters.min_corner.z = -5.0
        goal.request.workspace_parameters.max_corner.x = 5.0
        goal.request.workspace_parameters.max_corner.y = 5.0
        goal.request.workspace_parameters.max_corner.z = 5.0

        # use current state from planning scene
        goal.request.start_state.is_diff = True

        goal_constraint = Constraints()

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.ee_link
        pos_constraint.constraint_region = BoundingVolume()

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]  # 5mm

        pos_constraint.constraint_region.primitives = [sphere]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.ee_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0

        goal_constraint.position_constraints = [pos_constraint]
        goal_constraint.orientation_constraints = [ori_constraint]
        goal.request.goal_constraints = [goal_constraint]
        goal.planning_options.plan_only = True

        tag = f"[{label}] " if label else ""
        print(f"{tag}Planning → ({target_pose.pose.position.x:.3f}, "
              f"{target_pose.pose.position.y:.3f}, "
              f"{target_pose.pose.position.z:.3f})")

        future = self.move_group_client.send_goal_async(goal)
        timeout = self.planning_time + 2.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not future.done():
            print(f"{tag}[ERROR] Planning request timed out")
            return None

        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"{tag}[ERROR] Planning goal rejected")
            return None

        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not result_future.done():
            print(f"{tag}[ERROR] Planning result timed out")
            return None

        result = result_future.result().result

        if result.error_code.val != 1:
            print(f"\n{'='*60}")
            print(f"{tag}[PLANNING FAILED]")
            print(f"  Error code : {result.error_code.val}")
            self._print_error_code(result.error_code.val)
            print(f"  Planning time used: {result.planning_time:.2f}s")
            print(f"  Target: ({target_pose.pose.position.x:.3f}, "
                  f"{target_pose.pose.position.y:.3f}, "
                  f"{target_pose.pose.position.z:.3f})")
            print(f"  Frame : {target_pose.header.frame_id}  EE: {self.ee_link}")
            print(f"{'='*60}\n")
            return None

        n_pts = len(result.planned_trajectory.joint_trajectory.points)
        duration = 0.0
        if n_pts > 0:
            p = result.planned_trajectory.joint_trajectory.points[-1]
            duration = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        print(f"{tag}[SUCCESS] Waypoints: {n_pts}  Duration: {duration:.2f}s")
        return result

    def _plan_cartesian(self, target_pose: PoseStamped,
                        start_joint_names=None, start_joint_values=None,
                        label: str = ""):
        """
        EE 직선 경로 계획 (GetCartesianPath 서비스).
        start_joint_names/values=None → 플래닝 씬의 현재 상태 사용 (STEP 1용).
        값 제공 시 → 명시적 시작 상태 사용 (STEP 2: STEP 1 결과에서 출발).
        이전 waypoint를 IK seed로 쓰기 때문에 관절 연속성 보장 → 뒤틀림 없음.
        성공 시 GetCartesianPath.Response 반환, 실패 시 None.
        """
        req = GetCartesianPath.Request()
        req.header = target_pose.header
        req.group_name = self.planning_group
        req.link_name = self.ee_link
        req.waypoints = [target_pose.pose]
        req.max_step = 0.005          # 5mm 간격으로 보간 → 부드러운 직선
        req.jump_threshold = 0.0      # jump threshold 비활성화
        req.avoid_collisions = True

        if start_joint_names is not None and start_joint_values is not None:
            # 명시적 시작 상태 (STEP 2: STEP 1 결과의 마지막 관절값)
            req.start_state.is_diff = False
            req.start_state.joint_state.name = list(start_joint_names)
            req.start_state.joint_state.position = list(start_joint_values)
        else:
            # 현재 로봇 상태 사용 (STEP 1: 현재 위치에서 출발)
            req.start_state.is_diff = True

        tag = f"[{label}] " if label else ""
        print(f"{tag}Cartesian planning → ({target_pose.pose.position.x:.3f}, "
              f"{target_pose.pose.position.y:.3f}, "
              f"{target_pose.pose.position.z:.3f})")

        future = self.cartesian_path_client.call_async(req)
        timeout = 10.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not future.done():
            print(f"{tag}[ERROR] Cartesian path timed out")
            return None

        result = future.result()

        if result.fraction < 0.9:
            print(f"{tag}[ERROR] Cartesian path {result.fraction:.0%} 만 계획됨 "
                  f"(최소 90% 필요) → 경로 상 충돌 또는 특이점")
            return None

        n_pts = len(result.solution.joint_trajectory.points)
        print(f"{tag}[SUCCESS] Cartesian {result.fraction:.0%}, waypoints: {n_pts}")
        return result

    def _do_execute_arm(self, joint_trajectory):
        """execute_mode에 따라 arm 궤적 실행"""
        if self.execute_mode == 'direct_franka_topic':
            self._execute_direct_franka_topic(joint_trajectory)
        else:
            self._execute_trajectory(joint_trajectory)

    def _plan_step(self, goal_pose: PoseStamped, step_label: str,
                   start_joint_names=None, start_joint_values=None):
        """
        단일 스텝 계획 헬퍼.
        1) Cartesian path 우선 시도 (IK seed 연속성 → 뒤틀림 없음)
        2) Cartesian 실패 시 OMPL fallback
        confirm 후 joint_trajectory 반환, 취소/실패 시 None.
        """
        # ── Cartesian 시도 ──────────────────────────────────────────────
        print(f"\n[{step_label}] Cartesian planning...")
        cart = self._plan_cartesian(
            goal_pose,
            start_joint_names=start_joint_names,
            start_joint_values=start_joint_values,
            label=step_label,
        )
        if cart is not None:
            traj = cart.solution.joint_trajectory
            if self.gui:
                disp = DisplayTrajectory()
                disp.model_id = self.planning_group
                disp.trajectory_start = cart.start_state
                disp.trajectory.append(cart.solution)
                time.sleep(0.1)
                self.display_pub.publish(disp)
                time.sleep(0.5)
            if not self._confirm_execution(f"Execute {step_label}? (y/n): "):
                print(f"[CANCELED] {step_label} canceled")
                return None
            return traj

        # ── OMPL fallback ───────────────────────────────────────────────
        print(f"[FALLBACK] Cartesian 실패 → OMPL ({step_label})...")
        planner_tag = self.approach_planner_id if self.approach_planner_id else "default"
        ompl = self._plan_to_ee_pose(
            goal_pose, label=f"{step_label}(OMPL)",
            planner_id=self.approach_planner_id,
            num_attempts=self.approach_num_attempts,
        )
        if ompl is None:
            return None
        traj = ompl.planned_trajectory.joint_trajectory
        if self.gui:
            self._publish_display_trajectory(ompl)
        if not self._confirm_execution(f"Execute {step_label}? (y/n): "):
            print(f"[CANCELED] {step_label} canceled")
            return None
        return traj

    def _plan_and_execute_arm(self, target_pose: PoseStamped) -> bool:
        """
        2단계 arm 실행 (Cartesian 우선, OMPL fallback):
          STEP 1: 현재 위치 → approach pose (EE z축 -10cm)
          STEP 2: approach pose → target pose (Cartesian 직선 삽입)
        Cartesian path는 이전 waypoint를 IK seed로 사용 → 관절 연속성 보장.
        Returns True if both commands sent, False if failed/canceled.
        """
        approach_pose = self._compute_approach_pose(target_pose, offset_m=0.1)

        print(f"\n{'='*60}")
        print(f"[ARM] 2-step Cartesian execution plan")
        print(f"  STEP 1 approach : ({approach_pose.pose.position.x:.3f}, "
              f"{approach_pose.pose.position.y:.3f}, "
              f"{approach_pose.pose.position.z:.3f})  ← world Z +10cm")
        print(f"  STEP 2 target   : ({target_pose.pose.position.x:.3f}, "
              f"{target_pose.pose.position.y:.3f}, "
              f"{target_pose.pose.position.z:.3f})")
        print(f"{'='*60}")

        # ── STEP 1: 현재 위치 → approach (Cartesian, fallback OMPL) ─────
        jt1 = self._plan_step(approach_pose, "STEP 1/2 APPROACH")
        if jt1 is None:
            return False
        self._do_execute_arm(jt1)

        # ── STEP 2: approach → target (Cartesian, fallback OMPL) ────────
        # STEP 1 마지막 관절값을 시작 상태로 명시 → IK seed 연속성 보장
        final_joints = jt1.points[-1].positions if jt1.points else None
        jt2 = self._plan_step(
            target_pose, "STEP 2/2 TARGET",
            start_joint_names=jt1.joint_names,
            start_joint_values=final_joints,
        )
        if jt2 is None:
            return False
        self._do_execute_arm(jt2)
        return True

    def _publish_display_trajectory(self, planning_result):
        """Publish trajectory to RViz for visualization"""
        display_traj = DisplayTrajectory()
        display_traj.model_id = self.planning_group
        display_traj.trajectory_start = planning_result.trajectory_start
        display_traj.trajectory.append(planning_result.planned_trajectory)

        time.sleep(0.1)
        self.display_pub.publish(display_traj)
        time.sleep(0.5)

    def _confirm_execution(self, prompt: str = "Execute? (y/n): "):
        """Ask user confirmation for execution"""
        while True:
            response = input(prompt).strip().lower()
            if response == 'y':
                return True
            elif response == 'n':
                return False
            else:
                print("Please enter 'y' or 'n'")

    def _execute_trajectory(self, joint_trajectory):
        """Send arm trajectory to trajectory_forwarder"""
        print("[EXECUTING ARM] Sending trajectory to trajectory_forwarder...")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory

        future = self.traj_client.send_goal_async(goal)

        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not future.done():
            print("[ERROR] Trajectory execution request timed out")
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[ERROR] Trajectory execution rejected")
            return

        print("[SUCCESS] Arm trajectory sent to /trajectory_commands")
        print("          (PC2 will execute the trajectory)")

    def _execute_direct_franka_topic(self, joint_trajectory):
        """Send the final planned joint target to Franka target topics"""
        if not joint_trajectory.points:
            print("[ERROR] Planned trajectory has no waypoints")
            return

        final_point = joint_trajectory.points[-1]
        target_msg = Float64MultiArray()
        target_msg.data = list(final_point.positions)

        if len(target_msg.data) != 7:
            print(f"[ERROR] Expected 7 joint values, got {len(target_msg.data)}")
            return

        speed_msg = Float64()
        speed_msg.data = max(0.001, min(1.0, float(self.franka_speed_factor)))

        print("[EXECUTING ARM] Sending final joint target to Franka...")
        print(f"  Target topic: {self.franka_target_topic}")
        print(f"  Speed factor: {speed_msg.data:.3f}")
        print(f"  Final joints (rad): {[round(v, 4) for v in target_msg.data]}")

        self.franka_speed_pub.publish(speed_msg)
        time.sleep(0.05)
        self.franka_target_pub.publish(target_msg)

        print("[SUCCESS] Arm target sent")

    def _execute_lift(self, lift_m: float = 0.10):
        """현재 EE 위치에서 world Z축으로 lift_m 위로 한 번에 이동 (2단계 없음)."""
        print(f"\n[LIFT] Looking up current EE pose ({self.ee_link} in {self.ref_frame})...")
        try:
            tf = self.tf_buffer.lookup_transform(
                self.ref_frame, self.ee_link,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)
            )
        except Exception as e:
            print(f"[LIFT] [ERROR] TF lookup failed: {e}")
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        lift_pose = self._create_pose(
            t.x, t.y, t.z + lift_m,
            r.x, r.y, r.z, r.w
        )
        print(f"[LIFT] Target: ({t.x:.3f}, {t.y:.3f}, {t.z + lift_m:.3f})  (z +{lift_m*100:.0f}cm)")

        # Cartesian 직선 상승 시도
        cart = self._plan_cartesian(lift_pose, label="LIFT")
        if cart is not None:
            traj = cart.solution.joint_trajectory
            if self.gui:
                display_traj = DisplayTrajectory()
                display_traj.model_id = self.planning_group
                display_traj.trajectory_start = cart.start_state
                display_traj.trajectory.append(cart.solution)
                time.sleep(0.1)
                self.display_pub.publish(display_traj)
                time.sleep(0.5)
            if self._confirm_execution("Execute LIFT? (y/n): "):
                self._do_execute_arm(traj)
            else:
                print("[CANCELED] Lift canceled")
            return

        # Cartesian 실패 시 OMPL fallback
        print("[LIFT][FALLBACK] Cartesian failed → OMPL")
        ompl = self._plan_to_ee_pose(lift_pose, label="LIFT(OMPL)")
        if ompl is None:
            return
        traj = ompl.planned_trajectory.joint_trajectory
        if self.gui:
            self._publish_display_trajectory(ompl)
        if self._confirm_execution("Execute LIFT? (y/n): "):
            self._do_execute_arm(traj)
        else:
            print("[CANCELED] Lift canceled")

    def _execute_ready_pose(self):
        """Move arm + hand to ready pose via MoveIt joint-space planning."""
        print("\n" + "=" * 70)
        print("[READY POSE]")
        print(f"  Arm joints (rad): {READY_ARM_JOINTS}")
        print(f"  Hand joints (raw): all zeros (open hand)")
        print("=" * 70)

        # Arm: plan via MoveIt joint constraints (same safe path as normal mode)
        arm_ok = self._plan_and_execute_arm_joints(READY_ARM_JOINTS)
        if not arm_ok:
            print("[CANCELED/FAILED] Ready pose arm not executed")
            return

        # Hand confirmation (after arm command sent)
        print("\n[HAND] Press y when arm reaches ready pose to open hand.")
        if not self._confirm_execution("Execute hand open (all zeros)? (y/n): "):
            print("[CANCELED] Hand open canceled")
            return

        hand_msg = Int16MultiArray()
        hand_msg.data = list(READY_HAND_JOINTS_RAW)
        self.hand_target_pub.publish(hand_msg)
        print("[SUCCESS] Hand open (all zeros) sent")

    def _plan_and_execute_arm_joints(self, joint_values: list) -> bool:
        """
        Plan and execute arm to a specific joint configuration via MoveIt.
        Uses JointConstraint (joint-space planning) — same execute path as normal mode.
        Returns True if arm command was sent, False if failed or canceled.
        """
        print(f"\n[PLANNING] Joint target:")
        for name, val in zip(FR3_ARM_JOINT_NAMES, joint_values):
            print(f"  {name}: {val:.4f} rad")

        # Build MoveGroup goal with joint constraints
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        goal.request.workspace_parameters.header.frame_id = self.ref_frame
        goal.request.workspace_parameters.min_corner.x = -5.0
        goal.request.workspace_parameters.min_corner.y = -5.0
        goal.request.workspace_parameters.min_corner.z = -5.0
        goal.request.workspace_parameters.max_corner.x = 5.0
        goal.request.workspace_parameters.max_corner.y = 5.0
        goal.request.workspace_parameters.max_corner.z = 5.0
        goal.request.start_state.is_diff = True

        goal_constraint = Constraints()
        for name, value in zip(FR3_ARM_JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraint.joint_constraints.append(jc)

        goal.request.goal_constraints = [goal_constraint]
        goal.planning_options.plan_only = True

        print("[PLANNING] Requesting joint-space trajectory from MoveGroup...")
        future = self.move_group_client.send_goal_async(goal)

        timeout = self.planning_time + 2.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not future.done():
            print("[ERROR] Planning request timed out")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            print("[ERROR] Planning goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.01)

        if not result_future.done():
            print("[ERROR] Planning result timed out")
            return False

        result = result_future.result().result

        if result.error_code.val != 1:  # SUCCESS = 1
            print(f"\n{'='*60}")
            print(f"[PLANNING FAILED] Joint-space")
            print(f"  Error code : {result.error_code.val}")
            self._print_error_code(result.error_code.val)
            print(f"  Planning time used: {result.planning_time:.2f}s")
            print(f"  Target joints: {[round(v, 4) for v in joint_values]}")
            print(f"{'='*60}\n")
            return False

        trajectory = result.planned_trajectory
        n_points = len(trajectory.joint_trajectory.points)
        print(f"[SUCCESS] Planning succeeded! Waypoints: {n_points}")

        if n_points > 0:
            duration = (
                trajectory.joint_trajectory.points[-1].time_from_start.sec +
                trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
            )
            print(f"  Duration: {duration:.3f}s")

        if self.gui:
            self._publish_display_trajectory(result)
            print("[RVIZ] Trajectory published to /display_planned_path")

        if not self._confirm_execution("Execute arm to ready pose? (y/n): "):
            print("[CANCELED] Ready pose arm canceled")
            return False

        if self.execute_mode == 'direct_franka_topic':
            self._execute_direct_franka_topic(trajectory.joint_trajectory)
        else:
            self._execute_trajectory(trajectory.joint_trajectory)

        return True

    def _hand_state_callback(self, msg: Float32MultiArray):
        """현재 핸드 관절 위치 (degrees) 를 캐싱."""
        if len(msg.data) == 16:
            self._current_hand_deg = list(msg.data)

    def _execute_hand(self, hand_joints: list):
        """
        Hand joint 목표를 보간 궤적으로 전송.

        Input : 16 floats (radians, HW order: thumb/index/middle/ring)
        보간  : 현재 위치 → 목표 위치를 hand_traj_steps 단계로 나눠 전송
                각 단계 사이에 hand_traj_period 초 대기
        """
        target_raw = [rad_to_raw(v) for v in hand_joints]

        # 보간 시작점: 현재 핸드 상태(degrees) → raw, 없으면 open(0)
        DEG_TO_RAW = 8192.0 / 180.0
        if self._current_hand_deg is not None:
            start_raw = [int(round(d * DEG_TO_RAW)) for d in self._current_hand_deg]
        else:
            start_raw = [0] * 16

        steps = max(1, int(self.hand_traj_steps))
        period = float(self.hand_traj_period)
        total_time = steps * period

        print("[EXECUTING HAND] Sending hand trajectory...")
        print(f"  Topic  : {self.hand_target_topic}")
        print(f"  Steps  : {steps}  |  Period: {period:.2f}s  |  Total: {total_time:.1f}s")
        print(f"  --- target (rad, HW order) ---")
        print(f"  thumb : {[round(v, 4) for v in hand_joints[0:4]]}")
        print(f"  index : {[round(v, 4) for v in hand_joints[4:8]]}")
        print(f"  middle: {[round(v, 4) for v in hand_joints[8:12]]}")
        print(f"  ring  : {[round(v, 4) for v in hand_joints[12:16]]}")
        print(f"  --- target raw (int16) ---")
        print(f"  thumb : {target_raw[0:4]}")
        print(f"  index : {target_raw[4:8]}")
        print(f"  middle: {target_raw[8:12]}")
        print(f"  ring  : {target_raw[12:16]}")

        for i in range(1, steps + 1):
            t = i / steps
            interp_raw = [
                int(round(s + t * (g - s)))
                for s, g in zip(start_raw, target_raw)
            ]
            msg = Int16MultiArray()
            msg.data = interp_raw
            self.hand_target_pub.publish(msg)
            print(f"  [{i:2d}/{steps}] t={t:.2f}  raw={interp_raw}")
            if i < steps:
                time.sleep(period)

        print("[SUCCESS] Hand trajectory complete")

    def _print_error_code(self, code):
        """Print MoveIt error code with description and hint"""
        error_info = {
            # Planning failures
            -1:  ("PLANNING_FAILED",
                  "OMPL이 제한 시간 내에 경로를 찾지 못함 → planning_time 늘리거나 자세 재입력"),
            -2:  ("INVALID_MOTION_PLAN",
                  "플래닝은 됐지만 결과 궤적이 유효하지 않음"),
            -3:  ("MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                  "플래닝 중 환경(장애물 등)이 변경됨"),
            -4:  ("CONTROL_FAILED",
                  "실행 중 컨트롤러 오류"),
            -5:  ("UNABLE_TO_ACQUIRE_SENSOR_DATA",
                  "센서 데이터 수신 불가"),
            -6:  ("TIMED_OUT",
                  "플래닝 시간 초과 → planning_time 늘리기"),
            -7:  ("PREEMPTED",
                  "플래닝이 외부에서 취소됨"),
            # Start state
            -10: ("START_STATE_IN_COLLISION",
                  "현재 로봇 자세가 충돌 상태 → 충돌 없는 자세로 이동 필요"),
            -11: ("START_STATE_VIOLATES_PATH_CONSTRAINTS",
                  "시작 자세가 경로 제약 위반"),
            # Goal state
            -12: ("GOAL_IN_COLLISION",
                  "목표 자세가 충돌 상태 → 다른 목표 입력"),
            -13: ("GOAL_VIOLATES_PATH_CONSTRAINTS",
                  "목표가 경로 제약 위반"),
            -14: ("GOAL_CONSTRAINTS_VIOLATED",
                  "목표 제약 조건 위반"),
            # Invalid inputs
            -15: ("INVALID_GROUP_NAME",
                  "planning_group 이름 잘못됨 → fr3_arm 확인"),
            -16: ("INVALID_GOAL_CONSTRAINTS",
                  "목표 제약 구성이 잘못됨 (quaternion 정규화 등 확인)"),
            -17: ("INVALID_ROBOT_STATE",
                  "로봇 상태가 유효하지 않음 → joint_states 토픽 확인"),
            -18: ("INVALID_LINK_NAME",
                  "end_effector_link 이름 잘못됨 → fr3_link8 확인"),
            -19: ("INVALID_OBJECT_NAME",
                  "충돌 객체 이름 오류"),
            -20: ("FRAME_TRANSFORM_FAILURE",
                  "TF 변환 실패 → reference_frame 확인 (base / world)"),
            -21: ("COLLISION_CHECKING_UNAVAILABLE",
                  "충돌 검사 불가"),
            -22: ("ROBOT_STATE_STALE",
                  "로봇 상태 정보가 오래됨 → joint_states 토픽 확인"),
            -23: ("SENSOR_INFO_STALE",
                  "센서 정보 오래됨"),
            # IK
            -31: ("NO_IK_SOLUTION",
                  "IK 해 없음 → 도달 불가능한 자세이거나 특이점(singularity) 근처"),
            99999: ("FAILURE",
                    "포괄적 실패 → workspace_parameters 미설정/IK 샘플러 실패가 주원인. "
                    "또는 reference_frame 오류 (world vs base). "
                    "또는 joint_states 미수신 상태"),
        }
        if code in error_info:
            name, hint = error_info[code]
            print(f"  → {name}")
            print(f"  힌트: {hint}")
        else:
            print(f"  → UNKNOWN ERROR (code={code})")


def main(args=None):
    rclpy.init(args=args)
    node = PoseCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Pose Commander shutting down...')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
