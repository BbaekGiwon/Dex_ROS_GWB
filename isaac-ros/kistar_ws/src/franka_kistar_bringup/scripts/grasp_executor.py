#!/usr/bin/env python3
"""
grasp_executor.py — DRO-Grasp 출력을 읽어 FR3 + KISTAR 핸드를 실행합니다.

공유 디렉터리의 grasp_command.json 을 감지하면:
  1. MoveIt2 로 arm 궤적 계획 (fr3_arm, fr3_hand_tcp)
  2. 궤적 실행 (trajectory_forwarder 또는 direct_franka_topic)
  3. 핸드 관절 목표 전송 (/hand/target_joint, Int16MultiArray)
  4. 결과를 grasp_status.json 에 기록

사용법 (Docker 내부):
    ros2 run franka_kistar_bringup grasp_executor \\
        --ros-args \\
        -p watch_dir:=/shared/grasp_ros2 \\
        -p auto_execute:=true \\
        -p execute_mode:=trajectory_forwarder

공유 디렉터리 마운트 예시:
    docker run -v /home/kist/shared/grasp_ros2:/shared/grasp_ros2 ...

Author: generated for Affordance_grasp_DRO + dex_ros integration
"""

import json
import math
import os
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, Float64MultiArray, Int16MultiArray


class GraspExecutor(Node):
    """공유 파일을 감지해 DRO-Grasp 을 실행하는 ROS2 노드."""

    def __init__(self):
        super().__init__("grasp_executor")

        # ---------- 파라미터 ----------
        self.declare_parameter("watch_dir",         "/shared/grasp_ros2")
        self.declare_parameter("planning_group",    "fr3_arm")
        self.declare_parameter("end_effector_link", "fr3_hand_tcp")
        self.declare_parameter("reference_frame",   "world")
        self.declare_parameter("planning_time",     5.0)
        self.declare_parameter("execute_mode",      "trajectory_forwarder")
        self.declare_parameter("auto_execute",      False)
        self.declare_parameter("speed_factor",      0.1)
        self.declare_parameter("execute_hand",      True)
        self.declare_parameter("hand_target_topic", "/hand/target_joint")
        self.declare_parameter("hand_move_duration", 2.0)
        self.declare_parameter("poll_interval",     0.5)

        self.watch_dir       = Path(self.get_parameter("watch_dir").value)
        self.planning_group  = self.get_parameter("planning_group").value
        self.ee_link         = self.get_parameter("end_effector_link").value
        self.ref_frame       = self.get_parameter("reference_frame").value
        self.planning_time   = self.get_parameter("planning_time").value
        self.execute_mode    = self.get_parameter("execute_mode").value
        self.auto_execute    = self.get_parameter("auto_execute").value
        self.speed_factor    = self.get_parameter("speed_factor").value
        self.execute_hand    = self.get_parameter("execute_hand").value
        self.hand_topic      = self.get_parameter("hand_target_topic").value
        self.hand_duration   = self.get_parameter("hand_move_duration").value
        poll_sec             = self.get_parameter("poll_interval").value

        self._busy = False
        self.cb_group = ReentrantCallbackGroup()

        # ---------- Publishers / Action Clients ----------
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.cb_group
        )
        self.display_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )
        self.franka_target_pub = self.create_publisher(
            Float64MultiArray, "/franka/target_joint", 10
        )
        self.franka_speed_pub = self.create_publisher(
            Float64, "/franka/target_speed_factor", 10
        )
        self.hand_pub = self.create_publisher(
            Int16MultiArray, self.hand_topic, 10
        )

        if self.execute_mode == "trajectory_forwarder":
            self.traj_client = ActionClient(
                self,
                FollowJointTrajectory,
                "/fr3_arm_controller/follow_joint_trajectory",
                callback_group=self.cb_group,
            )
        else:
            self.traj_client = None

        # ---------- 감시 타이머 ----------
        self.timer = self.create_timer(
            poll_sec, self._poll_callback, callback_group=self.cb_group
        )

        self.get_logger().info("=" * 65)
        self.get_logger().info("Grasp Executor 시작")
        self.get_logger().info(f"  감시 디렉터리 : {self.watch_dir}")
        self.get_logger().info(f"  실행 모드      : {self.execute_mode}")
        self.get_logger().info(f"  자동 실행      : {self.auto_execute}")
        self.get_logger().info(f"  핸드 실행      : {self.execute_hand}")
        self.get_logger().info("=" * 65)
        self.get_logger().info("grasp_command.json 을 기다리는 중...")

    # ------------------------------------------------------------------
    # 파일 감시
    # ------------------------------------------------------------------

    def _poll_callback(self):
        if self._busy:
            return

        command_path = self.watch_dir / "grasp_command.json"
        if not command_path.exists():
            return

        # atomic rename → 이중 처리 방지
        processing_path = self.watch_dir / "grasp_command_processing.json"
        try:
            os.rename(command_path, processing_path)
        except OSError:
            return  # 다른 프로세스가 먼저 가져감

        self._busy = True
        thread = threading.Thread(
            target=self._execute_grasp, args=(processing_path,), daemon=True
        )
        thread.start()

    # ------------------------------------------------------------------
    # 메인 실행 흐름
    # ------------------------------------------------------------------

    def _execute_grasp(self, command_path: Path):
        status = {"status": "failed", "reason": "unknown"}
        try:
            with open(command_path) as f:
                cmd = json.load(f)

            robot = cmd.get("robot", "?")
            idx   = cmd.get("grasp_index", 0)
            pose  = cmd["pose"]
            hand_enc: list[int] = cmd.get("hand_enc", [])

            frame = pose.get("frame", "?")
            xyz   = pose["xyz"]
            quat  = pose["quat_xyzw"]   # [qx, qy, qz, qw]

            self.get_logger().info(f"\n{'='*65}")
            self.get_logger().info(f"Grasp 실행 요청  robot={robot}  index={idx}")
            self.get_logger().info(
                f"  Pose [{frame}] xyz=({xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f})"
            )
            self.get_logger().info(
                f"  quat=({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})"
            )

            if frame != "base":
                self.get_logger().warn(
                    f"pose frame='{frame}'. base frame이 아닙니다! "
                    "send_to_ros2.py 실행 전에 --calibration 옵션을 사용하세요."
                )

            # ── 1. Arm 계획 ──────────────────────────────────────────
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.ref_frame
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position    = Point(x=xyz[0], y=xyz[1], z=xyz[2])
            target_pose.pose.orientation = Quaternion(
                x=quat[0], y=quat[1], z=quat[2], w=quat[3]
            )

            trajectory, start_state = self._plan_arm(target_pose)
            if trajectory is None:
                status = {"status": "failed", "reason": "planning_failed"}
                return

            # RViz 시각화
            disp = DisplayTrajectory()
            disp.model_id = self.planning_group
            disp.trajectory_start = start_state
            disp.trajectory.append(trajectory)
            self.display_pub.publish(disp)
            self.get_logger().info("  RViz 궤적 퍼블리시 완료 (/display_planned_path)")

            if not self.auto_execute:
                self.get_logger().warn(
                    "auto_execute=false: 실행하려면 노드 파라미터를 "
                    "auto_execute:=true 로 설정하거나 grasp_confirm 파일을 생성하세요."
                )
                self._wait_for_confirm()

            # ── 2. Arm 실행 ──────────────────────────────────────────
            self.get_logger().info("[ARM] 궤적 실행 시작...")
            arm_ok = self._execute_arm(trajectory.joint_trajectory)
            if not arm_ok:
                status = {"status": "failed", "reason": "arm_execution_failed"}
                return
            self.get_logger().info("[ARM] 실행 완료")

            # ── 3. Hand 실행 ──────────────────────────────────────────
            if self.execute_hand and hand_enc:
                self.get_logger().info(
                    f"[HAND] 관절 목표 전송 ({self.hand_duration}s)  "
                    f"enc={hand_enc}"
                )
                hand_msg = Int16MultiArray()
                hand_msg.data = [int(v) for v in hand_enc]
                self.hand_pub.publish(hand_msg)
                time.sleep(self.hand_duration)
                self.get_logger().info("[HAND] 완료")
            elif self.execute_hand and not hand_enc:
                self.get_logger().warn("[HAND] hand_enc 없음, 핸드 실행 생략")

            status = {"status": "success"}
            self.get_logger().info("Grasp 실행 성공!")

        except Exception as e:
            self.get_logger().error(f"Grasp 실행 오류: {e}")
            import traceback
            traceback.print_exc()
            status = {"status": "failed", "reason": str(e)}

        finally:
            # 결과 기록
            status_path = self.watch_dir / "grasp_status.json"
            with open(status_path, "w") as f:
                json.dump(status, f, indent=2)
            self.get_logger().info(f"결과 → {status_path}")

            # 처리 완료된 command 파일 삭제
            try:
                command_path.unlink(missing_ok=True)
            except Exception:
                pass

            self._busy = False

    # ------------------------------------------------------------------
    # MoveIt2 계획
    # ------------------------------------------------------------------

    def _plan_arm(self, target_pose: PoseStamped):
        """MoveGroup 액션으로 Cartesian pose 계획. 성공 시 (trajectory, start) 반환."""
        goal = MoveGroup.Goal()
        goal.request.group_name              = self.planning_group
        goal.request.num_planning_attempts   = 5
        goal.request.allowed_planning_time   = self.planning_time
        goal.request.max_velocity_scaling_factor     = min(1.0, self.speed_factor)
        goal.request.max_acceleration_scaling_factor = min(1.0, self.speed_factor)

        # Position constraint
        pos_c = PositionConstraint()
        pos_c.header    = target_pose.header
        pos_c.link_name = self.ee_link
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]
        pos_c.constraint_region.primitives      = [sphere]
        pos_c.constraint_region.primitive_poses = [target_pose.pose]
        pos_c.weight = 1.0

        # Orientation constraint
        ori_c = OrientationConstraint()
        ori_c.header      = target_pose.header
        ori_c.link_name   = self.ee_link
        ori_c.orientation = target_pose.pose.orientation
        ori_c.absolute_x_axis_tolerance = 0.05
        ori_c.absolute_y_axis_tolerance = 0.05
        ori_c.absolute_z_axis_tolerance = 0.05
        ori_c.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints    = [pos_c]
        constraints.orientation_constraints = [ori_c]
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True

        self.get_logger().info("[PLAN] MoveGroup 에 계획 요청 중...")
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[PLAN] MoveGroup 서버 연결 실패")
            return None, None

        future = self.move_group_client.send_goal_async(goal)
        timeout = self.planning_time + 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)

        if not future.done() or not future.result().accepted:
            self.get_logger().error("[PLAN] 계획 요청 거부됨")
            return None, None

        res_future = future.result().get_result_async()
        start = time.time()
        while not res_future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)

        if not res_future.done():
            self.get_logger().error("[PLAN] 계획 타임아웃")
            return None, None

        result = res_future.result().result
        if result.error_code.val != 1:
            codes = {-1: "PLANNING_FAILED", -6: "TIMED_OUT", -10: "START_IN_COLLISION",
                     -31: "NO_IK_SOLUTION"}
            desc = codes.get(result.error_code.val, str(result.error_code.val))
            self.get_logger().error(f"[PLAN] 계획 실패: {desc}")
            return None, None

        n = len(result.planned_trajectory.joint_trajectory.points)
        self.get_logger().info(f"[PLAN] 성공  waypoints={n}")
        return result.planned_trajectory, result.trajectory_start

    # ------------------------------------------------------------------
    # Arm 실행
    # ------------------------------------------------------------------

    def _execute_arm(self, joint_trajectory) -> bool:
        if self.execute_mode == "trajectory_forwarder":
            return self._execute_via_forwarder(joint_trajectory)
        elif self.execute_mode == "direct_franka_topic":
            return self._execute_via_direct_topic(joint_trajectory)
        else:
            self.get_logger().error(f"알 수 없는 execute_mode: {self.execute_mode}")
            return False

    def _execute_via_forwarder(self, joint_trajectory) -> bool:
        if not self.traj_client:
            self.get_logger().error("traj_client 초기화 안됨")
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_trajectory

        future = self.traj_client.send_goal_async(goal)
        timeout = 10.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)

        if not future.done() or not future.result().accepted:
            self.get_logger().error("[ARM] trajectory_forwarder 목표 거부됨")
            return False

        # 실행 완료 대기
        traj_duration = 0.0
        if joint_trajectory.points:
            last = joint_trajectory.points[-1]
            traj_duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        wait_time = traj_duration + 3.0
        self.get_logger().info(f"[ARM] 실행 중... ({traj_duration:.1f}s 예상)")
        time.sleep(wait_time)
        return True

    def _execute_via_direct_topic(self, joint_trajectory) -> bool:
        if not joint_trajectory.points:
            self.get_logger().error("[ARM] 궤적이 비어 있음")
            return False

        final = joint_trajectory.points[-1]
        speed_msg = Float64()
        speed_msg.data = max(0.001, min(1.0, self.speed_factor))
        target_msg = Float64MultiArray()
        target_msg.data = list(final.positions)

        self.franka_speed_pub.publish(speed_msg)
        time.sleep(0.05)
        self.franka_target_pub.publish(target_msg)

        # 이동 완료 대기
        traj_duration = (final.time_from_start.sec
                         + final.time_from_start.nanosec * 1e-9)
        wait_time = max(traj_duration, 2.0) + 1.0
        self.get_logger().info(f"[ARM] 이동 중... ({wait_time:.1f}s 대기)")
        time.sleep(wait_time)
        return True

    # ------------------------------------------------------------------
    # 확인 대기 (auto_execute=false 일 때)
    # ------------------------------------------------------------------

    def _wait_for_confirm(self):
        confirm_path = self.watch_dir / "grasp_confirm"
        self.get_logger().info(
            f"실행 확인을 기다리는 중...\n"
            f"  실행:  touch {confirm_path}\n"
            f"  취소:  touch {self.watch_dir}/grasp_cancel"
        )
        cancel_path = self.watch_dir / "grasp_cancel"
        while True:
            if confirm_path.exists():
                confirm_path.unlink(missing_ok=True)
                self.get_logger().info("확인됨 → 실행합니다.")
                return
            if cancel_path.exists():
                cancel_path.unlink(missing_ok=True)
                raise RuntimeError("사용자가 실행을 취소했습니다.")
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = GraspExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Grasp Executor 종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
