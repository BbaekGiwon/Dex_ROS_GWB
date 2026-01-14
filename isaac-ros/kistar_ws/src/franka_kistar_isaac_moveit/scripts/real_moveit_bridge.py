#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from kistar_hand_ros2.msg import FrankaArmState, FrankaArmTarget


class RealMoveItBridge(Node):
    """MoveIt -> real Franka arm bridge (right arm by default)."""

    def __init__(self):
        super().__init__("real_moveit_bridge")

        self.declare_parameter("arm_side", "right")
        self.declare_parameter("arm_state_topic", "")
        self.declare_parameter("arm_target_topic", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "traj_action_name", "/fr3_arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("command_rate_hz", 200.0)
        self.declare_parameter(
            "joint_names",
            [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
        )
        self.declare_parameter("min_dt", 0.02)

        arm_side = self.get_parameter("arm_side").get_parameter_value().string_value
        if arm_side not in ("left", "right"):
            self.get_logger().warn(
                f"Invalid arm_side '{arm_side}', falling back to 'right'."
            )
            arm_side = "right"

        self.arm_id = 0 if arm_side == "right" else 1

        arm_state_topic = (
            self.get_parameter("arm_state_topic")
            .get_parameter_value()
            .string_value
        )
        arm_target_topic = (
            self.get_parameter("arm_target_topic")
            .get_parameter_value()
            .string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic")
            .get_parameter_value()
            .string_value
        )
        traj_action_name = (
            self.get_parameter("traj_action_name")
            .get_parameter_value()
            .string_value
        )

        if not arm_state_topic:
            arm_state_topic = f"/franka/arm_state/{arm_side}"
        if not arm_target_topic:
            arm_target_topic = f"/franka/arm_target/{arm_side}"

        joint_names_param = self.get_parameter("joint_names").get_parameter_value()
        self.joint_names = list(joint_names_param.string_array_value)
        if not self.joint_names:
            self.joint_names = [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ]

        self.command_rate_hz = max(
            self.get_parameter("command_rate_hz").get_parameter_value().double_value,
            1.0,
        )
        self.min_dt = max(
            self.get_parameter("min_dt").get_parameter_value().double_value, 0.001
        )

        self.js_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.state_sub = self.create_subscription(
            FrankaArmState, arm_state_topic, self._arm_state_cb, 10
        )
        self.cmd_pub = self.create_publisher(FrankaArmTarget, arm_target_topic, 10)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            traj_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"RealMoveItBridge started (arm_side={arm_side}, state={arm_state_topic}, "
            f"target={arm_target_topic})."
        )

    def _arm_state_cb(self, msg: FrankaArmState):
        positions = list(msg.joint_positions)
        if len(positions) != len(self.joint_names):
            self.get_logger().warn(
                f"ArmState length mismatch: {len(positions)} vs {len(self.joint_names)}"
            )
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions
        if len(msg.joint_torques) == len(self.joint_names):
            js.effort = list(msg.joint_torques)
        self.js_pub.publish(js)

    def goal_callback(self, goal_request):
        traj = goal_request.trajectory
        self.get_logger().info(
            f"[Action] Goal received: {len(traj.points)} points, "
            f"joints={list(traj.joint_names)}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("[Action] Cancel requested")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        n_points = len(traj.points)
        if n_points == 0:
            self.get_logger().warn("[Action] Empty trajectory")
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        goal_names = list(traj.joint_names)
        order_idx = None
        if goal_names and self.joint_names and goal_names != self.joint_names:
            try:
                order_idx = [goal_names.index(name) for name in self.joint_names]
                self.get_logger().warn(
                    "Joint order differs; reordering to match expected joint_names."
                )
            except ValueError:
                self.get_logger().warn(
                    "Joint name mismatch; using goal order without reordering."
                )
                order_idx = None

        self.get_logger().info(f"[Action] Executing trajectory ({n_points} points)")
        times = []
        positions_list = []
        last_t = 0.0
        for pt in traj.points:
            positions = list(pt.positions)
            if order_idx is not None:
                positions = [positions[idx] for idx in order_idx]

            if len(positions) != len(self.joint_names):
                self.get_logger().error(
                    "Position length mismatch; aborting trajectory execution."
                )
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                goal_handle.abort()
                return result

            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if t <= last_t:
                t = last_t + self.min_dt
            times.append(t)
            positions_list.append(positions)
            last_t = t

        if len(times) == 1:
            cmd = FrankaArmTarget()
            cmd.joint_targets = positions_list[0]
            cmd.arm_id = self.arm_id
            self.cmd_pub.publish(cmd)
        else:
            duration = times[-1]
            dt = 1.0 / self.command_rate_hz
            start_time = time.monotonic()
            next_tick = start_time
            idx = 0

            while True:
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("[Action] Goal canceled")
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                now = time.monotonic()
                t = now - start_time
                if t >= duration:
                    break

                while idx < len(times) - 2 and t > times[idx + 1]:
                    idx += 1

                t0 = times[idx]
                t1 = times[idx + 1]
                p0 = positions_list[idx]
                p1 = positions_list[idx + 1]
                if t1 <= t0:
                    alpha = 0.0
                else:
                    alpha = max(0.0, min((t - t0) / (t1 - t0), 1.0))

                interp = [
                    p0[j] + alpha * (p1[j] - p0[j]) for j in range(len(self.joint_names))
                ]

                cmd = FrankaArmTarget()
                cmd.joint_targets = interp
                cmd.arm_id = self.arm_id
                self.cmd_pub.publish(cmd)

                next_tick += dt
                sleep_time = max(0.0, next_tick - time.monotonic())
                time.sleep(sleep_time)

            cmd = FrankaArmTarget()
            cmd.joint_targets = positions_list[-1]
            cmd.arm_id = self.arm_id
            self.cmd_pub.publish(cmd)

        self.get_logger().info("[Action] Trajectory done")
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RealMoveItBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
