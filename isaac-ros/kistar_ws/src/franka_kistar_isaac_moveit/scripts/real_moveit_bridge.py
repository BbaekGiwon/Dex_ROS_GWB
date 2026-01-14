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
        last_t = 0.0
        for i, pt in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("[Action] Goal canceled")
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

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

            cmd = FrankaArmTarget()
            cmd.joint_targets = positions
            cmd.arm_id = self.arm_id
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"[Action] Sent point {i + 1}/{n_points}")

            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dt = max(t - last_t, self.min_dt)
            time.sleep(dt)
            last_t = t

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
