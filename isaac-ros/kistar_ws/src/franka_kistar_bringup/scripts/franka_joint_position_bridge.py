#!/usr/bin/env python3
"""Bridge Franka and KISTAR hand joint arrays to /joint_states."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray


ARM_JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

HAND_JOINT_NAMES = [
    "index_joint_0",
    "index_joint_1",
    "index_joint_2",
    "index_joint_3",
    "middle_joint_0",
    "middle_joint_1",
    "middle_joint_2",
    "middle_joint_3",
    "ring_joint_0",
    "ring_joint_1",
    "ring_joint_2",
    "ring_joint_3",
    "thumb_joint_0",
    "thumb_joint_1",
    "thumb_joint_2",
    "thumb_joint_3",
]


class FrankaJointPositionBridge(Node):
    """Publish sensor_msgs/JointState from arm and hand array topics."""

    def __init__(self) -> None:
        super().__init__("franka_joint_position_bridge")

        self.declare_parameter("joint_position_topic", "/franka/joint_position")
        self.declare_parameter("joint_velocity_topic", "/franka/joint_velocity")
        self.declare_parameter("joint_torque_topic", "/franka/joint_torque")
        self.declare_parameter("hand_joint_position_topic", "/hand/joint_position")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_hand_zero_joints", True)
        self.declare_parameter("hand_positions_in_degrees", True)

        self._latest_velocity: list[float] | None = None
        self._latest_effort: list[float] | None = None
        self._latest_hand_position: list[float] | None = None

        self._publish_hand_zero_joints = bool(
            self.get_parameter("publish_hand_zero_joints").value
        )
        self._hand_positions_in_degrees = bool(
            self.get_parameter("hand_positions_in_degrees").value
        )
        self._joint_states_pub = self.create_publisher(
            JointState,
            self.get_parameter("joint_states_topic").value,
            10,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("joint_position_topic").value,
            self._position_cb,
            sensor_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("joint_velocity_topic").value,
            self._velocity_cb,
            sensor_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("joint_torque_topic").value,
            self._effort_cb,
            sensor_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter("hand_joint_position_topic").value,
            self._hand_position_cb,
            sensor_qos,
        )

        self.get_logger().info(
            "Bridging /franka/joint_position + /hand/joint_position to /joint_states"
        )

    def _velocity_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= len(ARM_JOINT_NAMES):
            self._latest_velocity = list(msg.data[: len(ARM_JOINT_NAMES)])

    def _effort_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= len(ARM_JOINT_NAMES):
            self._latest_effort = list(msg.data[: len(ARM_JOINT_NAMES)])

    def _hand_position_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < len(HAND_JOINT_NAMES):
            self.get_logger().warn(
                f"Expected at least {len(HAND_JOINT_NAMES)} hand joint positions, "
                f"got {len(msg.data)}"
            )
            return

        hand_position = list(msg.data[: len(HAND_JOINT_NAMES)])
        if self._hand_positions_in_degrees:
            hand_position = [math.radians(value) for value in hand_position]
        self._latest_hand_position = hand_position

    def _position_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < len(ARM_JOINT_NAMES):
            self.get_logger().warn(
                f"Expected at least {len(ARM_JOINT_NAMES)} joint positions, "
                f"got {len(msg.data)}"
            )
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(ARM_JOINT_NAMES)
        joint_state.position = list(msg.data[: len(ARM_JOINT_NAMES)])

        if self._latest_velocity is not None:
            joint_state.velocity = list(self._latest_velocity)
        if self._latest_effort is not None:
            joint_state.effort = list(self._latest_effort)

        hand_position = self._latest_hand_position
        if hand_position is None and self._publish_hand_zero_joints:
            hand_position = [0.0] * len(HAND_JOINT_NAMES)

        if hand_position is not None:
            joint_state.name.extend(HAND_JOINT_NAMES)
            joint_state.position.extend(hand_position)
            if joint_state.velocity:
                joint_state.velocity.extend([0.0] * len(HAND_JOINT_NAMES))
            if joint_state.effort:
                joint_state.effort.extend([0.0] * len(HAND_JOINT_NAMES))

        self._joint_states_pub.publish(joint_state)


def main() -> None:
    rclpy.init()
    node = FrankaJointPositionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
