#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

import tf2_ros


class PlanningSceneStaticBoxes(Node):
    def __init__(self):
        super().__init__("planning_scene_static_boxes")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("table_frame", "table_link")
        self.declare_parameter("ttable_frame", "ttable_link")
        self.declare_parameter("table_size", [1.0, 0.8, 0.05])
        self.declare_parameter("ttable_size", [0.6, 0.6, 0.05])
        self.declare_parameter("timeout_sec", 10.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.table_frame = self.get_parameter("table_frame").value
        self.ttable_frame = self.get_parameter("ttable_frame").value
        self.table_size = self.get_parameter("table_size").value
        self.ttable_size = self.get_parameter("ttable_size").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.timer = self.create_timer(0.5, self._tick)
        self.start_time = self.get_clock().now()

    def _lookup_pose(self, target_frame: str) -> Pose:
        tf = self.tf_buffer.lookup_transform(self.world_frame, target_frame, rclpy.time.Time())
        p = Pose()
        p.position.x = tf.transform.translation.x
        p.position.y = tf.transform.translation.y
        p.position.z = tf.transform.translation.z
        p.orientation = tf.transform.rotation
        return p

    def _make_box(self, object_id: str, pose: Pose, size_xyz) -> CollisionObject:
        co = CollisionObject()
        co.id = object_id
        co.header.frame_id = self.world_frame

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size_xyz[0]), float(size_xyz[1]), float(size_xyz[2])]

        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def _tick(self):
        # timeout
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed > self.timeout_sec:
            self.get_logger().error("Timeout waiting for TF/service. Giving up.")
            rclpy.shutdown()
            return

        if not self.client.wait_for_service(timeout_sec=0.1):
            return

        try:
            table_pose = self._lookup_pose(self.table_frame)
            ttable_pose = self._lookup_pose(self.ttable_frame)

            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(self._make_box("table", table_pose, self.table_size))
            scene.world.collision_objects.append(self._make_box("ttable", ttable_pose, self.ttable_size))
            
            # Table Camera
            scene.world.collision_objects.append(self._make_box("camera", table_pose, [0.06, 0.06, 1.8]))

            req = ApplyPlanningScene.Request()
            req.scene = scene
            fut = self.client.call_async(req)

            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            if fut.result() and fut.result().success:
                self.get_logger().info("Applied planning scene: added table/ttable collision boxes.")
            else:
                self.get_logger().error("Failed to apply planning scene.")
            rclpy.shutdown()

        except Exception as e:
            # TF 아직 안 올라왔을 수 있음 -> 다음 tick에서 재시도
            self.get_logger().warn(f"Waiting... ({e})")


def main():
    rclpy.init()
    node = PlanningSceneStaticBoxes()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
