# IsaacLab policy 루프 안에서 쓸 예시 코드

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np


class PolicyPublisher(Node):
    def __init__(self, topic_name="/policy_action"):
        super().__init__("policy_publisher")
        self.pub = self.create_publisher(Float32MultiArray, topic_name, 10)

    def publish_action(self, action: np.ndarray):
        msg = Float32MultiArray()
        # numpy -> python list
        msg.data = action.astype(float).ravel().tolist()
        self.pub.publish(msg)


# ====== IsaacLab 쪽 메인 ======
def run_rl_with_ros():
    # IsaacLab env, policy 초기화 부분은 네 코드 그대로 두고,
    # 그 앞뒤에 rclpy 래핑만 추가하면 됨.

    rclpy.init(args=None)
    node = PolicyPublisher("/policy_action")

    try:
        while True:
            # IsaacLab 환경에서 observation 받아서
            obs = env.get_obs()
            # policy로 action 계산
            action = policy(obs)  # numpy array라고 가정

            # 시뮬레이터에 action 적용
            env.apply_action(action)

            # 동시에 ROS 2 토픽으로도 publish
            node.publish_action(action)

            # 콜백 처리 (서비스/구독이 필요 없으면 짧게만)
            rclpy.spin_once(node, timeout_sec=0.0)

            # IsaacLab 시뮬레이션 step
            env.step()
    finally:
        node.destroy_node()
        rclpy.shutdown()
