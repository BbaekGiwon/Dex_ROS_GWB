import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
"""
./isaaclab.sh \
  -p dex_ros/examples/run_soldering_env.py

"""

import omni.usd
import omni.kit.app as kit
import omni.timeline
import omni.graph.core as og

from isaacsim.core.api import PhysicsContext
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Sdf

# 로봇 아티큘레이션이 올라갈 prim path (MoveIt에서 제어할 타겟)
FRANKA_STAGE_PATH = "/World"
ROBOT_PATH = "/World/kistar"
# ROS에서 사용할 토픽 이름
JOINT_STATE_TOPIC = "isaac_joint_states"
JOINT_COMMAND_TOPIC = "isaac_joint_commands"

ROBOT_USD_PATH = "/home/cy/isaac_ws/dex_soldering/source/isaaclab_assets/isaaclab_assets/soldering_asset/soldering_ros_env.usd"


def create_ros_action_graph(robot_articulation_path: str):
    """ROS2 <-> 로봇 관절 제어용 OmniGraph(ActionGraph) 생성."""
    try:
        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                    (
                        "SubscribeJointState",
                        "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                    ),
                    (
                        "ArticulationController",
                        "isaacsim.core.nodes.IsaacArticulationController",
                    ),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "SubscribeJointState.inputs:execIn",
                    ),
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    (
                        "OnPlaybackTick.outputs:tick",
                        "ArticulationController.inputs:execIn",
                    ),
                    ("Context.outputs:context", "PublishJointState.inputs:context"),
                    ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishJointState.inputs:timeStamp",
                    ),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),
                    (
                        "SubscribeJointState.outputs:jointNames",
                        "ArticulationController.inputs:jointNames",
                    ),
                    (
                        "SubscribeJointState.outputs:positionCommand",
                        "ArticulationController.inputs:positionCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:velocityCommand",
                        "ArticulationController.inputs:velocityCommand",
                    ),
                    (
                        "SubscribeJointState.outputs:effortCommand",
                        "ArticulationController.inputs:effortCommand",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # 여기가 **실제 articulation prim** 이어야 함
                    (
                        "ArticulationController.inputs:robotPath",
                        robot_articulation_path,
                    ),
                    ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
                    ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
                    # 여기서 Sdf.Path 사용 (샘플 코드 형태와 동일)
                    (
                        "PublishJointState.inputs:targetPrim",
                        [Sdf.Path(robot_articulation_path)],
                    ),
                ],
            },
        )
        print(
            f"[INFO] Created ROS2 action graph for robot at {robot_articulation_path}"
        )
    except Exception as e:
        print("[ERROR] Failed to create ROS action graph:", e)


def main():
    app = kit.get_app()
    ext_manager = app.get_extension_manager()
    try:
        ext_manager.set_extension_enabled_immediate("isaaclab_tasks", False)
    except Exception:
        pass
    ext_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    print("[INFO] Enabled extension: isaacsim.ros2.bridge")
    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()

    # 로봇 USD 스폰 (FR3 + KISTAR + 테이블 포함된 usd라고 가정)

    # 물리 컨텍스트 생성 (dt 설정)

    create_prim(
        prim_path=FRANKA_STAGE_PATH,
        usd_path=ROBOT_USD_PATH,
    )
    set_camera_view(
        eye=[1.5, 0.0, 1.2],
        target=[0.0, 0.0, 0.5],
        camera_prim_path="/OmniverseKit_Persp",
    )
    PhysicsContext(physics_dt=1.0 / 60.0)
    print(f"[INFO] Spawning FR3+KISTAR from: {ROBOT_USD_PATH}")
    # 한 프레임 정도 업데이트해서 USD/PhysX 초기화 기다리기
    for _ in range(5):
        simulation_app.update()

    if not stage.GetPrimAtPath(ROBOT_PATH):
        print(f"[WARN] No prim at {ROBOT_PATH} — check your USD hierarchy.")
    else:
        print(f"[INFO] Found articulation prim at {ROBOT_PATH}")

    # ROS2 액션 그래프 생성
    create_ros_action_graph(ROBOT_PATH)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[INFO] Setup complete. Starting simulation loop...")

    # 메인 루프
    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()
    print("[INFO] Simulation closed.")


if __name__ == "__main__":
    main()
