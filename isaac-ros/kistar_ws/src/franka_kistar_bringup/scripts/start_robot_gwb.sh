#!/usr/bin/env bash
# =============================================================================
# start_robot_gwb.sh
#
# 1. fr3_interactive_pose_control.launch_GWB.py (MoveIt + RViz + bridge)를
#    백그라운드에서 실행
# 2. MoveGroup action server 준비 대기
# 3. pose_commander_GWB.py를 포그라운드에서 직접 실행 (stdin = 현재 터미널)
#
# Usage:
#   ./start_robot_gwb.sh [gui:=true] [execute_mode:=direct_franka_topic] ...
#
# Arguments (optional, same as the launch file):
#   gui                  true / false          (default: true)
#   use_fake_joint_states true / false          (default: false)
#   execute_mode         trajectory_forwarder / direct_franka_topic
#                                              (default: direct_franka_topic)
#   reference_frame      world / base          (default: base)
#   end_effector_link    fr3_link8 / ...       (default: fr3_link8)
#   planning_time        float seconds         (default: 5.0)
#   franka_speed_factor  0.0 ~ 1.0             (default: 0.1)
# =============================================================================

set -e

# ── conda 환경 완전 정리 (ROS2 Python과의 충돌 방지) ────────────────────────
# conda deactivate만으로는 PYTHONPATH 등이 남아 Python 3.13이 로드될 수 있음
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
unset CONDA_EXE CONDA_PYTHON_EXE CONDA_SHLVL
# conda bin을 PATH에서 제거
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/conda' | tr '\n' ':' | sed 's/:$//')

# ── 기본값 ──────────────────────────────────────────────────────────────────
GUI="true"
USE_FAKE="false"
EXEC_MODE="direct_franka_topic"
REF_FRAME="base"
EE_LINK="fr3_link8"
PLANNING_TIME="5.0"
SPEED_FACTOR="0.1"

# ── 인자 파싱 (key:=value 형식) ──────────────────────────────────────────────
for arg in "$@"; do
    case "$arg" in
        gui:=*)               GUI="${arg#gui:=}" ;;
        use_fake_joint_states:=*) USE_FAKE="${arg#use_fake_joint_states:=}" ;;
        execute_mode:=*)      EXEC_MODE="${arg#execute_mode:=}" ;;
        reference_frame:=*)   REF_FRAME="${arg#reference_frame:=}" ;;
        end_effector_link:=*) EE_LINK="${arg#end_effector_link:=}" ;;
        planning_time:=*)     PLANNING_TIME="${arg#planning_time:=}" ;;
        franka_speed_factor:=*) SPEED_FACTOR="${arg#franka_speed_factor:=}" ;;
        *)
            echo "[WARN] Unknown argument: $arg"
            ;;
    esac
done

# ── DISPLAY 없으면 gui 자동 off ──────────────────────────────────────────────
if [ "$GUI" = "true" ] && [ -z "$DISPLAY" ]; then
    echo "[WARN] DISPLAY not set — forcing gui:=false (RViz disabled)"
    GUI="false"
fi

echo "============================================================"
echo "  Starting Franka KISTAR Robot (GWB)"
echo "  gui              : $GUI"
echo "  use_fake_joints  : $USE_FAKE"
echo "  execute_mode     : $EXEC_MODE"
echo "  reference_frame  : $REF_FRAME"
echo "  end_effector_link: $EE_LINK"
echo "  planning_time    : ${PLANNING_TIME}s"
echo "  franka_speed     : $SPEED_FACTOR"
echo "============================================================"

# ── ROS2 환경 ────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"   # kistar_ws

# base ROS2 먼저 (PYTHONPATH 포함 기본 환경 보장)
# shellcheck source=/dev/null
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash

# 워크스페이스 overlay (local_setup.bash = base 중복 source 없이 workspace만 추가)
# shellcheck source=/dev/null
[ -f "$WS_ROOT/install/local_setup.bash" ] && source "$WS_ROOT/install/local_setup.bash"

# ── 1) 런치 파일 백그라운드 실행 ─────────────────────────────────────────────
echo "[INFO] Starting launch file in background..."
ros2 launch franka_kistar_bringup fr3_interactive_pose_control.launch_GWB.py \
    gui:="$GUI" \
    use_fake_joint_states:="$USE_FAKE" \
    execute_mode:="$EXEC_MODE" \
    reference_frame:="$REF_FRAME" \
    end_effector_link:="$EE_LINK" \
    planning_time:="$PLANNING_TIME" \
    franka_speed_factor:="$SPEED_FACTOR" \
    &
LAUNCH_PID=$!

# 종료 시 백그라운드 프로세스도 정리
cleanup() {
    echo ""
    echo "[INFO] Shutting down launch file (PID: $LAUNCH_PID)..."
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    echo "[INFO] Done."
}
trap cleanup EXIT INT TERM

# ── 2) MoveGroup 준비 대기 (고정 sleep) ─────────────────────────────────────
echo "[INFO] Waiting 15s for MoveGroup to start..."
sleep 15
echo "[INFO] Starting Pose Commander..."

# ── 3) pose_commander 포그라운드 실행 (stdin = 현재 터미널) ──────────────────
ros2 run franka_kistar_bringup pose_commander_GWB.py \
    --ros-args \
    -p gui:="$GUI" \
    -p planning_group:=fr3_arm \
    -p end_effector_link:="$EE_LINK" \
    -p planning_time:="$PLANNING_TIME" \
    -p reference_frame:="$REF_FRAME" \
    -p execute_mode:="$EXEC_MODE" \
    -p franka_speed_factor:="$SPEED_FACTOR"
