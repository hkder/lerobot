#!/usr/bin/env bash
# One command for the SO-101 setup. Run ./so101.sh with no arguments for help.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
set -a; source "$HERE/config.env"; [ -f "$HERE/config.local.env" ] && source "$HERE/config.local.env"; set +a
LEROBOT="$(cd "$HERE/.." && pwd)"
TOOLS="$HERE/tools"
FOLLOWER="$SO101_FOLLOWER"
LEADER="$SO101_LEADER"
CAM_OPTS="${SO101_CAM_FOURCC:+, fourcc: $SO101_CAM_FOURCC}${SO101_CAM_BACKEND:+, backend: $SO101_CAM_BACKEND}"
CAMERAS="{ wrist: {type: opencv, index_or_path: $SO101_WRIST_CAM, width: 1280, height: 720, fps: 30$CAM_OPTS}, top: {type: opencv, index_or_path: $SO101_TOP_CAM, width: 1280, height: 720, fps: 30$CAM_OPTS} }"

# Windows COMn is /dev/ttyS(n-1) in Git Bash.
port_exists() {
  case "$1" in
    COM[0-9]*) [ -e "/dev/ttyS$(( ${1#COM} - 1 ))" ] ;;
    *) [ -e "$1" ] ;;
  esac
}

need() {
  for port in "$@"; do
    if ! port_exists "$port"; then
      echo "Not connected: $port"
      echo "  follower = 12V board (…177611), leader = 6V board (…160051). Check USB and power."
      exit 1
    fi
  done
}

run() { cd "$LEROBOT" && uv run "$@"; }

# Ctrl+C stops the running program, then the script continues and parks the follower.
then_rest() {
  trap 'true' INT
  run "$@" || true
  trap - INT
  echo "Returning follower to rest pose..."
  run python "$TOOLS/rest.py" || true
}

case "${1:-help}" in
  teleop)
    need "$FOLLOWER" "$LEADER"
    echo "Teleop (stiffness P=${2:-32}). Match both arms' poses first. Ctrl+C to stop."
    then_rest lerobot-teleoperate \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower \
      --robot.position_p_coefficient="${2:-32}" --robot.disable_torque_on_disconnect=false \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id=my_leader
    ;;
  teleop-cam)
    need "$FOLLOWER" "$LEADER"
    echo "Teleop with camera view. Ctrl+C to stop."
    then_rest lerobot-teleoperate \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower \
      --robot.position_p_coefficient=32 --robot.disable_torque_on_disconnect=false --robot.cameras="$CAMERAS" \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id=my_leader \
      --display_data=true
    ;;
  check)
    for p in "$FOLLOWER" "$LEADER"; do port_exists "$p" && echo "OK   $p" || echo "MISSING $p"; done
    need "$FOLLOWER"
    port_exists "$LEADER" && export SO101_LEADER_OK=1
    run python - <<'EOF'
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
import os
arms = [("follower", SO101Follower(SO101FollowerConfig(port=os.environ["SO101_FOLLOWER"], id="my_follower")), 11.0, 13.0)]
if os.environ.get("SO101_LEADER_OK") == "1":
    arms.append(("leader", SO101Leader(SO101LeaderConfig(port=os.environ["SO101_LEADER"], id="my_leader")), 5.5, 8.0))
for name, dev, lo, hi in arms:
    dev.bus.connect(handshake=False)
    dev.bus.calibration = dev.calibration
    volts = [v / 10 for v in dev.bus.sync_read("Present_Voltage", normalize=False).values()]
    ok = all(lo <= v <= hi for v in volts)
    print(f"{name:9} voltage {min(volts):.1f}-{max(volts):.1f}V {'OK' if ok else 'WRONG ADAPTER?'}"
          f"   calibrated: {'yes' if dev.bus.is_calibrated else 'NO'}")
    dev.bus.disconnect(disable_torque=False)
EOF
    ;;
  install-calibration)
    mkdir -p ~/.cache/huggingface/lerobot/calibration
    cp -R "$HERE/calibration/." ~/.cache/huggingface/lerobot/calibration/
    echo "Copied leader/follower calibration to ~/.cache/huggingface/lerobot/calibration"
    ;;
  ports)
    echo "Serial ports now (serial number ends in 177611 = follower, 160051 = leader):"
    run python -m serial.tools.list_ports -v
    echo "Put them in $HERE/config.local.env (overrides config.env on this machine)."
    ;;
  cameras)
    rm -f "$LEROBOT"/outputs/captured_images/*.png
    run lerobot-find-cameras opencv 2>&1 | grep -E "Found|connected\.$" || true
    echo "Snapshots: $LEROBOT/outputs/captured_images"
    if command -v open >/dev/null; then open "$LEROBOT/outputs/captured_images"; else explorer.exe "$(cygpath -w "$LEROBOT/outputs/captured_images")" || true; fi
    ;;
  vision)
    need "$FOLLOWER"
    shift
    then_rest python "$TOOLS/vision_point.py" "${@:-1}"
    ;;
  log)
    need "$FOLLOWER" "$LEADER"
    then_rest python "$TOOLS/teleop_log.py" "${2:-32}" "${3:-0}"
    ;;
  calib)
    [ $# -eq 3 ] || { echo "usage: ./so101.sh calib <leader|follower> <joint 1-6>"; exit 1; }
    [ "$2" = leader ] && need "$LEADER" || need "$FOLLOWER"
    run python "$TOOLS/calib_one.py" "$2" "$3"
    ;;
  one-joint)
    [ $# -eq 2 ] || { echo "usage: ./so101.sh one-joint <joint 1-6>"; exit 1; }
    need "$FOLLOWER" "$LEADER"
    then_rest python "$TOOLS/one_joint.py" "$2"
    ;;
  record)
    [ $# -ge 3 ] || { echo 'usage: ./so101.sh record <hf-user/dataset-name> "<task sentence>" [episodes=20]'; exit 1; }
    need "$FOLLOWER" "$LEADER"
    echo "Recording ${4:-20} episodes of: $3"
    echo "Keys: -> next episode, <- redo episode, ESC stop (or n / r / q in this terminal)."
    then_rest lerobot-record \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower \
      --robot.position_p_coefficient=32 --robot.disable_torque_on_disconnect=false --robot.cameras="$CAMERAS" \
      --teleop.type=so101_leader --teleop.port="$LEADER" --teleop.id=my_leader \
      --display_data=true \
      --dataset.repo_id="$2" --dataset.single_task="$3" --dataset.num_episodes="${4:-20}" \
      --dataset.episode_time_s=20 --dataset.reset_time_s=10 \
      --dataset.streaming_encoding=true --dataset.encoder_threads=2
    ;;
  remote)
    [ $# -ge 4 ] || { echo 'usage: ./so101.sh remote <server-host:port> <policy_type> <hf-model-id> "<task sentence>"'; exit 1; }
    need "$FOLLOWER"
    echo "Robot runs model $3 ($2) served at $1. Ctrl+C to stop."
    then_rest python -m lerobot.async_inference.robot_client \
      --server_address="$1" \
      --robot.type=so101_follower --robot.port="$FOLLOWER" --robot.id=my_follower \
      --robot.position_p_coefficient=32 --robot.disable_torque_on_disconnect=false --robot.cameras="$CAMERAS" \
      --task="${5:-}" --policy_type="$2" --pretrained_name_or_path="$3" \
      --policy_device=cuda --actions_per_chunk=16 --chunk_size_threshold=0.5
    ;;
  cam)
    echo "Live cameras in Rerun, Hz shown below and in the viewer. Ctrl+C to stop."
    run python "$TOOLS/cam_view.py" "${@:2}"
    ;;
  save-rest)
    need "$FOLLOWER"
    run python "$TOOLS/rest.py" save
    ;;
  rest)
    need "$FOLLOWER"
    run python "$TOOLS/rest.py"
    ;;
  relax)
    need "$FOLLOWER"
    run python - <<'EOF'
import os
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
bus = FeetechMotorsBus(os.environ["SO101_FOLLOWER"], {f"m{i}": Motor(i, "sts3215", MotorNormMode.RANGE_M100_100) for i in range(1, 7)})
bus.connect(handshake=False)
for m in bus.motors:
    for _ in range(3):
        try:
            bus.write("Torque_Enable", m, 0, normalize=False)
            break
        except RuntimeError:
            pass
bus.disconnect(disable_torque=False)
print("Follower torque off. Support the arm, it is limp now.")
EOF
    ;;
  *)
    cat <<'EOF'
SO-101 commands (run from anywhere: ~/so101/so101.sh <command>)

  teleop [P]           leader drives follower (P = stiffness, default 32)
  teleop-cam           teleop + live camera view
  install-calibration  copy the saved calibration files into place (new machine)
  ports                list serial ports (to fill in config.env on a new machine)
  check                are both boards plugged in, right voltage, calibrated?
  record <repo> "<task>" [n]  record n demos (default 20), uploads to Hugging Face
  remote <host:port> <type> <model> "<task>"   run a model served on DGX Spark
  cam                  live view of both cameras with real Hz
  cameras              list cameras and save a snapshot from each
  vision [cam] [--reuse]  base turns to point at a colorful object (top camera = 1)
  log [P] [cap]        teleop and log leader/follower gap + load to CSV
  calib <arm> <joint>  recalibrate one joint, e.g. calib follower 2
  one-joint <joint>    teleop a single joint, e.g. one-joint 2
  save-rest            put the follower in its resting pose by hand, then save it
  rest                 move the follower smoothly to the rest pose, then relax
  relax                turn follower torque off (arm goes limp)

teleop, vision, log and one-joint return the follower to the rest pose when you stop them.
EOF
    ;;
esac
