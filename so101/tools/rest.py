# usage: uv run python rest.py save     save the current follower pose (move it by hand first)
#        uv run python rest.py          move the follower smoothly to the rest pose, then relax it
import os
import json
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

REST = Path(__file__).with_name("rest.json")
MOVE_S = 2.5
STEPS = 60

robot = SO101Follower(
    SO101FollowerConfig(port=os.environ["SO101_FOLLOWER"], id="my_follower", position_p_coefficient=32)
)

if sys.argv[1:] == ["save"]:
    robot.bus.connect(handshake=False)
    robot.bus.calibration = robot.calibration
    for m in robot.bus.motors:
        for _ in range(3):
            try:
                robot.bus.write("Torque_Enable", m, 0, normalize=False)
                break
            except RuntimeError:
                pass
    input("Move the follower BY HAND to its resting pose, then press Enter. ")
    pose = {f"{m}.pos": v for m, v in robot.bus.sync_read("Present_Position").items()}
    REST.write_text(json.dumps(pose, indent=2))
    robot.bus.disconnect(disable_torque=False)
    print("Saved rest pose:", {k: round(v, 1) for k, v in pose.items()})
    sys.exit()

if not REST.exists():
    print("No rest pose saved yet. Run: ~/so101/so101.sh save-rest")
    sys.exit(1)

rest = json.loads(REST.read_text())
robot.connect(calibrate=False)
start = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}
for i in range(1, STEPS + 1):
    a = i / STEPS
    robot.send_action({k: start[k] + (rest[k] - start[k]) * a for k in rest})
    time.sleep(MOVE_S / STEPS)
time.sleep(0.8)
robot.disconnect()
print("Follower at rest pose, torque off.")
