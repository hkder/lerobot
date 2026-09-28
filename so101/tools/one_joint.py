# Teleop ONE joint only: follower joint N follows leader joint N, other follower joints hold still.
# usage: uv run python one_joint.py 2      (1=shoulder_pan ... 6=gripper)
import os
import sys
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
joint = JOINTS[int(sys.argv[1]) - 1]
key = f"{joint}.pos"

f = SO101Follower(SO101FollowerConfig(port=os.environ["SO101_FOLLOWER"], id="my_follower"))
l = SO101Leader(SO101LeaderConfig(port=os.environ["SO101_LEADER"], id="my_leader"))
f.connect(calibrate=False)
l.connect(calibrate=False)

hold = {k: v for k, v in f.get_observation().items() if k.endswith(".pos")}
print(f"Only {joint} (L{sys.argv[1]} -> F{sys.argv[1]}) moves. Ctrl+C to stop.")
try:
    while True:
        lead = l.get_action()[key]
        f.send_action({**hold, key: lead})
        fol = f.get_observation()[key]
        print(f"\r{joint}: leader={lead:7.1f}  follower={fol:7.1f}  diff={lead - fol:6.1f}   ", end="", flush=True)
        time.sleep(1 / 30)
except KeyboardInterrupt:
    pass
finally:
    f.bus.disconnect(disable_torque=False)
    l.disconnect()
