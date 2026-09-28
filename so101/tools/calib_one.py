# Recalibrate ONE joint and keep the other joints in the calibration file untouched.
# usage: uv run python calib_one.py leader 2      (arm: leader|follower, joint: 1=shoulder_pan ... 6=gripper)
import os
import sys

from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
arm, joint = sys.argv[1], JOINTS[int(sys.argv[2]) - 1]

if arm == "follower":
    dev = SO101Follower(SO101FollowerConfig(port=os.environ["SO101_FOLLOWER"], id="my_follower"))
else:
    dev = SO101Leader(SO101LeaderConfig(port=os.environ["SO101_LEADER"], id="my_leader"))

bus = dev.bus
bus.connect(handshake=False)
for m in bus.motors:
    try:
        bus.write("Torque_Enable", m, 0, normalize=False)
    except RuntimeError:
        pass

input(f"[{arm}] Put {joint} in the MIDDLE of its range, then press ENTER...")
homing = bus.set_half_turn_homings([joint])[joint]

if joint == "wrist_roll":
    lo, hi = 0, 4095
else:
    print(f"Move ONLY {joint} slowly to BOTH ends. Press ENTER when done.")
    mins, maxes = bus.record_ranges_of_motion([joint])
    lo, hi = mins[joint], maxes[joint]

old = dev.calibration[joint]
dev.calibration[joint] = MotorCalibration(id=old.id, drive_mode=0, homing_offset=homing, range_min=lo, range_max=hi)
bus.write_calibration({joint: dev.calibration[joint]}, cache=False)
dev._save_calibration()
print(f"\n{joint}: {old.range_min}-{old.range_max} -> {lo}-{hi}  (saved {dev.calibration_fpath})")
bus.disconnect(disable_torque=False)
