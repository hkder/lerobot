# Teleop + log leader vs follower per joint, with follower load/current. Ctrl+C to stop and print a summary.
# usage: uv run python teleop_log.py [p_coefficient] [max_relative_target, 0 = no cap]
import os
import csv
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

P = int(sys.argv[1]) if len(sys.argv) > 1 else 16
CAP = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
FPS = 30
SETTLE_S = 2.0
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

follower = SO101Follower(
    SO101FollowerConfig(
        port=os.environ["SO101_FOLLOWER"],
        id="my_follower",
        max_relative_target=CAP or None,
        position_p_coefficient=P,
        disable_torque_on_disconnect=False,
    )
)
leader = SO101Leader(SO101LeaderConfig(port=os.environ["SO101_LEADER"], id="my_leader"))
follower.connect(calibrate=False)
leader.connect(calibrate=False)

out = Path(__file__).with_name(f"teleop_log_{time.strftime('%H%M%S')}.csv")
cols = ["t"]
for j in JOINTS:
    cols += [f"{j}.leader", f"{j}.follower", f"{j}.diff", f"{j}.diff_rel", f"{j}.load", f"{j}.current"]

rows, start_diff = [], None
t0 = time.perf_counter()
print(f"Logging at {FPS} Hz with P={P}, cap={CAP or None} -> {out.name}. Ctrl+C to stop.")
print(f"Hold the leader still for {SETTLE_S:.0f} s while the follower catches up.")
try:
    while True:
        tick = time.perf_counter()
        lead = leader.get_action()
        follower.send_action(lead)
        fol = follower.bus.sync_read("Present_Position")
        load = follower.bus.sync_read("Present_Load", normalize=False)
        cur = follower.bus.sync_read("Present_Current", normalize=False)
        diff = {j: lead[f"{j}.pos"] - fol[j] for j in JOINTS}
        if tick - t0 < SETTLE_S:
            time.sleep(max(0.0, 1 / FPS - (time.perf_counter() - tick)))
            continue
        if start_diff is None:
            start_diff = diff
            print("Baseline taken. Start the experiment.")
        row = [round(tick - t0, 3)]
        for j in JOINTS:
            row += [lead[f"{j}.pos"], fol[j], diff[j], diff[j] - start_diff[j], load[j], cur[j]]
        rows.append(row)
        time.sleep(max(0.0, 1 / FPS - (time.perf_counter() - tick)))
except KeyboardInterrupt:
    pass
finally:
    follower.disconnect()
    leader.disconnect()

with out.open("w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(cols)
    w.writerows(rows)

print(f"\n{len(rows)} samples saved to {out}")
print(f"{'joint':14} {'mean|diff|':>10} {'max|diff|':>10} {'max|load|':>10} {'corr(diff,load)':>16}")
for i, j in enumerate(JOINTS):
    d = [r[1 + 6 * i + 3] for r in rows]
    ld = [r[1 + 6 * i + 4] for r in rows]
    n = len(d)
    if n < 2:
        continue
    md, ml = sum(d) / n, sum(ld) / n
    cov = sum((a - md) * (b - ml) for a, b in zip(d, ld, strict=True))
    sd = (sum((a - md) ** 2 for a in d) * sum((b - ml) ** 2 for b in ld)) ** 0.5
    corr = cov / sd if sd else 0.0
    mean_d, max_d, max_l = sum(abs(x) for x in d) / n, max(abs(x) for x in d), max(abs(x) for x in ld)
    print(f"{j:14} {mean_d:10.2f} {max_d:10.2f} {max_l:10d} {corr:16.2f}")
