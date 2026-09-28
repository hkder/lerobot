# Follower base (shoulder_pan) turns to point at the most colorful object seen by the top camera.
# usage: uv run python vision_point.py [camera_index]            teach 4 points, then track
#        uv run python vision_point.py [camera_index] --reuse    reuse the saved teach calibration
import os
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

CAM = int(next((a for a in sys.argv[1:] if a.isdigit()), os.environ["SO101_TOP_CAM"]))
REUSE = "--reuse" in sys.argv
HERE = Path(__file__).parent
CALIB = HERE / "vision_point_calib.json"
TEACH_POINTS = 4
MAX_STEP_DEG = 2.0
LOST_AFTER_S = 0.5


def open_camera():
    cap = cv2.VideoCapture(CAM, int(os.environ.get("SO101_CAM_BACKEND") or cv2.CAP_AVFOUNDATION))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if fourcc := os.environ.get("SO101_CAM_FOURCC"):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    for _ in range(40):
        ok, frame = cap.read()
        if ok and frame.mean() > 5:
            return cap
        time.sleep(0.05)
    raise RuntimeError(f"camera {CAM} gives no image")


def desk_outline(hsv):
    dark = (hsv[..., 2] < 70).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return max(contours, key=cv2.contourArea) if contours else None


def on_desk(contour, desk):
    m = cv2.moments(contour)
    center = (m["m10"] / m["m00"], m["m01"] / m["m00"])
    return desk is not None and cv2.pointPolygonTest(desk, center, False) > 0


def find_object(frame):
    small = cv2.resize(frame, (640, 360))
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    mask = ((hsv[..., 1] > 120) & (hsv[..., 2] > 70)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    desk = desk_outline(hsv)
    contours = [c for c in contours if cv2.contourArea(c) > 0 and on_desk(c, desk)]
    if not contours:
        return None
    blob = max(contours, key=cv2.contourArea)
    if cv2.contourArea(blob) < 300:
        return None
    m = cv2.moments(blob)
    return 2 * m["m10"] / m["m00"], 2 * m["m01"] / m["m00"]


def latest_object(cap):
    for _ in range(3):
        cap.grab()
    ok, frame = cap.read()
    return frame, (find_object(frame) if ok else None)


def fit(points):
    """Find base pixel (bx, by), sign s and offset b so that pan ≈ s * atan2(y - by, x - bx) + b."""
    pts = np.array(points, dtype=float)
    xs, ys, pans = pts[:, 0], pts[:, 1], pts[:, 2]
    bx, by = np.meshgrid(np.arange(-640, 1921, 10), np.arange(-720, 1441, 10))
    theta = np.degrees(np.arctan2(ys[:, None, None] - by, xs[:, None, None] - bx))
    best = None
    for s in (1.0, -1.0):
        resid = pans[:, None, None] - s * theta
        b = np.degrees(np.arctan2(np.sin(np.radians(resid)).mean(0), np.cos(np.radians(resid)).mean(0)))
        err = (resid - b + 180) % 360 - 180
        rms = np.sqrt((err**2).mean(0))
        i = np.unravel_index(rms.argmin(), rms.shape)
        if best is None or rms[i] < best["rms"]:
            best = {"bx": float(bx[i]), "by": float(by[i]), "s": s, "b": float(b[i]), "rms": float(rms[i])}
    return best


def pan_for(c, xy):
    theta = np.degrees(np.arctan2(xy[1] - c["by"], xy[0] - c["bx"]))
    return (c["s"] * theta + c["b"] + 180) % 360 - 180


robot = SO101Follower(
    SO101FollowerConfig(
        port=os.environ["SO101_FOLLOWER"],
        id="my_follower",
        position_p_coefficient=32,
        disable_torque_on_disconnect=False,
    )
)
cap = open_camera()
robot.connect(calibrate=False)
hold = {k: v for k, v in robot.get_observation().items() if k.endswith(".pos")}

try:
    if REUSE:
        calib = json.loads(CALIB.read_text())
    else:
        robot.bus.disable_torque("shoulder_pan")
        points = []
        print("TEACH: put the colorful object somewhere, turn the follower base BY HAND to point at it,")
        print(f"then press Enter. Repeat in {TEACH_POINTS} spread-out spots (left, right, near, far).")
        while len(points) < TEACH_POINTS:
            input(f"  point {len(points) + 1}/{TEACH_POINTS}: aim the base, then Enter ")
            frame, xy = latest_object(cap)
            if xy is None:
                print("  no colorful object seen, try again")
                continue
            pan = robot.get_observation()["shoulder_pan.pos"]
            points.append((xy[0], xy[1], pan))
            cv2.circle(frame, (int(xy[0]), int(xy[1])), 12, (0, 255, 0), 3)
            cv2.imwrite(str(HERE / f"vision_teach_{len(points)}.jpg"), frame)
            print(f"  object at pixel ({xy[0]:.0f}, {xy[1]:.0f}), base pan {pan:.1f} deg")
        calib = fit(points)
        CALIB.write_text(json.dumps(calib, indent=2))
        print(f"fit: base pixel ({calib['bx']:.0f}, {calib['by']:.0f}), sign {calib['s']:+.0f}, "
              f"error {calib['rms']:.1f} deg")
        if calib["rms"] > 8:
            print("  error is large: the pointing or detection was inconsistent. Consider re-teaching.")
        hold["shoulder_pan.pos"] = robot.get_observation()["shoulder_pan.pos"]
        robot.bus.enable_torque("shoulder_pan")

    input("TRACK: hands clear of the follower, then Enter. Move the object around. Ctrl+C to stop. ")
    pan, last_seen = hold["shoulder_pan.pos"], time.time()
    while True:
        _, xy = latest_object(cap)
        if xy is not None:
            last_seen = time.time()
            target = pan_for(calib, xy)
            pan += float(np.clip(target - pan, -MAX_STEP_DEG, MAX_STEP_DEG))
            robot.send_action({**hold, "shoulder_pan.pos": pan})
            print(f"\robject ({xy[0]:4.0f},{xy[1]:4.0f})  target {target:6.1f}  pan {pan:6.1f}   ", end="")
        elif time.time() - last_seen > LOST_AFTER_S:
            print("\rno object - holding position                          ", end="")
except KeyboardInterrupt:
    pass
finally:
    robot.disconnect()
    cap.release()
    print()
