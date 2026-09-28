# Live view of the wrist + top cameras in Rerun, with the real frame rate (Hz) of each camera.
# usage: uv run python cam_view.py [width height fps]      default 1280 720 30
import os
import signal
import sys
import threading
import time

import rerun as rr

from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.utils.rerun_visualization import init_rerun

W, H, FPS = (int(a) for a in sys.argv[1:4]) if len(sys.argv) >= 4 else (1280, 720, 30)
CAMERAS = {"wrist": (int(os.environ["SO101_WRIST_CAM"]), 0), "top": (int(os.environ["SO101_TOP_CAM"]), 0)}

cams = {
    name: OpenCVCamera(
        OpenCVCameraConfig(
            index_or_path=idx,
            width=H if rot else W,
            height=W if rot else H,
            fps=FPS,
            rotation=rot,
        )
    )
    for name, (idx, rot) in CAMERAS.items()
}
latest, counts, stop = {}, dict.fromkeys(cams, 0), threading.Event()


def grab(name, cam):
    while not stop.is_set():
        try:
            latest[name] = cam.async_read(timeout_ms=1000)
            counts[name] += 1
        except Exception as e:
            print(f"\n{name}: {e}")


for cam in cams.values():
    cam.connect()
init_rerun(session_name="so101_cameras")
threads = [threading.Thread(target=grab, args=item, daemon=True) for item in cams.items()]
for t in threads:
    t.start()

print(f"Streaming {W}x{H} @ {FPS} requested. Ctrl+C to stop.")
last_counts, last_t = dict(counts), time.perf_counter()
try:
    while True:
        time.sleep(1 / 30)
        for name, frame in list(latest.items()):
            rr.log(f"camera/{name}", rr.Image(frame).compress())
        now = time.perf_counter()
        if now - last_t >= 1.0:
            hz = {n: (counts[n] - last_counts[n]) / (now - last_t) for n in cams}
            for n, v in hz.items():
                rr.log(f"hz/{n}", rr.Scalars(v))
            print("\r" + "   ".join(f"{n}: {v:5.1f} Hz" for n, v in hz.items()) + "   ", end="", flush=True)
            last_counts, last_t = dict(counts), now
except KeyboardInterrupt:
    pass
finally:
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    stop.set()
    for cam in cams.values():
        cam.disconnect()
    print()
