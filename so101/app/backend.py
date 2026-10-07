"""Robot side of the SO-101 operator app.

Every mode (idle monitor, teleop, record, run model, park, ...) is a Job running on its own thread and
owning the serial ports and cameras while it runs. Jobs publish the newest frames and joint values
into a Telemetry object; the window reads it on a timer, so a slow screen never slows the robot.
"""

from __future__ import annotations

import contextlib
import json
import logging
import socket
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SO101_DIR = Path(__file__).resolve().parents[1]
REST_FILE = SO101_DIR / "tools" / "rest.json"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CAMERAS = ("wrist", "top")
FOLLOWER_ID, LEADER_ID = "my_follower", "my_leader"
STIFFNESS_P = 32
POSE_OK = 10.0  # degrees: closer than this, teleop can start without moving the follower first
MATCH_SPEED = 60.0  # degrees per second when the follower glides onto the leader's pose

log = logging.getLogger("so101")


def load_settings() -> dict[str, str]:
    """config.env, then config.local.env on top (same files so101.sh reads)."""
    settings: dict[str, str] = {}
    for name in ("config.env", "config.local.env"):
        path = SO101_DIR / name
        if path.exists():
            for line in path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key, value = line.split("=", 1)
                    settings[key.strip()] = value.strip()
    return settings


# ---------------------------------------------------------------- configs


def camera_configs(s: dict[str, str]) -> dict:
    from lerobot.cameras.opencv import OpenCVCameraConfig

    def cam(index: str) -> OpenCVCameraConfig:
        return OpenCVCameraConfig(
            index_or_path=int(index),
            width=1280,
            height=720,
            fps=30,
            fourcc=s.get("SO101_CAM_FOURCC") or None,
            backend=int(s.get("SO101_CAM_BACKEND") or 0),
        )

    return {"wrist": cam(s["SO101_WRIST_CAM"]), "top": cam(s["SO101_TOP_CAM"])}


def follower_config(s: dict[str, str], cameras: bool, keep_torque: bool = True):
    from lerobot.robots.so_follower import SO101FollowerConfig

    return SO101FollowerConfig(
        port=s["SO101_FOLLOWER"],
        id=FOLLOWER_ID,
        position_p_coefficient=STIFFNESS_P,
        disable_torque_on_disconnect=not keep_torque,
        cameras=camera_configs(s) if cameras else {},
    )


def leader_config(s: dict[str, str]):
    from lerobot.teleoperators.so_leader import SO101LeaderConfig

    return SO101LeaderConfig(port=s["SO101_LEADER"], id=LEADER_ID)


def open_bus(device) -> None:
    """Open a device's servo bus for reading only: no handshake, no torque or config changes."""
    device.bus.connect(handshake=False)
    device.bus.calibration = device.calibration


def joint_ranges(device) -> dict[str, tuple[float, float]]:
    """Calibrated range of each joint in the units the app shows (degrees, gripper 0-100)."""
    ranges = {}
    for name, cal in device.calibration.items():
        if name == "gripper":
            ranges[name] = (0.0, 100.0)
        else:
            half = (cal.range_max - cal.range_min) / 2 * 360 / 4095
            ranges[name] = (-half, half)
    return ranges


def positions(values: dict) -> dict[str, float]:
    """{'shoulder_pan.pos': x, ...} -> {'shoulder_pan': x, ...}"""
    return {k[:-4]: float(v) for k, v in values.items() if k.endswith(".pos")}


# ---------------------------------------------------------------- telemetry


@dataclass
class Telemetry:
    """Newest values from whichever job is running. Written by jobs, read by the window."""

    lock: threading.Lock = field(default_factory=threading.Lock)
    frames: dict[str, np.ndarray] = field(default_factory=dict)
    frame_times: dict[str, float] = field(default_factory=dict)
    follower: dict[str, float] = field(default_factory=dict)
    target: dict[str, float] = field(default_factory=dict)  # leader pose, or the model's command
    temps: dict[str, float] = field(default_factory=dict)
    loads: dict[str, float] = field(default_factory=dict)
    ranges: dict[str, tuple[float, float]] = field(default_factory=dict)
    volts: dict[str, float | None] = field(default_factory=lambda: {"follower": None, "leader": None})
    arms_checked: bool = False  # the monitor has tried both ports at least once
    loop_hz: float = 0.0
    phase: str = ""
    phase_start: float = 0.0
    phase_len: float = 0.0
    episode: int = 0
    episodes_saved: int = 0
    dataset: str = ""

    def put(self, **values) -> None:
        with self.lock:
            for k, v in values.items():
                setattr(self, k, v)

    def put_frames(self, obs: dict) -> None:
        now = time.monotonic()
        with self.lock:
            for name in CAMERAS:
                frame = obs.get(name)
                if isinstance(frame, np.ndarray):
                    self.frames[name] = frame
                    self.frame_times[name] = now

    def drop_frame(self, name: str) -> None:
        with self.lock:
            self.frames.pop(name, None)
            self.frame_times.pop(name, None)

    def set_phase(self, phase: str, length: float = 0.0) -> None:
        self.put(phase=phase, phase_start=time.monotonic(), phase_len=length)

    def clear_live(self) -> None:
        with self.lock:
            self.target = {}
            self.loop_hz = 0.0
            self.phase = ""


class RateMeter:
    """Calls per second, from a moving average of the time between calls."""

    def __init__(self) -> None:
        self.last = time.perf_counter()
        self.dt = 0.0

    @property
    def hz(self) -> float:
        return 1 / self.dt if self.dt > 0 else 0.0

    def tick(self) -> float:
        now = time.perf_counter()
        dt, self.last = now - self.last, now
        self.dt = dt if self.dt == 0 else 0.9 * self.dt + 0.1 * dt
        return self.hz


# ---------------------------------------------------------------- cameras


class FastCamera:
    """OpenCV camera opened with every setting in the open call, read on its own thread (RGB frames).

    With DirectShow each later property change rebuilds the capture graph (~1.3 s each), so LeRobot's
    OpenCVCamera takes ~5 s to open; passing the settings to VideoCapture() gets a first frame in ~1 s.
    """

    def __init__(self, config) -> None:
        self.index, self.backend = config.index_or_path, int(config.backend)
        self.width, self.height, self.fps, self.fourcc = (
            config.width,
            config.height,
            config.fps,
            config.fourcc,
        )
        self.cap = None
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.new_frame = threading.Event()
        self.latest: np.ndarray | None = None
        self.latest_time = 0.0
        self.running = False

    @property
    def is_connected(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def connect(self) -> None:
        import cv2

        params = [cv2.CAP_PROP_FRAME_WIDTH, self.width, cv2.CAP_PROP_FRAME_HEIGHT, self.height]
        params += [cv2.CAP_PROP_FPS, self.fps]
        if self.fourcc:
            params += [cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc)]
        self.cap = cv2.VideoCapture(self.index, self.backend, params)
        if not self.cap.isOpened():
            self.cap = None
            raise ConnectionError(f"Could not open camera {self.index}")
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True, name=f"camera-{self.index}")
        self.thread.start()
        self.async_read(timeout_ms=3000)

    def _read_loop(self) -> None:
        import cv2

        while self.running:
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height))
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            with self.lock:
                self.latest, self.latest_time = rgb, time.perf_counter()
            self.new_frame.set()

    def async_read(self, timeout_ms: float = 200) -> np.ndarray:
        if not self.new_frame.wait(timeout_ms / 1000):
            raise TimeoutError(f"Camera {self.index}: no frame for {timeout_ms:.0f} ms")
        self.new_frame.clear()
        with self.lock:
            return self.latest

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        with self.lock:
            frame, stamp = self.latest, self.latest_time
        if frame is None:
            raise RuntimeError(f"Camera {self.index} has no frames yet")
        if (time.perf_counter() - stamp) * 1000 > max_age_ms:
            raise TimeoutError(f"Camera {self.index}: latest frame is older than {max_age_ms} ms")
        return frame

    def disconnect(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.cap is not None:
            self.cap.release()
            self.cap = None


def open_camera(s: dict[str, str], name: str):
    """A camera object for `name`, not yet connected.

    DirectShow (Windows) uses FastCamera, which needs only OpenCV: importing LeRobot's camera classes pulls in
    PyTorch (~4 s), which would hold the cameras back at app start.
    """
    if int(s.get("SO101_CAM_BACKEND") or 0) == 700:
        index = s["SO101_WRIST_CAM" if name == "wrist" else "SO101_TOP_CAM"]
        config = SimpleNamespace(
            index_or_path=int(index),
            width=1280,
            height=720,
            fps=30,
            fourcc=s.get("SO101_CAM_FOURCC"),
            backend=700,
        )
        return FastCamera(config)
    from lerobot.cameras.opencv import OpenCVCamera

    return OpenCVCamera(camera_configs(s)[name])


class CameraHub:
    """Keeps both cameras open and streaming for the whole session.

    Opening a camera takes ~5 s on Windows, so opening them per mode made every start slow. The hub opens
    them once, in parallel, publishes frames for the screen, and lends them to robots as SharedCamera.
    """

    def __init__(self, settings: dict[str, str], tel: Telemetry, on: dict[str, bool] | None = None) -> None:
        self.s, self.tel = settings, tel
        self.cams: dict = {}
        self.enabled = {name: threading.Event() for name in CAMERAS}
        for name in CAMERAS:
            if (on or {}).get(name, True):
                self.enabled[name].set()
        self.status = {n: "opening" if self.enabled[n].is_set() else "off" for n in CAMERAS}
        self.stopping = threading.Event()
        self.threads = [threading.Thread(target=self.run, args=(n,), daemon=True) for n in CAMERAS]

    def start(self) -> None:
        for t in self.threads:
            t.start()

    def set_on(self, name: str, on: bool) -> None:
        """Turn a camera on or off. Off closes the device; turning it back on reopens it (~5 s)."""
        if on:
            self.status[name] = "opening"
            self.enabled[name].set()
        else:
            self.enabled[name].clear()

    def run(self, name: str) -> None:
        while not self.stopping.is_set():
            if not self.enabled[name].is_set():
                if self.status[name] != "off":
                    self.status[name] = "off"
                    self.tel.drop_frame(name)
                    log.info("%s camera off", name.capitalize())
                self.stopping.wait(0.1)
                continue
            cam = open_camera(self.s, name)
            failed = False
            try:
                self.status[name] = "opening"
                opened = time.perf_counter()
                cam.connect()
                self.cams[name] = cam
                self.status[name] = "live"
                log.info("%s camera live (opened in %.1f s)", name.capitalize(), time.perf_counter() - opened)
                while not self.stopping.is_set() and self.enabled[name].is_set():
                    self.tel.put_frames({name: cam.async_read(timeout_ms=1000)})
            except Exception as e:  # noqa: BLE001 - unplugged or in use; retry
                failed = True
                if self.status[name] != "error":
                    log.warning(
                        "%s camera: %s", name.capitalize(), str(e).splitlines()[0] if str(e) else repr(e)
                    )
                self.status[name] = "error"
            finally:
                self.cams.pop(name, None)
                if cam.is_connected:
                    with contextlib.suppress(Exception):
                        cam.disconnect()
            if failed:
                self.stopping.wait(2.0)

    def wait_ready(self, timeout: float = 15.0) -> None:
        end = time.monotonic() + timeout
        while any(self.status[n] == "opening" for n in CAMERAS) and time.monotonic() < end:
            time.sleep(0.05)
        off = [n for n in CAMERAS if not self.enabled[n].is_set()]
        if off:
            raise ConnectionError(f"Camera turned off: {', '.join(off)}. Turn it on first.")
        missing = [n for n in CAMERAS if n not in self.cams]
        if missing:
            raise ConnectionError(f"Camera not available: {', '.join(missing)}. Check the USB cables.")

    def close(self) -> None:
        self.stopping.set()
        for t in self.threads:
            t.join(timeout=3)


class SharedCamera:
    """Stands in for a camera the hub keeps open: a robot reads it but never opens or closes it."""

    use_rgb, use_depth = True, False

    def __init__(self, cam) -> None:
        self.cam, self.width, self.height = cam, cam.width, cam.height

    @property
    def is_connected(self) -> bool:
        return self.cam.is_connected

    def connect(self, warmup: bool = True) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def read_latest(self, max_age_ms: int = 500):
        return self.cam.read_latest(max_age_ms)

    def async_read(self, timeout_ms: float = 200):
        return self.cam.read_latest()  # async_read would steal the hub's new-frame signal

    def read(self, color_mode=None):
        return self.cam.read_latest()


HUB: CameraHub | None = None  # set by the app


def with_shared_cameras(make_robot):
    """Wrap make_robot_from_config so the robot uses the hub's open cameras instead of opening its own."""

    def make(config):
        robot = make_robot(config)
        if robot.cameras:
            HUB.wait_ready()
            robot.cameras = {name: SharedCamera(HUB.cams[name]) for name in robot.cameras}
        return robot

    return make


# ---------------------------------------------------------------- jobs


class Job(threading.Thread):
    """One mode of operation. Owns the arms/cameras for as long as it runs."""

    title = "BUSY"
    parks_follower = False  # move the follower to rest when it ends normally

    def __init__(self, settings: dict[str, str], tel: Telemetry) -> None:
        super().__init__(daemon=True, name=self.__class__.__name__)
        self.s = settings
        self.tel = tel
        self.stop_event = threading.Event()
        self.created = time.monotonic()
        self.estopped = False
        self.error: str | None = None

    def stop(self, estop: bool = False) -> None:
        self.estopped = self.estopped or estop
        self.stop_event.set()

    def run(self) -> None:
        try:
            self.work()
        except Exception as e:  # noqa: BLE001 - show every failure in the window
            self.error = f"{type(e).__name__}: {e}"
            log.exception("%s failed", self.title)
        finally:
            self.tel.clear_live()
        if self.parks_follower and not self.estopped and self.error is None:
            try:
                park(self.s, self.tel)
            except Exception:  # noqa: BLE001
                log.exception("Could not park the follower")

    def work(self) -> None:
        raise NotImplementedError


class Monitor(Job):
    """Idle mode: reads both arms a few times a second without moving anything."""

    title = "IDLE"
    last_errors: dict[str, str] = {}  # shared across restarts so a missing arm is reported once

    def work(self) -> None:
        from lerobot.robots.so_follower import SO101Follower
        from lerobot.teleoperators.so_leader import SO101Leader

        while not self.stop_event.is_set():
            follower = SO101Follower(follower_config(self.s, cameras=False))
            leader = SO101Leader(leader_config(self.s))
            opened = []
            try:
                for dev in (follower, leader):
                    try:
                        open_bus(dev)
                        opened.append(dev)
                        self.last_errors.pop(dev.id, None)
                    except Exception as e:  # noqa: BLE001 - unplugged or busy port
                        msg = f"{type(e).__name__}: {e}".strip().splitlines()[0]
                        if self.last_errors.get(dev.id) != msg:  # say it once, not every retry
                            log.warning("Can't open %s: %s", dev.id, msg)
                            self.last_errors[dev.id] = msg
                if follower in opened:
                    self.tel.put(ranges=joint_ranges(follower))
                while not self.stop_event.is_set():
                    self.read(follower if follower in opened else None, leader if leader in opened else None)
                    if len(opened) < 2:
                        self.stop_event.wait(2.0)
                        break  # reopen to pick up an arm that was plugged in
                    self.stop_event.wait(0.2)
            except Exception as e:  # noqa: BLE001 - arm unplugged while reading
                log.warning("Lost connection to an arm: %s", e)
                self.tel.put(volts={"follower": None, "leader": None})
                self.stop_event.wait(1.0)
            finally:
                for dev in opened:
                    with contextlib.suppress(Exception):
                        dev.bus.disconnect(disable_torque=False)

    def read(self, follower, leader) -> None:
        volts: dict[str, float | None] = {"follower": None, "leader": None}
        values = {}
        if follower is not None:
            values["follower"] = positions(
                {f"{k}.pos": v for k, v in follower.bus.sync_read("Present_Position").items()}
            )
            volts["follower"] = min(follower.bus.sync_read("Present_Voltage", normalize=False).values()) / 10
            values.update(read_health(follower))
        else:
            values["follower"] = {}
        if leader is not None:
            values["target"] = positions(
                {f"{k}.pos": v for k, v in leader.bus.sync_read("Present_Position").items()}
            )
            volts["leader"] = min(leader.bus.sync_read("Present_Voltage", normalize=False).values()) / 10
        else:
            values["target"] = {}
        self.tel.put(volts=volts, arms_checked=True, **values)


class Teleop(Job):
    title = "TELEOP"
    parks_follower = True

    def work(self) -> None:
        from lerobot.robots.so_follower import SO101Follower
        from lerobot.teleoperators.so_leader import SO101Leader
        from lerobot.utils.robot_utils import precise_sleep

        # No cameras on the robot: the CameraHub keeps them streaming, so teleop starts in under a second.
        follower = SO101Follower(follower_config(self.s, cameras=False))
        leader = SO101Leader(leader_config(self.s))
        self.tel.set_phase("Connecting arms")
        leader.connect(calibrate=False)
        try:
            connect_with_retry(follower)
            self.tel.put(ranges=joint_ranges(follower))
            glide_to_leader(follower, leader, self.tel, self.stop_event)
            self.tel.set_phase("Teleop")
            log.info("Teleop live %.1f s after start", time.monotonic() - self.created)
            rate, step = RateMeter(), 0
            while not self.stop_event.is_set():
                start = time.perf_counter()
                action = leader.get_action()
                follower.send_action(action)
                obs = follower.get_observation()
                self.tel.put(follower=positions(obs), target=positions(action), loop_hz=rate.tick())
                step += 1
                if step % 30 == 0:  # twice a second; ~2 ms, well inside the 16.7 ms step
                    self.tel.put(**read_health(follower))
                precise_sleep(max(1 / 60 - (time.perf_counter() - start), 0))
        finally:
            if follower.is_connected:
                follower.disconnect()  # keeps torque on: the arm holds its pose
            if leader.is_connected:
                leader.disconnect()


class Record(Job):
    """Runs lerobot-record in this process, with its keys, display and voice hooks pointed at the app."""

    title = "RECORDING"
    parks_follower = True

    def __init__(self, settings, tel, repo_id, task, episodes, episode_s, reset_s, voice, upload) -> None:
        super().__init__(settings, tel)
        self.repo_id, self.task = repo_id, task
        self.episodes, self.episode_s, self.reset_s = episodes, episode_s, reset_s
        self.voice, self.upload = voice, upload
        self.events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
        self.rate = RateMeter()

    def next_episode(self) -> None:
        self.events["exit_early"] = True

    def redo_episode(self) -> None:
        self.events["rerecord_episode"] = True
        self.events["exit_early"] = True

    def stop(self, estop: bool = False) -> None:
        # Finish: saves the finished episodes and uploads. E-stop also throws away the current one.
        if estop:
            self.events["rerecord_episode"] = True
        self.events["stop_recording"] = True
        self.events["exit_early"] = True
        super().stop(estop)

    def work(self) -> None:
        import lerobot.scripts.lerobot_record as rec
        from lerobot.configs.dataset import DatasetRecordConfig

        cfg = rec.RecordConfig(
            robot=follower_config(self.s, cameras=True),
            teleop=leader_config(self.s),
            dataset=DatasetRecordConfig(
                repo_id=self.repo_id,
                single_task=self.task,
                num_episodes=self.episodes,
                episode_time_s=self.episode_s,
                reset_time_s=self.reset_s,
                streaming_encoding=True,
                encoder_threads=2,
                push_to_hub=self.upload,
            ),
            display_data=True,
            play_sounds=self.voice,
        )
        original_say = rec.log_say
        hooks = {
            "init_logging": lambda *a, **k: None,
            "init_keyboard_listener": lambda: (None, self.events),
            "init_visualization": lambda *a, **k: None,
            "shutdown_visualization": lambda *a, **k: None,
            "log_visualization_data": self.show,
            "log_say": lambda text, play_sounds=True, blocking=False: self.say(original_say, text, blocking),
        }
        hooks["make_robot_from_config"] = with_shared_cameras(rec.make_robot_from_config)
        saved = {name: getattr(rec, name) for name in hooks}
        for name, fn in hooks.items():
            setattr(rec, name, fn)
        self.tel.set_phase("Connecting arms")
        match_pose(self.s, self.tel, self.stop_event)
        try:
            try:
                rec.record(cfg)
            except ConnectionError as e:
                # A servo sometimes misses one packet while the arm is being set up. Retry once if that
                # happened before any episode started; later failures are real and are reported.
                if self.tel.episode or self.stop_event.is_set():
                    raise
                log.warning("Retrying after a servo communication error: %s", e)
                cfg.dataset.repo_id = self.repo_id
                rec.record(cfg)
        finally:
            for name, fn in saved.items():
                setattr(rec, name, fn)
        self.tel.put(dataset=cfg.dataset.repo_id)
        log.info("Dataset saved as %s", cfg.dataset.repo_id)

    def show(self, display_mode, observation=None, action=None, compress_images=False) -> None:
        obs = observation or {}
        self.tel.put(follower=positions(obs), target=positions(action or {}), loop_hz=self.rate.tick())

    def say(self, original, text: str, blocking: bool) -> None:
        log.info(text)
        if text.startswith("Recording episode"):
            # lerobot-record numbers episodes by how many the dataset already holds.
            saved = int(text.rsplit(" ", 1)[-1])
            self.tel.put(episode=saved + 1, episodes_saved=saved)
            self.tel.set_phase("Recording", self.episode_s)
        elif text.startswith("Reset"):
            self.tel.set_phase("Reset the scene", self.reset_s)
        elif text.startswith("Stop recording"):
            self.tel.set_phase("Saving and uploading")
        if self.voice:
            original(text, True, blocking)


class RunModel(Job):
    """Robot client for a policy served on the DGX Spark (same as so101.sh remote)."""

    title = "AUTONOMOUS"
    parks_follower = True

    def __init__(self, settings, tel, server, model, task, threshold) -> None:
        super().__init__(settings, tel)
        self.server, self.model, self.task, self.threshold = server, model, task, threshold
        self.client = None

    def stop(self, estop: bool = False) -> None:
        super().stop(estop)
        if self.client is not None:
            self.client.shutdown_event.set()

    def work(self) -> None:
        import lerobot.async_inference.robot_client as rc
        from lerobot.async_inference.configs import RobotClientConfig

        cfg = RobotClientConfig(
            policy_type="groot",
            pretrained_name_or_path=self.model,
            robot=follower_config(self.s, cameras=True),
            actions_per_chunk=16,
            task=self.task,
            server_address=self.server,
            policy_device="cuda",
            chunk_size_threshold=self.threshold,
        )
        self.tel.set_phase("Connecting to the robot and the model server")
        make_robot = rc.make_robot_from_config
        rc.make_robot_from_config = with_shared_cameras(make_robot)
        try:
            client = rc.RobotClient(cfg)
        finally:
            rc.make_robot_from_config = make_robot
        robot, rate = client.robot, RateMeter()
        get_obs, send = robot.get_observation, robot.send_action

        def observe():
            obs = get_obs()
            self.tel.put(follower=positions(obs))
            return obs

        def act(action):
            self.tel.put(target=positions(action), loop_hz=rate.tick())
            return send(action)

        robot.get_observation, robot.send_action = observe, act
        self.client = client
        receiver = None
        try:
            if self.stop_event.is_set() or not client.start():
                raise ConnectionError(f"Could not reach the model server at {self.server}")
            self.tel.set_phase("Loading the model on the server (first time can take minutes)")
            receiver = threading.Thread(target=client.receive_actions, daemon=True)
            receiver.start()
            threading.Thread(target=self.watch_first_action, daemon=True).start()
            client.control_loop(task=self.task)
        finally:
            client.stop()  # also disconnects the robot (torque stays on)
            if receiver is not None:
                receiver.join(timeout=5)

    def watch_first_action(self) -> None:
        while not self.stop_event.is_set():
            if self.tel.loop_hz > 0:
                self.tel.set_phase("Running the model")
                return
            time.sleep(0.2)


class Park(Job):
    title = "PARKING"

    def work(self) -> None:
        park(self.s, self.tel)


class Relax(Job):
    title = "RELAX"

    def work(self) -> None:
        from lerobot.robots.so_follower import SO101Follower

        follower = SO101Follower(follower_config(self.s, cameras=False))
        open_bus(follower)
        try:
            torque_off(follower)
        finally:
            follower.bus.disconnect(disable_torque=False)
        log.info("Follower torque off. It is limp now.")


class SaveRest(Job):
    title = "SAVE REST"

    def work(self) -> None:
        from lerobot.robots.so_follower import SO101Follower

        follower = SO101Follower(follower_config(self.s, cameras=False))
        open_bus(follower)
        try:
            pose = {f"{k}.pos": v for k, v in follower.bus.sync_read("Present_Position").items()}
        finally:
            follower.bus.disconnect(disable_torque=False)
        REST_FILE.write_text(json.dumps(pose, indent=2))
        log.info("Saved rest pose: %s", {k: round(v, 1) for k, v in pose.items()})


def connect_with_retry(robot, attempts: int = 2) -> None:
    """robot.connect(), retried once: a servo occasionally misses a single packet during setup."""
    for attempt in range(attempts):
        try:
            robot.connect(calibrate=False)
            return
        except ConnectionError as e:
            # connect() may have stopped halfway, so close the pieces one by one.
            for cam in robot.cameras.values():
                if cam.is_connected:
                    cam.disconnect()
            if robot.bus.is_connected:
                with contextlib.suppress(Exception):
                    robot.bus.disconnect(disable_torque=False)
            if attempt == attempts - 1:
                raise
            log.warning("Retrying after a servo communication error: %s", e)
            time.sleep(0.5)


def glide_to_leader(follower, leader, tel: Telemetry, stop_event: threading.Event) -> None:
    """Move the follower smoothly onto the leader's pose, so teleop starts without a jump."""
    start = {k: v for k, v in follower.get_observation().items() if k.endswith(".pos")}
    target = leader.get_action()
    gap = max(abs(target[k] - start[k]) for k in start if not k.startswith("gripper"))
    if gap < POSE_OK:
        return
    duration = min(max(gap / MATCH_SPEED, 0.6), 4.0)
    log.info("Matching the leader's pose: %.0f° apart, %.1f s", gap, duration)
    tel.set_phase("Matching the leader's pose", duration)
    t0 = time.perf_counter()
    while not stop_event.is_set():
        a = min((time.perf_counter() - t0) / duration, 1.0)
        eased = a * a * (3 - 2 * a)  # smoothstep: gentle start and stop
        target = leader.get_action()  # keeps tracking the leader if it moves meanwhile
        goal = {k: start[k] + (target[k] - start[k]) * eased for k in start}
        follower.send_action(goal)
        tel.put(follower=positions(goal), target=positions(target))
        if a >= 1.0:
            return
        time.sleep(1 / 60)


def match_pose(settings: dict[str, str], tel: Telemetry, stop_event: threading.Event) -> None:
    """Glide the follower onto the leader with short-lived connections (before lerobot-record connects)."""
    from lerobot.robots.so_follower import SO101Follower
    from lerobot.teleoperators.so_leader import SO101Leader

    follower = SO101Follower(follower_config(settings, cameras=False))
    leader = SO101Leader(leader_config(settings))
    leader.connect(calibrate=False)
    try:
        connect_with_retry(follower)
        try:
            glide_to_leader(follower, leader, tel, stop_event)
        finally:
            follower.disconnect()  # torque stays on: it holds the matched pose
    finally:
        leader.disconnect()


def read_health(follower) -> dict:
    """Servo temperatures (°C) and loads (% of max torque)."""
    temps = follower.bus.sync_read("Present_Temperature", normalize=False)
    loads = follower.bus.sync_read("Present_Load", normalize=False)  # signed: -1000..1000
    return {
        "temps": {k: float(v) for k, v in temps.items()},
        "loads": {k: abs(int(v)) / 10 for k, v in loads.items()},
    }


def torque_off(follower) -> None:
    for motor in follower.bus.motors:
        for _ in range(3):
            try:
                follower.bus.write("Torque_Enable", motor, 0, normalize=False)
                break
            except RuntimeError:
                pass


LEADER_PARK_P = 16  # half the follower's stiffness: a hand may still be on the leader's handle


def park(settings: dict[str, str], tel: Telemetry, move_s: float = 2.5, steps: int = 60) -> None:
    """Move both arms smoothly to the saved rest pose at the same time, then turn their torque off."""
    if not REST_FILE.exists():
        log.warning("No rest pose saved yet; use Tools > Save current pose as rest.")
        return
    rest = json.loads(REST_FILE.read_text())
    tel.set_phase("Parking both arms", move_s)
    leader_error: list[BaseException] = []

    def leader_side() -> None:
        try:
            park_leader(settings, tel, rest, move_s, steps)
        except Exception as e:  # noqa: BLE001 - a missing leader must not stop the follower parking
            leader_error.append(e)

    leader_thread = threading.Thread(target=leader_side, daemon=True)
    leader_thread.start()
    park_follower(settings, tel, rest, move_s, steps)
    leader_thread.join(timeout=move_s + 10)
    if leader_error:
        log.warning("Leader not parked: %s", leader_error[0])
        log.info("Follower at rest pose, torque off.")
    else:
        log.info("Both arms at rest pose, torque off.")


def eased_path(start: dict[str, float], goal: dict[str, float], steps: int):
    """Poses from start to goal with a gentle start and stop (smoothstep)."""
    for i in range(1, steps + 1):
        a = i / steps
        a = a * a * (3 - 2 * a)
        yield {k: start[k] + (goal[k] - start[k]) * a for k in goal}


def park_follower(settings, tel: Telemetry, rest: dict[str, float], move_s: float, steps: int) -> None:
    from lerobot.robots.so_follower import SO101Follower

    follower = SO101Follower(follower_config(settings, cameras=False))
    connect_with_retry(follower)
    try:
        start = {k: v for k, v in follower.get_observation().items() if k.endswith(".pos")}
        for goal in eased_path(start, rest, steps):
            follower.send_action(goal)
            tel.put(follower=positions(goal))
            time.sleep(move_s / steps)
        time.sleep(0.8)
        torque_off(follower)
    finally:
        follower.disconnect()


def park_leader(settings, tel: Telemetry, rest: dict[str, float], move_s: float, steps: int) -> None:
    """Drive the leader to the rest pose, then leave it limp again (it is normally moved by hand)."""
    from lerobot.motors.feetech import OperatingMode
    from lerobot.teleoperators.so_leader import SO101Leader

    leader = SO101Leader(leader_config(settings))
    open_bus(leader)
    bus = leader.bus
    try:
        start = {f"{m}.pos": v for m, v in bus.sync_read("Present_Position").items()}
        for motor in bus.motors:
            bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            bus.write("P_Coefficient", motor, LEADER_PARK_P)
        bus.sync_write("Goal_Position", {k[:-4]: v for k, v in start.items()})  # no jump at torque on
        bus.enable_torque(num_retry=2)
        for goal in eased_path(start, rest, steps):
            bus.sync_write("Goal_Position", {k[:-4]: v for k, v in goal.items()})
            tel.put(target=positions(goal))
            time.sleep(move_s / steps)
        time.sleep(0.8)
    finally:
        with contextlib.suppress(Exception):
            bus.disable_torque(num_retry=3)
        bus.disconnect(disable_torque=False)


def ping_server(address: str, timeout: float = 2.0) -> float | None:
    """TCP connect time to host:port in ms, or None if it doesn't answer."""
    host, _, port = address.rpartition(":")
    try:
        start = time.perf_counter()
        with socket.create_connection((host, int(port)), timeout=timeout):
            return (time.perf_counter() - start) * 1000
    except (OSError, ValueError):
        return None
