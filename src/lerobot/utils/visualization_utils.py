# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Backend-agnostic visualization dispatch.

Selects a visualization backend at runtime via a display-mode string (e.g. a ``--display_mode`` CLI
flag) so callers never branch on the backend. The concrete implementations live in
:mod:`lerobot.utils.rerun_visualization` and :mod:`lerobot.utils.foxglove_visualization`; importing
this module does not import ``rerun`` or ``foxglove`` (each backend imports its SDK lazily behind a
``require_package`` guard).

Logging runs on a background thread so a slow viewer never stalls the robot control loop: only the
newest pending observation/action is kept, and older ones that were not logged yet are dropped.
"""

import logging
import threading
from collections.abc import Callable

from lerobot.lerobot_types import RobotAction, RobotObservation

from .foxglove_visualization import init_foxglove, log_foxglove_data, shutdown_foxglove
from .rerun_visualization import init_rerun, log_rerun_data, shutdown_rerun

# Visualization backends selectable at runtime via a display-mode string (e.g. a --display_mode flag).
VISUALIZATION_MODES = ("rerun", "foxglove")


class _LatestOnlyWorker:
    """Runs submitted calls on a daemon thread, keeping only the newest call that has not started yet."""

    def __init__(self) -> None:
        self._pending: Callable[[], None] | None = None
        self._stopped = False
        self._cond = threading.Condition()
        self._thread = threading.Thread(target=self._run, name="visualization-logger", daemon=True)
        self._thread.start()

    def submit(self, fn: Callable[[], None]) -> None:
        with self._cond:
            self._pending = fn
            self._cond.notify()

    def close(self, timeout: float = 2.0) -> None:
        """Stops the thread after it runs the last pending call."""
        with self._cond:
            self._stopped = True
            self._cond.notify()
        self._thread.join(timeout)

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._pending is None and not self._stopped:
                    self._cond.wait()
                if self._pending is None:
                    return
                fn, self._pending = self._pending, None
            try:
                fn()
            except Exception:
                logging.exception("Visualization logging failed")


_worker: _LatestOnlyWorker | None = None


def init_visualization(
    display_mode: str,
    *,
    session_name: str = "lerobot_control_loop",
    ip: str | None = None,
    port: int | None = None,
) -> None:
    """Initializes the visualization backend selected by ``display_mode``.

    For ``"rerun"``, ``ip``/``port`` point at an optional remote Rerun server. For ``"foxglove"``,
    ``ip`` is the interface to bind the WebSocket server to (``127.0.0.1`` for local only, ``0.0.0.0``
    for all interfaces) and ``port`` is its port.
    """

    if display_mode == "rerun":
        init_rerun(session_name=session_name, ip=ip, port=port)
    elif display_mode == "foxglove":
        init_foxglove(host=ip or "127.0.0.1", port=port)
    else:
        raise ValueError(f"Unknown display_mode '{display_mode}'. Expected one of {VISUALIZATION_MODES}.")


def log_visualization_data(
    display_mode: str,
    observation: RobotObservation | None = None,
    action: RobotAction | None = None,
    compress_images: bool = False,
) -> None:
    """Logs observation/action data to the backend selected by ``display_mode`` without blocking.

    The data is logged on a background thread. If the backend is still busy with an earlier call, that
    earlier data is replaced by this call's data.
    """

    if display_mode == "rerun":
        log_fn = log_rerun_data
    elif display_mode == "foxglove":
        log_fn = log_foxglove_data
    else:
        raise ValueError(f"Unknown display_mode '{display_mode}'. Expected one of {VISUALIZATION_MODES}.")

    global _worker
    if _worker is None:
        _worker = _LatestOnlyWorker()
    _worker.submit(lambda: log_fn(observation=observation, action=action, compress_images=compress_images))


def shutdown_visualization(display_mode: str) -> None:
    """Shuts down the backend selected by ``display_mode``."""

    if display_mode not in VISUALIZATION_MODES:
        raise ValueError(f"Unknown display_mode '{display_mode}'. Expected one of {VISUALIZATION_MODES}.")

    global _worker
    if _worker is not None:
        _worker.close()
        _worker = None

    if display_mode == "rerun":
        shutdown_rerun()
    elif display_mode == "foxglove":
        shutdown_foxglove()
    else:
        raise ValueError(f"Unknown display_mode '{display_mode}'. Expected one of {VISUALIZATION_MODES}.")
