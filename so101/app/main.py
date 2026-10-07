"""SO-101 operator console: live cameras and joints, teleop, recording, running a model, E-STOP.

Start it with so101/SO101.exe on Windows (built by so101/app/build_exe.cmd) or so101/SO101.command on macOS.
Both install PySide6 into the project venv once, then run: .venv python so101/app/main.py
"""

from __future__ import annotations

import logging
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, QTimer
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QGuiApplication,
    QIcon,
    QImage,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QShortcut,
)
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import backend as be  # noqa: E402

APP_DIR = Path(__file__).resolve().parent
ICON = APP_DIR / "assets" / "so101.ico"
LOG_FILE = APP_DIR.parents[1] / "outputs" / "so101_app.log"

BG, PANEL, PANEL2, BORDER = "#0b0f14", "#111821", "#17202b", "#223041"
TEXT, MUTED = "#dce6f0", "#7b8a9a"
ACCENT, GREEN, AMBER, RED, VIOLET = "#27c4f5", "#2ecc71", "#f5a623", "#ff4040", "#a78bfa"
MODE_COLORS = {
    "IDLE": MUTED,
    "TELEOP": ACCENT,
    "RECORDING": RED,
    "AUTONOMOUS": VIOLET,
    "PARKING": AMBER,
    "CAMERAS": ACCENT,
    "RELAX": AMBER,
    "SAVE REST": AMBER,
    "E-STOP": RED,
}
POSE_OK, POSE_WARN = be.POSE_OK, 25.0  # degrees between leader and follower

STYLE = f"""
QWidget {{
    background: {BG}; color: {TEXT}; font-family: 'Segoe UI', 'Helvetica Neue', sans-serif; font-size: 10pt;
}}
QFrame#panel {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 12px; }}
QLabel, QCheckBox {{ background: transparent; }}
QWidget#page, QScrollArea, QScrollArea > QWidget > QWidget {{ background: {PANEL}; }}
QLabel#muted {{ color: {MUTED}; }}
QLabel#section {{ color: {MUTED}; font-size: 8pt; font-weight: 700; letter-spacing: 1px; }}
QLabel#big {{ font-size: 20pt; font-weight: 800; }}
QLineEdit, QSpinBox, QDoubleSpinBox {{
    background: #0d131a; border: 1px solid {BORDER}; border-radius: 7px; padding: 7px 9px; color: {TEXT};
    selection-background-color: {ACCENT};
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border-color: {ACCENT}; }}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {{ color: {MUTED}; border-color: #19222d; }}
QPushButton {{
    background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 9px; padding: 11px 14px;
    font-weight: 700; color: {TEXT};
}}
QPushButton:hover {{ border-color: {ACCENT}; }}
QPushButton:pressed {{ background: #0f1720; }}
QPushButton:disabled {{ color: #3f4b58; background: #0f151c; border-color: #161e27; }}
QPushButton#primary {{ background: {ACCENT}; color: #03131b; border: none; }}
QPushButton#primary:hover {{ background: #5fd6fb; }}
QPushButton#record {{ background: #d32f2f; color: white; border: none; }}
QPushButton#record:hover {{ background: #ef4444; }}
QPushButton#primary:disabled, QPushButton#record:disabled {{ background: #152029; color: #3f4b58; }}
QPushButton#estop {{
    background: qradialgradient(cx:0.5, cy:0.4, radius:0.7, stop:0 #ff5a5a, stop:1 #b3001b);
    border: 5px solid #f5c400; border-radius: 46px; color: white; font-size: 12pt; font-weight: 900;
    min-width: 92px; max-width: 92px; min-height: 92px; max-height: 92px; padding: 0;
}}
QPushButton#estop:pressed {{ background: #8a0015; }}
QTabWidget::pane {{ border: none; top: -1px; }}
QTabBar::tab {{
    background: transparent; color: {MUTED}; padding: 10px 12px; font-weight: 800; font-size: 9pt;
    border-bottom: 2px solid {BORDER};
}}
QTabBar::tab:selected {{ color: {TEXT}; border-bottom: 2px solid {ACCENT}; }}
QTabBar::tab:hover {{ color: {TEXT}; }}
QProgressBar {{
    background: #0d131a; border: 1px solid {BORDER}; border-radius: 6px; height: 16px; text-align: center;
    color: {TEXT}; font-weight: 700; font-size: 8pt;
}}
QProgressBar::chunk {{ background: {ACCENT}; border-radius: 5px; }}
QPlainTextEdit {{
    background: #070a0e; border: 1px solid {BORDER}; border-radius: 10px; padding: 6px;
    font-family: Consolas, Menlo, monospace; font-size: 9pt; color: #aab7c4;
}}
QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{
    width: 16px; height: 16px; border-radius: 4px; border: 1px solid {BORDER}; background: #0d131a;
}}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}
QToolTip {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {BORDER}; }}
QScrollBar:vertical {{ background: transparent; width: 10px; }}
QScrollBar::handle:vertical {{ background: {BORDER}; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
"""


# ---------------------------------------------------------------- icon


def draw_icon(size: int = 256) -> QImage:
    """App icon: a stylized arm on a dark rounded tile."""
    img = QImage(size, size, QImage.Format.Format_ARGB32)
    img.fill(Qt.GlobalColor.transparent)
    p = QPainter(img)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    s = size / 256
    grad = QLinearGradient(0, 0, size, size)
    grad.setColorAt(0, QColor("#16222e"))
    grad.setColorAt(1, QColor("#070b10"))
    p.setBrush(grad)
    p.setPen(QPen(QColor(ACCENT), 6 * s))
    p.drawRoundedRect(QRectF(8 * s, 8 * s, 240 * s, 240 * s), 52 * s, 52 * s)
    pen = QPen(
        QColor(ACCENT), 22 * s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin
    )
    p.setPen(pen)
    base, shoulder, elbow, wrist = QPointF(80, 196), QPointF(80, 150), QPointF(150, 88), QPointF(196, 128)
    path = QPainterPath(base * s)
    for pt in (shoulder, elbow, wrist):
        path.lineTo(pt * s)
    p.setBrush(Qt.BrushStyle.NoBrush)
    p.drawPath(path)
    p.setPen(QPen(QColor("#e8f6ff"), 12 * s, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
    p.drawLine(QPointF(196, 128) * s, QPointF(214, 158) * s)
    p.drawLine(QPointF(196, 128) * s, QPointF(226, 130) * s)
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QColor("#e8f6ff"))
    for pt in (shoulder, elbow, wrist):
        p.drawEllipse(pt * s, 15 * s, 15 * s)
    p.setBrush(QColor(ACCENT))
    p.drawRoundedRect(QRectF(46 * s, 192 * s, 68 * s, 22 * s), 8 * s, 8 * s)
    p.end()
    return img


# ---------------------------------------------------------------- widgets


def panel() -> QFrame:
    f = QFrame()
    f.setObjectName("panel")
    return f


def label(text: str = "", name: str | None = None) -> QLabel:
    lab = QLabel(text)
    if name:
        lab.setObjectName(name)
    return lab


def button(text: str, name: str | None = None) -> QPushButton:
    b = QPushButton(text)
    b.setCursor(Qt.CursorShape.PointingHandCursor)
    if name:
        b.setObjectName(name)
    return b


class StatusPill(QWidget):
    """Colored light + title + detail, e.g. ● FOLLOWER  COM3 · 12.1 V."""

    def __init__(self, title: str) -> None:
        super().__init__()
        self.title, self.detail, self.color = title, "—", MUTED
        self.setFixedSize(200, 46)

    def set(self, detail: str, color: str) -> None:
        if (detail, color) != (self.detail, self.color):
            self.detail, self.color = detail, color
            self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(QColor(PANEL))
        p.drawRoundedRect(r, 10, 10)
        glow = QColor(self.color)
        glow.setAlpha(60)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(glow)
        p.drawEllipse(QPointF(20, r.center().y()), 9, 9)
        p.setBrush(QColor(self.color))
        p.drawEllipse(QPointF(20, r.center().y()), 5, 5)
        p.setPen(QColor(MUTED))
        f = QFont(self.font())
        f.setPointSizeF(7.5)
        f.setBold(True)
        p.setFont(f)
        p.drawText(QRectF(36, 5, r.width() - 40, 16), Qt.AlignmentFlag.AlignLeft, self.title)
        p.setPen(QColor(TEXT))
        f.setPointSizeF(9.5)
        p.setFont(f)
        p.drawText(QRectF(36, 21, r.width() - 40, 20), Qt.AlignmentFlag.AlignLeft, self.detail)


class ModeBadge(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.mode, self.sub, self.blink = "IDLE", "", False
        self.setFixedSize(230, 46)

    def set(self, mode: str, sub: str, blink: bool) -> None:
        self.mode, self.sub, self.blink = mode, sub, blink
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(MODE_COLORS.get(self.mode, ACCENT))
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        fill = QColor(color)
        fill.setAlpha(40 if not (self.blink and int(time.monotonic() * 2) % 2) else 90)
        p.setBrush(fill)
        p.setPen(QPen(color, 1.5))
        p.drawRoundedRect(r, 10, 10)
        f = QFont(self.font())
        f.setBold(True)
        f.setPointSizeF(13)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.5)
        p.setFont(f)
        p.setPen(color)
        p.drawText(
            QRectF(14, 4, r.width() - 20, 24),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            self.mode,
        )
        f.setPointSizeF(8)
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 0)
        f.setBold(False)
        p.setFont(f)
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(14, 25, r.width() - 20, 16), Qt.AlignmentFlag.AlignLeft, self.sub)


class CameraView(QWidget):
    def __init__(self, title: str) -> None:
        super().__init__()
        self.title = title
        self.pixmap: QPixmap | None = None
        self.source: np.ndarray | None = None
        self.fps_note = ""
        self.live = False
        self.hint = "Opening camera…"
        self.off = False
        self.setMinimumSize(240, 120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.toggle = QPushButton("ON", self)
        self.toggle.setCheckable(True)
        self.toggle.setChecked(True)
        self.toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.toggle.setFixedSize(58, 24)
        self.toggle.setToolTip("Turn this camera on or off")
        self.toggle.setStyleSheet(
            "QPushButton { background: #3a1216; color: #ff9aa2; border: 1px solid #6b1f27;"
            " border-radius: 12px; padding: 0; font-size: 8pt; font-weight: 800; }"
            f"QPushButton:checked {{ background: #0f3b2a; color: {GREEN}; border-color: #1f6b4a; }}"
            f"QPushButton:disabled {{ color: #4b5563; background: #111820; border-color: #1b2430; }}"
        )
        self.toggle.toggled.connect(lambda on: self.toggle.setText("ON" if on else "OFF"))

    def resizeEvent(self, event) -> None:
        self.toggle.move(self.width() - self.toggle.width() - 10, 10)
        super().resizeEvent(event)

    def show_frame(self, frame: np.ndarray | None, live: bool, fps_note: str) -> None:
        self.live, self.fps_note = live, fps_note
        if frame is not None and frame is not self.source:
            self.source = frame
            h, w = frame.shape[:2]
            img = QImage(np.ascontiguousarray(frame).data, w, h, 3 * w, QImage.Format.Format_RGB888)
            self.pixmap = QPixmap.fromImage(
                img.scaled(
                    self.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        clip = QPainterPath()
        clip.addRoundedRect(r, 12, 12)
        p.setClipPath(clip)
        p.fillRect(r, QColor("#05080b"))
        if self.live and self.pixmap is not None:
            pm = self.pixmap
            p.drawPixmap(int(r.center().x() - pm.width() / 2), int(r.center().y() - pm.height() / 2), pm)
        else:
            p.setPen(QColor("#3a4654"))
            f = QFont(self.font())
            f.setPointSizeF(13)
            f.setBold(True)
            p.setFont(f)
            p.drawText(
                r.adjusted(0, -14, 0, -14),
                Qt.AlignmentFlag.AlignCenter,
                "CAMERA OFF" if self.off else "NO SIGNAL",
            )
            f.setPointSizeF(8.5)
            f.setBold(False)
            p.setFont(f)
            p.drawText(r.adjusted(0, 22, 0, 22), Qt.AlignmentFlag.AlignCenter, self.hint)
        p.setClipping(False)
        p.setPen(QPen(QColor(BORDER), 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawRoundedRect(r, 12, 12)
        # title chip
        f = QFont(self.font())
        f.setPointSizeF(8)
        f.setBold(True)
        p.setFont(f)
        chip = QRectF(10, 10, 14 + p.fontMetrics().horizontalAdvance(self.title) + 16, 22)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(0, 0, 0, 150))
        p.drawRoundedRect(chip, 6, 6)
        p.setBrush(QColor(GREEN if self.live else "#4b5563"))
        p.drawEllipse(QPointF(chip.left() + 11, chip.center().y()), 4, 4)
        p.setPen(QColor(TEXT))
        p.drawText(chip.adjusted(20, 0, 0, 0), Qt.AlignmentFlag.AlignVCenter, self.title)
        if self.live and self.fps_note:
            w = p.fontMetrics().horizontalAdvance(self.fps_note) + 16
            box = QRectF(r.right() - w - 78, 11, w, 22)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(0, 0, 0, 150))
            p.drawRoundedRect(box, 6, 6)
            p.setPen(QColor(TEXT))
            p.drawText(box, Qt.AlignmentFlag.AlignCenter, self.fps_note)


class JointPanel(QWidget):
    """Six gauges: follower position (bar), leader/target (white marker), the gap, temperature and load."""

    ROW = 40

    def __init__(self) -> None:
        super().__init__()
        self.follower: dict[str, float] = {}
        self.target: dict[str, float] = {}
        self.ranges: dict[str, tuple[float, float]] = {}
        self.temps: dict[str, float] = {}
        self.loads: dict[str, float] = {}
        self.target_name = "LEADER"
        self.setFixedHeight(self.ROW * 6 + 30)

    def set(self, tel: be.Telemetry, target_name: str) -> None:
        with tel.lock:
            self.follower, self.target = dict(tel.follower), dict(tel.target)
            self.ranges, self.temps, self.loads = dict(tel.ranges), dict(tel.temps), dict(tel.loads)
        self.target_name = target_name
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w = self.width()
        name_w, val_w = 118, 250
        bar_l, bar_r = name_w, w - val_w - 12
        f = QFont(self.font())
        f.setPointSizeF(7.5)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(0, 0, name_w, 20), Qt.AlignmentFlag.AlignVCenter, "JOINT")
        legend_x = bar_l
        for text, color in (("FOLLOWER", ACCENT), (self.target_name, "#ffffff")):
            p.setBrush(QColor(color))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(QRectF(legend_x, 6, 10, 8), 2, 2)
            p.setPen(QColor(MUTED))
            p.drawText(QRectF(legend_x + 14, 0, 100, 20), Qt.AlignmentFlag.AlignVCenter, text)
            legend_x += 30 + p.fontMetrics().horizontalAdvance(text)
        cols = [("POS", 0), (self.target_name[:6], 64), ("GAP", 128), ("TEMP / LOAD", 178)]
        for text, dx in cols:
            p.drawText(QRectF(w - val_w + dx, 0, 80, 20), Qt.AlignmentFlag.AlignVCenter, text)

        mono = QFont("Consolas")
        mono.setPointSizeF(10)
        for i, joint in enumerate(be.JOINTS):
            y = 26 + i * self.ROW
            lo, hi = self.ranges.get(joint, (0.0, 100.0) if joint == "gripper" else (-180.0, 180.0))
            unit = "%" if joint == "gripper" else "°"
            pos, tgt = self.follower.get(joint), self.target.get(joint)
            p.setFont(f)
            p.setPen(QColor(TEXT))
            f.setPointSizeF(9)
            p.setFont(f)
            p.drawText(
                QRectF(0, y, name_w, 22), Qt.AlignmentFlag.AlignVCenter, joint.replace("_", " ").upper()
            )
            f.setPointSizeF(7.5)

            track = QRectF(bar_l, y + 6, max(bar_r - bar_l, 10), 10)
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor("#0d141b"))
            p.drawRoundedRect(track, 5, 5)

            def x_of(v: float, track=track, lo=lo, hi=hi) -> float:
                return track.left() + (min(max(v, lo), hi) - lo) / (hi - lo or 1) * track.width()

            zero = x_of(0.0 if lo < 0 < hi else lo)
            p.setPen(QPen(QColor("#2b3746"), 1))
            p.drawLine(QPointF(zero, track.top() - 3), QPointF(zero, track.bottom() + 3))
            if pos is not None:
                x = x_of(pos)
                grad = QLinearGradient(zero, 0, x, 0)
                grad.setColorAt(0, QColor(ACCENT).darker(180))
                grad.setColorAt(1, QColor(ACCENT))
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(grad))
                p.drawRoundedRect(QRectF(min(zero, x), track.top(), abs(x - zero), track.height()), 5, 5)
                p.setBrush(QColor(ACCENT))
                p.drawEllipse(QPointF(x, track.center().y()), 7, 7)
            if tgt is not None:
                x = x_of(tgt)
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QColor("#ffffff"))
                tri = QPolygonF(
                    [
                        QPointF(x - 6, track.top() - 8),
                        QPointF(x + 6, track.top() - 8),
                        QPointF(x, track.top() - 1),
                    ]
                )
                p.drawPolygon(tri)
                p.drawRect(QRectF(x - 1, track.top(), 2, track.height()))
            # scale labels
            p.setPen(QColor("#4b5866"))
            small = QFont(self.font())
            small.setPointSizeF(7)
            p.setFont(small)
            p.drawText(
                QRectF(track.left(), track.bottom() + 2, 60, 12), Qt.AlignmentFlag.AlignLeft, f"{lo:.0f}"
            )
            p.drawText(
                QRectF(track.right() - 60, track.bottom() + 2, 60, 12),
                Qt.AlignmentFlag.AlignRight,
                f"{hi:.0f}",
            )

            p.setFont(mono)
            vx = w - val_w
            p.setPen(QColor(ACCENT))
            p.drawText(
                QRectF(vx, y, 62, 22),
                Qt.AlignmentFlag.AlignVCenter,
                "—" if pos is None else f"{pos:6.1f}{unit}",
            )
            p.setPen(QColor("#ffffff"))
            p.drawText(
                QRectF(vx + 64, y, 62, 22),
                Qt.AlignmentFlag.AlignVCenter,
                "—" if tgt is None else f"{tgt:6.1f}{unit}",
            )
            if pos is not None and tgt is not None:
                gap = abs(pos - tgt)
                p.setPen(QColor(GREEN if gap < POSE_OK else AMBER if gap < POSE_WARN else RED))
                p.drawText(QRectF(vx + 128, y, 50, 22), Qt.AlignmentFlag.AlignVCenter, f"{gap:5.1f}")
            temp, load = self.temps.get(joint), self.loads.get(joint)
            if temp is not None:
                p.setPen(QColor(RED if temp >= 60 else AMBER if temp >= 50 else MUTED))
                p.drawText(
                    QRectF(vx + 178, y, 80, 22),
                    Qt.AlignmentFlag.AlignVCenter,
                    f"{temp:3.0f}°C {load or 0:3.0f}%",
                )
            p.setFont(f)


# ---------------------------------------------------------------- logging


class QueueHandler(logging.Handler):
    def __init__(self, q: queue.Queue) -> None:
        super().__init__()
        self.q = q
        self.setFormatter(logging.Formatter("%(asctime)s  %(message)s", "%H:%M:%S"))

    def emit(self, record: logging.LogRecord) -> None:
        self.q.put((record.levelno, self.format(record)))


class StreamToLog:
    """Sends print() output (and pythonw's missing console) into the log."""

    def __init__(self, level: int) -> None:
        self.level = level

    def write(self, text: str) -> None:
        for line in text.rstrip().splitlines():
            if line.strip():
                logging.getLogger("so101.print").log(self.level, line)

    def flush(self) -> None:
        pass


# ---------------------------------------------------------------- window


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.settings = be.load_settings()
        self.tel = be.Telemetry()
        self.job: be.Job | None = None
        self.monitor: be.Monitor | None = None
        self.estop_note = False
        self.server_ms: float | None = None
        self.server_pinged = False
        self.log_q: queue.Queue = queue.Queue()
        self.cam_rates = {name: be.RateMeter() for name in be.CAMERAS}
        self.cam_last: dict[str, float] = {}

        self.setWindowTitle("SO-101 Operator Console")
        if ICON.exists():
            self.setWindowIcon(QIcon(str(ICON)))
        self.resize(1500, 900)
        self.build()
        self.setup_logging()

        QShortcut(
            QKeySequence(Qt.Key.Key_Escape),
            self,
            activated=self.estop,
            context=Qt.ShortcutContext.ApplicationShortcut,
        )
        QShortcut(QKeySequence(Qt.Key.Key_Right), self, activated=lambda: self.record_control("next"))
        QShortcut(QKeySequence(Qt.Key.Key_Left), self, activated=lambda: self.record_control("redo"))

        self.prefs = QSettings("so101", "operator-console")
        on = {name: self.prefs.value(f"camera/{name}", True, type=bool) for name in be.CAMERAS}
        be.HUB = be.CameraHub(self.settings, self.tel, on)
        be.HUB.start()
        for name, view in self.cam_views.items():
            view.toggle.setChecked(on[name])
            view.toggle.toggled.connect(lambda checked, n=name: self.set_camera(n, checked))

        self.fast = QTimer(self, interval=15, timeout=self.refresh_live)
        self.fast.setTimerType(Qt.TimerType.PreciseTimer)
        self.fast.start()
        self.slow = QTimer(self, interval=150, timeout=self.refresh_state)
        self.slow.start()
        threading.Thread(target=self.ping_loop, daemon=True).start()
        logging.getLogger("so101").info(
            "Follower %s, leader %s, cameras wrist=%s top=%s",
            self.settings.get("SO101_FOLLOWER"),
            self.settings.get("SO101_LEADER"),
            self.settings.get("SO101_WRIST_CAM"),
            self.settings.get("SO101_TOP_CAM"),
        )

    # ----- layout

    def build(self) -> None:
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(12)
        self.setCentralWidget(root)

        # top bar
        top = QHBoxLayout()
        top.setSpacing(10)
        logo = QLabel()
        logo.setPixmap(
            QPixmap.fromImage(draw_icon(96)).scaled(
                46, 46, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation
            )
        )
        top.addWidget(logo)
        title = QVBoxLayout()
        title.setSpacing(0)
        name = label("SO-101", "big")
        title.addWidget(name)
        title.addWidget(label("OPERATOR CONSOLE", "section"))
        top.addLayout(title)
        top.addSpacing(18)
        self.pill_follower = StatusPill("FOLLOWER · 12 V")
        self.pill_leader = StatusPill("LEADER · 6 V")
        self.pill_server = StatusPill("MODEL SERVER")
        for pill in (self.pill_follower, self.pill_leader, self.pill_server):
            top.addWidget(pill)
        top.addStretch(1)
        self.hz = label("", "muted")
        self.hz.setFont(QFont("Consolas", 11))
        top.addWidget(self.hz)
        top.addSpacing(8)
        self.badge = ModeBadge()
        top.addWidget(self.badge)
        top.addSpacing(8)
        self.estop_btn = button("E-STOP\nEsc", "estop")
        self.estop_btn.setToolTip("Stop now. The follower holds its pose (torque stays on).")
        self.estop_btn.clicked.connect(self.estop)
        top.addWidget(self.estop_btn)
        outer.addLayout(top)

        # banner
        self.banner = QLabel()
        self.banner.setWordWrap(True)
        self.banner.hide()
        self.banner.setToolTip("Click to dismiss")
        self.banner.mousePressEvent = lambda _event: self.banner.hide()
        outer.addWidget(self.banner)

        body = QHBoxLayout()
        body.setSpacing(12)
        outer.addLayout(body, 1)

        # left: cameras + joints
        left = QVBoxLayout()
        left.setSpacing(12)
        cams = QHBoxLayout()
        cams.setSpacing(12)
        self.cam_views = {"wrist": CameraView("WRIST CAM"), "top": CameraView("TOP CAM")}
        for view in self.cam_views.values():
            cams.addWidget(view)
        left.addLayout(cams, 3)
        joints_panel = panel()
        jl = QVBoxLayout(joints_panel)
        jl.setContentsMargins(16, 12, 16, 12)
        self.joints = JointPanel()
        jl.addWidget(self.joints)
        left.addWidget(joints_panel, 0)
        joints_panel.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        body.addLayout(left, 1)

        # right: controls
        side = panel()
        side.setFixedWidth(420)
        sl = QVBoxLayout(side)
        sl.setContentsMargins(14, 10, 14, 14)
        self.tabs = QTabWidget()
        for page, name in (
            (self.build_teleop(), "TELEOP"),
            (self.build_record(), "RECORD"),
            (self.build_autonomous(), "AUTONOMOUS"),
            (self.build_tools(), "TOOLS"),
        ):
            # Scroll inside a tab instead of making the window taller than a laptop screen.
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.Shape.NoFrame)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setWidget(page)
            self.tabs.addTab(scroll, name)
        sl.addWidget(self.tabs, 3)
        sl.addSpacing(8)
        sl.addWidget(label("EVENT LOG", "section"))
        self.log_view = QPlainTextEdit(readOnly=True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setMinimumHeight(90)
        sl.addWidget(self.log_view, 1)
        body.addWidget(side)

    def card(self, title: str) -> tuple[QFrame, QVBoxLayout]:
        f = QFrame()
        f.setStyleSheet(
            f"QFrame {{ background: {PANEL2}; border: 1px solid {BORDER}; border-radius: 10px; }}"
            "QLabel { border: none; background: transparent; }"
        )
        lay = QVBoxLayout(f)
        lay.setContentsMargins(14, 12, 14, 12)
        lay.setSpacing(6)
        lay.addWidget(label(title, "section"))
        return f, lay

    def build_teleop(self) -> QWidget:
        w = QWidget()
        w.setObjectName("page")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(2, 14, 2, 2)
        lay.setSpacing(12)
        card, cl = self.card("POSE MATCH")
        self.pose_state = label("—")
        self.pose_state.setStyleSheet("font-size: 16pt; font-weight: 800;")
        self.pose_detail = label("", "muted")
        self.pose_detail.setWordWrap(True)
        cl.addWidget(self.pose_state)
        cl.addWidget(self.pose_detail)
        lay.addWidget(card)
        self.teleop_start = button("▶  START TELEOP", "primary")
        self.teleop_start.clicked.connect(self.start_teleop)
        self.teleop_stop = button("■  STOP AND PARK")
        self.teleop_stop.clicked.connect(self.stop_job)
        lay.addWidget(self.teleop_start)
        lay.addWidget(self.teleop_stop)
        note = label(
            "If the arms don't match, the follower first glides to the leader's pose "
            f"({be.MATCH_SPEED:.0f}°/s), then teleop takes over. Stiffness P={be.STIFFNESS_P}. "
            "Stop parks both arms at their rest pose.",
            "muted",
        )
        note.setWordWrap(True)
        lay.addWidget(note)
        lay.addStretch(1)
        return w

    def build_record(self) -> QWidget:
        w = QWidget()
        w.setObjectName("page")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(2, 14, 2, 2)
        lay.setSpacing(10)
        form = QFormLayout()
        form.setSpacing(8)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)
        self.rec_repo = QLineEdit("hkder/so101-pick-cube")
        self.rec_task = QLineEdit("Pick up the red cube and put it in the cup")
        self.rec_eps = QSpinBox(minimum=1, maximum=500, value=20)
        self.rec_len = QSpinBox(minimum=3, maximum=600, value=20, suffix=" s")
        self.rec_reset = QSpinBox(minimum=0, maximum=600, value=10, suffix=" s")
        self.rec_voice = QCheckBox("Voice prompts")
        self.rec_voice.setChecked(True)
        self.rec_upload = QCheckBox("Upload to Hugging Face")
        self.rec_upload.setChecked(True)
        form.addRow("Dataset", self.rec_repo)
        form.addRow("Task", self.rec_task)
        row = QHBoxLayout()
        row.addWidget(self.rec_eps)
        row.addWidget(label("each", "muted"))
        row.addWidget(self.rec_len)
        form.addRow("Episodes", row)
        form.addRow("Reset time", self.rec_reset)
        checks = QHBoxLayout()
        checks.addWidget(self.rec_voice)
        checks.addWidget(self.rec_upload)
        form.addRow("", checks)
        lay.addLayout(form)

        card, cl = self.card("SESSION")
        self.rec_phase = label("Ready")
        self.rec_phase.setStyleSheet("font-size: 17pt; font-weight: 800;")
        self.rec_episode = label("", "muted")
        self.rec_progress = QProgressBar()
        self.rec_progress.setRange(0, 1000)
        self.rec_progress.setTextVisible(True)
        self.rec_progress.setFormat("")
        cl.addWidget(self.rec_phase)
        cl.addWidget(self.rec_episode)
        cl.addWidget(self.rec_progress)
        lay.addWidget(card)

        self.rec_start = button("●  START RECORDING", "record")
        self.rec_start.clicked.connect(self.start_record)
        lay.addWidget(self.rec_start)
        grid = QGridLayout()
        grid.setSpacing(8)
        self.rec_next = button("NEXT  ▶\n(→)")
        self.rec_redo = button("↺  REDO\n(←)")
        self.rec_finish = button("■  FINISH AND UPLOAD")
        self.rec_next.clicked.connect(lambda: self.record_control("next"))
        self.rec_redo.clicked.connect(lambda: self.record_control("redo"))
        self.rec_finish.clicked.connect(self.stop_job)
        grid.addWidget(self.rec_redo, 0, 0)
        grid.addWidget(self.rec_next, 0, 1)
        grid.addWidget(self.rec_finish, 1, 0, 1, 2)
        lay.addLayout(grid)
        self.rec_saved = label("", "muted")
        self.rec_saved.setWordWrap(True)
        self.rec_saved.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self.rec_saved)
        lay.addStretch(1)
        return w

    def build_autonomous(self) -> QWidget:
        w = QWidget()
        w.setObjectName("page")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(2, 14, 2, 2)
        lay.setSpacing(10)
        form = QFormLayout()
        form.setSpacing(8)
        self.run_server = QLineEdit("10.31.247.142:8080")
        self.run_model = QLineEdit("hkder/so101-pick-cube-groot")
        self.run_task = QLineEdit("Pick up the red cube and put it in the cup")
        self.run_threshold = QDoubleSpinBox(minimum=0.1, maximum=1.0, singleStep=0.05, value=0.7)
        self.run_threshold.setToolTip(
            "Ask for the next action chunk when this fraction is left. Use 0.7 on slow links (ping > 100 ms)."
        )
        form.addRow("Server", self.run_server)
        form.addRow("Model", self.run_model)
        form.addRow("Task", self.run_task)
        form.addRow("Chunk threshold", self.run_threshold)
        lay.addLayout(form)
        card, cl = self.card("MODEL")
        self.run_state = label("Ready")
        self.run_state.setStyleSheet("font-size: 15pt; font-weight: 800;")
        self.run_detail = label("GR00T N1.7 served by lerobot policy_server on the DGX Spark.", "muted")
        self.run_detail.setWordWrap(True)
        cl.addWidget(self.run_state)
        cl.addWidget(self.run_detail)
        lay.addWidget(card)
        self.run_start = button("▶  RUN MODEL", "primary")
        self.run_start.clicked.connect(self.start_model)
        self.run_stop = button("■  STOP AND PARK")
        self.run_stop.clicked.connect(self.stop_job)
        lay.addWidget(self.run_start)
        lay.addWidget(self.run_stop)
        note = label(
            "Keep a hand near E-STOP. Cameras, their positions and the task sentence must match the "
            "recording.",
            "muted",
        )
        note.setWordWrap(True)
        lay.addWidget(note)
        lay.addStretch(1)
        return w

    def build_tools(self) -> QWidget:
        w = QWidget()
        w.setObjectName("page")
        lay = QVBoxLayout(w)
        lay.setContentsMargins(2, 14, 2, 2)
        lay.setSpacing(10)
        self.tool_buttons = []
        for text, tip, fn in [
            (
                "Park both arms",
                "Move the follower and the leader smoothly to the rest pose, then relax them.",
                lambda: self.start_job(be.Park(self.settings, self.tel)),
            ),
            ("Relax follower (torque off)", "The follower goes limp at once. Hold it first.", self.relax),
            ("Save current pose as rest", "Relax the follower, move it by hand, then save.", self.save_rest),
        ]:
            b = button(text)
            b.setToolTip(tip)
            b.clicked.connect(fn)
            lay.addWidget(b)
            hint = label(tip, "muted")
            hint.setWordWrap(True)
            lay.addWidget(hint)
            self.tool_buttons.append(b)
        lay.addSpacing(10)
        card, cl = self.card("THIS MACHINE")
        s = self.settings
        info = label(
            f"Follower {s.get('SO101_FOLLOWER')}   Leader {s.get('SO101_LEADER')}\n"
            f"Cameras: wrist {s.get('SO101_WRIST_CAM')}, top {s.get('SO101_TOP_CAM')}"
            f"  ({s.get('SO101_CAM_FOURCC') or 'auto'}, backend {s.get('SO101_CAM_BACKEND') or 'auto'})\n"
            "Edit so101/config.local.env to change.",
            "muted",
        )
        cl.addWidget(info)
        lay.addWidget(card)
        lay.addStretch(1)
        return w

    def setup_logging(self) -> None:
        LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        root = logging.getLogger()
        root.setLevel(logging.INFO)
        root.addHandler(QueueHandler(self.log_q))
        file_handler = logging.FileHandler(LOG_FILE, encoding="utf-8")
        file_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        root.addHandler(file_handler)
        sys.stdout = StreamToLog(logging.INFO)
        sys.stderr = StreamToLog(logging.WARNING)

    # ----- jobs

    def busy(self) -> bool:
        return self.job is not None and self.job.is_alive()

    def start_job(self, job: be.Job) -> None:
        if self.busy():
            return
        self.estop_note = False
        self.banner.hide()
        if self.monitor is not None:
            self.monitor.stop()
            self.monitor.join(timeout=3)
            self.monitor = None
        self.job = job
        job.start()
        logging.getLogger("so101").info("Started %s", job.title.lower())

    def stop_job(self) -> None:
        if self.busy():
            self.job.stop()
            if isinstance(self.job, be.Record):
                self.tel.set_phase("Finishing: saving the episodes, then uploading")

    def estop(self) -> None:
        if self.busy():
            self.job.stop(estop=True)
            self.estop_note = True
            self.show_banner(
                "E-STOP — control stopped. The follower is holding its pose with torque on. "
                "Use TOOLS › Park or Relax when it is safe.",
                RED,
            )
            logging.getLogger("so101").warning("E-STOP pressed")

    def record_control(self, what: str) -> None:
        if self.busy() and isinstance(self.job, be.Record):
            (self.job.next_episode if what == "next" else self.job.redo_episode)()
            logging.getLogger("so101").info(
                "Record: %s", "next episode" if what == "next" else "redo episode"
            )

    def pose_gap(self) -> tuple[float | None, str]:
        with self.tel.lock:
            f, t = dict(self.tel.follower), dict(self.tel.target)
        gaps = {j: abs(f[j] - t[j]) for j in be.JOINTS[:-1] if j in f and j in t}
        if not gaps:
            return None, ""
        joint = max(gaps, key=gaps.get)
        return gaps[joint], joint

    def start_teleop(self) -> None:
        # If the arms don't match, the job glides the follower onto the leader first.
        self.start_job(be.Teleop(self.settings, self.tel))

    def start_record(self) -> None:
        repo, task = self.rec_repo.text().strip(), self.rec_task.text().strip()
        if "/" not in repo or not task:
            QMessageBox.warning(self, "Record", "Enter a dataset like user/name and a task sentence.")
            return
        if not self.cameras_on():
            return
        self.tel.put(dataset="", episode=0, episodes_saved=0)
        self.rec_saved.setText("")
        self.start_job(
            be.Record(
                self.settings,
                self.tel,
                repo,
                task,
                self.rec_eps.value(),
                self.rec_len.value(),
                self.rec_reset.value(),
                self.rec_voice.isChecked(),
                self.rec_upload.isChecked(),
            )
        )

    def start_model(self) -> None:
        server, model, task = (
            self.run_server.text().strip(),
            self.run_model.text().strip(),
            self.run_task.text().strip(),
        )
        if not (server and model and task):
            QMessageBox.warning(self, "Run model", "Fill in server, model and task.")
            return
        if not self.cameras_on():
            return
        self.start_job(be.RunModel(self.settings, self.tel, server, model, task, self.run_threshold.value()))

    def relax(self) -> None:
        if (
            QMessageBox.question(self, "Relax", "The follower will go limp immediately. Are you holding it?")
            == QMessageBox.StandardButton.Yes
        ):
            self.start_job(be.Relax(self.settings, self.tel))

    def save_rest(self) -> None:
        if (
            QMessageBox.question(
                self,
                "Save rest pose",
                "Save the follower's current pose as its rest pose?\n\n(Relax it first and move it by hand.)",
            )
            == QMessageBox.StandardButton.Yes
        ):
            self.start_job(be.SaveRest(self.settings, self.tel))

    def set_camera(self, name: str, on: bool) -> None:
        be.HUB.set_on(name, on)
        self.prefs.setValue(f"camera/{name}", on)

    def cameras_on(self) -> bool:
        off = [n for n in be.CAMERAS if not be.HUB.enabled[n].is_set()]
        if off:
            QMessageBox.warning(
                self,
                "Cameras off",
                f"Turn on the {' and '.join(off)} camera first (ON switch on the camera view).",
            )
        return not off

    def ping_loop(self) -> None:
        while True:
            self.server_ms = be.ping_server(self.run_server.text().strip())
            self.server_pinged = True
            time.sleep(5)

    def show_banner(self, text: str, color: str) -> None:
        self.banner.setText(text)
        self.banner.setStyleSheet(
            f"background: {QColor(color).darker(260).name()}; border: 1px solid {color}; border-radius: 10px;"
            f"padding: 10px 14px; font-weight: 700; color: {TEXT};"
        )
        self.banner.show()

    # ----- refresh

    def refresh_live(self) -> None:
        now = time.monotonic()
        with self.tel.lock:
            frames, times = dict(self.tel.frames), dict(self.tel.frame_times)
        for name, view in self.cam_views.items():
            t = times.get(name)
            if t is not None and t != self.cam_last.get(name):
                self.cam_last[name] = t
                self.cam_rates[name].tick()
            live = t is not None and now - t < 1.0
            view.show_frame(frames.get(name), live, f"{self.cam_rates[name].hz:4.1f} fps" if live else "")
        target_name = "MODEL" if isinstance(self.job, be.RunModel) and self.busy() else "LEADER"
        self.joints.set(self.tel, target_name)
        self.badge.update()

    def refresh_state(self) -> None:
        # job lifecycle
        if self.job is not None and not self.job.is_alive():
            job, self.job = self.job, None
            if job.error:
                self.show_banner(f"{job.title} stopped with an error: {job.error}", RED)
            elif isinstance(job, be.Record) and self.tel.dataset:
                where = "Saved and uploaded as" if job.upload else "Saved locally (not uploaded) as"
                self.rec_saved.setText(f"{where}:\n{self.tel.dataset}")
                self.show_banner(f"Recording done → {self.tel.dataset}", GREEN)
            logging.getLogger("so101").info("%s finished", job.title.capitalize())
        if not self.busy() and (self.monitor is None or not self.monitor.is_alive()):
            self.monitor = be.Monitor(self.settings, self.tel)
            self.monitor.start()

        self.drain_log()
        busy = self.busy()
        job = self.job if busy else None
        mode = job.title if job else ("E-STOP" if self.estop_note else "IDLE")
        with self.tel.lock:
            phase, start, length = self.tel.phase, self.tel.phase_start, self.tel.phase_len
            hz, volts, checked = self.tel.loop_hz, dict(self.tel.volts), self.tel.arms_checked
            episode, saved = self.tel.episode, self.tel.episodes_saved
        sub = phase or ("" if job else "Watching both arms" if checked else "Starting up…")
        self.badge.set(mode, sub[:38], blink=mode in ("RECORDING", "E-STOP"))
        self.hz.setText(f"{hz:5.1f} Hz" if busy and hz > 0 else "")

        # status pills
        s = self.settings
        for pill, key, port, lo, hi in (
            (self.pill_follower, "follower", s.get("SO101_FOLLOWER"), 11.0, 13.0),
            (self.pill_leader, "leader", s.get("SO101_LEADER"), 5.5, 8.0),
        ):
            v = volts.get(key)
            if busy:
                pill.set(f"{port} · in use", ACCENT)
            elif v is None and not checked:
                pill.set(f"{port} · checking…", MUTED)
            elif v is None:
                pill.set(f"{port} · not found", RED)
            else:
                pill.set(f"{port} · {v:.1f} V", GREEN if lo <= v <= hi else AMBER)
        ms = self.server_ms
        host = self.run_server.text().split(":")[0]
        if not self.server_pinged:
            self.pill_server.set(f"{host} · checking…", MUTED)
        else:
            self.pill_server.set(
                f"{host} · {'offline' if ms is None else f'{ms:.0f} ms'}",
                RED if ms is None else GREEN if ms < 100 else AMBER,
            )

        # pose match (idle only: leader vs follower)
        gap, joint = self.pose_gap()
        if busy:
            self.pose_state.setText("Teleop running" if isinstance(job, be.Teleop) else "Arm in use")
            self.pose_state.setStyleSheet(f"font-size: 16pt; font-weight: 800; color: {ACCENT};")
            self.pose_detail.setText("")
        elif not checked:
            self.pose_state.setText("Starting up…")
            self.pose_state.setStyleSheet(f"font-size: 16pt; font-weight: 800; color: {MUTED};")
            self.pose_detail.setText("Loading the robot drivers (about 5 s).")
        elif gap is None:
            self.pose_state.setText("Waiting for both arms")
            self.pose_state.setStyleSheet(f"font-size: 16pt; font-weight: 800; color: {MUTED};")
            self.pose_detail.setText("Plug in both boards: follower on 12 V, leader on 6 V.")
        else:
            color = GREEN if gap < POSE_OK else AMBER if gap < POSE_WARN else RED
            self.pose_state.setText("POSES MATCH" if gap < POSE_OK else f"GAP {gap:.0f}°")
            self.pose_state.setStyleSheet(f"font-size: 16pt; font-weight: 800; color: {color};")
            self.pose_detail.setText(
                "Ready to start."
                if gap < POSE_OK
                else f"Largest on {joint.replace('_', ' ')}. On start, the follower glides to the leader."
            )

        # recording panel
        recording = isinstance(job, be.Record)
        if recording:
            color = RED if phase == "Recording" else AMBER if phase.startswith("Reset") else ACCENT
            self.rec_phase.setText(phase.upper() or "STARTING")
            self.rec_phase.setStyleSheet(f"font-size: 17pt; font-weight: 800; color: {color};")
            self.rec_episode.setText(
                f"Episode {episode} of {job.episodes}   ·   {saved} saved" if episode else ""
            )
            if length > 0:
                left = max(length - (time.monotonic() - start), 0)
                self.rec_progress.setValue(int(1000 * (1 - left / length)))
                self.rec_progress.setFormat(f"{left:4.1f} s left")
                self.rec_progress.setStyleSheet(
                    f"QProgressBar::chunk {{ background: {color}; border-radius: 5px; }}"
                )
            else:
                self.rec_progress.setValue(0)
                self.rec_progress.setFormat("")
        elif not busy:
            self.rec_phase.setText("Ready")
            self.rec_phase.setStyleSheet("font-size: 17pt; font-weight: 800;")
            self.rec_episode.setText("")
            self.rec_progress.setValue(0)
            self.rec_progress.setFormat("")

        # model panel
        if isinstance(job, be.RunModel):
            self.run_state.setText(phase or "Starting")
            self.run_state.setStyleSheet(f"font-size: 15pt; font-weight: 800; color: {VIOLET};")
        elif not busy:
            self.run_state.setText("Ready")
            self.run_state.setStyleSheet("font-size: 15pt; font-weight: 800;")

        # enable / disable
        # The idle monitor's first read means the robot drivers (PyTorch & co, ~5 s) are loaded.
        for b in (self.teleop_start, self.rec_start, self.run_start, *self.tool_buttons):
            b.setEnabled(not busy and checked)
        for b, text in (
            (self.teleop_start, "▶  START TELEOP"),
            (self.rec_start, "●  START RECORDING"),
            (self.run_start, "▶  RUN MODEL"),
        ):
            b.setText(text if checked else "Loading robot drivers…")
        for field in (
            self.rec_repo,
            self.rec_task,
            self.rec_eps,
            self.rec_len,
            self.rec_reset,
            self.rec_voice,
            self.rec_upload,
            self.run_server,
            self.run_model,
            self.run_task,
            self.run_threshold,
        ):
            field.setEnabled(not busy)
        self.teleop_stop.setEnabled(isinstance(job, be.Teleop))
        self.run_stop.setEnabled(isinstance(job, be.RunModel))
        for b in (self.rec_next, self.rec_redo, self.rec_finish):
            b.setEnabled(recording)
        self.estop_btn.setEnabled(busy)
        cams_locked = isinstance(job, (be.Record, be.RunModel))  # they read the cameras
        for name, view in self.cam_views.items():
            status = be.HUB.status.get(name) if be.HUB else "opening"
            view.off = status == "off"
            view.toggle.setEnabled(not cams_locked)
            view.hint = {
                "opening": "Opening camera…",
                "off": "Turn it on with the switch (top right)",
                "error": "Camera not found. Check the USB cable (retrying).",
            }.get(status, "Waiting for frames…")

    def drain_log(self) -> None:
        colors = {logging.WARNING: AMBER, logging.ERROR: RED, logging.CRITICAL: RED}
        try:
            while True:
                level, text = self.log_q.get_nowait()
                color = colors.get(level, "#aab7c4")
                safe = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                self.log_view.appendHtml(f'<span style="color:{color}">{safe}</span>')
        except queue.Empty:
            pass

    def closeEvent(self, event) -> None:
        if self.busy():
            answer = QMessageBox.question(self, "Quit", f"{self.job.title} is running. Stop it and quit?")
            if answer != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
            self.job.stop()
            self.job.join(timeout=15)
        if self.monitor is not None:
            self.monitor.stop()
            self.monitor.join(timeout=3)
        if be.HUB is not None:
            be.HUB.close()
        event.accept()


def main() -> None:
    if "--write-icon" in sys.argv:
        QGuiApplication(sys.argv)
        path = Path(sys.argv[sys.argv.index("--write-icon") + 1])
        path.parent.mkdir(parents=True, exist_ok=True)
        draw_icon(256).save(str(path))
        return
    if sys.platform == "win32":
        import ctypes

        # Own taskbar entry and icon instead of Python's.
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("so101.operator")
    app = QApplication(sys.argv)
    app.setApplicationName("SO-101 Operator Console")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    app.setWindowIcon(QIcon(str(ICON)) if ICON.exists() else QIcon(QPixmap.fromImage(draw_icon())))
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
