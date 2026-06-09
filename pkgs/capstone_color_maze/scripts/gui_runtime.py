#!/usr/bin/env python3
"""
gui_runtime.py — 런타임 모니터 GUI  (완전 반응형)

maze_tour.py 실행 중 로봇 상태를 실시간으로 표시하는 발표용 화면.
구독:
  /target_color     (std_msgs/String)
  /explorer_phase   (std_msgs/String)
  /detected_color   (std_msgs/String)
  /detected_digit   (std_msgs/Int32)
  /color_signal     (std_msgs/Float32MultiArray)  [color_id, cx_norm, coverage]
  /maze_done        (std_msgs/Bool)
  /camera/image_raw (sensor_msgs/Image)
  /map              (nav_msgs/OccupancyGrid)

실행:
  python3 gui_runtime.py
  python3 gui_runtime.py --cam-topic /tb3/cam/image_raw
"""
import sys, time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32, Float32MultiArray
from rcl_interfaces.msg import Log

try:
    from nav_msgs.msg import OccupancyGrid
    _HAS_NAV = True
except ImportError:
    _HAS_NAV = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QScrollArea, QSizePolicy, QGraphicsOpacityEffect,
)
from PyQt5.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap

from gui_common import (
    apply_dark_palette, BASE_QSS, RosThread, font,
    BG_DARK, BG_CARD, BG_CARD2, BORDER, TEXT_PRI, TEXT_SEC,
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_MAP,
    STATUS_OK, STATUS_WARN, STATUS_IDLE, ACCENT,
)

# ── CLI ─────────────────────────────────────────────────────────
def _arg(flag, default=None):
    for i, a in enumerate(sys.argv):
        if a == flag and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default

CAM_TOPIC = _arg('--cam-topic', '/camera/image_raw')
BASE_H    = 660
ROI_RATIO = 0.7
FPS_LIMIT = 15

DRAW_BGR = {
    'RED':   (60,  60,  230),
    'GREEN': (50,  200, 50),
    'BLUE':  (230, 100, 50),
    'NONE':  (140, 140, 140),
}


# ── 변환 헬퍼 ───────────────────────────────────────────────────
def imgmsg_to_bgr(msg: Image):
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding
    if enc == 'bgr8':
        return arr.reshape(msg.height, msg.width, 3).copy()
    if enc == 'rgb8':
        return arr.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    if enc in ('mono8', '8UC1'):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    return None


def bgr_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    h, w = bgr.shape[:2]
    rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    qi   = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
    return QPixmap.fromImage(qi.copy())


def occupancy_to_bgr(msg) -> np.ndarray:
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape(h, w)
    img  = np.full((h, w, 3), 45, dtype=np.uint8)
    img[data == 0]   = [62,  66,  72]
    img[data == 100] = [210, 215, 220]
    return np.flipud(img)


# ── ROS2 노드 ───────────────────────────────────────────────────
class RuntimeNode(Node, QObject):
    sig_target = pyqtSignal(str)
    sig_phase  = pyqtSignal(str)
    sig_color  = pyqtSignal(str)
    sig_digit  = pyqtSignal(int)
    sig_signal = pyqtSignal(float, float, float)
    sig_done   = pyqtSignal(bool)
    sig_frame  = pyqtSignal(object)
    sig_map    = pyqtSignal(object)
    sig_log    = pyqtSignal(str)

    # /rosout 에서 이 노드들의 INFO 이상 메시지를 current action 으로 표시
    _WATCH_NODES = {'maze_tour', 'phase1_explorer', 'phase2_visitor'}

    def __init__(self, cam_topic: str):
        Node.__init__(self, 'gui_runtime')
        QObject.__init__(self)
        self._color_str  = 'NONE'
        self._cx_norm    = 0.0
        self._coverage   = 0.0
        self._last_cam_t = 0.0
        self._last_map_t = 0.0

        self.create_subscription(String, '/target_color',   self._target, 10)
        self.create_subscription(String, '/explorer_phase', self._phase,  10)
        self.create_subscription(String, '/detected_color', self._color,  10)
        self.create_subscription(Int32,  '/detected_digit', self._digit,  10)
        self.create_subscription(Float32MultiArray, '/color_signal', self._sig, 10)
        self.create_subscription(Bool,   '/maze_done',      self._done,   10)
        self.create_subscription(Log,   '/rosout',         self._rosout, 10)
        self.create_subscription(Image, cam_topic, self._img,
                                 qos_profile_sensor_data)
        if _HAS_NAV:
            _map_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, '/map', self._map, _map_qos)

    def _target(self, m): self.sig_target.emit(m.data)
    def _phase(self,  m): self.sig_phase.emit(m.data)
    def _rosout(self, m: Log):
        if m.name in self._WATCH_NODES and m.level >= 20:   # INFO=20
            self.sig_log.emit(m.msg)
    def _color(self,  m):
        self._color_str = m.data
        self.sig_color.emit(m.data)
    def _digit(self,  m): self.sig_digit.emit(m.data)
    def _sig(self, m):
        if len(m.data) >= 3:
            self._cx_norm  = float(m.data[1])
            self._coverage = float(m.data[2])
            self.sig_signal.emit(float(m.data[0]), self._cx_norm, self._coverage)
    def _done(self,   m): self.sig_done.emit(m.data)

    def _img(self, msg: Image):
        now = time.monotonic()
        if now - self._last_cam_t < 1 / FPS_LIMIT:
            return
        self._last_cam_t = now
        bgr = imgmsg_to_bgr(msg)
        if bgr is None:
            return
        self.sig_frame.emit(self._annotate(bgr))

    def _map(self, msg):
        now = time.monotonic()
        if now - self._last_map_t < 0.5:
            return
        self._last_map_t = now
        self.sig_map.emit(occupancy_to_bgr(msg))

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        h, w   = frame.shape[:2]
        rw, rh = int(w * ROI_RATIO), int(h * ROI_RATIO)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        bgr    = DRAW_BGR.get(self._color_str, DRAW_BGR['NONE'])
        cv2.rectangle(frame, (x0, y0), (x0 + rw, y0 + rh), bgr, 2)
        if self._color_str != 'NONE':
            cx_px = int(x0 + rw / 2 + self._cx_norm * rw / 2)
            cv2.line(frame, (cx_px, y0), (cx_px, y0 + rh), bgr, 1)
        label = f"{self._color_str}  cov={self._coverage:.2f}  cx={self._cx_norm:+.2f}"
        cv2.putText(frame, label, (x0 + 4, max(18, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1, cv2.LINE_AA)
        return frame


# ── 이미지 패널 ─────────────────────────────────────────────────
class ImagePanel(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("card")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pm = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(3)

        self._sec = QLabel(title.upper())
        self._sec.setObjectName("section")
        lay.addWidget(self._sec)

        self._img = QLabel()
        self._img.setAlignment(Qt.AlignCenter)
        self._img.setStyleSheet("background:#000;border-radius:4px;")
        self._img.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self._img, stretch=1)
        self._show_no_signal()

    def _show_no_signal(self):
        self._img.setText(f"<span style='color:{TEXT_SEC};'>NO SIGNAL</span>")
        self._img.setPixmap(QPixmap())

    def update_pixmap(self, pm: QPixmap):
        self._pm = pm
        self._redraw()

    def _redraw(self):
        if not self._pm or self._pm.isNull():
            self._show_no_signal()
            return
        self._img.setPixmap(
            self._pm.scaled(max(1, self._img.width()), max(1, self._img.height()),
                            Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._redraw()

    def update_sec_font(self, f):
        self._sec.setFont(f)


# ── 로그 위젯 ───────────────────────────────────────────────────
class LogWidget(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setStyleSheet(f"background:{BG_CARD};border:none;")
        self._c = QWidget()
        self._lay = QVBoxLayout(self._c)
        self._lay.setContentsMargins(6, 6, 6, 6)
        self._lay.setSpacing(2)
        self._lay.addStretch()
        self.setWidget(self._c)
        self._lines = []

    def append(self, text, color=TEXT_SEC):
        ts  = time.strftime("%H:%M:%S")
        lbl = QLabel(
            f"<span style='color:{TEXT_SEC};font-size:9px;'>{ts}</span>"
            f"&nbsp;&nbsp;<span style='color:{color};'>{text}</span>")
        lbl.setFont(font(10))
        lbl.setTextFormat(Qt.RichText)
        lbl.setWordWrap(True)
        self._lay.insertWidget(self._lay.count() - 1, lbl)
        self._lines.append(lbl)
        if len(self._lines) > 100:
            self._lines.pop(0).setParent(None)
        QTimer.singleShot(40, lambda: self.verticalScrollBar().setValue(
            self.verticalScrollBar().maximum()))


# ── 완료 오버레이 ────────────────────────────────────────────────
class DoneOverlay(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setStyleSheet("background:rgba(13,17,23,0.88);")
        self.hide()

        lay = QVBoxLayout(self)
        lay.setAlignment(Qt.AlignCenter)
        lay.setSpacing(16)

        self.check = QLabel("✓")
        self.check.setStyleSheet(f"color:{STATUS_OK};")
        self.check.setAlignment(Qt.AlignCenter)

        self.msg = QLabel("미션 완료")
        self.msg.setStyleSheet(f"color:{TEXT_PRI};")
        self.msg.setAlignment(Qt.AlignCenter)

        self.sub_lbl = QLabel("")
        self.sub_lbl.setStyleSheet(f"color:{TEXT_SEC};")
        self.sub_lbl.setAlignment(Qt.AlignCenter)

        for w in (self.check, self.msg, self.sub_lbl):
            lay.addWidget(w)

        self._fonts = [(self.check, 72, True), (self.msg, 30, True), (self.sub_lbl, 13, False)]
        for w, pt, b in self._fonts:
            w.setFont(font(pt, b))

    def show_done(self, target: str):
        hex_c = COLOR_MAP.get(target, ACCENT)
        self.sub_lbl.setText(
            f"목표 색상 &nbsp;<span style='color:{hex_c};font-weight:bold;'>"
            f"{target}</span>&nbsp; 벽에 도착했습니다")
        self.sub_lbl.setTextFormat(Qt.RichText)
        self.resize(self.parentWidget().size())
        self.show()
        self.raise_()

    def scale_fonts(self, scale):
        for w, pt, b in self._fonts:
            w.setFont(font(max(7, round(pt * scale)), b))

    def resizeEvent(self, e):
        if self.parentWidget():
            self.resize(self.parentWidget().size())
        super().resizeEvent(e)


# ── 메인 윈도우 ─────────────────────────────────────────────────
class RuntimeWindow(QMainWindow):
    def __init__(self, node: RuntimeNode):
        super().__init__()
        self.node          = node
        self._target_color = "—"
        self._start_time   = time.time()
        self._fonts: list  = []

        self.setWindowTitle("Color Maze — Runtime Monitor")
        self.resize(1120, 680)
        self.setMinimumSize(800, 500)
        self.setStyleSheet(BASE_QSS)

        root = QWidget()
        self.setCentralWidget(root)
        R = QVBoxLayout(root)
        R.setContentsMargins(18, 18, 18, 18)
        R.setSpacing(12)

        # ── 헤더 ──
        hdr = QHBoxLayout()
        self.title_lbl = self._lbl("Runtime Monitor", 18, bold=True, color=TEXT_PRI)
        hdr.addWidget(self.title_lbl)
        hdr.addStretch()
        self.status_lbl = self._lbl("● IDLE", 10, color=STATUS_IDLE)
        hdr.addWidget(self.status_lbl)
        R.addLayout(hdr)

        # ── 현재 행동 카드 ──
        act_card = self._card()
        al = QVBoxLayout(act_card)
        al.setContentsMargins(18, 10, 18, 10)
        al.setSpacing(4)
        al.addWidget(self._sec("CURRENT ACTION"))
        self.phase_lbl = self._lbl("대기 중 — 목표 색을 선택하세요", 10, color=TEXT_SEC)
        self.phase_lbl.setWordWrap(True)
        al.addWidget(self.phase_lbl)
        R.addWidget(act_card)

        # ── 메인 영역 ──
        main = QHBoxLayout()
        main.setSpacing(12)

        # 좌측: 목표색 + 감지 정보 + 배지
        left = QVBoxLayout()
        left.setSpacing(8)

        # 목표색 카드
        tgt_card = self._card()
        tl = QVBoxLayout(tgt_card)
        tl.setContentsMargins(14, 12, 14, 12)
        tl.setSpacing(6)
        tl.addWidget(self._sec("목표"))
        tl.addStretch()
        self.target_dot = self._lbl("●", 28, color=TEXT_SEC, align=Qt.AlignCenter)
        self.target_lbl = self._lbl("—", 24, bold=True, color=TEXT_SEC, align=Qt.AlignCenter)
        tl.addWidget(self.target_dot)
        tl.addWidget(self.target_lbl)
        tl.addStretch()
        left.addWidget(tgt_card, stretch=1)

        # 감지 카드
        det_card = self._card()
        dl = QVBoxLayout(det_card)
        dl.setContentsMargins(14, 12, 14, 12)
        dl.setSpacing(6)
        dl.addWidget(self._sec("현재 감지"))
        dl.addStretch()
        self.det_color = self._lbl("—", 22, bold=True, align=Qt.AlignCenter)
        self.det_digit = self._lbl("digit: —", 13, color=TEXT_SEC, align=Qt.AlignCenter)
        self.det_cov   = self._lbl("cov: —",    9, color=TEXT_SEC, align=Qt.AlignCenter)
        dl.addWidget(self.det_color)
        dl.addWidget(self.det_digit)
        dl.addWidget(self.det_cov)
        dl.addStretch()
        left.addWidget(det_card, stretch=1)

        # 경과/상태 카드
        st_card = self._card()
        sl = QVBoxLayout(st_card)
        sl.setContentsMargins(14, 10, 14, 10)
        sl.setSpacing(8)
        sl.addWidget(self._sec("상태"))
        self.elapsed_lbl = self._lbl("0s",   13, bold=True, color=TEXT_SEC)
        self.state_lbl   = self._lbl("IDLE", 13, bold=True, color=STATUS_IDLE)
        sl.addWidget(self.elapsed_lbl)
        sl.addWidget(self.state_lbl)
        left.addWidget(st_card, stretch=1)

        main.addLayout(left, stretch=1)

        # 중앙+우측: 카메라 + 로그
        right = QVBoxLayout()
        right.setSpacing(8)

        cam_row = QHBoxLayout()
        cam_row.setSpacing(8)
        self.cam_vision = ImagePanel("Vision")
        self.cam_map    = ImagePanel("Map")
        cam_row.addWidget(self.cam_vision, stretch=1)
        cam_row.addWidget(self.cam_map,    stretch=1)
        right.addLayout(cam_row, stretch=3)

        log_card = self._card()
        ll = QVBoxLayout(log_card)
        ll.setContentsMargins(10, 10, 10, 10)
        ll.setSpacing(6)
        ll.addWidget(self._sec("이벤트 로그"))
        self.log = LogWidget()
        ll.addWidget(self.log)
        right.addWidget(log_card, stretch=1)

        main.addLayout(right, stretch=3)
        R.addLayout(main, stretch=1)

        # ── 완료 오버레이 ──
        self.overlay = DoneOverlay(root)

        # ── 타이머 ──
        self._clock = QTimer(self)
        self._clock.setInterval(1000)
        self._clock.timeout.connect(self._tick)

        node.sig_target.connect(self._on_target)
        node.sig_phase.connect(self._on_phase)
        node.sig_log.connect(self._on_phase)   # /rosout maze_tour 로그 → current action
        node.sig_color.connect(self._on_color)
        node.sig_digit.connect(self._on_digit)
        node.sig_signal.connect(self._on_signal)
        node.sig_done.connect(self._on_done)
        node.sig_frame.connect(lambda arr: self.cam_vision.update_pixmap(bgr_to_qpixmap(arr)))
        node.sig_map.connect(lambda arr:   self.cam_map.update_pixmap(bgr_to_qpixmap(arr)))

    # ── 헬퍼 ──────────────────────────────────────────────────
    def _lbl(self, text, pt, bold=False, color=TEXT_PRI, align=None):
        l = QLabel(text)
        l.setFont(font(pt, bold))
        l.setStyleSheet(f"color:{color};")
        if align is not None:
            l.setAlignment(align)
        self._fonts.append((l, pt, bold))
        return l

    def _sec(self, text):
        l = QLabel(text.upper())
        l.setObjectName("section")
        l.setFont(font(9))
        return l

    def _card(self):
        f = QFrame()
        f.setObjectName("card")
        f.setFrameShape(QFrame.StyledPanel)
        return f

    # ── 폰트 스케일 ────────────────────────────────────────────
    def resizeEvent(self, e):
        super().resizeEvent(e)
        scale = max(0.6, min(2.0, self.height() / BASE_H))
        for w, pt, b in self._fonts:
            w.setFont(font(max(7, round(pt * scale)), b))
        for panel in (self.cam_vision, self.cam_map):
            panel.update_sec_font(font(max(7, round(9 * scale))))
        if hasattr(self, 'overlay'):
            self.overlay.scale_fonts(scale)
            cw = self.centralWidget()
            if cw:
                self.overlay.resize(cw.size())

    # ── 슬롯 ──────────────────────────────────────────────────
    def _on_target(self, color: str):
        self._target_color = color
        hex_c = COLOR_MAP.get(color, ACCENT)
        self.target_lbl.setText(color)
        self.target_lbl.setStyleSheet(f"color:{hex_c};font-weight:bold;")
        self.target_dot.setStyleSheet(f"color:{hex_c};")
        self._start_time = time.time()
        self._clock.start()
        self.status_lbl.setText("● RUNNING")
        self.status_lbl.setStyleSheet(f"color:{STATUS_WARN};")
        self.state_lbl.setText("RUNNING")
        self.state_lbl.setStyleSheet(f"color:{STATUS_WARN};")
        self.log.append(f"목표 설정: {color}", hex_c)

    def _on_phase(self, text: str):
        self.phase_lbl.setText(text)
        self.log.append(text, ACCENT)

    def _on_color(self, color: str):
        c = COLOR_MAP.get(color, TEXT_SEC)
        self.det_color.setText(color if color != "NONE" else "—")
        self.det_color.setStyleSheet(f"color:{c};")

    def _on_digit(self, digit: int):
        self.det_digit.setText(f"digit: {digit}" if digit >= 0 else "digit: —")
        if digit >= 0:
            self.log.append(f"digit 인식: {digit}", TEXT_PRI)

    def _on_signal(self, _id, cx, cov):
        self.det_cov.setText(f"cov: {cov:.2f}  cx: {cx:+.2f}")

    def _on_done(self, done: bool):
        if not done:
            return
        self._clock.stop()
        self.status_lbl.setText("● DONE")
        self.status_lbl.setStyleSheet(f"color:{STATUS_OK};")
        self.state_lbl.setText("DONE")
        self.state_lbl.setStyleSheet(f"color:{STATUS_OK};")
        elapsed = int(time.time() - self._start_time)
        m, s = divmod(elapsed, 60)
        self.elapsed_lbl.setText(f"{m}:{s:02d}" if m else f"{s}s")
        self.elapsed_lbl.setStyleSheet(f"color:{STATUS_OK};")
        self.log.append("미션 완료 ✓", STATUS_OK)
        self.overlay.show_done(self._target_color)

    def _tick(self):
        elapsed = int(time.time() - self._start_time)
        m, s = divmod(elapsed, 60)
        self.elapsed_lbl.setText(f"{m}:{s:02d}" if m else f"{s}s")


# ── 진입점 ──────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = RuntimeNode(CAM_TOPIC)

    app = QApplication(sys.argv)
    apply_dark_palette(app)

    win = RuntimeWindow(node)
    win.show()

    ros_thread = RosThread(node)
    ros_thread.start()

    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
