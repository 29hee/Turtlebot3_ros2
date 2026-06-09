#!/usr/bin/env python3
"""
gui_mapping.py — 매핑 모니터 GUI  (완전 반응형)

창 크기 변경 시 패널·글씨 모두 비율 유지.

구독:
  /explorer_phase  (std_msgs/String)
  /phase1_done     (std_msgs/Bool)
  /phase2_done     (std_msgs/Bool)
  /detected_color  (std_msgs/String)
  /detected_digit  (std_msgs/Int32)
  /color_signal    (std_msgs/Float32MultiArray)  [color_id, cx_norm, coverage]
  /camera/image_raw (sensor_msgs/Image)          — VISION 오버레이용
  /map             (nav_msgs/OccupancyGrid)      — 실시간 맵

실행:
  python3 gui_mapping.py
  python3 gui_mapping.py --sim
  python3 gui_mapping.py --cam-topic /tb3/cam/image_raw
"""
import sys, os, time, math
from collections import defaultdict

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Int32, Float32MultiArray

try:
    from nav_msgs.msg import OccupancyGrid
    _HAS_NAV = True
except ImportError:
    _HAS_NAV = False

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QScrollArea, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap

from gui_common import (
    apply_dark_palette, BASE_QSS, RosThread, font,
    BG_DARK, BG_CARD, BG_CARD2, BORDER, TEXT_PRI, TEXT_SEC,
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_MAP,
    STATUS_OK, STATUS_WARN, STATUS_ERR, STATUS_IDLE, ACCENT,
)

# ── CLI ─────────────────────────────────────────────────────────
def _arg(flag, default=None):
    for i, a in enumerate(sys.argv):
        if a == flag and i + 1 < len(sys.argv):
            return sys.argv[i + 1]
    return default

SIM_MODE  = '--sim' in sys.argv
CAM_TOPIC = _arg('--cam-topic', '/camera/image_raw_rot')
BASE_H    = 660   # 폰트 스케일 기준 높이 [px]
ROI_RATIO = 0.7   # vision_node 기본값과 동일
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
    img  = np.full((h, w, 3), 45, dtype=np.uint8)   # unknown → 어두운 회색
    img[data == 0]   = [62,  66,  72]                # free    → 중간 회색
    img[data == 100] = [210, 215, 220]               # wall    → 밝은 회색
    return np.flipud(img)                            # ROS Y축 반전


# ── ROS2 노드 ───────────────────────────────────────────────────
class MappingNode(Node, QObject):
    sig_phase   = pyqtSignal(str)
    sig_p1_done = pyqtSignal(bool)
    sig_p2_done = pyqtSignal(bool)
    sig_color   = pyqtSignal(str)
    sig_digit   = pyqtSignal(int)
    sig_signal  = pyqtSignal(float, float, float)
    sig_frame   = pyqtSignal(object)   # annotated ndarray
    sig_map     = pyqtSignal(object)   # map ndarray

    def __init__(self, cam_topic: str):
        Node.__init__(self, 'gui_mapping')
        QObject.__init__(self)
        self._color_str  = 'NONE'
        self._cx_norm    = 0.0
        self._coverage   = 0.0
        self._last_cam_t = 0.0
        self._last_map_t = 0.0

        self.create_subscription(String, '/explorer_phase', self._ph,  10)
        self.create_subscription(Bool,   '/phase1_done',    self._p1,  10)
        self.create_subscription(Bool,   '/phase2_done',    self._p2,  10)
        self.create_subscription(String, '/detected_color', self._col, 10)
        self.create_subscription(Int32,  '/detected_digit', self._dig, 10)
        self.create_subscription(Float32MultiArray, '/color_signal', self._sig, 10)
        self.create_subscription(Image, cam_topic, self._img,
                                 qos_profile_sensor_data)
        if _HAS_NAV:
            _map_qos = QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(OccupancyGrid, '/map', self._map, _map_qos)

    def _ph(self,  m): self.sig_phase.emit(m.data)
    def _p1(self,  m): self.sig_p1_done.emit(m.data)
    def _p2(self,  m): self.sig_p2_done.emit(m.data)
    def _col(self, m):
        self._color_str = m.data
        self.sig_color.emit(m.data)
    def _dig(self, m): self.sig_digit.emit(m.data)
    def _sig(self, m):
        if len(m.data) >= 3:
            self._cx_norm  = float(m.data[1])
            self._coverage = float(m.data[2])
            self.sig_signal.emit(float(m.data[0]), self._cx_norm, self._coverage)

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
        if now - self._last_map_t < 0.5:   # 2fps 충분
            return
        self._last_map_t = now
        self.sig_map.emit(occupancy_to_bgr(msg))

    def _annotate(self, frame: np.ndarray) -> np.ndarray:
        h, w  = frame.shape[:2]
        rw, rh = int(w * ROI_RATIO), int(h * ROI_RATIO)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        bgr    = DRAW_BGR.get(self._color_str, DRAW_BGR['NONE'])
        cv2.rectangle(frame, (x0, y0), (x0 + rw, y0 + rh), bgr, 2)
        if self._color_str != 'NONE':
            cx_px = int(x0 + rw / 2 + self._cx_norm * rw / 2)
            cv2.line(frame, (cx_px, y0), (cx_px, y0 + rh), bgr, 1)
        label = f"{self._color_str}  cov={self._coverage:.2f}  cx={self._cx_norm:+.2f}"
        cv2.putText(frame, label,
                    (x0 + 4, max(18, y0 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, bgr, 1, cv2.LINE_AA)
        return frame


# ── 스케일 가능한 이미지 패널 ────────────────────────────────────
class ImagePanel(QFrame):
    """픽스맵을 aspect-ratio 유지하며 꽉 채우는 카드."""
    def __init__(self, title: str):
        super().__init__()
        self.setObjectName("card")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pm = None
        self._title = title

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(3)

        self._sec = QLabel(title.upper())
        self._sec.setObjectName("section")
        lay.addWidget(self._sec)

        self._img = QLabel()
        self._img.setAlignment(Qt.AlignCenter)
        self._img.setStyleSheet("background: #000; border-radius: 4px;")
        self._img.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self._img, stretch=1)
        self._show_no_signal()

    def _show_no_signal(self):
        self._img.setText(
            f"<span style='color:{TEXT_SEC};'>NO SIGNAL</span>")
        self._img.setPixmap(QPixmap())

    def update_pixmap(self, pm: QPixmap):
        self._pm = pm
        self._redraw()

    def _redraw(self):
        if not self._pm or self._pm.isNull():
            self._show_no_signal()
            return
        w = max(1, self._img.width())
        h = max(1, self._img.height())
        self._img.setPixmap(
            self._pm.scaled(w, h, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._redraw()

    def update_sec_font(self, f):
        self._sec.setFont(f)


# ── 색 카운터 카드 ───────────────────────────────────────────────
class ColorCounter(QFrame):
    def __init__(self, color_name, hex_color):
        super().__init__()
        self.setObjectName("card")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.hex_color = hex_color

        lay = QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(4)

        top = QHBoxLayout()
        self.dot = QLabel("●")
        self.dot.setStyleSheet(f"color: {hex_color};")
        top.addWidget(self.dot)
        top.addStretch()
        lay.addLayout(top)

        self.count_lbl = QLabel("0")
        self.count_lbl.setStyleSheet(f"color: {hex_color};")
        self.count_lbl.setAlignment(Qt.AlignLeft)
        lay.addWidget(self.count_lbl)

        self.name_lbl = QLabel(color_name)
        self.name_lbl.setStyleSheet(f"color: {TEXT_SEC};")
        lay.addWidget(self.name_lbl)

        self.digit_lbl = QLabel("digits: —")
        self.digit_lbl.setStyleSheet(f"color: {TEXT_SEC};")
        lay.addWidget(self.digit_lbl)

    def update_count(self, count, digits=None):
        self.count_lbl.setText(str(count))
        self.digit_lbl.setText(
            "digits: " + ", ".join(str(d) for d in sorted(digits))
            if digits else "digits: —")


# ── 이벤트 로그 ─────────────────────────────────────────────────
class LogWidget(QScrollArea):
    def __init__(self):
        super().__init__()
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setStyleSheet(f"background: {BG_CARD}; border: none;")
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
        lbl.setFont(font(9))
        lbl.setTextFormat(Qt.RichText)
        lbl.setWordWrap(True)
        self._lay.insertWidget(self._lay.count() - 1, lbl)
        self._lines.append(lbl)
        if len(self._lines) > 80:
            self._lines.pop(0).setParent(None)
        QTimer.singleShot(40, lambda: self.verticalScrollBar().setValue(
            self.verticalScrollBar().maximum()))


# ── Phase 진행 바 ────────────────────────────────────────────────
class PhaseBar(QWidget):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self.steps = []
        labels = [("Phase 1", "SLAM 탐색"), ("Phase 2", "정면 방문"), ("완료", "저장")]
        for i, (title, sub) in enumerate(labels):
            col = QVBoxLayout()
            col.setSpacing(1)

            self.num = num = QLabel(str(i + 1))
            num.setAlignment(Qt.AlignCenter)
            num.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
            num.setStyleSheet(f"""
                background:{BG_CARD2};color:{TEXT_SEC};
                border-radius:14px;border:2px solid {BORDER};
                min-width:26px;min-height:26px;
            """)

            t = QLabel(title)
            t.setStyleSheet(f"color:{TEXT_SEC};")
            t.setAlignment(Qt.AlignCenter)

            s = QLabel(sub)
            s.setStyleSheet(f"color:{TEXT_SEC};")
            s.setAlignment(Qt.AlignCenter)

            col.addWidget(num, alignment=Qt.AlignHCenter)
            col.addWidget(t,   alignment=Qt.AlignHCenter)
            col.addWidget(s,   alignment=Qt.AlignHCenter)
            lay.addLayout(col)
            self.steps.append((num, t, s))

            if i < len(labels) - 1:
                line = QFrame()
                line.setFrameShape(QFrame.HLine)
                line.setFixedHeight(2)
                line.setStyleSheet(f"background:{BORDER};border:none;")
                lay.addWidget(line)
        self.set_step(0)

    def set_step(self, active):
        for i, (num, t, s) in enumerate(self.steps):
            if i < active:
                num.setStyleSheet(f"background:{STATUS_OK};color:{BG_DARK};"
                                  f"border-radius:14px;border:2px solid {STATUS_OK};"
                                  f"min-width:26px;min-height:26px;")
                t.setStyleSheet(f"color:{STATUS_OK};")
            elif i == active:
                num.setStyleSheet(f"background:{ACCENT};color:{BG_DARK};"
                                  f"border-radius:14px;border:2px solid {ACCENT};"
                                  f"min-width:26px;min-height:26px;")
                t.setStyleSheet(f"color:{ACCENT};")
            else:
                num.setStyleSheet(f"background:{BG_CARD2};color:{TEXT_SEC};"
                                  f"border-radius:14px;border:2px solid {BORDER};"
                                  f"min-width:26px;min-height:26px;")
                t.setStyleSheet(f"color:{TEXT_SEC};")


# ── 메인 윈도우 ─────────────────────────────────────────────────
class MappingWindow(QMainWindow):
    def __init__(self, node: MappingNode):
        super().__init__()
        self.node  = node
        self.found = defaultdict(set)
        self._fonts: list = []   # (label, base_pt, bold)

        self.setWindowTitle("Color Maze — Mapping Monitor" + ("  [SIM]" if SIM_MODE else ""))
        self.resize(1120, 680)
        self.setMinimumSize(800, 500)
        self.setStyleSheet(BASE_QSS)

        root = QWidget()
        self.setCentralWidget(root)
        R = QVBoxLayout(root)
        R.setContentsMargins(18, 18, 18, 18)
        R.setSpacing(12)

        # ── 헤더 ──────────────────────────────────────────────
        hdr = QHBoxLayout()
        title = self._lbl("Mapping Monitor", 18, bold=True, color=TEXT_PRI)
        hdr.addWidget(title)
        if SIM_MODE:
            sim_b = QLabel("  SIM  ")
            sim_b.setFont(font(9, bold=True))
            sim_b.setStyleSheet(
                f"color:{STATUS_WARN};background:{STATUS_WARN}22;"
                f"border:1px solid {STATUS_WARN};border-radius:4px;padding:2px 6px;")
            hdr.addWidget(sim_b)
        hdr.addStretch()
        self.status_dot = self._lbl("● IDLE", 10, color=STATUS_IDLE, align=Qt.AlignRight)
        hdr.addWidget(self.status_dot)
        R.addLayout(hdr)

        # ── Phase 바 ──────────────────────────────────────────
        ph_card = self._card()
        ph_lay  = QVBoxLayout(ph_card)
        ph_lay.setContentsMargins(18, 10, 18, 10)
        ph_lay.setSpacing(6)
        self.phase_bar  = PhaseBar()
        self.phase_text = self._lbl("대기 중 — 매핑을 시작하세요", 10,
                                    color=TEXT_SEC, align=Qt.AlignCenter)
        ph_lay.addWidget(self.phase_bar)
        ph_lay.addWidget(self.phase_text)
        R.addWidget(ph_card)

        # ── 메인 영역 ──────────────────────────────────────────
        main = QHBoxLayout()
        main.setSpacing(12)

        # 좌측: 색 카운터 3개 + 감지 정보
        left = QVBoxLayout()
        left.setSpacing(8)
        self.counters = {}
        for c, h in [("RED", COLOR_RED), ("GREEN", COLOR_GREEN), ("BLUE", COLOR_BLUE)]:
            w = ColorCounter(c, h)
            left.addWidget(w, stretch=1)
            self.counters[c] = w
            self._fonts.append((w.dot,       11, False))
            self._fonts.append((w.count_lbl, 26, True))
            self._fonts.append((w.name_lbl,  10, False))
            self._fonts.append((w.digit_lbl,  9, False))

        det = self._card()
        dl  = QVBoxLayout(det)
        dl.setContentsMargins(14, 10, 14, 10)
        dl.setSpacing(6)
        dl.addWidget(self._sec("현재 감지"))
        dl.addStretch()
        self.det_color = self._lbl("—", 22, bold=True, align=Qt.AlignCenter)
        self.det_digit = self._lbl("digit: —", 13, color=TEXT_SEC, align=Qt.AlignCenter)
        self.det_cov   = self._lbl("cov: —", 9,  color=TEXT_SEC, align=Qt.AlignCenter)
        dl.addWidget(self.det_color)
        dl.addWidget(self.det_digit)
        dl.addWidget(self.det_cov)
        dl.addStretch()
        left.addWidget(det, stretch=1)
        main.addLayout(left, stretch=1)

        # 중앙+우측: 카메라 패널 + 로그
        right = QVBoxLayout()
        right.setSpacing(8)

        cam_row = QHBoxLayout()
        cam_row.setSpacing(8)
        self.cam_vision = ImagePanel("Vision")
        self.cam_map    = ImagePanel("Map (SLAM)")
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

        # ── 시그널 ────────────────────────────────────────────
        node.sig_phase.connect(self._on_phase)
        node.sig_p1_done.connect(self._on_p1)
        node.sig_p2_done.connect(self._on_p2)
        node.sig_color.connect(self._on_color)
        node.sig_digit.connect(self._on_digit)
        node.sig_signal.connect(self._on_signal)
        node.sig_frame.connect(lambda arr: self.cam_vision.update_pixmap(bgr_to_qpixmap(arr)))
        node.sig_map.connect(lambda arr:   self.cam_map.update_pixmap(bgr_to_qpixmap(arr)))

        self._cur_color = "NONE"
        self._cur_digit = -1

    # ── 위젯 헬퍼 ──────────────────────────────────────────────
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
        for lbl, base_pt, bold in self._fonts:
            lbl.setFont(font(max(7, round(base_pt * scale)), bold))
        # 카메라 패널 섹션 라벨도 스케일
        for panel in (self.cam_vision, self.cam_map):
            panel.update_sec_font(font(max(7, round(9 * scale))))

    # ── 슬롯 ──────────────────────────────────────────────────
    def _on_phase(self, text):
        self.phase_text.setText(text)
        self.status_dot.setText("● RUNNING")
        self.status_dot.setStyleSheet(f"color:{STATUS_WARN};")
        self.log.append(text, ACCENT)
        if "CAPTURE" in text or "확인" in text:
            self._record_current()

    def _on_p1(self, done):
        if done:
            self.phase_bar.set_step(1)
            self.log.append("Phase 1 완료 ✓", STATUS_OK)

    def _on_p2(self, done):
        if done:
            self.phase_bar.set_step(2)
            self.status_dot.setText("● DONE")
            self.status_dot.setStyleSheet(f"color:{STATUS_OK};")
            self.log.append("Phase 2 완료 — color_landmarks.yaml 저장됨 ✓", STATUS_OK)

    def _on_color(self, color):
        self._cur_color = color
        c = COLOR_MAP.get(color, TEXT_SEC)
        self.det_color.setText(color if color != "NONE" else "—")
        self.det_color.setStyleSheet(f"color:{c};")

    def _on_digit(self, digit):
        self._cur_digit = digit
        self.det_digit.setText(f"digit: {digit}" if digit >= 0 else "digit: —")

    def _on_signal(self, _id, cx, cov):
        self.det_cov.setText(f"cov: {cov:.2f}  cx: {cx:+.2f}")

    def _record_current(self):
        c, d = self._cur_color, self._cur_digit
        if c not in ("NONE", "") and d >= 0:
            self.found[c].add(d)
            cnt = self.counters.get(c)
            if cnt:
                cnt.update_count(len(self.found[c]), self.found[c])
            self.log.append(f"기록: {c} digit={d}", COLOR_MAP.get(c, TEXT_SEC))


# ── 진입점 ──────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MappingNode(CAM_TOPIC)

    app = QApplication(sys.argv)
    apply_dark_palette(app)

    win = MappingWindow(node)
    win.show()

    ros_thread = RosThread(node)
    ros_thread.start()

    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
