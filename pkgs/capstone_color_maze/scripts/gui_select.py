#!/usr/bin/env python3
"""
gui_select.py — 색상 선택 화면  (완전 반응형)

color_landmarks.yaml 에서 확인된 색+digit 목록을 표시하고
사용자가 원하는 색을 선택하면 /target_color (std_msgs/String) 발행.

실행:
  python3 gui_select.py
"""
import sys
import os
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QPushButton, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject

from gui_common import (
    apply_dark_palette, BASE_QSS, RosThread, font,
    BG_DARK, BG_CARD, BG_CARD2, BORDER, TEXT_PRI, TEXT_SEC,
    COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_MAP,
    STATUS_OK, STATUS_WARN, ACCENT,
)

BASE_H = 560   # 폰트 스케일 기준 높이


def default_landmarks_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


def load_landmarks(path):
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        result = {}
        for entry in data.get('landmarks', []):
            c = entry.get('color', '').upper()
            if c:
                result.setdefault(c, []).append(entry)
        return result
    except Exception:
        return {}


# ── ROS2 노드 ───────────────────────────────────────────────────
class SelectNode(Node, QObject):
    sig_selected = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'gui_select')
        QObject.__init__(self)
        self.pub = self.create_publisher(String, '/target_color', 10)

    def publish_color(self, color: str):
        self.pub.publish(String(data=color))
        self.sig_selected.emit(color)


# ── 색상 선택 카드 ──────────────────────────────────────────────
class ColorCard(QFrame):
    clicked = pyqtSignal(str)

    COLORS = {'RED': COLOR_RED, 'GREEN': COLOR_GREEN, 'BLUE': COLOR_BLUE}

    def __init__(self, color_name: str, landmarks: list):
        super().__init__()
        self.color_name = color_name
        self.hex_color  = self.COLORS.get(color_name, TEXT_SEC)
        self._selected  = False
        self.setObjectName("card")
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(20, 22, 20, 22)
        lay.setSpacing(10)

        self.dot = QLabel("●")
        self.dot.setStyleSheet(f"color: {self.hex_color};")
        self.dot.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.dot)

        self.name_lbl = QLabel(color_name)
        self.name_lbl.setStyleSheet(f"color: {self.hex_color};")
        self.name_lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.name_lbl)

        digits = sorted({e.get('digit') for e in landmarks
                         if e.get('digit') is not None})
        digit_text = "digit  " + "  ".join(str(d) for d in digits) if digits else "digit  —"
        self.digit_lbl = QLabel(digit_text)
        self.digit_lbl.setStyleSheet(
            f"color: {TEXT_PRI};" if digits else f"color: {TEXT_SEC};")
        self.digit_lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.digit_lbl)

        self.count_lbl = QLabel(
            f"벽 {len(landmarks)}개 확인됨" if landmarks else "미확인")
        self.count_lbl.setStyleSheet(
            f"color: {STATUS_OK};" if landmarks else f"color: {STATUS_WARN};")
        self.count_lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.count_lbl)

        lay.addStretch()

        self.btn = QPushButton("선택")
        self.btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self._set_btn_style(False)
        self.btn.clicked.connect(lambda: self.clicked.emit(self.color_name))
        lay.addWidget(self.btn)

    def scalable(self):
        """창 스케일러에 등록할 (위젯, base_pt, bold) 목록."""
        return [
            (self.dot,       36, False),
            (self.name_lbl,  22, True),
            (self.digit_lbl, 14, True),
            (self.count_lbl, 10, False),
            (self.btn,       13, True),
        ]

    def _set_btn_style(self, selected: bool):
        base = (f"background:{self.hex_color};color:{BG_DARK};border:none;"
                if selected else
                f"background:{BG_CARD2};color:{self.hex_color};"
                f"border:2px solid {self.hex_color};")
        hover = (f"background:{self.hex_color}dd;" if selected
                 else f"background:{self.hex_color}22;")
        self.btn.setStyleSheet(
            f"QPushButton{{{base}border-radius:8px;font-weight:bold;}}"
            f"QPushButton:hover{{{hover}}}")

    def set_selected(self, selected: bool):
        self._selected = selected
        self._set_btn_style(selected)
        if selected:
            self.setStyleSheet(
                f"QFrame#card{{background:{self.hex_color}18;"
                f"border:2px solid {self.hex_color};border-radius:8px;}}")
        else:
            self.setStyleSheet("")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit(self.color_name)
        super().mousePressEvent(event)


# ── 메인 윈도우 ─────────────────────────────────────────────────
class SelectWindow(QMainWindow):
    def __init__(self, node: SelectNode, landmarks_path: str = None):
        super().__init__()
        self.node      = node
        self._selected = None
        self._fonts: list = []   # (widget, base_pt, bold)

        path = landmarks_path or default_landmarks_path()
        self.landmarks = load_landmarks(path)

        self.setWindowTitle("Color Maze — 색상 선택")
        self.resize(860, 560)
        self.setMinimumSize(620, 400)
        self.setStyleSheet(BASE_QSS)

        root = QWidget()
        self.setCentralWidget(root)
        R = QVBoxLayout(root)
        R.setContentsMargins(28, 28, 28, 28)
        R.setSpacing(16)

        # ── 헤더 ──
        hdr = QHBoxLayout()
        self.title_lbl = self._lbl("색상 선택", 20, bold=True, color=TEXT_PRI)
        hdr.addWidget(self.title_lbl)
        hdr.addStretch()
        self.status_lbl = self._lbl("목표 색을 선택하세요", 10,
                                    color=TEXT_SEC, align=Qt.AlignRight | Qt.AlignVCenter)
        hdr.addWidget(self.status_lbl)
        R.addLayout(hdr)

        self.sub_lbl = self._lbl("선택한 색의 벽으로 로봇이 이동합니다",
                                 11, color=TEXT_SEC)
        R.addWidget(self.sub_lbl)

        # ── 색 카드 3열 ──
        card_row = QHBoxLayout()
        card_row.setSpacing(14)
        self.cards = {}
        for color in ('RED', 'GREEN', 'BLUE'):
            c = ColorCard(color, self.landmarks.get(color, []))
            c.clicked.connect(self._on_select)
            card_row.addWidget(c)
            self.cards[color] = c
            self._fonts.extend(c.scalable())
        R.addLayout(card_row, stretch=1)

        # ── 출발 버튼 ──
        self.confirm_btn = QPushButton("출발 →")
        self.confirm_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.confirm_btn.setEnabled(False)
        self._set_confirm_idle()
        self.confirm_btn.clicked.connect(self._on_confirm)
        self._fonts.append((self.confirm_btn, 14, True))
        R.addWidget(self.confirm_btn)

        node.sig_selected.connect(self._on_published)

    # ── 위젯 헬퍼 ──────────────────────────────────────────────
    def _lbl(self, text, pt, bold=False, color=TEXT_PRI, align=None):
        l = QLabel(text)
        l.setFont(font(pt, bold))
        l.setStyleSheet(f"color:{color};")
        if align is not None:
            l.setAlignment(align)
        self._fonts.append((l, pt, bold))
        return l

    def _set_confirm_idle(self):
        self.confirm_btn.setStyleSheet(
            f"QPushButton{{background:{BG_CARD2};color:{TEXT_SEC};"
            f"border:1px solid {BORDER};border-radius:10px;font-weight:bold;}}")

    # ── 폰트 스케일 ────────────────────────────────────────────
    def resizeEvent(self, e):
        super().resizeEvent(e)
        scale = max(0.6, min(2.0, self.height() / BASE_H))
        for w, base_pt, bold in self._fonts:
            w.setFont(font(max(7, round(base_pt * scale)), bold))
        # 버튼 최소 높이도 비례
        btn_h = max(36, round(52 * scale))
        self.confirm_btn.setMinimumHeight(btn_h)
        for card in self.cards.values():
            card.btn.setMinimumHeight(max(32, round(44 * scale)))

    # ── 슬롯 ──────────────────────────────────────────────────
    def _on_select(self, color: str):
        for c, card in self.cards.items():
            card.set_selected(c == color)
        self._selected = color
        self.status_lbl.setText(f"선택됨: {color}")
        self.status_lbl.setStyleSheet(f"color:{TEXT_PRI};")
        self.confirm_btn.setEnabled(True)
        hex_c = COLOR_MAP.get(color, ACCENT)
        self.confirm_btn.setStyleSheet(
            f"QPushButton{{background:{hex_c};color:{BG_DARK};border:none;"
            f"border-radius:10px;font-weight:bold;}}"
            f"QPushButton:hover{{background:{hex_c}dd;}}")

    def _on_confirm(self):
        if self._selected:
            self.node.publish_color(self._selected)

    def _on_published(self, color: str):
        self.status_lbl.setText(f"● {color} 전송 완료")
        self.status_lbl.setStyleSheet(f"color:{STATUS_OK};")


# ── 진입점 ──────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SelectNode()

    app = QApplication(sys.argv)
    apply_dark_palette(app)

    win = SelectWindow(node)
    win.show()

    ros_thread = RosThread(node)
    ros_thread.start()

    ret = app.exec_()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
