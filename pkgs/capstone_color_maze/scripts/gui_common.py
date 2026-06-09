"""
gui_common.py — 공통 스타일 / ROS2 브릿지 / 위젯 헬퍼
"""
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QColor, QPalette
from PyQt5.QtWidgets import QApplication

# ── 색상 팔레트 ─────────────────────────────────────────────────
BG_DARK    = "#0d1117"
BG_CARD    = "#161b22"
BG_CARD2   = "#1c2128"
BORDER     = "#30363d"
TEXT_PRI   = "#e6edf3"
TEXT_SEC   = "#8b949e"
ACCENT     = "#58a6ff"

COLOR_RED   = "#f85149"
COLOR_GREEN = "#3fb950"
COLOR_BLUE  = "#58a6ff"
COLOR_MAP   = {"RED": COLOR_RED, "GREEN": COLOR_GREEN, "BLUE": COLOR_BLUE}

STATUS_OK   = "#3fb950"
STATUS_WARN = "#d29922"
STATUS_ERR  = "#f85149"
STATUS_IDLE = "#8b949e"

# ── 글꼴 ────────────────────────────────────────────────────────
FONT_FAMILY = "Segoe UI, Arial, sans-serif"

def font(size=11, bold=False):
    f = QFont("Segoe UI", size)
    f.setBold(bold)
    return f

# ── 앱 전역 다크 팔레트 ─────────────────────────────────────────
def apply_dark_palette(app: QApplication):
    app.setStyle("Fusion")
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(BG_DARK))
    pal.setColor(QPalette.WindowText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Base,            QColor(BG_CARD))
    pal.setColor(QPalette.AlternateBase,   QColor(BG_CARD2))
    pal.setColor(QPalette.Text,            QColor(TEXT_PRI))
    pal.setColor(QPalette.ButtonText,      QColor(TEXT_PRI))
    pal.setColor(QPalette.Button,          QColor(BG_CARD2))
    pal.setColor(QPalette.Highlight,       QColor(ACCENT))
    pal.setColor(QPalette.HighlightedText, QColor(BG_DARK))
    app.setPalette(pal)

# ── 공통 QSS ────────────────────────────────────────────────────
BASE_QSS = f"""
QWidget {{
    background: {BG_DARK};
    color: {TEXT_PRI};
    font-family: 'Segoe UI', Arial, sans-serif;
}}
QFrame#card {{
    background: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#section {{
    color: {TEXT_SEC};
    font-size: 10px;
    letter-spacing: 1px;
}}
QScrollBar:vertical {{
    background: {BG_CARD};
    width: 6px;
    border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: {BORDER};
    border-radius: 3px;
    min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
"""

# ── ROS2 수신 스레드 ────────────────────────────────────────────
class RosThread(QThread):
    """백그라운드에서 rclpy.spin 을 돌리고 신호로 Qt 에 전달."""
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        import rclpy
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
