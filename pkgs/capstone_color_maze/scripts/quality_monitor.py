#!/usr/bin/env python3
"""
quality_monitor.py — 매핑 중 색·숫자 맵 품질을 라이브로 점검·출력.

color_landmarks.yaml 을 주기적으로 읽어 색별 '진짜 벽'(resolve_target_walls) 개수와
각 벽의 digit/votes 를 요약하고, 비었거나 digit 미상인 항목을 ⚠ 로 경고한다.
→ 매핑을 끝내기 전에 '아직 못 잡은 색 / 숫자 못 읽은 패널' 을 눈으로 확인하고 그 구역을
   다시 돌지 판단한다(같은 자리 빙빙/누락 방지의 마지막 안전망). /explorer_phase 로 탐사기
   현재 국면도 함께 표시.

실행:
  python3 quality_monitor.py
  # 기대 개수 기본 = 색당 3개(총 9). 다르면:  -p expect:="RED:3,GREEN:3,BLUE:3"
"""
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import yaml

from maze_common import VALID_COLORS, resolve_target_walls


def default_landmarks_path():
    here = os.path.dirname(os.path.realpath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'color_landmarks.yaml')


def parse_expect(s):
    """'RED:3,GREEN:1' → {'RED':3,'GREEN':1}. 비면 {}."""
    out = {}
    for tok in str(s or '').split(','):
        tok = tok.strip()
        if ':' not in tok:
            continue
        c, n = tok.split(':', 1)
        c = c.strip().upper()
        try:
            out[c] = int(n)
        except ValueError:
            pass
    return out


class QualityMonitor(Node):
    def __init__(self):
        super().__init__('quality_monitor')
        self.declare_parameter('landmarks_path', default_landmarks_path())
        self.declare_parameter('period', 4.0)
        # 미션 사양: 색당 3개 × 3색 = 총 9개 패널. 기본을 9개 기준으로 둔다.
        self.declare_parameter('expect', 'RED:3,GREEN:3,BLUE:3')

        self.path = self.get_parameter('landmarks_path').value
        self.period = float(self.get_parameter('period').value)
        self.expect = parse_expect(self.get_parameter('expect').value)

        self.phase = '-'
        self.create_subscription(String, '/explorer_phase', self._on_phase, 10)
        self.create_timer(self.period, self.report)
        self.get_logger().info(f"quality_monitor 시작 — {self.path} ({self.period:.0f}s 주기)")

    def _on_phase(self, msg):
        self.phase = msg.data

    def report(self):
        try:
            with open(self.path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().info(f"[품질] 아직 맵 파일 없음(곧 생성): {self.path}")
            return
        except Exception as e:
            self.get_logger().warn(f"[품질] 읽기 실패: {e}")
            return

        lines = [f"━━ 매핑 품질 체크 (탐사 국면: {self.phase}) ━━"]
        total_found, total_exp, short = 0, 0, False
        for color in VALID_COLORS:
            walls = resolve_target_walls(data, color)   # yaml 은 색+숫자 둘 다인 칸만
            total_found += len(walls)
            exp = self.expect.get(color)
            head = f"{color:5s}: {len(walls)}개"
            if exp is not None:
                total_exp += exp
                head += f"/{exp} 기대"
                if len(walls) < exp:
                    head += " ⚠ 부족"
                    short = True
            if not walls:
                head += "  ⚠ 미발견"
            lines.append(head)
            # 같은 색 내 digit 중복(같은 번호 두 번 잡힘)도 품질 문제 → 표시
            seen_digits = [w.get('digit') for w in walls]
            for w in walls:
                digit = w.get('digit')
                dtxt = f"digit={digit}" if digit is not None else "digit=? ⚠ 숫자미상"
                if digit is not None and seen_digits.count(digit) > 1:
                    dtxt += " ⚠ 중복"
                lines.append(
                    f"    #{w['id']} ({w['x']:+.2f},{w['y']:+.2f}) votes={int(w['votes'])} {dtxt}")
        if total_exp:
            ok = (not short) and total_found >= total_exp
            verdict = '✅ 전부 확보 — 맵 저장 OK' if ok else '⚠ 미완 — 더 돌 것(저장 보류)'
            lines.append(f"합계: {total_found}/{total_exp} 패널(색+숫자 둘 다) {verdict}")
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = QualityMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
