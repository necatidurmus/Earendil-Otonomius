#!/usr/bin/env python3
"""
5 Rastgele Nokta Nav2 Testi
Map frame XY koordinatlarını doğrudan navigate_to_pose action'a gönderir.
Engelleri dikkate alarak seçilmiş noktalar.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math, time

# ── 5 test noktası (map frame XY, metre) ───────────────────────────────────
# Kutu engeller (2×2m): (5,3),(-7,4),(10,-3),(-10,-5),(3,-8),(8,8),(-5,-12),
#                       (12,5),(-12,10),(0,10),(-3,6),(6,-12),(-14,0),(14,-8)
# Duvarlar: y=17 (K), x=17 (D)
# Tüm noktalar engel merkezlerinden ≥3m uzakta seçildi.
WAYPOINTS = [
    {"x":  4.0, "y":  6.0, "name": "WP1 (K-D +4/+6)"},
    {"x": -5.0, "y":  1.0, "name": "WP2 (B   -5/+1)"},
    {"x":  6.0, "y": -5.0, "name": "WP3 (G-D +6/-5)"},
    {"x":-10.0, "y":  7.0, "name": "WP4 (K-B -10/+7)"},
    {"x":  0.0, "y": -5.0, "name": "WP5 (G    0/-5)"},
]

class Nav5WP(Node):
    def __init__(self):
        super().__init__('nav_5wp_test')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.idx = 0
        self.results = []
        self.start_time = time.time()
        self.wp_start = 0.0

    def run(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('  5 RASTGELE NOKTA NAVİGASYON TESTİ')
        self.get_logger().info('=' * 55)
        for i, wp in enumerate(WAYPOINTS):
            self.get_logger().info(f'  [{i+1}] {wp["name"]:25s}  →  ({wp["x"]:+.1f}, {wp["y"]:+.1f}) m')
        self.get_logger().info('=' * 55)
        self.get_logger().info('Nav2 action server bekleniyor...')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 hazır değil!')
            return
        self.send_next()

    def send_next(self):
        if self.idx >= len(WAYPOINTS):
            self._print_summary()
            rclpy.shutdown()
            return
        wp = WAYPOINTS[self.idx]
        self.get_logger().info('')
        self.get_logger().info(f'[{self.idx+1}/5] {wp["name"]}  →  ({wp["x"]:+.1f}, {wp["y"]:+.1f}) m')
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.orientation.w = 1.0
        self.wp_start = time.time()
        f = self.client.send_goal_async(goal, feedback_callback=self._fb)
        f.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('  ❌ Hedef REDDEDİLDİ')
            self.results.append({'name': WAYPOINTS[self.idx]['name'], 'status': 'REJECTED', 'time': 0})
            self.idx += 1
            self.send_next()
            return
        self.get_logger().info('  ✔  Hedef kabul edildi, gidiliyor...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _fb(self, fb_msg):
        fb = fb_msg.feedback
        p = fb.current_pose.pose.position
        self.get_logger().info(
            f'  Konum:({p.x:+.1f},{p.y:+.1f})  Kalan:{fb.distance_remaining:.1f}m',
            throttle_duration_sec=4.0)

    def _result_cb(self, future):
        elapsed = round(time.time() - self.wp_start, 1)
        res = future.result()
        status_map = {4: 'BAŞARDI ✅', 6: 'İPTAL ⚠️', 5: 'DURDURULDU'}
        status = status_map.get(res.status, f'DURUM={res.status}')
        self.get_logger().info(f'  → {status}  ({elapsed}s)')
        self.results.append({'name': WAYPOINTS[self.idx]['name'], 'status': status, 'time': elapsed})
        self.idx += 1
        self.send_next()

    def _print_summary(self):
        total = round(time.time() - self.start_time, 1)
        self.get_logger().info('')
        self.get_logger().info('=' * 55)
        self.get_logger().info('  SONUÇLAR')
        self.get_logger().info('=' * 55)
        ok = 0
        for r in self.results:
            self.get_logger().info(f'  {r["name"]:25s}  {r["status"]:12s}  {r["time"]}s')
            if 'BAŞARDI' in r['status']:
                ok += 1
        self.get_logger().info(f'\n  TOPLAM: {ok}/{len(self.results)} başarılı  |  {total}s')
        self.get_logger().info('=' * 55)


def main():
    rclpy.init()
    node = Nav5WP()
    node.run()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

if __name__ == '__main__':
    main()
