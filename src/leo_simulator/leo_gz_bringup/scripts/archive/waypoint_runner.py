#!/usr/bin/env python3
"""
Waypoint Runner — Shared GPS→Map Navigation Library (v0.2.1)
=============================================================
Ortak kütüphane: GPS waypoint → map XY dönüşümü, Nav2 goal gönderme,
sonuç raporlama. test_gps_nav.py, mission_manager.py ve
gps_waypoint_nav.py tarafından kullanılır.

Kullanım:
  from waypoint_runner import WaypointRunner
  runner = WaypointRunner(node, waypoints)
  runner.precompute()
  runner.run()
"""

import math
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import FromLL


def load_waypoints_yaml(filepath):
    """YAML dosyasından waypoint listesi yükle."""
    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])
    except Exception as e:
        print(f'Waypoints dosyası okunamadı: {e}')
        return None


class WaypointRunner:
    """GPS waypoint navigasyon motoru — bir ROS 2 Node'a bağlanır."""

    def __init__(self, node: Node, waypoints: list):
        self.node = node
        self.waypoints = waypoints
        self.wp_index = 0
        self.wp_results = []
        self._map_cache = {}
        self.start_time = time.time()
        self.wp_start = 0.0

        # GPS monitoring
        self.current_gps = None
        self.gps_status = -1
        node.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)

        # Nav2 action client
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

        # /fromLL service
        self.fromll_client = node.create_client(FromLL, '/fromLL')

        # Completion callback (mission_manager kullanır)
        self.on_complete = None

    def _gps_cb(self, msg):
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)
        self.gps_status = msg.status.status

    # ── GPS → Map dönüşümü ────────────────────────────────────────────
    def _gps_to_map(self, lat, lon):
        """GPS→Map dönüşümü. Sadece precompute() içinden çaışırılmalı
        (spin aktifken çaığırılırsa deadlock oluştuğur)."""
        if not self.fromll_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error('/fromLL servisi bulunamadı!')
            return None, None
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0
        future = self.fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is not None:
            r = future.result()
            return r.map_point.x, r.map_point.y
        self.node.get_logger().error('/fromLL çağrısı başarısız!')
        return None, None

    def precompute(self):
        """Tüm GPS waypoint'lerini önceden map XY'ye çevir."""
        self.node.get_logger().info('GPS waypoint → Map XY dönüşümleri:')
        for i, wp in enumerate(self.waypoints):
            x, y = self._gps_to_map(wp['lat'], wp['lon'])
            if x is None:
                self._map_cache[i] = None
                self.node.get_logger().error(
                    f'  WP{i+1} ({wp.get("name","")}) dönüşüm başarısız!')
            else:
                self._map_cache[i] = (x, y)
                self.node.get_logger().info(
                    f'  WP{i+1}: ({wp["lat"]:.6f}, {wp["lon"]:.6f}) → ({x:+.2f}, {y:+.2f})')

    def _compute_yaw(self, idx):
        if idx + 1 >= len(self.waypoints):
            return 0.0
        curr = self._map_cache.get(idx)
        nxt = self._map_cache.get(idx + 1)
        if curr is None or nxt is None:
            return 0.0
        return math.atan2(nxt[1] - curr[1], nxt[0] - curr[0])

    # ── Navigasyon akışı ──────────────────────────────────────────────
    def wait_for_services(self, gps_timeout=30.0, nav_timeout=30.0):
        """GPS fix, /fromLL ve Nav2 action server'ı bekle."""
        log = self.node.get_logger()
        log.info('GPS fix bekleniyor...')
        while self.current_gps is None:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        lat, lon, _ = self.current_gps
        log.info(f'GPS fix: ({lat:.6f}, {lon:.6f})')

        log.info('/fromLL servisi bekleniyor...')
        if not self.fromll_client.wait_for_service(timeout_sec=gps_timeout):
            log.error('/fromLL servisi bulunamadı!')
            return False

        log.info('Nav2 action server bekleniyor...')
        if not self.nav_client.wait_for_server(timeout_sec=nav_timeout):
            log.error('Nav2 action server bulunamadı!')
            return False
        return True

    def run(self, start_index=0):
        """Waypoint navigasyonunu başlat."""
        self.wp_index = start_index
        self._send_next()

    def _send_next(self):
        if self.wp_index >= len(self.waypoints):
            self._finish()
            return

        wp = self.waypoints[self.wp_index]
        name = wp.get('name', f'WP{self.wp_index+1}')
        cached = self._map_cache.get(self.wp_index)

        if cached is None:
            self.node.get_logger().error(f'  {name}: harita koordinatı yok, atlanıyor')
            self.wp_results.append({
                'wp': self.wp_index + 1, 'name': name, 'status': 'SKIP', 'time': 0
            })
            self.wp_index += 1
            self._send_next()
            return

        x, y = cached
        yaw = self._compute_yaw(self.wp_index)
        log = self.node.get_logger()
        log.info(f'[{self.wp_index+1}/{len(self.waypoints)}] {name}')
        log.info(f'  Map: ({x:+.2f}, {y:+.2f})  Yaw: {math.degrees(yaw):.1f}°')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.wp_start = time.time()
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            name = self.waypoints[self.wp_index].get('name', '')
            self.node.get_logger().error(f'  {name}: Hedef REDDEDİLDİ!')
            self.wp_results.append({
                'wp': self.wp_index + 1, 'name': name,
                'status': 'REJECTED', 'time': 0
            })
            self.wp_index += 1
            self._send_next()
            return
        self.node.get_logger().info('  Hedef kabul edildi, navigasyon başladı...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        pos = fb.current_pose.pose.position
        remaining = fb.distance_remaining
        extra = ''
        if self.current_gps:
            lat, lon, _ = self.current_gps
            extra = f'  GPS:({lat:.5f},{lon:.5f})'
        self.node.get_logger().info(
            f'  Nav:({pos.x:+.1f},{pos.y:+.1f}) Kalan:{remaining:.1f}m{extra}',
            throttle_duration_sec=4.0)

    def _result_cb(self, future):
        result = future.result()
        elapsed = round(time.time() - self.wp_start, 1)
        name = self.waypoints[self.wp_index].get('name', '')
        status_map = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        status = status_map.get(result.status, f'UNKNOWN({result.status})')

        self.wp_results.append({
            'wp': self.wp_index + 1, 'name': name,
            'status': status, 'time': elapsed
        })

        if result.status == 4:
            self.node.get_logger().info(f'  ✅ {name} → {elapsed}s')
        else:
            self.node.get_logger().warn(f'  ⚠️ {name}: {status} ({elapsed}s)')

        self.wp_index += 1
        self._send_next()

    def _finish(self):
        self.print_results()
        if self.on_complete:
            self.on_complete(self.wp_results)
        else:
            rclpy.shutdown()

    def print_results(self):
        total = round(time.time() - self.start_time, 1)
        log = self.node.get_logger()
        log.info('')
        log.info('=' * 60)
        log.info('  GPS NAVIGATION RESULTS')
        log.info('=' * 60)
        for r in self.wp_results:
            log.info(
                f"  WP{r['wp']}: {r['status']:10s} {r.get('time','?')}s  {r.get('name','')}")
        ok = sum(1 for r in self.wp_results if r['status'] == 'SUCCEEDED')
        log.info(f'\n  Total: {ok}/{len(self.wp_results)} OK  |  {total}s')
        log.info('=' * 60)

    @property
    def success_count(self):
        return sum(1 for r in self.wp_results if r['status'] == 'SUCCEEDED')
