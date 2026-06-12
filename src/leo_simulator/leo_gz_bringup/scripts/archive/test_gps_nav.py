#!/usr/bin/env python3
"""
GPS-only Navigation Test (v0.2.1)
==================================
v0.1 GPS-only navigasyonu test eder (regresyon testi).
waypoint_runner kütüphanesini kullanır.
4 GPS waypoint'i sırayla gezer.

Kullanım:
  ros2 run leo_gz_bringup test_gps_nav -- --waypoints /path/to/waypoints.yaml
  python3 test_gps_nav.py
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
import math
import time
import yaml
import os
import argparse
import threading
import sys

# waypoint_runner aynı dizinde
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from waypoint_runner import WaypointRunner, load_waypoints_yaml


DEFAULT_WAYPOINTS = [
    {"lat": 39.925063, "lon": 32.836991, "name": "WP1: KD (+3,+5) uzk=3.0m"},
    {"lat": 39.925117, "lon": 32.836886, "name": "WP2: KB (-6,+11) uzk=3.0m"},
    {"lat": 39.925027, "lon": 32.837120, "name": "WP3: D (+14,+1) uzk=2.9m"},
    {"lat": 39.924973, "lon": 32.836921, "name": "WP4: GB (-3,-5) uzk=3.9m"},
]


class GPSNavTest(Node):
    def __init__(self, waypoints):
        super().__init__('test_gps_nav')

        # UKF diagnostik izleme
        self._counts = {
            'odom_local': 0, 'odom_filtered': 0, 'odom_gps': 0,
            'imu': 0, 'navsat': 0, 'odom_raw': 0,
        }
        self._lock = threading.Lock()
        self._start = time.time()

        self.create_subscription(Odometry, '/odometry/local', self._cnt('odom_local'), 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._cnt('odom_filtered'), 10)
        self.create_subscription(Odometry, '/odometry/gps', self._cnt('odom_gps'), 10)
        self.create_subscription(Odometry, '/odom', self._cnt('odom_raw'), 10)
        self.create_subscription(Imu, '/imu/data_raw', self._cnt('imu'), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.runner = WaypointRunner(self, waypoints)

    def _cnt(self, key):
        def cb(msg):
            with self._lock:
                self._counts[key] += 1
        return cb

    def verify_pipeline(self):
        """Dual-UKF pipeline doğrulama."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('  DUAL-UKF PIPELINE DOĞRULAMA')
        self.get_logger().info('=' * 60)

        self.get_logger().info('  Sensör verileri bekleniyor (15s)...')
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            with self._lock:
                if (self._counts['odom_local'] > 0 and
                    self._counts['odom_filtered'] > 0):
                    break

        elapsed = time.time() - self._start
        checks = 0
        with self._lock:
            topics = [
                ('/odom', 'odom_raw'), ('/imu/data_raw', 'imu'),
                ('/odometry/local', 'odom_local'), ('/odometry/gps', 'odom_gps'),
                ('/odometry/filtered', 'odom_filtered'),
            ]
            for label, key in topics:
                cnt = self._counts[key]
                rate = cnt / elapsed if elapsed > 0 else 0
                s = '✅' if cnt > 0 else '❌'
                self.get_logger().info(f'  {label:25s} {s} {cnt} msg ({rate:.1f} Hz)')
                if cnt > 0:
                    checks += 1

        # TF
        for parent, child in [('odom', 'base_footprint'), ('map', 'odom')]:
            try:
                self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.get_logger().info(f'  {parent}→{child:15s} ✅')
                checks += 1
            except Exception:
                self.get_logger().warn(f'  {parent}→{child:15s} ❌')

        ok = checks >= 5
        self.get_logger().info(f'  Sonuç: {"✅ BAŞARILI" if ok else "⚠️ EKSİK"} ({checks}/7)')
        self.get_logger().info('=' * 60)
        return ok

    def run_test(self):
        if not self.runner.wait_for_services():
            return
        self.runner.precompute()
        self.runner.run()


def main():
    parser = argparse.ArgumentParser(description='GPS Navigation Test')
    parser.add_argument('--waypoints', '-w', type=str, default=None)
    parser.add_argument('--skip-verify', action='store_true')
    args, _ = parser.parse_known_args()

    waypoints = None
    if args.waypoints:
        waypoints = load_waypoints_yaml(args.waypoints)

    if not waypoints:
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('leo_gz_bringup')
            candidates.append(os.path.join(pkg, 'config', 'waypoints.yaml'))
        except Exception:
            pass
        candidates.append(os.path.join(
            os.path.dirname(__file__), '..', 'config', 'waypoints.yaml'))
        for c in candidates:
            if os.path.isfile(c):
                waypoints = load_waypoints_yaml(c)
                if waypoints:
                    break

    if not waypoints:
        waypoints = DEFAULT_WAYPOINTS

    rclpy.init()
    node = GPSNavTest(waypoints)
    time.sleep(2)

    if not args.skip_verify:
        node.verify_pipeline()

    node.run_test()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
