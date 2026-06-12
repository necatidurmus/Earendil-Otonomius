#!/usr/bin/env python3
"""
Hybrid Mission Test (v0.2.1)
=============================
Hibrit GPS+SLAM faz tabanlı navigasyonu test eder.
mission_manager'ı kullanarak fazlı waypoint navigasyonu yapar.
GPS→SLAM ve SLAM→GPS mod geçişlerini doğrular.

Kullanım:
  ros2 run leo_gz_bringup test_hybrid_mission -- --mission /path/to/hybrid_waypoints.yaml
  python3 test_hybrid_mission.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tf2_ros
import time
import os
import argparse
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from mission_manager import MissionManager


class HybridMissionTest(MissionManager):
    """MissionManager'ı genişletir — ek TF ve mod geçiş doğrulamaları."""

    def __init__(self, mission_file):
        super().__init__(mission_file)
        self.mode_transitions = []
        self._last_mode = 'GPS'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _mode_cb(self, msg):
        if msg.data != self._last_mode:
            ts = time.time() - self.start_time
            self.mode_transitions.append({
                'time': round(ts, 1),
                'from': self._last_mode,
                'to': msg.data,
            })
            self.get_logger().warn(
                f'  MOD GEÇİŞİ: {self._last_mode} → {msg.data} (t={ts:.1f}s)')
            self._last_mode = msg.data
        self.current_mode = msg.data

    def _print_summary(self):
        super()._print_summary()

        log = self.get_logger()
        log.info('')
        log.info('  ── Mod Geçişleri ──')
        if self.mode_transitions:
            for t in self.mode_transitions:
                log.info(f"  t={t['time']:6.1f}s  {t['from']} → {t['to']}")
        else:
            log.info('  (mod geçişi olmadı)')

        # TF doğrulama
        log.info('')
        log.info('  ── TF Doğrulama ──')
        for parent, child in [('map', 'map_slam'), ('map_slam', 'odom'), ('map', 'base_footprint')]:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                p = t.transform.translation
                log.info(f'  {parent}→{child}: ({p.x:+.2f}, {p.y:+.2f}) ✅')
            except Exception:
                log.info(f'  {parent}→{child}: ❌')
        log.info('=' * 65)


def main():
    parser = argparse.ArgumentParser(description='Hybrid Mission Test')
    parser.add_argument('--mission', '-m', type=str, default=None)
    args, _ = parser.parse_known_args()

    if args.mission is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('leo_gz_bringup')
            args.mission = os.path.join(pkg, 'config', 'hybrid_waypoints.yaml')
        except Exception:
            args.mission = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'hybrid_waypoints.yaml')

    rclpy.init()
    node = HybridMissionTest(args.mission)
    time.sleep(2)
    node.start_mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
