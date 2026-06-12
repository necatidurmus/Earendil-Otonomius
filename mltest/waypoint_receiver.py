#!/usr/bin/env python3
"""
Waypoint Receiver — MATLAB'dan gelen waypoint'leri Nav2'ye iletir
=================================================================

MATLAB GUI'si bu node'a waypoint publish eder.
Node, waypoint'leri sırayla Nav2 navigate_to_pose action'ına gönderir.

Topic'ler:
  - /ml_waypoint (geometry_msgs/PoseStamped)  → Tek waypoint
  - /ml_mission   (std_msgs/String)           → JSON formatında waypoint listesi
  - /ml_control   (std_msgs/String)           → "start", "stop", "reset" komutları

Publish:
  - /goal_pose (geometry_msgs/PoseStamped)    → Nav2'ye goal
  - /ml_status (std_msgs/String)              → Durum raporu

Kullanım (Docker içinde):
  python3 waypoint_receiver.py [--frame map|odom]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import time
import argparse


class WaypointReceiver(Node):
    def __init__(self, frame='map'):
        super().__init__('waypoint_receiver',
                         parameter_overrides=[
                             rclpy.parameter.Parameter(
                                 'use_sim_time',
                                 rclpy.parameter.Parameter.Type.BOOL, True)
                         ])

        self.frame = frame
        self.waypoints = []
        self.current_idx = 0
        self.is_running = False
        self.start_time = 0.0
        self.results = []

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.wp_sub = self.create_subscription(
            PoseStamped, '/ml_waypoint', self._on_waypoint, 10)
        self.mission_sub = self.create_subscription(
            String, '/ml_mission', self._on_mission, 10)
        self.control_sub = self.create_subscription(
            String, '/ml_control', self._on_control, 10)

        # Publisher — durum raporu
        self.status_pub = self.create_publisher(String, '/ml_status', 10)

        # Timer — status publish
        self.status_timer = self.create_timer(2.0, self._publish_status)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  WAYPOINT RECEIVER')
        self.get_logger().info(f'  Frame: {self.frame}')
        self.get_logger().info(f'  /ml_waypoint  → tek waypoint')
        self.get_logger().info(f'  /ml_mission   → JSON waypoint listesi')
        self.get_logger().info(f'  /ml_control   → start/stop/reset')
        self.get_logger().info('=' * 50)

    def _on_waypoint(self, msg):
        """Tek waypoint geldi — listeye ekle ve hemen gönder."""
        wp = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'yaw': self._quat_to_yaw(msg.pose.orientation)
        }
        self.waypoints.append(wp)
        idx = len(self.waypoints)
        self.get_logger().info(
            f'WP{idx} eklendi: ({wp["x"]:.2f}, {wp["y"]:.2f}) '
            f'yaw={wp["yaw"]:.1f}°')

        # Eğer çalışmıyorsa otomatik başlat
        if not self.is_running:
            self._start_navigation()

    def _on_mission(self, msg):
        """JSON formatında waypoint listesi geldi.

        Format:
          [{"x": 1.0, "y": 2.0, "yaw": 0.0}, ...]
        veya:
          [{"x": 1.0, "y": 2.0}, ...]
        """
        try:
            wps = json.loads(msg.data)
            self.waypoints = []
            for wp in wps:
                self.waypoints.append({
                    'x': float(wp.get('x', 0.0)),
                    'y': float(wp.get('y', 0.0)),
                    'yaw': float(wp.get('yaw', 0.0))
                })
            self.get_logger().info(
                f'{len(self.waypoints)} waypoint yüklendi')
            self._start_navigation()
        except Exception as e:
            self.get_logger().error(f'JSON parse hatası: {e}')

    def _on_control(self, msg):
        """Kontrol komutu geldi."""
        cmd = msg.data.strip().upper()
        if cmd == 'START':
            self._start_navigation()
        elif cmd == 'STOP':
            self.is_running = False
            self.get_logger().warn('Navigasyon durduruldu')
        elif cmd == 'RESET':
            self.waypoints = []
            self.current_idx = 0
            self.is_running = False
            self.results = []
            self.get_logger().info('Sıfırlandı')
        else:
            self.get_logger().warn(f'Bilinmeyen komut: {cmd}')

    def _start_navigation(self):
        """Waypoint listesini sırayla göndermeye başla."""
        if not self.waypoints:
            self.get_logger().warn('Waypoint listesi boş!')
            return

        if self.is_running:
            self.get_logger().info('Zaten çalışıyor')
            return

        self.current_idx = 0
        self.is_running = True
        self.start_time = time.time()
        self.results = []
        self.get_logger().info(
            f'Navigasyon başladı: {len(self.waypoints)} waypoint')
        self._send_next()

    def _send_next(self):
        """Sıradaki waypoint'i Nav2'ye gönder."""
        if not self.is_running:
            return

        if self.current_idx >= len(self.waypoints):
            self._finish()
            return

        wp = self.waypoints[self.current_idx]
        idx = self.current_idx + 1
        total = len(self.waypoints)

        self.get_logger().info(
            f'  [{idx}/{total}] ({wp["x"]:.2f}, {wp["y"]:.2f}) '
            f'yaw={wp["yaw"]:.1f}°')

        # Nav2'ye goal gönder
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadı!')
            self.is_running = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp['x']
        goal.pose.pose.position.y = wp['y']
        goal.pose.pose.position.z = 0.0

        yaw_rad = wp['yaw'] * 3.14159265 / 180.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
        goal.pose.pose.orientation.z = 0.5 * (2.0 * 0.0 * 0.0 + 0.0)  # placeholder
        # Quaternion from yaw
        import math
        goal.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self._wp_start = time.time()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(
                f'WP{self.current_idx + 1} REDDEDİLDİ!')
            self.results.append({
                'wp': self.current_idx + 1,
                'status': 'REJECTED',
                'time': 0
            })
            self.current_idx += 1
            self._send_next()
            return

        self.get_logger().info(f'WP{self.current_idx + 1} kabul edildi')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        elapsed = round(time.time() - self._wp_start, 1)
        status_map = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        status = status_map.get(result.status, f'UNKNOWN({result.status})')

        self.results.append({
            'wp': self.current_idx + 1,
            'status': status,
            'time': elapsed
        })

        if result.status == 4:
            self.get_logger().info(
                f'  ✅ WP{self.current_idx + 1} tamamlandı ({elapsed}s)')
        else:
            self.get_logger().warn(
                f'  ⚠️ WP{self.current_idx + 1}: {status} ({elapsed}s)')

        self.current_idx += 1
        self._send_next()

    def _finish(self):
        """Tüm waypoint'ler tamamlandı."""
        total_time = round(time.time() - self.start_time, 1)
        ok = sum(1 for r in self.results if r['status'] == 'SUCCEEDED')

        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('  SONUÇ')
        self.get_logger().info('=' * 50)
        for r in self.results:
            self.get_logger().info(
                f"  WP{r['wp']}: {r['status']} ({r['time']}s)")
        self.get_logger().info(
            f'\n  Toplam: {ok}/{len(self.results)} başarılı | {total_time}s')
        self.get_logger().info('=' * 50)

        self.is_running = False
        self._publish_result(ok, len(self.results), total_time)

    def _publish_status(self):
        """Durum raporu publish et."""
        msg = String()
        if self.is_running:
            msg.data = json.dumps({
                'state': 'running',
                'current_wp': self.current_idx + 1,
                'total_wps': len(self.waypoints),
                'elapsed': round(time.time() - self.start_time, 1)
            })
        else:
            msg.data = json.dumps({
                'state': 'idle',
                'waypoints_queued': len(self.waypoints) - self.current_idx
            })
        self.status_pub.publish(msg)

    def _publish_result(self, ok, total, elapsed):
        """Sonuç raporu publish et."""
        msg = String()
        msg.data = json.dumps({
            'state': 'completed',
            'succeeded': ok,
            'total': total,
            'elapsed': elapsed
        })
        self.status_pub.publish(msg)

    @staticmethod
    def _quat_to_yaw(q):
        """Quaternion'dan yaw (derece) hesapla."""
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def main():
    parser = argparse.ArgumentParser(description='Waypoint Receiver')
    parser.add_argument('--frame', type=str, default='map',
                        help='Frame ID (map veya odom)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = WaypointReceiver(frame=args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
