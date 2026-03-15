#!/usr/bin/env python3
"""
Tunnel GPS Spoofer
==================
Robot tünel bölgesine girince /navsat mesajını otomatik olarak
STATUS_NO_FIX'e çevirir — Gazebo'nun GPS'i kesememesini telafi eder.

Bu node bir proxy gibi çalışır:
  /navsat  (Gazebo bridge'den gerçek GPS) → işle → /navsat_filtered (navsat_transform)

Robot pozisyonunu TF'den okur (map → base_footprint):
  Tünel bölgesi  (x: 20-30, |y| < 1.35) → STATUS_NO_FIX  yayınla
  Kapı bölgesi   (x: 34-36, |y| < 1.0)  → Zayıf GPS yayınla
  Dışarısı                               → orijinal GPS'i geçir

Kullanım:
  navigation_hybrid.launch.py tarafından otomatik başlatılır.
  Hybrid modda navsat_transform /navsat_filtered'a subscribe olur.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import tf2_ros


# ── GPS kaybı simülasyonu için kovaryans değerleri ──────────────────────
COVARIANCE_NO_FIX    = [9999.0] + [0.0]*8
COVARIANCE_BAD_FIX   = [25.0, 0, 0, 0, 25.0, 0, 0, 0, 100.0]


class TunnelGPSSpoofer(Node):
    def __init__(self):
        super().__init__('tunnel_gps_spoofer')

        # ── Parametreler ─────────────────────────────────────────────────────
        self.declare_parameter('tunnel_x_min', 19.5)
        self.declare_parameter('tunnel_x_max', 30.5)
        self.declare_parameter('tunnel_y_max', 1.35)
        self.declare_parameter('door_x_min',   33.5)
        self.declare_parameter('door_x_max',   36.5)
        self.declare_parameter('door_y_max',   1.0)
        self.declare_parameter('verbose',      True)
        self.declare_parameter('base_frame',   'base_footprint')
        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('control_topic', '/gps_spoofer/control')
        self.declare_parameter('state_topic',   '/gps_spoofer/state')

        self.t_xmin  = self.get_parameter('tunnel_x_min').value
        self.t_xmax  = self.get_parameter('tunnel_x_max').value
        self.t_ymax  = self.get_parameter('tunnel_y_max').value
        self.d_xmin  = self.get_parameter('door_x_min').value
        self.d_xmax  = self.get_parameter('door_x_max').value
        self.d_ymax  = self.get_parameter('door_y_max').value
        self.verbose = self.get_parameter('verbose').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value
        self.control_topic = self.get_parameter('control_topic').value
        self.state_topic = self.get_parameter('state_topic').value

        # ── Durum ────────────────────────────────────────────────────────────
        self.robot_x  = 0.0
        self.robot_y  = 0.0
        self.last_zone = 'NORMAL'
        self.override_mode = 'AUTO'
        self.last_effective_mode = 'NORMAL'
        self.tf_available = False

        # ── TF listener (odom yerine — drift yok) ────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Abonelikler ──────────────────────────────────────────────────────
        self.create_subscription(
            NavSatFix, '/navsat', self._gps_cb, 10)
        self.create_subscription(
            String, self.control_topic, self._control_cb, 10)

        # ── Yayıncı ──────────────────────────────────────────────────────────
        self.gps_pub = self.create_publisher(NavSatFix, '/navsat_filtered', 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        # ── Periyodik TF okuma (10 Hz) ────────────────────────────────────────
        self.create_timer(0.1, self._update_robot_pose)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  TUNNEL GPS SPOOFER aktif')
        self.get_logger().info(f'  Tunel: x=[{self.t_xmin},{self.t_xmax}], |y|<={self.t_ymax}')
        self.get_logger().info(f'  Kapi : x=[{self.d_xmin},{self.d_xmax}], |y|<={self.d_ymax}')
        self.get_logger().info(f'  TF: {self.map_frame} -> {self.base_frame}')
        self.get_logger().info('  /navsat -> [spoof] -> /navsat_filtered')
        self.get_logger().info(f'  Kontrol: {self.control_topic} (AUTO/GPS_ON/GPS_OFF)')
        self.get_logger().info(f'  Durum  : {self.state_topic}')
        self.get_logger().info('=' * 55)
        self._publish_state('init')

    def _publish_state(self, reason: str = ''):
        msg = String()
        msg.data = (
            f'override={self.override_mode};zone={self.last_zone};'
            f'effective={self.last_effective_mode};x={self.robot_x:.2f};y={self.robot_y:.2f};reason={reason}'
        )
        self.state_pub.publish(msg)

    def _control_cb(self, msg: String):
        requested = msg.data.strip().upper()
        aliases = {
            'AUTO': 'AUTO',
            'GPS_ON': 'GPS_ON',
            'ON': 'GPS_ON',
            'GPS_OFF': 'GPS_OFF',
            'OFF': 'GPS_OFF',
            'SLAM': 'GPS_OFF',
        }
        if requested not in aliases:
            self.get_logger().warn(f'Gecersiz override: {msg.data!r}')
            return

        new_mode = aliases[requested]
        if new_mode != self.override_mode:
            self.get_logger().warn(f'GPS override degisti: {self.override_mode} -> {new_mode}')
            self.override_mode = new_mode
            self._publish_state('override_change')

    # ── TF'den robot konumunu oku ─────────────────────────────────────────────
    # tf_mode_relay v0.3 map→odom'u startup'ta identity olarak self-bootstrap
    # yapıyor. Bu sayede spoofer güvenle map→base_footprint kullanabilir;
    # bölge koordinatları da map_gps dünya referansıyla uyumlu kalır.
    def _update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            if not self.tf_available:
                self.tf_available = True
                self.get_logger().info(
                    f'TF hazir: {self.map_frame}->{self.base_frame} '
                    f'pos=({self.robot_x:.1f}, {self.robot_y:.1f})')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    # ── GPS callback: bölgeye göre mesajı modifiye et ────────────────────────
    def _gps_cb(self, msg: NavSatFix):
        zone = self._get_zone(self.robot_x, self.robot_y)
        effective_zone = zone
        if self.override_mode == 'GPS_OFF':
            effective_zone = 'FORCED_OFF'
        elif self.override_mode == 'GPS_ON':
            effective_zone = 'FORCED_ON'

        # Bölge değişimi logla
        if zone != self.last_zone:
            self.get_logger().warn(
                f'GPS Bolge: {self.last_zone} -> {zone}  '
                f'(robot pos: x={self.robot_x:.1f}, y={self.robot_y:.1f})')
            self.last_zone = zone

        spoofed = self._spoof_message(msg, effective_zone)
        self.gps_pub.publish(spoofed)
        self.last_effective_mode = effective_zone
        self._publish_state('gps_cb')

        if self.verbose and effective_zone != 'NORMAL':
            self.get_logger().info(
                f'  [{effective_zone}] GPS status: {spoofed.status.status} '
                f'| pos: ({self.robot_x:.1f}, {self.robot_y:.1f})',
                throttle_duration_sec=3.0)

    # ── Bölge belirleme ──────────────────────────────────────────────────────
    def _get_zone(self, x: float, y: float) -> str:
        ay = abs(y)
        if self.t_xmin <= x <= self.t_xmax and ay <= self.t_ymax:
            return 'TUNNEL'
        if self.d_xmin <= x <= self.d_xmax and ay <= self.d_ymax:
            return 'DOOR'
        return 'NORMAL'

    # ── Mesaj spoofing ───────────────────────────────────────────────────────
    def _spoof_message(self, original: NavSatFix, zone: str) -> NavSatFix:
        msg = NavSatFix()
        msg.header = original.header
        msg.latitude  = original.latitude
        msg.longitude = original.longitude
        msg.altitude  = original.altitude

        if zone in ('TUNNEL', 'FORCED_OFF'):
            msg.status.status  = NavSatStatus.STATUS_NO_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.position_covariance = COVARIANCE_NO_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        elif zone == 'DOOR':
            msg.status.status  = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.position_covariance = COVARIANCE_BAD_FIX
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        elif zone == 'FORCED_ON':
            msg.status.status  = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS
            msg.position_covariance = list(original.position_covariance)
            msg.position_covariance_type = original.position_covariance_type

        else:
            msg.status  = original.status
            msg.position_covariance = list(original.position_covariance)
            msg.position_covariance_type = original.position_covariance_type

        return msg


def main():
    rclpy.init()
    node = TunnelGPSSpoofer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
