#!/usr/bin/env python3
"""
GPS Quality Monitor — Hybrid Navigation Mode Switcher (v0.4)
=====================================================
/navsat_filtered topic'indeki GPS fix kalitesini izler.
GPS zayıfladığında (tünel/kapalı alan) SLAM moduna geç sinyali yayınlar.
GPS geri döndüğünde GPS+UKF moduna dön.

Mod geçişi:
  GPS modu  → tf_mode_relay map_gps→odom'u map→odom olarak relay eder
  SLAM modu → tf_mode_relay map_slam→odom'u map→odom olarak relay eder

Yayınlanan topic'ler:
  /nav_mode        (std_msgs/String)  : "GPS" veya "SLAM"
  /gps_quality     (std_msgs/Float32) : 0.0 (kötü) – 1.0 (mükemmel)

Abonelikler:
  input_topic param (varsayılan /navsat_filtered, use_spoofer=false ise /navsat)

Kullanım:
  ros2 run leo_gz_bringup gps_monitor
  ya da launch dosyasıyla otomatik başlatılır
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Float32
from robot_localization.srv import SetPose
from collections import deque
import math


class GPSMonitor(Node):
    def __init__(self):
        super().__init__('gps_monitor')

        # ── Parametreler ─────────────────────────────────────────────────────
        self.declare_parameter('gps_to_slam_threshold', 0.3)
        self.declare_parameter('slam_to_gps_threshold', 0.6)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('hysteresis_secs', 3.0)
        self.declare_parameter('gps_timeout_secs', 3.0)
        self.declare_parameter('confirmation_count', 3)
        self.declare_parameter('input_topic', '/navsat_filtered')
        self.declare_parameter('verbose', True)

        self.gps_to_slam_thr    = self.get_parameter('gps_to_slam_threshold').value
        self.slam_to_gps_thr    = self.get_parameter('slam_to_gps_threshold').value
        self.window_size        = self.get_parameter('window_size').value
        self.hysteresis_secs    = self.get_parameter('hysteresis_secs').value
        self.gps_timeout_secs   = self.get_parameter('gps_timeout_secs').value
        self.confirmation_count = self.get_parameter('confirmation_count').value
        self.input_topic        = self.get_parameter('input_topic').value
        self.verbose            = self.get_parameter('verbose').value

        # ── Durum ────────────────────────────────────────────────────────────
        self.current_mode     = 'GPS'
        self.quality_window   = deque(maxlen=self.window_size)
        self.last_switch_time = self.get_clock().now()
        self.last_navsat_time = self.get_clock().now()
        self.last_fix_status  = None
        self._confirm_counter = 0        # ardışık eşik aşımı sayacı
        self._pending_mode    = None     # geçiş onay bekleyen mod

        # ── Yayıncılar ───────────────────────────────────────────────────────
        self.mode_pub    = self.create_publisher(String,  '/nav_mode',    10)
        self.quality_pub = self.create_publisher(Float32, '/gps_quality', 10)

        # ── Abonelikler ──────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, self.input_topic, self._navsat_cb, 10)
        self.latest_gps_odom = None
        self.create_subscription(Odometry, '/odometry/gps', self._gps_odom_cb, 10)

        # ── UKF Global reset service (SLAM→GPS geçişinde) ────────────────
        self.set_pose_client = self.create_client(
            SetPose, '/ukf_filter_node/set_pose')

        # ── Periyodik yayın (GPS gelmese bile modu yay — 5Hz) ─────────────
        self.create_timer(0.2, self._publish_mode)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  GPS QUALITY MONITOR + MOD KONTROL')
        self.get_logger().info(f'  GPS→SLAM eşiği : {self.gps_to_slam_thr:.2f}')
        self.get_logger().info(f'  SLAM→GPS eşiği : {self.slam_to_gps_thr:.2f}')
        self.get_logger().info(f'  Pencere boyutu : {self.window_size}')
        self.get_logger().info(f'  Histerezis     : {self.hysteresis_secs}s')
        self.get_logger().info(f'  GPS timeout    : {self.gps_timeout_secs}s')
        self.get_logger().info(f'  Onay sayısı    : {self.confirmation_count}')
        self.get_logger().info('=' * 55)

    # ── GPS odometry callback (UKF reset için) ────────────────────────────────
    def _gps_odom_cb(self, msg: Odometry):
        self.latest_gps_odom = msg

    # ── UKF Global state reset (SLAM→GPS geçişinde) ──────────────────────────
    def _reset_ukf_global(self):
        if self.latest_gps_odom is None:
            self.get_logger().warn('UKF reset: /odometry/gps verisi yok, atlanıyor')
            return
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warn('UKF reset: set_pose servisi hazır değil')
            return

        req = SetPose.Request()
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map_gps'
        pose.pose.pose = self.latest_gps_odom.pose.pose
        cov = [0.0] * 36
        cov[0] = 1.0    # x
        cov[7] = 1.0    # y
        cov[35] = 0.1   # yaw
        pose.pose.covariance = cov
        req.pose = pose

        self.set_pose_client.call_async(req)
        p = pose.pose.pose.position
        self.get_logger().info(
            f'UKF Global sıfırlandı → ({p.x:+.1f}, {p.y:+.1f})')

    # ── GPS kalite skoru hesaplama ────────────────────────────────────────────
    def _compute_quality(self, msg: NavSatFix) -> float:
        status = msg.status.status
        if status < 0:
            return 0.0
        status_score = {0: 0.5, 1: 0.8, 2: 1.0}.get(status, 0.5)
        cov = msg.position_covariance
        if msg.position_covariance_type > 0 and len(cov) >= 5:
            sigma_h = math.sqrt(max(cov[0], 0) + max(cov[4], 0))
            cov_score = max(0.0, 1.0 - sigma_h / 10.0)
            return 0.5 * status_score + 0.5 * cov_score
        return status_score

    # ── GPS callback ─────────────────────────────────────────────────────────
    def _navsat_cb(self, msg: NavSatFix):
        quality = self._compute_quality(msg)
        self.quality_window.append(quality)
        self.last_fix_status = msg.status.status
        self.last_navsat_time = self.get_clock().now()

        q_msg = Float32()
        q_msg.data = quality
        self.quality_pub.publish(q_msg)

        if len(self.quality_window) >= 3:
            avg_quality = sum(self.quality_window) / len(self.quality_window)
            self._evaluate_mode_switch(avg_quality)

    # ── Mod geçiş mantığı ────────────────────────────────────────────────────
    def _evaluate_mode_switch(self, avg_quality: float):
        now = self.get_clock().now()
        elapsed = (now - self.last_switch_time).nanoseconds / 1e9

        if elapsed < self.hysteresis_secs:
            self._confirm_counter = 0
            self._pending_mode = None
            return

        # Hangi moda geçmek gerekiyor?
        desired_mode = self.current_mode
        if self.current_mode == 'GPS' and avg_quality < self.gps_to_slam_thr:
            desired_mode = 'SLAM'
        elif self.current_mode == 'SLAM' and avg_quality > self.slam_to_gps_thr:
            desired_mode = 'GPS'

        if desired_mode == self.current_mode:
            # Eşik aşılmadı — sayacı sıfırla
            self._confirm_counter = 0
            self._pending_mode = None
            return

        # Ardışık onay sayısı kontrolü (chatter önleme)
        if desired_mode == self._pending_mode:
            self._confirm_counter += 1
        else:
            self._pending_mode = desired_mode
            self._confirm_counter = 1

        if self._confirm_counter >= self.confirmation_count:
            self.get_logger().warn(
                f'MOD DEGISIKLIGI: {self.current_mode} -> {desired_mode} '
                f'(kalite: {avg_quality:.2f}, status: {self.last_fix_status}, '
                f'onay: {self._confirm_counter})'
            )
            # SLAM→GPS geçişinde UKF Global filtresini GPS konumuna sıfırla
            if self.current_mode == 'SLAM' and desired_mode == 'GPS':
                self._reset_ukf_global()
            self.current_mode = desired_mode
            self.last_switch_time = now
            self._confirm_counter = 0
            self._pending_mode = None

    # ── Periyodik yayın — mod + kontrol sinyalleri ────────────────────────────
    def _publish_mode(self):
        # GPS timeout kontrolü: mesaj gelmiyorsa kaliteyi 0'a düşür (#6)
        now = self.get_clock().now()
        gps_silence = (now - self.last_navsat_time).nanoseconds / 1e9
        if gps_silence > self.gps_timeout_secs:
            # Sadece bir kez 0.0 ekle, her tick'te değil (zehirlenme önleme)
            if not getattr(self, '_timeout_injected', False):
                self.quality_window.append(0.0)
                self._timeout_injected = True
            if len(self.quality_window) >= 3:
                avg_q = sum(self.quality_window) / len(self.quality_window)
                self._evaluate_mode_switch(avg_q)
                # Timeout sırasında da quality topic'i güncelle
                q_msg = Float32()
                q_msg.data = float(avg_q)
                self.quality_pub.publish(q_msg)
        else:
            self._timeout_injected = False

        # Mod topic
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        avg_q = (sum(self.quality_window) / len(self.quality_window)
                 if self.quality_window else 0.0)

        if self.verbose:
            src = 'UKF(map_gps)' if self.current_mode == 'GPS' else 'SLAM(map_slam)'
            self.get_logger().info(
                f'Mod: {self.current_mode:4s} | TF: {src} | '
                f'Kalite: {avg_q:.2f} | GPS status: {self.last_fix_status}',
                throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    node = GPSMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
