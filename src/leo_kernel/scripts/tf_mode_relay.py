#!/usr/bin/env python3
"""
TF Mode Relay — Deterministic GPS/SLAM Frame Selector (v0.5)
===============================================================
Publishes map→map_slam TF based on /nav_mode.

This maintains a SINGLE TF tree:
    map → map_slam → odom → base_footprint → ...

  SLAM Toolbox → map_slam→odom   (always on main /tf)
  DiffDrive    → odom→base_footprint (always on main /tf)
  tf_mode_relay:
    SLAM mode → map→map_slam = identity (SLAM is truth)
    GPS  mode → map→map_slam = GPS correction offset
               computed from /odometry/filtered (map_gps→base_footprint)
               and SLAM's map_slam→base_footprint chain

Nav2 uses global_frame: map → sees: map→map_slam→odom→base_footprint

Usage:
  navigation_hybrid.launch.py tarafından otomatik başlatılır.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
import tf2_ros
import math


def _lerp(a, b, t):
    return a + (b - a) * t


def _slerp_quat(q1, q2, t):
    dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w
    if dot < 0:
        q2 = Quaternion(x=-q2.x, y=-q2.y, z=-q2.z, w=-q2.w)
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        r = Quaternion()
        r.x = _lerp(q1.x, q2.x, t)
        r.y = _lerp(q1.y, q2.y, t)
        r.z = _lerp(q1.z, q2.z, t)
        r.w = _lerp(q1.w, q2.w, t)
        norm = math.sqrt(r.x**2 + r.y**2 + r.z**2 + r.w**2)
        if norm > 0:
            r.x /= norm; r.y /= norm; r.z /= norm; r.w /= norm
        return r
    theta_0 = math.acos(dot)
    sin_t0 = math.sin(theta_0)
    s1 = math.sin((1 - t) * theta_0) / sin_t0
    s2 = math.sin(t * theta_0) / sin_t0
    r = Quaternion()
    r.x = s1*q1.x + s2*q2.x
    r.y = s1*q1.y + s2*q2.y
    r.z = s1*q1.z + s2*q2.z
    r.w = s1*q1.w + s2*q2.w
    return r


def _interpolate_transform(t_from, t_to, alpha):
    result = Transform()
    result.translation.x = _lerp(t_from.translation.x, t_to.translation.x, alpha)
    result.translation.y = _lerp(t_from.translation.y, t_to.translation.y, alpha)
    result.translation.z = _lerp(t_from.translation.z, t_to.translation.z, alpha)
    result.rotation = _slerp_quat(t_from.rotation, t_to.rotation, alpha)
    return result


def _identity_transform():
    t = Transform()
    t.translation = Vector3(x=0.0, y=0.0, z=0.0)
    t.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return t


# ── Quaternion math helpers ──────────────────────────────────────────────

def _quat_conjugate(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _quat_multiply(a, b):
    return Quaternion(
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
    )


def _rotate_vector(q, v):
    vq = Quaternion(x=v.x, y=v.y, z=v.z, w=0.0)
    result = _quat_multiply(_quat_multiply(q, vq), _quat_conjugate(q))
    return Vector3(x=result.x, y=result.y, z=result.z)


def _compose_transforms(t_ab, t_bc):
    """Compute T_AC = T_AB * T_BC."""
    result = Transform()
    rotated = _rotate_vector(t_ab.rotation, t_bc.translation)
    result.translation = Vector3(
        x=t_ab.translation.x + rotated.x,
        y=t_ab.translation.y + rotated.y,
        z=t_ab.translation.z + rotated.z,
    )
    result.rotation = _quat_multiply(t_ab.rotation, t_bc.rotation)
    return result


def _invert_transform(t):
    """Compute T⁻¹."""
    result = Transform()
    q_inv = _quat_conjugate(t.rotation)
    neg_t = Vector3(x=-t.translation.x, y=-t.translation.y, z=-t.translation.z)
    t_inv = _rotate_vector(q_inv, neg_t)
    result.translation = t_inv
    result.rotation = q_inv
    return result


class TFModeRelay(Node):
    def __init__(self):
        super().__init__('tf_mode_relay')

        self.current_mode = 'GPS'

        # Smooth transition state
        self._transition_active = False
        self._transition_start_time = None
        self._transition_duration = 1.0
        self._transition_from = None
        self._transition_to_mode = None
        self._last_published = _identity_transform()

        # TF broadcaster — map→map_slam yayınlar
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # tf2 buffer — SLAM's map_slam→odom→base_footprint chain
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # GPS source: /odometry/filtered (map_gps→base_footprint)
        self._last_gps_odom = None
        self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_filtered_cb, 10)

        # Mod sinyali
        self.create_subscription(String, '/nav_mode', self._mode_cb, 10)

        # Publish map→map_slam at 50 Hz
        self.create_timer(0.02, self._relay_tf)

        self.get_logger().info('TF Mode Relay v0.5 baslatildi')
        self.get_logger().info('  TF zinciri: map → map_slam → odom → base_footprint')
        self.get_logger().info('  GPS  modu → map→map_slam = GPS correction offset')
        self.get_logger().info('  SLAM modu → map→map_slam = identity')
        self.get_logger().info(f'  Gecis suresi: {self._transition_duration}s')

    def _odom_filtered_cb(self, msg: Odometry):
        self._last_gps_odom = msg

    def _mode_cb(self, msg: String):
        if msg.data != self.current_mode:
            self.get_logger().warn(
                f'TF Relay mod gecisi: {self.current_mode} -> {msg.data}')
            self._transition_active = True
            self._transition_start_time = self.get_clock().now()
            self._transition_from = self._copy_transform(self._last_published)
            self._transition_to_mode = msg.data
            self.current_mode = msg.data

    def _get_gps_correction(self):
        """GPS modunda map→map_slam offset hesapla.

        map→map_slam = map_gps→base_footprint * inv(map_slam→base_footprint)

        map_gps→bf  : /odometry/filtered topic'inden
        map_slam→bf : tf2 buffer'dan (map_slam→odom→base_footprint chain)
        """
        odom_msg = self._last_gps_odom
        if odom_msg is None:
            return None

        # map_gps→base_footprint from UKF Global odometry
        gps_to_bf = Transform()
        gps_to_bf.translation.x = odom_msg.pose.pose.position.x
        gps_to_bf.translation.y = odom_msg.pose.pose.position.y
        gps_to_bf.translation.z = odom_msg.pose.pose.position.z
        gps_to_bf.rotation = odom_msg.pose.pose.orientation

        # map_slam→base_footprint from SLAM chain
        try:
            ts = self._tf_buffer.lookup_transform(
                'map_slam', 'base_footprint', rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        slam_to_bf = ts.transform

        # map→map_slam = map_gps→bf * inv(map_slam→bf)
        # We treat map ≡ map_gps (GPS-anchored global frame)
        return _compose_transforms(gps_to_bf, _invert_transform(slam_to_bf))

    def _get_source_tf(self, mode):
        if mode == 'GPS':
            return self._get_gps_correction()
        else:
            return _identity_transform()

    def _relay_tf(self):
        if self._transition_active:
            self._handle_transition()
            return

        transform = self._get_source_tf(self.current_mode)
        if transform is None:
            transform = self._last_published
        self._publish_map_map_slam(transform)

    def _handle_transition(self):
        now = self.get_clock().now()
        elapsed = (now - self._transition_start_time).nanoseconds / 1e9
        alpha = min(elapsed / self._transition_duration, 1.0)

        target_tf = self._get_source_tf(self._transition_to_mode)
        if target_tf is None:
            self._publish_map_map_slam(self._last_published)
            return

        if alpha >= 1.0:
            self._transition_active = False
            self._publish_map_map_slam(target_tf)
            self.get_logger().info(
                f'TF gecis tamamlandi -> {self._transition_to_mode}')
            return

        blended = _interpolate_transform(self._transition_from, target_tf, alpha)
        self._publish_map_map_slam(blended)

    def _publish_map_map_slam(self, transform):
        relay = TransformStamped()
        relay.header.stamp = self.get_clock().now().to_msg()
        relay.header.frame_id = 'map'
        relay.child_frame_id = 'map_slam'
        relay.transform = transform
        self.tf_broadcaster.sendTransform(relay)
        self._last_published = self._copy_transform(transform)

    @staticmethod
    def _copy_transform(t):
        c = Transform()
        c.translation.x = t.translation.x
        c.translation.y = t.translation.y
        c.translation.z = t.translation.z
        c.rotation.x = t.rotation.x
        c.rotation.y = t.rotation.y
        c.rotation.z = t.rotation.z
        c.rotation.w = t.rotation.w
        return c


def main():
    rclpy.init()
    node = TFModeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
