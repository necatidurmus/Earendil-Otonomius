#!/usr/bin/env python3
"""
Web-based WASD Teleop Server for Leo Rover
Serves a beautiful web UI and handles keyboard commands via HTTP.
Publishes cmd_vel and reads sensor data from ROS 2.

Usage: python3 teleop_server.py
Then open http://localhost:8080 in your browser.
"""
import os
import json
import threading
import time
from http.server import HTTPServer, SimpleHTTPRequestHandler
from functools import partial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from std_msgs.msg import String


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_web')
        # use_sim_time is automatically declared by rclpy Node in Humble

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gps_control_pub = self.create_publisher(String, '/gps_spoofer/control', 10)

        # State
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.max_linear = 1.5
        self.max_angular = 3.5
        self.drive_mode = 'MANUAL'
        self.gps_override = 'AUTO'

        # Sensor data
        self.odom_data = {}
        self.map_odom_data = {}
        self.imu_data = {}
        self.gps_data = {}
        self.filtered_gps_data = {}
        self.lidar_data = {}
        self.nav_mode = 'UNKNOWN'
        self.gps_spoofer_state = {}

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._map_odom_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 10)
        self.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)
        self.create_subscription(NavSatFix, '/navsat_filtered', self._filtered_gps_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._lidar_cb, 10)
        self.create_subscription(String, '/nav_mode', self._nav_mode_cb, 10)
        self.create_subscription(String, '/gps_spoofer/state', self._gps_spoofer_state_cb, 10)

        # timer removed
        self.get_logger().info('Teleop Web Node started')

    def _odom_cb(self, msg):
        self.odom_data = {
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'vx': round(msg.twist.twist.linear.x, 3),
            'vz': round(msg.twist.twist.angular.z, 3),
        }

    def _imu_cb(self, msg):
        self.imu_data = {
            'ax': round(msg.linear_acceleration.x, 2),
            'ay': round(msg.linear_acceleration.y, 2),
            'az': round(msg.linear_acceleration.z, 2),
            'gz': round(msg.angular_velocity.z, 3),
        }

    def _map_odom_cb(self, msg):
        self.map_odom_data = {
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'vx': round(msg.twist.twist.linear.x, 3),
            'vz': round(msg.twist.twist.angular.z, 3),
        }

    def _gps_cb(self, msg):
        self.gps_data = {
            'lat': round(msg.latitude, 8),
            'lon': round(msg.longitude, 8),
            'alt': round(msg.altitude, 1),
            'status': int(msg.status.status),
            'cov0': round(msg.position_covariance[0], 2) if msg.position_covariance else 0.0,
        }

    def _filtered_gps_cb(self, msg):
        self.filtered_gps_data = {
            'lat': round(msg.latitude, 8),
            'lon': round(msg.longitude, 8),
            'alt': round(msg.altitude, 1),
            'status': int(msg.status.status),
            'cov0': round(msg.position_covariance[0], 2) if msg.position_covariance else 0.0,
        }

    def _lidar_cb(self, msg):
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        self.lidar_data = {
            'valid': len(ranges),
            'total': len(msg.ranges),
            'min': round(min(ranges), 2) if ranges else 0,
            'avg': round(sum(ranges) / len(ranges), 2) if ranges else 0,
        }

    def _nav_mode_cb(self, msg):
        self.nav_mode = msg.data.strip() or 'UNKNOWN'

    def _gps_spoofer_state_cb(self, msg):
        state = {}
        for part in msg.data.split(';'):
            if '=' not in part:
                continue
            key, value = part.split('=', 1)
            state[key] = value
        self.gps_spoofer_state = state

    def set_velocity(self, linear, angular):
        if self.drive_mode != 'MANUAL':
            linear = 0.0
            angular = 0.0
        self.linear_x = max(-self.max_linear, min(self.max_linear, linear))
        self.angular_z = max(-self.max_angular, min(self.max_angular, angular))
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.angular.z = float(self.angular_z)
        self.cmd_pub.publish(msg)

    def set_drive_mode(self, mode):
        requested = str(mode).strip().upper()
        if requested not in {'MANUAL', 'AUTO'}:
            raise ValueError(f'Invalid drive mode: {mode}')
        self.drive_mode = requested
        if requested != 'MANUAL':
            self.set_velocity(0.0, 0.0)

    def set_gps_override(self, mode):
        requested = str(mode).strip().upper()
        aliases = {
            'AUTO': 'AUTO',
            'GPS_ON': 'GPS_ON',
            'ON': 'GPS_ON',
            'GPS_OFF': 'GPS_OFF',
            'OFF': 'GPS_OFF',
        }
        if requested not in aliases:
            raise ValueError(f'Invalid GPS override: {mode}')
        self.gps_override = aliases[requested]
        msg = String()
        msg.data = self.gps_override
        self.gps_control_pub.publish(msg)

    def get_status(self):
        return {
            'drive_mode': self.drive_mode,
            'gps_override': self.gps_override,
            'nav_mode': self.nav_mode,
            'cmd': {'linear': round(self.linear_x, 3), 'angular': round(self.angular_z, 3)},
            'odom': self.odom_data,
            'map_odom': self.map_odom_data,
            'imu': self.imu_data,
            'gps': self.gps_data,
            'gps_filtered': self.filtered_gps_data,
            'lidar': self.lidar_data,
            'gps_spoofer': self.gps_spoofer_state,
        }


class TeleopHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, teleop_node, *args, **kwargs):
        self.teleop_node = teleop_node
        super().__init__(*args, directory=os.path.dirname(__file__), **kwargs)

    def _read_json_body(self):
        """POST body'yi güvenli şekilde parse et."""
        try:
            length = int(self.headers.get('Content-Length', 0))
            if length <= 0 or length > 4096:
                return None
            return json.loads(self.rfile.read(length))
        except (json.JSONDecodeError, ValueError):
            return None

    def do_POST(self):
        body = self._read_json_body()
        if body is None:
            self._json_response({'ok': False, 'error': 'invalid JSON'}, status=400)
            return

        if self.path == '/cmd':
            linear = float(body.get('linear', 0.0))
            angular = float(body.get('angular', 0.0))
            self.teleop_node.set_velocity(linear, angular)
            self._json_response({'ok': True})
        elif self.path == '/speed':
            self.teleop_node.max_linear = float(body.get('max_linear', 1.5))
            self.teleop_node.max_angular = float(body.get('max_angular', 3.5))
            self._json_response({'ok': True})
        elif self.path == '/drive_mode':
            try:
                self.teleop_node.set_drive_mode(body.get('mode', 'MANUAL'))
                self._json_response({'ok': True, 'mode': self.teleop_node.drive_mode})
            except ValueError as e:
                self._json_response({'ok': False, 'error': str(e)}, status=400)
        elif self.path == '/gps_override':
            try:
                self.teleop_node.set_gps_override(body.get('mode', 'AUTO'))
                self._json_response({'ok': True, 'mode': self.teleop_node.gps_override})
            except ValueError as e:
                self._json_response({'ok': False, 'error': str(e)}, status=400)
        else:
            self.send_error(404)

    def do_GET(self):
        if self.path == '/status':
            self._json_response(self.teleop_node.get_status())
        elif self.path == '/':
            self.path = '/index.html'
            super().do_GET()
        else:
            super().do_GET()

    def _json_response(self, data, status=200):
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args):
        pass  # Suppress HTTP logs


def main():
    rclpy.init()
    node = TeleopNode()

    # Run ROS 2 in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Start HTTP server
    handler = partial(TeleopHTTPHandler, node)

    class ReusableHTTPServer(HTTPServer):
        allow_reuse_address = True

    server = ReusableHTTPServer(('0.0.0.0', 8080), handler)
    print('\n' + '=' * 50)
    print('  🎮 Leo Rover Teleop Web Interface')
    print('  Open: http://localhost:8080')
    print('=' * 50 + '\n')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.set_velocity(0.0, 0.0)
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
