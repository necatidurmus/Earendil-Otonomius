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


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_web')
        # use_sim_time is automatically declared by rclpy Node in Humble

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.max_linear = 0.5
        self.max_angular = 1.0

        # Sensor data
        self.odom_data = {}
        self.imu_data = {}
        self.gps_data = {}
        self.lidar_data = {}

        # Subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 10)
        self.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._lidar_cb, 10)

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

    def _gps_cb(self, msg):
        self.gps_data = {
            'lat': round(msg.latitude, 8),
            'lon': round(msg.longitude, 8),
            'alt': round(msg.altitude, 1),
        }

    def _lidar_cb(self, msg):
        ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
        self.lidar_data = {
            'valid': len(ranges),
            'total': len(msg.ranges),
            'min': round(min(ranges), 2) if ranges else 0,
            'avg': round(sum(ranges) / len(ranges), 2) if ranges else 0,
        }

    def set_velocity(self, linear, angular):
        self.linear_x = max(-self.max_linear, min(self.max_linear, linear))
        self.angular_z = max(-self.max_angular, min(self.max_angular, angular))
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.angular.z = float(self.angular_z)
        self.cmd_pub.publish(msg)

    def get_status(self):
        return {
            'cmd': {'linear': round(self.linear_x, 3), 'angular': round(self.angular_z, 3)},
            'odom': self.odom_data,
            'imu': self.imu_data,
            'gps': self.gps_data,
            'lidar': self.lidar_data,
        }


class TeleopHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, teleop_node, *args, **kwargs):
        self.teleop_node = teleop_node
        super().__init__(*args, directory=os.path.dirname(__file__), **kwargs)

    def do_POST(self):
        if self.path == '/cmd':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            self.teleop_node.set_velocity(
                body.get('linear', 0.0),
                body.get('angular', 0.0)
            )
            self._json_response({'ok': True})
        elif self.path == '/speed':
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length))
            self.teleop_node.max_linear = body.get('max_linear', 0.5)
            self.teleop_node.max_angular = body.get('max_angular', 1.0)
            self._json_response({'ok': True})
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

    def _json_response(self, data):
        self.send_response(200)
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
