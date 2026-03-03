#!/usr/bin/env python3
"""
SLAM Mapping Drive Script
Drives the robot systematically around the marsyard to build a complete map.
Uses a pattern: forward → turn → forward → turn (coverage pattern)
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class MappingDriver(Node):
    def __init__(self):
        super().__init__('mapping_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        self.odom = msg

    def drive(self, linear, angular, duration, label=""):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        end = time.time() + duration
        while time.time() < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        # Brief stop
        self.cmd_pub.publish(Twist())
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.odom:
            p = self.odom.pose.pose.position
            print(f"  [{label}] pos=({p.x:.2f}, {p.y:.2f})")

    def mapping_pattern(self):
        """Drive a systematic exploration pattern."""
        print("=== SLAM Mapping Drive ===")
        print("Driving systematic pattern for map coverage...\n")

        # Phase 1: Forward exploration
        self.drive(0.25, 0.0, 4.0, "Forward 1")
        self.drive(0.0, 0.4, 3.5, "Turn left 90°")
        self.drive(0.25, 0.0, 3.0, "Forward 2")
        self.drive(0.0, 0.4, 3.5, "Turn left 90°")
        self.drive(0.25, 0.0, 4.0, "Forward 3")
        self.drive(0.0, 0.4, 3.5, "Turn left 90°")
        self.drive(0.25, 0.0, 3.0, "Forward 4")
        self.drive(0.0, 0.4, 3.5, "Turn left 90°")

        # Phase 2: Expand outward
        self.drive(0.25, 0.0, 5.0, "Forward long 1")
        self.drive(0.0, -0.4, 3.5, "Turn right 90°")
        self.drive(0.25, 0.0, 4.0, "Forward long 2")
        self.drive(0.0, -0.4, 3.5, "Turn right 90°")
        self.drive(0.25, 0.0, 5.0, "Forward long 3")
        self.drive(0.0, -0.4, 3.5, "Turn right 90°")
        self.drive(0.25, 0.0, 4.0, "Forward long 4")

        # Phase 3: Arc sweeps for wider coverage
        self.drive(0.2, 0.15, 6.0, "Arc sweep 1")
        self.drive(0.2, -0.15, 6.0, "Arc sweep 2")
        self.drive(0.2, 0.15, 6.0, "Arc sweep 3")

        # Return and fill gaps
        self.drive(0.0, 0.4, 7.0, "Full rotation scan")
        self.drive(0.25, 0.0, 3.0, "Final forward")
        self.drive(0.0, 0.4, 7.0, "Final rotation scan")

        # Stop
        self.cmd_pub.publish(Twist())
        print("\n=== Mapping drive complete! ===")
        print("Map should now have good coverage of the area.")


def main():
    rclpy.init()
    driver = MappingDriver()
    time.sleep(2)
    for _ in range(20):
        rclpy.spin_once(driver, timeout_sec=0.1)
    driver.mapping_pattern()
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
