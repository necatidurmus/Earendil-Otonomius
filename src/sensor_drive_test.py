#!/usr/bin/env python3
"""
Marsyard Dynamic Sensor Test
- Aracı ileri, sola, saga, geri sürer
- Her adımda tüm sensörleri kontrol eder
- Sonuçları tablo olarak yazdırır
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, JointState, NavSatFix
import time
import math


class SensorTester(Node):
    def __init__(self):
        super().__init__('sensor_tester')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sensor data storage
        self.odom = None
        self.scan = None
        self.imu = None
        self.joints = None
        self.gps = None

        # Subscribers
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Imu, '/imu/data_raw', lambda m: setattr(self, 'imu', m), 10)
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', m), 10)
        self.create_subscription(NavSatFix, '/navsat', lambda m: setattr(self, 'gps', m), 10)

        self.results = []
        self.start_time = time.time()

    def send_cmd(self, linear_x, angular_z, duration, label):
        """Send velocity command for given duration."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        stop = Twist()
        self.cmd_pub.publish(stop)
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.1)

        # Record sensor state
        self.record_state(label)

    def record_state(self, label):
        """Record current sensor state."""
        rclpy.spin_once(self, timeout_sec=0.5)

        state = {'label': label, 'time': time.time() - self.start_time}

        # Odom
        if self.odom:
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            state['odom_x'] = p.x
            state['odom_y'] = p.y
            state['odom_yaw'] = math.degrees(yaw)
            state['odom_ok'] = True
        else:
            state['odom_ok'] = False

        # LiDAR
        if self.scan:
            valid = [r for r in self.scan.ranges
                     if r > self.scan.range_min and r < self.scan.range_max]
            state['lidar_rays'] = len(self.scan.ranges)
            state['lidar_valid'] = len(valid)
            state['lidar_min'] = min(valid) if valid else -1
            state['lidar_avg'] = sum(valid)/len(valid) if valid else -1
            state['lidar_ok'] = True
        else:
            state['lidar_ok'] = False

        # IMU
        if self.imu:
            state['imu_ax'] = self.imu.linear_acceleration.x
            state['imu_ay'] = self.imu.linear_acceleration.y
            state['imu_az'] = self.imu.linear_acceleration.z
            state['imu_gz'] = self.imu.angular_velocity.z
            state['imu_ok'] = True
        else:
            state['imu_ok'] = False

        # GPS
        if self.gps:
            state['gps_lat'] = self.gps.latitude
            state['gps_lon'] = self.gps.longitude
            state['gps_alt'] = self.gps.altitude
            state['gps_ok'] = True
        else:
            state['gps_ok'] = False

        # Joints
        if self.joints:
            state['joints_ok'] = len(self.joints.name) > 0
            state['joints_count'] = len(self.joints.name)
        else:
            state['joints_ok'] = False

        self.results.append(state)

    def print_results(self):
        """Print results as a table."""
        print("\n" + "=" * 120)
        print("MARSYARD DYNAMIC SENSOR TEST RESULTS")
        print("=" * 120)

        # Odom table
        print("\n--- ODOMETRY ---")
        print(f"{'Step':<20} {'Time':>6} {'X (m)':>8} {'Y (m)':>8} {'Yaw (°)':>8} {'Status':>8}")
        print("-" * 60)
        for s in self.results:
            if s.get('odom_ok'):
                print(f"{s['label']:<20} {s['time']:>6.1f} {s['odom_x']:>8.3f} {s['odom_y']:>8.3f} {s['odom_yaw']:>8.1f} {'OK':>8}")
            else:
                print(f"{s['label']:<20} {s['time']:>6.1f} {'---':>8} {'---':>8} {'---':>8} {'FAIL':>8}")

        # LiDAR table
        print("\n--- LIDAR ---")
        print(f"{'Step':<20} {'Valid/Total':>12} {'Min (m)':>8} {'Avg (m)':>8} {'Status':>8}")
        print("-" * 60)
        for s in self.results:
            if s.get('lidar_ok'):
                print(f"{s['label']:<20} {s['lidar_valid']:>4}/{s['lidar_rays']:<6} {s['lidar_min']:>8.2f} {s['lidar_avg']:>8.2f} {'OK':>8}")
            else:
                print(f"{s['label']:<20} {'---':>12} {'---':>8} {'---':>8} {'FAIL':>8}")

        # GPS table
        print("\n--- GPS ---")
        print(f"{'Step':<20} {'Latitude':>14} {'Longitude':>14} {'Alt (m)':>8} {'Status':>8}")
        print("-" * 70)
        for s in self.results:
            if s.get('gps_ok'):
                print(f"{s['label']:<20} {s['gps_lat']:>14.8f} {s['gps_lon']:>14.8f} {s['gps_alt']:>8.1f} {'OK':>8}")
            else:
                print(f"{s['label']:<20} {'---':>14} {'---':>14} {'---':>8} {'FAIL':>8}")

        # IMU table
        print("\n--- IMU ---")
        print(f"{'Step':<20} {'Accel X':>8} {'Accel Y':>8} {'Accel Z':>8} {'Gyro Z':>8} {'Status':>8}")
        print("-" * 60)
        for s in self.results:
            if s.get('imu_ok'):
                print(f"{s['label']:<20} {s['imu_ax']:>8.3f} {s['imu_ay']:>8.3f} {s['imu_az']:>8.3f} {s['imu_gz']:>8.4f} {'OK':>8}")
            else:
                print(f"{s['label']:<20} {'---':>8} {'---':>8} {'---':>8} {'---':>8} {'FAIL':>8}")

        # Summary
        print("\n--- SUMMARY ---")
        total = len(self.results)
        odom_ok = sum(1 for s in self.results if s.get('odom_ok'))
        lidar_ok = sum(1 for s in self.results if s.get('lidar_ok'))
        gps_ok = sum(1 for s in self.results if s.get('gps_ok'))
        imu_ok = sum(1 for s in self.results if s.get('imu_ok'))
        joints_ok = sum(1 for s in self.results if s.get('joints_ok'))
        print(f"Odom:   {odom_ok}/{total} OK")
        print(f"LiDAR:  {lidar_ok}/{total} OK")
        print(f"GPS:    {gps_ok}/{total} OK")
        print(f"IMU:    {imu_ok}/{total} OK")
        print(f"Joints: {joints_ok}/{total} OK")

        if odom_ok == total and lidar_ok == total and gps_ok == total and imu_ok == total:
            print("\n*** ALL SENSORS PASSED! ***")
        else:
            print("\n*** SOME SENSORS FAILED ***")
        print("=" * 120)


def main():
    rclpy.init()
    tester = SensorTester()

    # Wait for sensors to come online
    print("Waiting for sensor data...")
    time.sleep(2)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.1)

    print("Starting drive test in marsyard2020...")

    # Test sequence
    tester.record_state("1. Baslangic")

    tester.send_cmd(0.2, 0.0, 3.0, "2. Ileri 3s")

    tester.send_cmd(0.0, 0.5, 2.0, "3. Sola don 2s")

    tester.send_cmd(0.3, 0.0, 3.0, "4. Ileri 3s")

    tester.send_cmd(0.0, -0.5, 2.0, "5. Saga don 2s")

    tester.send_cmd(0.2, 0.0, 2.0, "6. Ileri 2s")

    tester.send_cmd(0.0, 0.5, 3.0, "7. Sola don 3s")

    tester.send_cmd(-0.15, 0.0, 2.0, "8. Geri 2s")

    tester.send_cmd(0.2, 0.2, 3.0, "9. Arc hareket 3s")

    tester.record_state("10. Son durum")

    # Print results
    tester.print_results()

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
