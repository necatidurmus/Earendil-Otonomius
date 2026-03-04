#!/usr/bin/env python3
"""
GPS Waypoint Navigation for Leo Rover
Loads waypoints from YAML, converts lat/lon via /fromLL service,
computes heading to next waypoint, sends Nav2 goals.

Usage:
  ros2 run leo_gz_bringup gps_waypoint_nav  (if packaged)
  python3 gps_waypoint_nav.py --waypoints /path/to/waypoints.yaml
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import FromLL
import math
import time
import yaml
import os
import argparse


class GPSWaypointNav(Node):
    def __init__(self, waypoints_file):
        super().__init__('gps_waypoint_nav')

        # Load waypoints from YAML
        self.waypoints = self._load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded!')
            raise SystemExit(1)

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Service client for GPS -> map XY conversion
        self.fromll_client = self.create_client(FromLL, '/fromLL')

        # GPS status monitoring
        self.current_gps = None
        self.gps_status = -1  # STATUS_NO_FIX
        self.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)

        # State
        self.wp_index = 0
        self.wp_results = []
        self.start_time = time.time()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  GPS WAYPOINT NAVIGATION')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Source: {waypoints_file}')
        self.get_logger().info('=' * 60)

    def _load_waypoints(self, filepath):
        """Load waypoints from YAML file."""
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            wps = data.get('waypoints', [])
            self.get_logger().info(f'Loaded {len(wps)} waypoints from {filepath}')
            return wps
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def _gps_cb(self, msg):
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)
        self.gps_status = msg.status.status

    def _gps_to_map(self, lat, lon):
        """Convert lat/lon to map XY using /fromLL service."""
        if not self.fromll_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/fromLL service not available!')
            return None, None

        req = FromLL.Request()
        req.ll_point = Point()
        req.ll_point.x = lat   # latitude
        req.ll_point.y = lon   # longitude
        req.ll_point.z = 0.0

        future = self.fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            return result.map_point.x, result.map_point.y
        else:
            self.get_logger().error('/fromLL call failed!')
            return None, None

    def _compute_yaw(self, current_wp_idx):
        """Compute heading (yaw) toward the next waypoint."""
        if current_wp_idx + 1 >= len(self.waypoints):
            return 0.0  # Last waypoint: no heading preference

        # Convert both current and next WP to map XY
        curr = self.waypoints[current_wp_idx]
        nxt = self.waypoints[current_wp_idx + 1]

        cx, cy = self._gps_to_map(curr['lat'], curr['lon'])
        nx, ny = self._gps_to_map(nxt['lat'], nxt['lon'])

        if cx is None or nx is None:
            return 0.0

        return math.atan2(ny - cy, nx - cx)

    def start_navigation(self):
        """Wait for GPS fix and /fromLL service, then start sending goals."""
        self.get_logger().info('Waiting for GPS fix...')
        while self.current_gps is None:
            rclpy.spin_once(self, timeout_sec=0.5)

        lat, lon, _ = self.current_gps
        self.get_logger().info(f'GPS fix: ({lat:.6f}, {lon:.6f}), status={self.gps_status}')

        self.get_logger().info('Waiting for /fromLL service...')
        if not self.fromll_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('/fromLL service not available! Is navsat_transform running?')
            return

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Nav2 action server not available!')
            return

        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            self.print_results()
            rclpy.shutdown()
            return

        wp = self.waypoints[self.wp_index]
        lat, lon, name = wp['lat'], wp['lon'], wp.get('name', f'WP{self.wp_index+1}')

        # Convert GPS to map XY via /fromLL
        x, y = self._gps_to_map(lat, lon)
        if x is None:
            self.get_logger().error(f'  Failed to convert {name} to map XY, skipping')
            self.wp_results.append({'wp': self.wp_index+1, 'name': name, 'status': 'SKIP'})
            self.wp_index += 1
            self.send_next_waypoint()
            return

        # Compute yaw toward next waypoint
        yaw = self._compute_yaw(self.wp_index)

        self.get_logger().info('')
        self.get_logger().info(f'[{self.wp_index+1}/{len(self.waypoints)}] {name}')
        self.get_logger().info(f'  GPS: ({lat:.6f}, {lon:.6f}) -> Map: ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'  Yaw: {math.degrees(yaw):.1f} deg')

        # Build goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.wp_start = time.time()
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('  Goal REJECTED!')
            self.wp_results.append({
                'wp': self.wp_index+1,
                'name': self.waypoints[self.wp_index].get('name', ''),
                'status': 'REJECTED'
            })
            self.wp_index += 1
            self.send_next_waypoint()
            return
        self.get_logger().info('  Goal accepted, navigating...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        pos = fb.current_pose.pose.position
        remaining = fb.distance_remaining
        if self.current_gps:
            lat, lon, _ = self.current_gps
            self.get_logger().info(
                f'  GPS:({lat:.6f},{lon:.6f}) Pos:({pos.x:.1f},{pos.y:.1f}) '
                f'Remain:{remaining:.1f}m',
                throttle_duration_sec=3.0)

    def _result_cb(self, future):
        result = future.result()
        elapsed = round(time.time() - self.wp_start, 1)
        name = self.waypoints[self.wp_index].get('name', '')
        status_map = {4: 'SUCCEEDED', 6: 'ABORTED', 5: 'CANCELED'}
        status = status_map.get(result.status, f'UNKNOWN({result.status})')

        self.wp_results.append({
            'wp': self.wp_index+1, 'name': name,
            'status': status, 'time': elapsed
        })

        if result.status == 4:
            self.get_logger().info(f'  ✅ {name} reached in {elapsed}s')
        else:
            self.get_logger().warn(f'  ⚠️ {name}: {status} ({elapsed}s)')

        self.wp_index += 1
        self.send_next_waypoint()

    def print_results(self):
        total = round(time.time() - self.start_time, 1)
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  GPS NAVIGATION RESULTS')
        self.get_logger().info('=' * 60)
        for r in self.wp_results:
            self.get_logger().info(
                f"  WP{r['wp']}: {r['status']:10s} {r.get('time','?')}s  {r.get('name','')}")
        ok = sum(1 for r in self.wp_results if r['status'] == 'SUCCEEDED')
        self.get_logger().info(f'\n  Total: {ok}/{len(self.wp_results)} OK  |  {total}s')
        self.get_logger().info('=' * 60)


def main():
    parser = argparse.ArgumentParser(description='GPS Waypoint Navigation')
    parser.add_argument('--waypoints', '-w', type=str, default=None,
                        help='Path to waypoints YAML file')
    args, unknown = parser.parse_known_args()

    # Default waypoints path
    if args.waypoints is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('leo_gz_bringup')
            args.waypoints = os.path.join(pkg, 'config', 'waypoints.yaml')
        except Exception:
            args.waypoints = os.path.join(
                os.path.dirname(__file__), '..', 'leo_simulator',
                'leo_gz_bringup', 'config', 'waypoints.yaml')

    rclpy.init()
    node = GPSWaypointNav(args.waypoints)
    time.sleep(1)
    node.start_navigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
