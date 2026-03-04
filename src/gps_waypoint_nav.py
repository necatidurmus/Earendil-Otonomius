#!/usr/bin/env python3
"""
GPS Waypoint Navigation for Leo Rover
Reads Gazebo /navsat GPS data, converts to local XY, sends Nav2 goals.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import math
import time

# Reference GPS origin (same as Gazebo NavSat plugin config)
# Ankara coordinates from URDF
REF_LAT = 39.925054
REF_LON = 32.836991

# GPS waypoints to visit (lat, lon, name)
GPS_WAYPOINTS = [
    (39.925090, 32.837050, "GPS-WP1: Kuzey-Dogu 4m"),
    (39.925130, 32.836930, "GPS-WP2: Kuzey-Bati 8m"),
    (39.925054, 32.837100, "GPS-WP3: Dogu 8m"),
    (39.925000, 32.836900, "GPS-WP4: Guney-Bati 7m"),
    (39.925054, 32.836991, "GPS-WP5: Baslangica don"),
]

def gps_to_local(lat, lon, ref_lat=REF_LAT, ref_lon=REF_LON):
    """Convert GPS (lat, lon) to local (x, y) in meters."""
    R = 6371000.0  # Earth radius in meters
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    x = dlon * R * math.cos(math.radians(ref_lat))  # East = +X
    y = dlat * R  # North = +Y
    return x, y


class GPSWaypointNav(Node):
    def __init__(self):
        super().__init__('gps_waypoint_nav')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.current_gps = None
        self.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)
        
        self.wp_index = 0
        self.wp_results = []
        self.start_time = time.time()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('  GPS WAYPOINT NAVIGATION TEST')
        self.get_logger().info(f'  Reference: ({REF_LAT}, {REF_LON})')
        self.get_logger().info(f'  Waypoints: {len(GPS_WAYPOINTS)}')
        self.get_logger().info('=' * 60)

    def _gps_cb(self, msg):
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)

    def start_navigation(self):
        self.get_logger().info('Waiting for GPS fix...')
        while self.current_gps is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        
        lat, lon, alt = self.current_gps
        sx, sy = gps_to_local(lat, lon)
        self.get_logger().info(f'GPS fix: ({lat:.6f}, {lon:.6f}) -> local: ({sx:.1f}, {sy:.1f})')
        
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.wp_index >= len(GPS_WAYPOINTS):
            self.print_results()
            rclpy.shutdown()
            return

        lat, lon, name = GPS_WAYPOINTS[self.wp_index]
        x, y = gps_to_local(lat, lon)

        self.get_logger().info('')
        self.get_logger().info(f'[{self.wp_index+1}/{len(GPS_WAYPOINTS)}] {name}')
        self.get_logger().info(f'  GPS: ({lat:.6f}, {lon:.6f}) -> Local: ({x:.1f}, {y:.1f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.wp_start = time.time()
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal REJECTED!')
            self.wp_results.append({'wp': self.wp_index+1, 'status': 'REJECTED'})
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
                f'Remain:{remaining:.1f}m', throttle_duration_sec=3.0)

    def _result_cb(self, future):
        result = future.result()
        elapsed = round(time.time() - self.wp_start, 1)
        _, _, name = GPS_WAYPOINTS[self.wp_index]
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
    rclpy.init()
    node = GPSWaypointNav()
    time.sleep(1)
    node.start_navigation()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
