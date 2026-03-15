#!/usr/bin/env python3
"""
Navigate the robot through obstacle field to the tunnel and back.
Sends sequential Nav2 goals with feedback.
Waypoints chosen to avoid all obstacles in leo_obstacles.sdf.

Obstacle layout:
  box_01: (6,4)   box_03: (10,-3)   box_14: (14,-8)
  box_05: (3,-8)  wall_east: x=17, y=[-8,8]
  Tunnel: x=[20,30], y=0, entrance at x=20

Route: south to avoid boxes, then southeast to go under wall_east (y<-8),
       then east past the wall, then north to tunnel entrance.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import sys
import time


# Waypoints to navigate around obstacles to the tunnel
WAYPOINTS_TO_TUNNEL = [
    # Step 1: Go south to avoid box_01 (6,4) and box_05 (3,-8)
    (3.0, -5.0, "South of obstacles"),
    # Step 2: East, between box_03 (10,-3) gap and south
    (8.0, -6.0, "East, avoiding box_03"),
    # Step 3: Further east, clear area
    (12.0, -6.0, "Approaching wall_east"),
    # Step 4: South to go under wall_east (y<-8) and avoid box_14 (14,-8 -> y=[-9,-7])
    (12.0, -11.0, "South of wall_east"),
    # Step 5: Past the wall
    (18.0, -11.0, "Past wall_east"),
    # Step 6: North toward tunnel entrance
    (20.0, -5.0, "Approaching tunnel"),
    # Step 7: Tunnel entrance
    (20.0, 0.0, "Tunnel entrance"),
    # Step 8: Inside tunnel (should trigger GPS->SLAM)
    (25.0, 0.0, "Inside tunnel (GPS should be lost)"),
    # Step 9: Tunnel exit
    (31.0, 0.0, "Tunnel exit (GPS should return)"),
    # Step 10: Past tunnel, clear area
    (33.0, 0.0, "Past tunnel, GPS restored"),
]


class TunnelNavigator(Node):
    def __init__(self):
        super().__init__('tunnel_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_wp = 0
        self._done = False

    def send_goal(self, x, y, label):
        self.get_logger().info(f'--- WP {self._current_wp + 1}/{len(WAYPOINTS_TO_TUNNEL)}: ({x:.1f}, {y:.1f}) - {label} ---')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self._send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        self._send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED!')
            self._advance()
            return
        self.get_logger().info('Goal accepted, navigating...')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'WP {self._current_wp + 1} REACHED!')
        elif status == 6:  # ABORTED
            self.get_logger().warn(f'WP {self._current_wp + 1} ABORTED (path blocked?)')
        elif status == 5:  # CANCELED
            self.get_logger().warn(f'WP {self._current_wp + 1} CANCELED')
        else:
            self.get_logger().warn(f'WP {self._current_wp + 1} finished with status={status}')
        self._advance()

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        remaining = fb.distance_remaining
        if remaining < 1.0 or self._current_wp % 3 == 0:
            self.get_logger().info(
                f'  Distance remaining: {remaining:.2f}m',
                throttle_duration_sec=5.0)

    def _advance(self):
        self._current_wp += 1
        if self._current_wp < len(WAYPOINTS_TO_TUNNEL):
            x, y, label = WAYPOINTS_TO_TUNNEL[self._current_wp]
            self.send_goal(x, y, label)
        else:
            self.get_logger().info('=== ALL WAYPOINTS COMPLETE ===')
            self._done = True

    def run(self):
        if len(WAYPOINTS_TO_TUNNEL) == 0:
            return
        x, y, label = WAYPOINTS_TO_TUNNEL[self._current_wp]
        self.send_goal(x, y, label)

        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.5)


def main():
    rclpy.init()
    nav = TunnelNavigator()
    try:
        nav.run()
    except KeyboardInterrupt:
        pass
    finally:
        nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
