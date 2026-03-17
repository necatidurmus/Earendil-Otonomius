#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool


class GoalPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_bridge', parameter_overrides=[
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.parameter.Parameter.Type.BOOL,
                True,
            )
        ])

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10,
        )
        self.cancel_sub = self.create_subscription(
            Bool,
            '/external_goal_cancel',
            self.cancel_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, '/external_goal_status', 10)

        self.current_goal_handle = None
        self.current_goal_future = None
        self.current_result_future = None

        self.publish_status('READY: waiting for /goal_pose')
        self.get_logger().info('GoalPoseBridge hazır. /goal_pose dinleniyor → navigate_to_pose')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def goal_callback(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'

        # stamp boş gelirse doldur
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1e-9:
            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
        else:
            msg.pose.orientation.x /= norm
            msg.pose.orientation.y /= norm
            msg.pose.orientation.z /= norm
            msg.pose.orientation.w /= norm

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.publish_status('ERROR: navigate_to_pose action server bulunamadı')
            return

        self.publish_status(
            f'GOAL_RECEIVED: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}, frame={msg.header.frame_id}'
        )

        # Varsa mevcut goal’i iptal et
        if self.current_goal_handle is not None:
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                self.publish_status('INFO: previous goal cancel requested')
            except Exception as exc:
                self.publish_status(f'WARN: previous goal cancel failed: {exc}')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        self.current_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        self.current_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.publish_status(f'ERROR: send_goal failed: {exc}')
            return

        if not goal_handle.accepted:
            self.current_goal_handle = None
            self.publish_status('GOAL_REJECTED')
            return

        self.current_goal_handle = goal_handle
        self.publish_status('GOAL_ACCEPTED')
        self.current_result_future = goal_handle.get_result_async()
        self.current_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        pose = feedback.current_pose.pose.position
        self.publish_status(f'FEEDBACK: x={pose.x:.2f}, y={pose.y:.2f}')

    def result_callback(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.publish_status(f'ERROR: result failed: {exc}')
            return

        status = result.status
        status_map = {
            0: 'UNKNOWN',
            1: 'ACCEPTED',
            2: 'EXECUTING',
            3: 'CANCELING',
            4: 'SUCCEEDED',
            5: 'CANCELED',
            6: 'ABORTED',
        }
        self.publish_status(f'GOAL_{status_map.get(status, str(status))}')
        self.current_goal_handle = None
        self.current_goal_future = None
        self.current_result_future = None

    def cancel_callback(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.current_goal_handle is None:
            self.publish_status('INFO: no active goal to cancel')
            return

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda _: self.publish_status('CANCEL_REQUESTED'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
