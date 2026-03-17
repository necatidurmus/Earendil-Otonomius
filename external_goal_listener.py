#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String


STATUS_TEXT = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class ExternalGoalListener(Node):
    def __init__(self) -> None:
        super().__init__('external_goal_listener')

        self.declare_parameter('goal_topic', '/external_goal')
        self.declare_parameter('cancel_topic', '/external_goal_cancel')
        self.declare_parameter('status_topic', '/external_goal_status')
        self.declare_parameter('default_frame', 'map')
        self.declare_parameter('allow_preempt', True)
        self.declare_parameter('wait_for_server_sec', 60.0)
        self.declare_parameter('feedback_log_period_sec', 1.0)

        self.goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        self.cancel_topic = self.get_parameter('cancel_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.default_frame = self.get_parameter('default_frame').get_parameter_value().string_value
        self.allow_preempt = self.get_parameter('allow_preempt').get_parameter_value().bool_value
        self.wait_for_server_sec = self.get_parameter('wait_for_server_sec').get_parameter_value().double_value
        self.feedback_log_period_sec = self.get_parameter('feedback_log_period_sec').get_parameter_value().double_value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self._goal_cb, 10)
        self.cancel_sub = self.create_subscription(Bool, self.cancel_topic, self._cancel_cb, 10)

        self._active_goal_handle = None
        self._pending_goal: Optional[PoseStamped] = None
        self._last_feedback_log_time = 0.0

        self.get_logger().info('external_goal_listener basliyor...')
        self.get_logger().info(f'Goal topic   : {self.goal_topic}')
        self.get_logger().info(f'Cancel topic : {self.cancel_topic}')
        self.get_logger().info(f'Status topic : {self.status_topic}')
        self.get_logger().info(f'Default frame: {self.default_frame}')
        self.get_logger().info(f'Preempt      : {self.allow_preempt}')

        if self.nav_client.wait_for_server(timeout_sec=self.wait_for_server_sec):
            self._publish_status('READY')
            self.get_logger().info('navigate_to_pose action server hazir.')
        else:
            self._publish_status('SERVER_NOT_READY')
            self.get_logger().error('navigate_to_pose action server bulunamadi.')

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _normalize_pose(self, msg: PoseStamped) -> PoseStamped:
        if not msg.header.frame_id:
            msg.header.frame_id = self.default_frame

        q = msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm == 0.0:
            msg.pose.orientation.w = 1.0
        return msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        msg = self._normalize_pose(msg)
        x = msg.pose.position.x
        y = msg.pose.position.y
        frame = msg.header.frame_id

        self.get_logger().info(f'Yeni external goal alindi: frame={frame}, x={x:.3f}, y={y:.3f}')
        self._publish_status(f'GOAL_RECEIVED frame={frame} x={x:.3f} y={y:.3f}')

        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Action server hazir degil, goal reddedildi.')
            self._publish_status('SERVER_NOT_READY')
            return

        if self._active_goal_handle is not None:
            if not self.allow_preempt:
                self.get_logger().warn('Aktif bir goal varken yeni goal yoksayildi.')
                self._publish_status('GOAL_IGNORED_ACTIVE_GOAL')
                return

            self.get_logger().info('Aktif goal iptal edilip yeni goal gonderilecek...')
            self._pending_goal = msg
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_for_preempt)
            self._publish_status('PREEMPT_REQUESTED')
            return

        self._send_goal(msg)

    def _cancel_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._active_goal_handle is None:
            self.get_logger().info('Iptal istegi geldi ama aktif goal yok.')
            self._publish_status('CANCEL_IGNORED_NO_ACTIVE_GOAL')
            return

        self.get_logger().info('Aktif goal icin iptal istegi gonderiliyor...')
        cancel_future = self._active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_manual)
        self._publish_status('CANCEL_REQUESTED')

    def _cancel_done_for_preempt(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f'Preempt icin cancel hatasi: {exc}')
            self._publish_status('PREEMPT_CANCEL_FAILED')
            return

        self._active_goal_handle = None
        pending = self._pending_goal
        self._pending_goal = None
        self._publish_status('PREEMPT_CANCEL_DONE')
        if pending is not None:
            self._send_goal(pending)

    def _cancel_done_manual(self, future) -> None:
        try:
            future.result()
            self.get_logger().info('Cancel istegi Nav2 tarafina iletildi.')
            self._publish_status('CANCEL_SENT_TO_NAV2')
        except Exception as exc:
            self.get_logger().error(f'Cancel hatasi: {exc}')
            self._publish_status('CANCEL_FAILED')

    def _send_goal(self, pose: PoseStamped) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

        frame = pose.header.frame_id
        x = pose.pose.position.x
        y = pose.pose.position.y
        self.get_logger().info(f'Goal Nav2\'ya gonderildi: frame={frame}, x={x:.3f}, y={y:.3f}')
        self._publish_status(f'GOAL_SENT frame={frame} x={x:.3f} y={y:.3f}')

    def _goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Goal response hatasi: {exc}')
            self._publish_status('GOAL_RESPONSE_ERROR')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Goal Nav2 tarafindan reddedildi.')
            self._publish_status('GOAL_REJECTED')
            self._active_goal_handle = None
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info('Goal kabul edildi.')
        self._publish_status('GOAL_ACCEPTED')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_feedback_log_time < self.feedback_log_period_sec:
            return
        self._last_feedback_log_time = now

        try:
            dist = feedback_msg.feedback.distance_remaining
            eta = feedback_msg.feedback.estimated_time_remaining.sec
            self.get_logger().info(f'Feedback: distance_remaining={dist:.3f} m, eta~{eta}s')
            self._publish_status(f'FEEDBACK dist={dist:.3f} eta={eta}')
        except Exception:
            self.get_logger().info('Feedback alindi.')
            self._publish_status('FEEDBACK')

    def _result_cb(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Result hatasi: {exc}')
            self._publish_status('RESULT_ERROR')
            self._active_goal_handle = None
            return

        status = int(result.status)
        status_text = STATUS_TEXT.get(status, f'CODE_{status}')
        self.get_logger().info(f'Goal sonucu: {status_text}')
        self._publish_status(f'GOAL_{status_text}')
        self._active_goal_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExternalGoalListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
