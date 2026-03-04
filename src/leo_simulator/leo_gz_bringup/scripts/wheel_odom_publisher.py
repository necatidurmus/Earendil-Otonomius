#!/usr/bin/env python3
"""
Wheel Odometry Publisher
Subscribes to /joint_states and publishes individual wheel odometry
for each of the 4 wheels as separate topics.

Published topics:
  /wheel_odom/FL  (nav_msgs/Odometry) - Front Left wheel
  /wheel_odom/FR  (nav_msgs/Odometry) - Front Right wheel
  /wheel_odom/RL  (nav_msgs/Odometry) - Rear Left wheel
  /wheel_odom/RR  (nav_msgs/Odometry) - Rear Right wheel

Each wheel odom contains:
  - Linear distance traveled (from wheel rotation * radius)
  - Linear velocity (angular_vel * radius)
  - Angular position and velocity of the wheel
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math


class WheelOdomPublisher(Node):
    # Wheel radius from URDF (macros.xacro)
    WHEEL_RADIUS = 0.125  # meters

    # Joint name to wheel mapping
    WHEEL_JOINTS = {
        'wheel_FL_joint': 'FL',
        'wheel_FR_joint': 'FR',
        'wheel_RL_joint': 'RL',
        'wheel_RR_joint': 'RR',
    }

    def __init__(self):
        super().__init__('wheel_odom_publisher')

        # Declare parameters
        self.declare_parameter('wheel_radius', self.WHEEL_RADIUS)
        self.wheel_radius = self.get_parameter('wheel_radius').value

        # Previous values for delta calculations
        self.prev_positions = {}
        self.prev_time = None

        # Publishers for each wheel
        self.wheel_pubs = {}
        for joint_name, wheel_id in self.WHEEL_JOINTS.items():
            topic = f'/wheel_odom/{wheel_id}'
            self.wheel_pubs[joint_name] = self.create_publisher(
                Odometry, topic, 10
            )
            self.get_logger().info(f'Publishing wheel odom: {topic}')

        # Subscribe to joint states
        self.create_subscription(
            JointState, '/joint_states', self.joint_states_cb, 10
        )

        self.get_logger().info(
            f'Wheel Odom Publisher started (radius={self.wheel_radius}m)'
        )

    def joint_states_cb(self, msg: JointState):
        current_time = self.get_clock().now()

        for i, joint_name in enumerate(msg.name):
            if joint_name not in self.WHEEL_JOINTS:
                continue

            wheel_id = self.WHEEL_JOINTS[joint_name]
            position = msg.position[i]  # radians
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

            # Linear distance and velocity
            linear_distance = position * self.wheel_radius  # meters
            linear_velocity = velocity * self.wheel_radius  # m/s

            # Build odometry message
            odom = Odometry()
            odom.header.stamp = msg.header.stamp if msg.header.stamp.sec > 0 \
                else current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = f'wheel_{wheel_id}_link'

            # Position: total distance traveled by this wheel
            odom.pose.pose.position.x = linear_distance
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.position.z = 0.0

            # Orientation from wheel angle (rotation around Y axis)
            half_angle = position / 2.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = math.sin(half_angle)
            odom.pose.pose.orientation.z = 0.0
            odom.pose.pose.orientation.w = math.cos(half_angle)

            # Velocity
            odom.twist.twist.linear.x = linear_velocity
            odom.twist.twist.angular.z = velocity  # angular vel of wheel

            self.wheel_pubs[joint_name].publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
