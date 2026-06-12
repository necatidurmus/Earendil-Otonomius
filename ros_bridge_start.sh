#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/ros/ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist" \
  "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry" \
  "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V" \
  "/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU" \
  "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model" \
  "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan" \
  "/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat" \
  --ros-args -r __node:=parameter_bridge \
  -p "qos_overrides./tf_static.publisher.durability:=transient_local"
