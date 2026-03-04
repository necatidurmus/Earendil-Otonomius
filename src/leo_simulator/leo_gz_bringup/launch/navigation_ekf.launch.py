# Copyright 2026 Necati Durmus
# Navigation with EKF sensor fusion (GPS + Odometry + IMU)
# Usage: ros2 launch leo_gz_bringup navigation_ekf.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_project = get_package_share_directory("leo_gz_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    ekf_config = os.path.join(pkg_project, "config", "ekf.yaml")
    nav2_params = os.path.join(pkg_project, "config", "nav2_params.yaml")
    default_map = os.path.join(
        pkg_project, "maps", "empty_50x50_map.yaml"
    )

    # EKF node — fuses odom + IMU
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/filtered")],
    )

    # NavSat Transform — converts GPS to local frame, outputs /odometry/gps
    # EKF fuses this GPS odometry as odom1 for position correction
    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu",               "/imu/data_raw"),
            ("gps/fix",           "/navsat"),
            ("odometry/filtered", "odometry/filtered"),
        ],
    )

    # Static TF: map -> odom (required for Nav2 TF chain)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": default_map,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        ekf_node,
        navsat_node,
        static_tf,
        nav2_bringup,
    ])
