# GPS Mapless Navigation with Dual-EKF Sensor Fusion
# Architecture:
#   ekf_local  → odom->base_footprint (wheel + IMU)
#   ekf_global → map->odom (local EKF + GPS)
#   navsat     → GPS->odom conversion, /fromLL service
#   Nav2       → navigation_launch (no AMCL, no map_server)
#
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

    ekf_local_config = os.path.join(pkg_project, "config", "ekf_local.yaml")
    ekf_global_config = os.path.join(pkg_project, "config", "ekf_global.yaml")
    navsat_config = os.path.join(pkg_project, "config", "navsat.yaml")
    nav2_params = os.path.join(pkg_project, "config", "nav2_params_gps.yaml")

    # ── EKF Local: odom -> base_footprint (smooth, high-freq) ──
    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[ekf_local_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/local")],
    )

    # ── EKF Global: map -> odom (GPS-anchored) ──
    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[ekf_global_config, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    # ── NavSat Transform: GPS -> local odom + /fromLL service ──
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu",               "/imu/data_raw"),
            ("gps/fix",           "/navsat"),
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    # ── Nav2: navigation only (no AMCL, no map_server) ──
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        ekf_local,
        ekf_global,
        navsat,
        nav2,
    ])
