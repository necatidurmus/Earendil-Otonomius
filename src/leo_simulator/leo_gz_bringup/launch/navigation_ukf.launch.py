# GPS Mapless Navigation with Dual-UKF + NavSat Sensor Fusion
#
# Architecture:
#   DiffDrive    → odom→base_footprint (Gazebo diff_drive plugin)
#   UKF Local    → /odometry/local   (odom+IMU, world_frame: odom)
#   UKF Global   → /odometry/filtered (local velocities, world_frame: map)
#                → map→odom TF       (corrected by GPS once NavSat feeds in)
#   NavSat       → /odometry/gps     (GPS→map, yaw from /odometry/filtered [UKF Global])
#   Nav2         → goal-following    (no AMCL, no map_server)
#
# Startup sequence:
#   t=0s  :  static_tf bootstrap + UKF Local (odom+IMU — no GPS dependency)
#   t=4s  :  UKF Global (needs /odometry/local; starts before NavSat so it can
#             publish /odometry/filtered which NavSat needs for yaw)
#   t=8s  :  NavSat (needs /odometry/filtered for yaw; outputs /odometry/gps in map frame)
#   t=25s :  Nav2 (needs complete TF chain: map→odom→base_footprint)
#
# Usage:
#   ros2 launch leo_gz_bringup navigation_ukf.launch.py
#   ros2 launch leo_gz_bringup navigation_ukf.launch.py use_sim_time:=true

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_project     = get_package_share_directory("leo_gz_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    ukf_local_config  = os.path.join(pkg_project, "config", "ukf_local.yaml")
    ukf_global_config = os.path.join(pkg_project, "config", "ukf_global.yaml")
    navsat_config     = os.path.join(pkg_project, "config", "navsat.yaml")
    nav2_params       = os.path.join(pkg_project, "config", "nav2_params_gps.yaml")

    # ── Bootstrap: static map→odom identity TF ─────────────────────────
    # UKF Global overrides this as soon as GPS data flows in.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── UKF Local: odom + IMU → /odometry/local ─────────────────────────
    # world_frame: odom — continuous, drift-free short-term estimate.
    # publish_tf: false — DiffDrive already handles odom→base_footprint.
    ukf_local_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_local_node",
        output="screen",
        parameters=[ukf_local_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "/odometry/local"),
        ],
    )

    # ── NavSat Transform: GPS → /odometry/gps + /fromLL service ─────────
    # Uses /odometry/filtered (UKF Global output, frame=map) for yaw.
    # This way /odometry/gps is published in map frame → UKF Global
    # can correctly anchor robot position.  The loop converges:
    #   UKF Global → /odometry/filtered(map) → NavSat → /odometry/gps(map)
    #   → UKF Global corrects map→odom TF  (no divergence)
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu",     "/imu/data_raw"),
            ("gps/fix", "/navsat"),
            # No odometry/filtered remapping → uses /odometry/filtered from UKF Global
        ],
    )

    # ── UKF Global: /odometry/local + /odometry/gps → map→odom TF ───────
    # world_frame: map — GPS-anchored global localization.
    # Publishes /odometry/filtered consumed by Nav2.
    ukf_global_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[ukf_global_config, {"use_sim_time": use_sim_time}],
    )

    # ── Nav2: navigation only (no AMCL, no map_server) ──────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file":  nav2_params,
            "autostart":    "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # t=0s — bootstrap TF + UKF Local (no external dependencies)
        static_tf,
        ukf_local_node,

        # t=4s — UKF Global (needs /odometry/local; starts early to publish /odometry/filtered for NavSat)
        TimerAction(period=4.0, actions=[ukf_global_node]),

        # t=8s — NavSat (needs /odometry/filtered from UKF Global for yaw; outputs /odometry/gps in map frame)
        TimerAction(period=8.0, actions=[navsat]),

        # t=25s — Nav2 (needs map→odom→base_footprint TF chain; extra margin for node services)
        TimerAction(period=25.0, actions=[nav2]),
    ])
