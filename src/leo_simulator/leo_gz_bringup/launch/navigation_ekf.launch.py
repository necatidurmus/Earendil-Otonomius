# GPS Mapless Navigation with EKF + NavSat Sensor Fusion
# Architecture:
#   DiffDrive   → odom->base_footprint (Gazebo)
#   EKF         → map->odom (wheel+IMU+GPS fused, world_frame: map)
#   navsat      → GPS->odom conversion, /fromLL service
#   Nav2        → navigation_launch (no AMCL, no map_server)
#
# Bootstrap: static map->odom identity TF ensures EKF can start,
#            then EKF overrides it with GPS-corrected transform.
#
# Usage: ros2 launch leo_gz_bringup navigation_ekf.launch.py

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ── sim_config.yaml oku ──────────────────────────────────────────────────
def _load_config():
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidate = os.path.join(here, 'sim_config.yaml')
        if os.path.isfile(candidate):
            with open(candidate) as f:
                return yaml.safe_load(f)
        here = os.path.dirname(here)
    return {}

_CFG = _load_config()

def _t(keys, default):
    d = _CFG
    for k in keys:
        if not isinstance(d, dict) or k not in d:
            return default
        d = d[k]
    return d


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_project = get_package_share_directory("leo_gz_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")

    ekf_config = os.path.join(pkg_project, "config", "ekf_global.yaml")
    navsat_config = os.path.join(pkg_project, "config", "navsat.yaml")
    nav2_params = os.path.join(pkg_project, "config", "nav2_params_gps.yaml")

    # ── Bootstrap: static map -> odom (identity, EKF overrides later) ──
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── EKF: map -> odom (GPS-corrected, overrides static TF) ──
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    # ── NavSat Transform: GPS -> odometry/gps + /fromLL service ──
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[navsat_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("imu",               "/imu/data_raw"),
            ("gps/fix",           "/navsat"),
            ("odometry/filtered", "/odometry/filtered"),
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

    # ── Timing değerleri sim_config.yaml'dan ───────────────────────────
    # EKF mimarisi daha basit, navsat daha erken başlayabilir
    T_NAVSAT = float(_t(['timing', 'navsat_start'], 8.0)) / 2.0   # EKF tek katman, daha hızlı
    T_NAV2   = float(_t(['timing', 'nav2_start'],  40.0))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Static TF bootstrap (EKF overrides once data flows)
        static_tf,
        # EKF starts immediately
        ekf_node,
        # NavSat needs EKF filtered output
        TimerAction(period=T_NAVSAT, actions=[navsat]),
        # Nav2 needs map->odom->base_footprint TF chain
        TimerAction(period=T_NAV2, actions=[nav2]),
    ])
