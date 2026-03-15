# Hybrid GPS + SLAM Navigation Launch — Kernel Mode
# =====================================================
# Simülasyondan arındırılmış otonom navigasyon çekirdeği.
#
# Mimari:
#   AÇIK ALAN  : GPS + Dual-UKF → /odometry/filtered
#   KAPALI ALAN: SLAM Toolbox (LiDAR) → map_slam→odom TF
#
# TF Zinciri: map → map_slam → odom → base_footprint
#   DiffDrive    → odom→base_footprint
#   SLAM Toolbox → map_slam→odom
#   UKF Global   → /odometry/filtered (publish_tf: false)
#   tf_mode_relay → map→map_slam (GPS: correction, SLAM: identity)
#
# Mod geçişi:
#   gps_monitor: /navsat GPS kalitesini izler → /nav_mode yayınlar
#   tf_mode_relay: /nav_mode'a göre GPS veya SLAM TF yayınlar
#
# Startup sırası:
#   t=0s : UKF Local + SLAM Toolbox + GPS Monitor + TF Relay
#   t=2s : UKF Global
#   t=4s : NavSat Transform
#   t=8s : Nav2

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_project      = get_package_share_directory('leo_kernel')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Config dosyaları ─────────────────────────────────────────────────────
    ukf_local_config  = os.path.join(pkg_project, 'config', 'ukf_local.yaml')
    ukf_global_config = os.path.join(pkg_project, 'config', 'ukf_global_hybrid.yaml')
    navsat_config     = os.path.join(pkg_project, 'config', 'navsat.yaml')
    slam_config       = os.path.join(pkg_project, 'config', 'slam_toolbox_params.yaml')
    nav2_params       = os.path.join(pkg_project, 'config', 'nav2_params_hybrid.yaml')

    # ── UKF Local: odom + IMU → /odometry/local ──────────────────────────────
    ukf_local_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_local_node',
        output='screen',
        parameters=[ukf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    # ── SLAM Toolbox: LiDAR → harita ─────────────────────────────────────────
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    # ── GPS Quality Monitor → /nav_mode ───────────────────────────────────────
    gps_monitor = Node(
        package='leo_kernel',
        executable='gps_monitor.py',
        name='gps_monitor',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'gps_to_slam_threshold': 0.3,
            'slam_to_gps_threshold': 0.6,
            'window_size':           10,
            'hysteresis_secs':       3.0,
            'gps_timeout_secs':      3.0,
            'confirmation_count':    3,
            'input_topic':           '/navsat',
            'verbose':               True,
        }],
    )

    # ── TF Mode Relay: GPS/SLAM mod geçişini map→map_slam TF ile uygular ───
    tf_mode_relay = Node(
        package='leo_kernel',
        executable='tf_mode_relay.py',
        name='tf_mode_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── NavSat Transform: GPS → /odometry/gps + /fromLL service ─────────────
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu',     '/imu/data_raw'),
            ('gps/fix', '/navsat'),
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    # ── UKF Global: /odometry/local + /odometry/gps → /odometry/filtered ────
    ukf_global_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[ukf_global_config, {'use_sim_time': use_sim_time}],
    )

    # ── Nav2 (navigation only, no AMCL, no map_server) ───────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
            'autostart':    'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # t=0s — sensor fusion + SLAM + GPS monitor + relay
        ukf_local_node,
        slam_toolbox_node,
        gps_monitor,
        tf_mode_relay,

        # Zamanlı başlatmalar
        TimerAction(period=2.0,  actions=[ukf_global_node]),
        TimerAction(period=4.0,  actions=[navsat]),
        TimerAction(period=8.0,  actions=[nav2]),
    ])
