# Hybrid GPS + SLAM Navigation Launch (v0.4)
# =============================================
# Mimari:
#   AÇIK ALAN  : GPS + Dual-UKF → /odometry/filtered (map_gps frame)
#   KAPALI ALAN: SLAM Toolbox (LiDAR) → map_slam→odom TF
#
# TF Frame Ownership (v0.5):
#   DiffDrive    → odom→base_footprint   (her zaman, ana /tf)
#   SLAM Toolbox → map_slam→odom         (her zaman, ana /tf)
#   UKF Global   → /odometry/filtered    (publish_tf: false, sadece topic)
#   tf_mode_relay → map→map_slam         (GPS: correction offset,
#                                          SLAM: identity)
#   TF zinciri: map → map_slam → odom → base_footprint → ...
#   Nav2 → global_frame: map             (tek ağaç, multi-parent yok)
#
# Mod geçişi:
#   tunnel_gps_spoofer: /navsat → /navsat_filtered (tünelde STATUS_NO_FIX)
#   gps_monitor: /navsat_filtered'dan GPS kalitesini izler → /nav_mode yayınlar
#   tf_mode_relay: /nav_mode'a göre GPS veya SLAM TF'sini map→odom yapar
#
# Startup sırası:
#   t=0s  : UKF Local + SLAM Toolbox + GPS Monitor + Spoofer + Relay
#           + GPS Monitor + Spoofer + Relay
#   t=4s  : UKF Global (ukf_global_hybrid.yaml → map_gps frame)
#   t=8s  : NavSat Transform (gps/fix → /navsat_filtered)
#   t=40s : Nav2
#   t=42s : RViz (opsiyonel)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    pkg_nav2_bringup  = get_package_share_directory('nav2_bringup')
    pkg_project       = get_package_share_directory('leo_gz_bringup')

    use_sim_time  = LaunchConfiguration('use_sim_time')
    launch_rviz   = LaunchConfiguration('launch_rviz')
    use_spoofer   = LaunchConfiguration('use_spoofer')

    # GPS monitor input_topic: spoofer açıksa /navsat_filtered, kapalıysa /navsat
    gps_input_topic = PythonExpression([
        "'/navsat_filtered' if '", use_spoofer, "' == 'true' else '/navsat'"
    ])

    # ── Config dosyaları ─────────────────────────────────────────────────────
    ukf_local_config   = os.path.join(pkg_project, 'config', 'ukf_local.yaml')
    ukf_global_config  = os.path.join(pkg_project, 'config', 'ukf_global_hybrid.yaml')
    navsat_config      = os.path.join(pkg_project, 'config', 'navsat.yaml')
    slam_config        = os.path.join(pkg_project, 'config', 'slam_toolbox_params.yaml')
    nav2_params        = os.path.join(pkg_project, 'config', 'nav2_params_hybrid.yaml')

    # ── Bootstrap: static TF yok — artık gerekmez ─────────────────
    # SLAM Toolbox kendi map_slam→odom TF'sini yayınlar.
    # UKF Global publish_tf: false — TF yayınlamaz.
    # tf_mode_relay başlangıçta identity map→odom TF yayınlar (self-bootstrap).

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
    # map_slam→odom TF'yi ana /tf'ye yayınlar.
    # tf_mode_relay SLAM modunda bunu map→odom olarak relay eder.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    # ── GPS Quality Monitor + Mod Kontrol ─────────────────────────────────────
    # /navsat_filtered'dan GPS kalitesini izler, /nav_mode yayınlar
    gps_monitor = Node(
        package='leo_gz_bringup',
        executable='gps_monitor.py',
        name='gps_monitor',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'gps_to_slam_threshold': _t(['gps_monitor','gps_to_slam_threshold'], 0.3),
            'slam_to_gps_threshold': _t(['gps_monitor','slam_to_gps_threshold'], 0.6),
            'window_size':           _t(['gps_monitor','window_size'], 10),
            'hysteresis_secs':       _t(['gps_monitor','hysteresis_secs'], 3.0),
            'gps_timeout_secs':      _t(['gps_monitor','gps_timeout_secs'], 3.0),
            'confirmation_count':    _t(['gps_monitor','confirmation_count'], 3),
            'input_topic':           gps_input_topic,
            'verbose':               _t(['gps_monitor','verbose'], True),
        }],
    )

    # ── Tunnel GPS Spoofer: /navsat → /navsat_filtered ─────────────────
    # Tünelde STATUS_NO_FIX, dışarıda orijinal GPS'i geçirir────────────
    tunnel_spoofer = Node(
        package='leo_gz_bringup',
        executable='tunnel_gps_spoofer.py',
        name='tunnel_gps_spoofer',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'verbose':       _t(['gps_monitor','verbose'], True),
            'tunnel_x_min':  _t(['tunnel','x_min'], 19.5),
            'tunnel_x_max':  _t(['tunnel','x_max'], 30.5),
            'tunnel_y_max':  _t(['tunnel','y_max'], 1.85),
            'door_x_min':    _t(['door','x_min'],   33.5),
            'door_x_max':    _t(['door','x_max'],   36.5),
            'door_y_max':    _t(['door','y_max'],    1.0),
            'map_frame':     'map',  # map frame = düzeltilmiş dünya koordinatları (odom drift eder)
        }],
        condition=IfCondition(use_spoofer),
    )

    # ── TF Mode Relay: GPS/SLAM mod geçişini deterministic TF ile uygular ──
    # map_gps→odom veya map_slam→odom'u map→odom olarak relay eder
    tf_mode_relay = Node(
        package='leo_gz_bringup',
        executable='tf_mode_relay.py',
        name='tf_mode_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── NavSat Transform: GPS → /odometry/gps + /fromLL service ─────────────
    # Hybrid modda: spoofer açıksa /navsat_filtered, kapalıysa /navsat okur
    navsat_gps_topic = PythonExpression([
        "'/navsat_filtered' if '", use_spoofer, "' == 'true' else '/navsat'"
    ])
    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu',     '/imu/data_raw'),
            ('gps/fix', navsat_gps_topic),
            # UKF Global map_gps→odom TF'si artık ayrı topic'te olduğundan
            # navsat_transform "map_gps→base_footprint" TF bulamıyordu.
            # /odometry/local (frame_id: odom) kullanarak odom→base_footprint
            # TF'sini bulabilir (bu her zaman Gazebo DiffDrive tarafından yayınlanır).
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    # ── UKF Global: /odometry/local + /odometry/gps → /odometry/filtered ─────
    # UKF Global: /odometry/local + /odometry/gps → /odometry/filtered
    # publish_tf: false — TF çakışmasını önler, tf_mode_relay TF hesaplar.
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

    # ── RViz (opsiyonel — launch_rviz:=true ile) ──────────────────────────────
    rviz_config = os.path.join(pkg_project, 'config', 'rviz_hybrid.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    # ── Timing değerleri sim_config.yaml'dan ───────────────────────────────
    T_UKF_GLOBAL = float(_t(['timing','ukf_global_start'],  4.0))
    T_NAVSAT     = float(_t(['timing','navsat_start'],       8.0))
    T_NAV2       = float(_t(['timing','nav2_start'],        40.0))
    T_RVIZ       = float(_t(['timing','rviz_start'],        42.0))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='RViz gorsellestrmeyi baslat'),
        DeclareLaunchArgument(
            'use_spoofer', default_value='true',
            description='Tunel GPS spoofer kullan (sim icin true, gercek robot icin false)'),

        # t=0s — sensor fusion + SLAM + GPS monitor + spoofer + relay
        ukf_local_node,
        slam_toolbox_node,
        gps_monitor,
        tunnel_spoofer,
        tf_mode_relay,

        # Zamanlı başlatmalar
        TimerAction(period=T_UKF_GLOBAL, actions=[ukf_global_node]),
        TimerAction(period=T_NAVSAT,     actions=[navsat]),
        TimerAction(period=T_NAV2,       actions=[nav2]),
        TimerAction(period=T_RVIZ,       actions=[rviz]),
    ])
