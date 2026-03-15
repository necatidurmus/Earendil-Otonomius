#!/usr/bin/env python3
"""
Mission Manager — Phase-Based Waypoint Orchestrator (v0.4)
=============================================================
YAML'dan fazları okur (GPS waypoints, SLAM waypoints), sıralı goal gönderir.
Faz geçişinde beklenen moda geçişi bekler. Timeout ve retry destekler.

Kullanım:
  ros2 run leo_kernel mission_manager -- --mission /path/to/hybrid_waypoints.yaml
  ya da navigation_hybrid.launch.py tarafından çağrılır.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import FromLL
from std_srvs.srv import Empty as EmptySrv
from std_msgs.msg import String
import tf2_ros
import math
import time
import yaml
import os
import argparse


class MissionManager(Node):
    def __init__(self, mission_file):
        super().__init__('mission_manager')

        self.phases = self._load_mission(mission_file)
        if not self.phases:
            self.get_logger().error('Görev fazları yüklenemedi!')
            raise SystemExit(1)

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # /fromLL service
        self.fromll_client = self.create_client(FromLL, '/fromLL')

        # TF2 listener — SLAM faz başlangıcında offset hesabı için
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Costmap clear service clients
        self._clear_local = self.create_client(
            EmptySrv, '/local_costmap/clear_entirely_local_costmap')
        self._clear_global = self.create_client(
            EmptySrv, '/global_costmap/clear_entirely_global_costmap')

        # GPS monitoring
        self.current_gps = None
        self.create_subscription(NavSatFix, '/navsat', self._gps_cb, 10)

        # Mod monitoring
        self.current_mode = 'GPS'
        self.create_subscription(String, '/nav_mode', self._mode_cb, 10)

        # State
        self.phase_idx = 0
        self.wp_idx = 0
        self._map_cache = {}
        self.results = []
        self.start_time = time.time()
        self.wp_start = 0.0
        self._done = False                  # mission tamamlandığında set edilir
        self._gps_transition_pose = None    # GPS→SLAM geçişinde kaydedilen GPS pozu

        total_wps = sum(len(p.get('waypoints', [])) for p in self.phases)
        self.get_logger().info('=' * 60)
        self.get_logger().info('  MISSION MANAGER v0.4')
        self.get_logger().info(f'  Fazlar : {len(self.phases)}')
        self.get_logger().info(f'  Toplam WP: {total_wps}')
        self.get_logger().info(f'  Dosya  : {mission_file}')
        self.get_logger().info('=' * 60)

    def _load_mission(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('phases', [])
        except Exception as e:
            self.get_logger().error(f'Görev dosyası okunamadı: {e}')
            return []

    def _gps_cb(self, msg):
        self.current_gps = (msg.latitude, msg.longitude, msg.altitude)

    def _mode_cb(self, msg):
        self.current_mode = msg.data

    def _gps_to_map(self, lat, lon):
        if not self.fromll_client.wait_for_service(timeout_sec=10.0):
            return None, None
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0
        future = self.fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            r = future.result()
            return r.map_point.x, r.map_point.y
        return None, None

    def _precompute_all(self):
        """Tüm fazlardaki waypoint'leri map XY'ye çevir.

        GPS fazları: lat/lon → /fromLL servisi ile map XY
        SLAM fazları (forward/lateral): geçiş anında hesaplanır, burada sadece kaydedilir
        """
        self.get_logger().info('Tüm waypoint dönüşümleri yapılıyor...')
        for pi, phase in enumerate(self.phases):
            mode = phase.get('mode', 'GPS').upper()
            wps = phase.get('waypoints', [])
            for wi, wp in enumerate(wps):
                key = (pi, wi)
                if mode == 'SLAM' and 'forward' in wp:
                    fwd = float(wp['forward'])
                    lat = float(wp.get('lateral', 0.0))
                    self._map_cache[key] = ('relative', fwd, lat)
                    self.get_logger().info(
                        f'  [{phase["name"]}] WP{wi+1}: '
                        f'relative (fwd={fwd:.1f}m, lat={lat:.1f}m)')
                elif mode == 'SLAM' and 'x' in wp and 'y' in wp:
                    x, y = float(wp['x']), float(wp['y'])
                    self._map_cache[key] = (x, y)
                    self.get_logger().info(
                        f'  [{phase["name"]}] WP{wi+1}: '
                        f'local ({x:+.2f}, {y:+.2f})')
                else:
                    x, y = self._gps_to_map(wp['lat'], wp['lon'])
                    if x is None:
                        self._map_cache[key] = None
                        self.get_logger().error(
                            f'  [{phase["name"]}] WP{wi+1} dönüşüm başarısız!')
                    else:
                        self._map_cache[key] = (x, y)
                        self.get_logger().info(
                            f'  [{phase["name"]}] WP{wi+1}: '
                            f'({wp["lat"]:.6f}, {wp["lon"]:.6f}) → ({x:+.2f}, {y:+.2f})')

    # ── Ana akış ──────────────────────────────────────────────────────
    def start_mission(self):
        """GPS fix, services bekle, görev başlat."""
        log = self.get_logger()
        log.info('GPS fix bekleniyor...')
        while self.current_gps is None:
            rclpy.spin_once(self, timeout_sec=0.5)

        log.info('/fromLL servisi bekleniyor...')
        if not self.fromll_client.wait_for_service(timeout_sec=30.0):
            log.error('/fromLL servisi bulunamadı!')
            return

        log.info('Nav2 action server bekleniyor...')
        if not self.nav_client.wait_for_server(timeout_sec=60.0):
            log.error('Nav2 action server bulunamadı!')
            return

        self._precompute_all()
        self._start_phase()

    def _start_phase(self):
        """Tüm kalan fazları özyineleme yapmadan iter döngüsü ile çalıştır.

        NOT: Bu metot callback zincirleri içinden de çağrılabilir
        (_result_cb → _send_wp → _start_phase). Bu yüzden mod bekleme
        için spin_once KULLANILMAZ; bunun yerine timer-tabanlı
        polling kullanılır.
        """
        while True:
            if self.phase_idx >= len(self.phases):
                self._print_summary()
                return  # görev bitti; _done flag'ini _print_summary içindeki set eder

            phase = self.phases[self.phase_idx]
            expected_mode = phase.get('mode', 'GPS')
            name = phase.get('name', f'Phase{self.phase_idx+1}')
            wps = phase.get('waypoints', [])

            self.get_logger().info('')
            self.get_logger().info(f'═══ FAZ {self.phase_idx+1}: {name} ═══')
            self.get_logger().info(f'  Beklenen mod: {expected_mode}')
            self.get_logger().info(f'  Waypoint sayısı: {len(wps)}')

            # SLAM fazına geçmeden önce GPS-corrected pozu kaydet
            if expected_mode == 'SLAM':
                self._save_gps_pose()

            # Mod bekleme (timeout: 60s)
            # Hemen uyumluysa devam et; değilse timer ile bekle
            if self.current_mode != expected_mode:
                self.get_logger().info(
                    f'  Mod geçişi bekleniyor: {self.current_mode} → {expected_mode}...')
                self._phase_mode_deadline = time.time() + 60.0
                self._phase_expected_mode = expected_mode
                # Timer ile kontrol et — re-entrant spin yok
                self._mode_wait_timer = self.create_timer(
                    0.5, self._check_mode_transition)
                return  # timer callback devam edecek

            self.get_logger().info(f'  Aktif mod: {self.current_mode}')

            # Mod geçişi sonrası costmap'leri temizle
            self._clear_costmaps()

            if expected_mode == 'SLAM':
                # TF geçiş süresi (1s) tamamlanana kadar bekle
                self.get_logger().info('  TF geçişi için 2s bekleniyor...')
                self._slam_delay_timer = self.create_timer(
                    2.0, self._after_slam_delay)
                return
            self.wp_idx = 0
            self._send_wp()
            return  # WP süreci async; sonraki faz _result_cb zinciriyle başlatılır

    def _after_slam_delay(self):
        """TF geçişi tamamlandıktan sonra SLAM WP'leri hesapla ve navigasyonu başlat."""
        self._slam_delay_timer.cancel()
        self.destroy_timer(self._slam_delay_timer)
        self.get_logger().info('  TF geçişi tamamlandı, SLAM WP hesaplanıyor...')
        self._apply_slam_offset()
        self.wp_idx = 0
        self._send_wp()

    def _check_mode_transition(self):
        """Timer callback: mod geçişini spin_once olmadan bekle."""
        expected = self._phase_expected_mode

        if self.current_mode == expected:
            self._mode_wait_timer.cancel()
            self.destroy_timer(self._mode_wait_timer)
            self.get_logger().info(f'  Aktif mod: {self.current_mode}')

            # Mod geçişi sonrası costmap'leri temizle
            self._clear_costmaps()

            if expected == 'SLAM':
                # TF geçiş süresi (1s) tamamlanana kadar bekle
                self.get_logger().info('  TF geçişi için 2s bekleniyor...')
                self._slam_delay_timer = self.create_timer(
                    2.0, self._after_slam_delay)
                return
            self.wp_idx = 0
            self._send_wp()
            return

        if time.time() > self._phase_mode_deadline:
            self._mode_wait_timer.cancel()
            self.destroy_timer(self._mode_wait_timer)
            phase = self.phases[self.phase_idx]
            name = phase.get('name', f'Phase{self.phase_idx+1}')
            wps = phase.get('waypoints', [])
            self.get_logger().warn(
                f'  Mod {expected} zaman aşımı — faz atlanıyor!')
            for wi, wp in enumerate(wps):
                self.results.append({
                    'phase': name,
                    'wp': wi + 1,
                    'name': wp.get('name', f'WP{wi+1}'),
                    'status': 'MODE_TIMEOUT',
                    'time': 0
                })
            self.phase_idx += 1
            self._start_phase()

    def _save_gps_pose(self):
        """GPS→SLAM geçişinden ÖNCE, GPS-corrected robot pozunu kaydet (artık kullanılmıyor)."""
        pass

    def _clear_costmaps(self):
        """Faz geçişi sonrası local ve global costmap'leri temizle."""
        log = self.get_logger()
        for name, cli in [('local', self._clear_local), ('global', self._clear_global)]:
            if cli.service_is_ready():
                cli.call_async(EmptySrv.Request())
                log.info(f'  Costmap temizlendi: {name}')
            else:
                log.info(f'  Costmap servisi henüz hazır değil: {name}')

    def _apply_slam_offset(self):
        """SLAM fazına girerken, robotun mevcut pozisyon+heading'inden göreceli WP hesapla.

        Forward/lateral tanımlı WP'leri robotun 'odom' frame'deki pozisyonuna
        ve yaw açısına göre mutlak koordinatlara çevirir.

        NEDEN odom frame: map frame SLAM/GPS geçişlerinde değişir (map→map_slam
        identity olunca SLAM Toolbox'un kendi koordinat sistemi görünür).
        odom frame ise Gazebo DiffDrive plugin'i tarafından üretilir,
        mod geçişlerinden etkilenmez ve fiziksel robot pozisyonunu yansıtır.
        Goal'lar da 'odom' frame_id ile gönderilir.
        """
        log = self.get_logger()

        # ── Robotun mevcut pozisyonu odom frame'de (stabil, geçişlerden etkilenmez) ──
        try:
            t = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            robot_x = t.transform.translation.x
            robot_y = t.transform.translation.y
            q = t.transform.rotation
            robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception as e:
            log.warn(f'  SLAM offset: odom TF lookup başarısız: {e}')
            return

        cos_h = math.cos(robot_yaw)
        sin_h = math.sin(robot_yaw)

        log.info(f'  Robot pose (odom): ({robot_x:+.1f},{robot_y:+.1f}) '
                 f'yaw={math.degrees(robot_yaw):.1f}°')

        # ── Faz düzeyinde heading override (tünel ekseni gibi sabit yön) ──
        phase = self.phases[self.phase_idx]
        if 'heading_deg' in phase:
            robot_yaw = math.radians(float(phase['heading_deg']))
            log.info(f'  Heading override: {phase["heading_deg"]}° (tünel ekseni)')

        cos_h = math.cos(robot_yaw)
        sin_h = math.sin(robot_yaw)

        # ── Her SLAM WP'yi heading'e göre odom frame'de hesapla ──
        wps = phase.get('waypoints', [])
        for wi in range(len(wps)):
            key = (self.phase_idx, wi)
            cached = self._map_cache.get(key)
            if cached is None:
                continue
            if isinstance(cached, tuple) and len(cached) == 3 and cached[0] == 'relative':
                _, fwd, lat = cached
                # İleri (forward) = heading yönünde, lateral = heading'e dik (sola +)
                new_x = robot_x + fwd * cos_h - lat * sin_h
                new_y = robot_y + fwd * sin_h + lat * cos_h
                self._map_cache[key] = ('odom', new_x, new_y)
                name = wps[wi].get('name', f'WP{wi+1}')
                log.info(f'    {name}: fwd={fwd:.0f}m lat={lat:.0f}m → '
                         f'odom({new_x:+.1f},{new_y:+.1f})')

    def _send_wp(self):
        phase = self.phases[self.phase_idx]
        wps = phase.get('waypoints', [])

        if self.wp_idx >= len(wps):
            self.phase_idx += 1
            self._start_phase()
            return

        wp = wps[self.wp_idx]
        name = wp.get('name', f'WP{self.wp_idx+1}')
        cached = self._map_cache.get((self.phase_idx, self.wp_idx))

        if cached is None:
            self.get_logger().error(f'  {name}: koordinat yok, atlanıyor')
            self.results.append({
                'phase': phase.get('name', ''),
                'wp': self.wp_idx + 1, 'name': name,
                'status': 'SKIP', 'time': 0
            })
            self.wp_idx += 1
            self._send_wp()
            return

        # Relative WP henüz çözülmediyse atla
        if isinstance(cached, tuple) and len(cached) == 3 and cached[0] == 'relative':
            self.get_logger().error(f'  {name}: relative WP çözülemedi, atlanıyor')
            self.results.append({
                'phase': phase.get('name', ''),
                'wp': self.wp_idx + 1, 'name': name,
                'status': 'SKIP', 'time': 0
            })
            self.wp_idx += 1
            self._send_wp()
            return

        # odom frame WP: ('odom', x, y)
        if isinstance(cached, tuple) and len(cached) == 3 and cached[0] == 'odom':
            frame_id = 'odom'
            x, y = cached[1], cached[2]
        else:
            frame_id = 'map'
            x, y = cached

        # Sonraki WP'ye yaw hesapla
        yaw = 0.0
        next_cached = self._map_cache.get((self.phase_idx, self.wp_idx + 1))
        if next_cached and isinstance(next_cached, tuple):
            if len(next_cached) == 2:
                yaw = math.atan2(next_cached[1] - y, next_cached[0] - x)
            elif len(next_cached) == 3 and next_cached[0] == 'odom':
                yaw = math.atan2(next_cached[2] - y, next_cached[1] - x)

        self.get_logger().info(
            f'  [{self.wp_idx+1}/{len(wps)}] {name} → {frame_id}({x:+.2f}, {y:+.2f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame_id
        goal.pose.header.stamp = rclpy.time.Time().to_msg()  # Time(0) = latest TF
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.wp_start = time.time()
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        phase = self.phases[self.phase_idx]
        wps = phase.get('waypoints', [])
        name = wps[self.wp_idx].get('name', '')

        if not handle.accepted:
            self.get_logger().error(f'  {name}: REDDEDİLDİ!')
            self.results.append({
                'phase': phase.get('name', ''),
                'wp': self.wp_idx + 1, 'name': name,
                'status': 'REJECTED', 'time': 0
            })
            self.wp_idx += 1
            self._send_wp()
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        pos = fb.current_pose.pose.position
        self.get_logger().info(
            f'    Nav:({pos.x:+.1f},{pos.y:+.1f}) Kalan:{fb.distance_remaining:.1f}m '
            f'Mod:{self.current_mode}',
            throttle_duration_sec=4.0)

    def _result_cb(self, future):
        result = future.result()
        elapsed = round(time.time() - self.wp_start, 1)
        phase = self.phases[self.phase_idx]
        wps = phase.get('waypoints', [])
        name = wps[self.wp_idx].get('name', '')
        status_map = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        status = status_map.get(result.status, f'UNKNOWN({result.status})')

        self.results.append({
            'phase': phase.get('name', ''),
            'wp': self.wp_idx + 1, 'name': name,
            'status': status, 'time': elapsed
        })

        if result.status == 4:
            self.get_logger().info(f'  ✅ {name} → {elapsed}s')
        else:
            self.get_logger().warn(f'  ⚠️ {name}: {status} ({elapsed}s)')

        self.wp_idx += 1
        self._send_wp()

    def _print_summary(self):
        total = round(time.time() - self.start_time, 1)
        log = self.get_logger()
        log.info('')
        log.info('=' * 65)
        log.info('  MISSION RESULTS')
        log.info('=' * 65)

        current_phase = ''
        for r in self.results:
            if r['phase'] != current_phase:
                current_phase = r['phase']
                log.info(f'\n  ── {current_phase} ──')
            log.info(
                f"  WP{r['wp']}: {r['status']:12s} {r['time']}s  {r['name']}")

        ok = sum(1 for r in self.results if r['status'] == 'SUCCEEDED')
        log.info(f'\n  Toplam: {ok}/{len(self.results)} başarılı  |  {total}s')
        log.info('=' * 65)

        # Görev tamam—döngüyü callback içinden güvenli kapat
        self._done = True
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Mission Manager')
    parser.add_argument('--mission', '-m', type=str, required=False,
                        help='Görev YAML dosyası yolu')
    args, unknown = parser.parse_known_args()

    # Varsayılan görev dosyası
    if args.mission is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('leo_kernel')
            args.mission = os.path.join(pkg, 'config', 'hybrid_waypoints.yaml')
        except Exception:
            args.mission = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'hybrid_waypoints.yaml')

    rclpy.init()
    node = MissionManager(args.mission)
    time.sleep(2)
    node.start_mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
