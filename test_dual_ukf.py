#!/usr/bin/env python3
"""
Dual-UKF GPS Navigasyon Doğrulama Testi
========================================
Dual-UKF mimarisini test eder:
  UKF Local  (odom+IMU)        → /odometry/local
  UKF Global (local+GPS)       → /odometry/filtered  +  map→odom TF
  NavSat     (GPS→map dönüşüm) → /odometry/gps       +  /fromLL servisi

5 GPS waypoint'ini sırayla gezer ve her aşamada sensör füzyon
durumunu raporlar.

Kullanım (container içinde):
  source /home/ros/ws/install/setup.bash
  python3 test_dual_ukf.py
  python3 test_dual_ukf.py --waypoints /path/to/waypoints.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from robot_localization.srv import FromLL
import tf2_ros
import math
import time
import yaml
import os
import argparse
import threading


# ── 5 GPS Waypoint (Ankara civarı, datum: 39.925018, 32.836956) ─────────
DEFAULT_WAYPOINTS = [
    {"lat": 39.925090, "lon": 32.837050, "name": "WP1: Kuzey-Dogu ~4m"},
    {"lat": 39.925130, "lon": 32.836930, "name": "WP2: Kuzey-Bati ~8m"},
    {"lat": 39.925054, "lon": 32.837100, "name": "WP3: Dogu ~8m"},
    {"lat": 39.925000, "lon": 32.836900, "name": "WP4: Guney-Bati ~7m"},
    {"lat": 39.925054, "lon": 32.836991, "name": "WP5: Baslangica don"},
]


class DualUKFTest(Node):
    def __init__(self, waypoints):
        super().__init__('dual_ukf_test')

        self.waypoints = waypoints
        self.wp_index = 0
        self.wp_results = []
        self.start_time = time.time()
        self.wp_start = 0.0
        self._map_cache = {}

        # ── Topic izleme sayaçları ──────────────────────────────────────
        self._counts = {
            'odom_local': 0,
            'odom_filtered': 0,
            'odom_gps': 0,
            'imu': 0,
            'navsat': 0,
            'odom_raw': 0,
        }
        self._last_msgs = {
            'odom_local': None,
            'odom_filtered': None,
            'odom_gps': None,
            'navsat': None,
        }
        self._count_lock = threading.Lock()
        self._monitor_start = time.time()

        # ── Subscriber'lar — Dual-UKF pipeline izleme ──────────────────
        self.create_subscription(Odometry,   '/odometry/local',    self._cb_local,    10)
        self.create_subscription(Odometry,   '/odometry/filtered', self._cb_filtered, 10)
        self.create_subscription(Odometry,   '/odometry/gps',      self._cb_gps,      10)
        self.create_subscription(NavSatFix,  '/navsat',            self._cb_navsat,   10)
        self.create_subscription(Imu,        '/imu/data_raw',      self._cb_imu,      10)
        self.create_subscription(Odometry,   '/odom',              self._cb_odom_raw, 10)

        # ── TF dinleyici ────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action client ───────────────────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ── /fromLL servisi ─────────────────────────────────────────────
        self.fromll_client = self.create_client(FromLL, '/fromLL')

    # ── Callback'ler ────────────────────────────────────────────────────
    def _cb_local(self, msg):
        with self._count_lock:
            self._counts['odom_local'] += 1
            self._last_msgs['odom_local'] = msg

    def _cb_filtered(self, msg):
        with self._count_lock:
            self._counts['odom_filtered'] += 1
            self._last_msgs['odom_filtered'] = msg

    def _cb_gps(self, msg):
        with self._count_lock:
            self._counts['odom_gps'] += 1
            self._last_msgs['odom_gps'] = msg

    def _cb_navsat(self, msg):
        with self._count_lock:
            self._counts['navsat'] += 1
            self._last_msgs['navsat'] = msg

    def _cb_imu(self, msg):
        with self._count_lock:
            self._counts['imu'] += 1

    def _cb_odom_raw(self, msg):
        with self._count_lock:
            self._counts['odom_raw'] += 1

    # ── Dual-UKF Doğrulama ─────────────────────────────────────────────
    def verify_dual_ukf(self):
        """Dual-UKF pipeline'ının çalıştığını doğrula."""
        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('  DUAL-UKF DOĞRULAMA TESTİ')
        self.get_logger().info('=' * 65)

        # Sensör verilerinin gelmesini bekle (max 15s)
        self.get_logger().info('  Sensör verileri bekleniyor (15s)...')
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            with self._count_lock:
                if (self._counts['odom_local'] > 0 and
                    self._counts['odom_filtered'] > 0 and
                    self._counts['odom_gps'] > 0):
                    break

        elapsed = time.time() - self._monitor_start
        self.get_logger().info('')
        self.get_logger().info(f'  ── Topic Durumu ({elapsed:.0f}s içinde) ──')

        checks_passed = 0
        total_checks = 6

        with self._count_lock:
            topics = [
                ('  /odom              (DiffDrive)',      'odom_raw',      'UKF Local girdi'),
                ('  /imu/data_raw      (IMU)',            'imu',           'UKF Local girdi'),
                ('  /navsat            (GPS fix)',        'navsat',        'NavSat girdi'),
                ('  /odometry/local    (UKF Local çıktı)','odom_local',    'UKF Global girdi'),
                ('  /odometry/gps      (NavSat çıktı)',  'odom_gps',      'UKF Global girdi'),
                ('  /odometry/filtered (UKF Global çıktı)','odom_filtered','Nav2 girdi'),
            ]
            for label, key, role in topics:
                cnt = self._counts[key]
                rate = cnt / elapsed if elapsed > 0 else 0
                if cnt > 0:
                    status = '✅'
                    checks_passed += 1
                else:
                    status = '❌'
                self.get_logger().info(f'{label}  {status}  {cnt} msg ({rate:.1f} Hz) [{role}]')

        # TF chain kontrol
        self.get_logger().info('')
        self.get_logger().info('  ── TF Zinciri Kontrolü ──')

        tf_pairs = [
            ('odom', 'base_footprint', 'DiffDrive plugin'),
            ('map',  'odom',           'UKF Global (map→odom)'),
            ('map',  'base_footprint', 'Tam zincir'),
        ]
        tf_ok = 0
        for parent, child, desc in tf_pairs:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                pos = t.transform.translation
                self.get_logger().info(
                    f'  {parent}→{child}  ✅  ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})  [{desc}]')
                tf_ok += 1
            except Exception:
                self.get_logger().warn(
                    f'  {parent}→{child}  ❌  TF bulunamadı!  [{desc}]')

        # Pozisyon karşılaştırma
        self.get_logger().info('')
        self.get_logger().info('  ── Pozisyon Karşılaştırması ──')
        with self._count_lock:
            if self._last_msgs['odom_local'] and self._last_msgs['odom_filtered']:
                pl = self._last_msgs['odom_local'].pose.pose.position
                pf = self._last_msgs['odom_filtered'].pose.pose.position
                diff = math.sqrt((pl.x - pf.x)**2 + (pl.y - pf.y)**2)
                self.get_logger().info(f'  UKF Local  pozisyon: ({pl.x:+.3f}, {pl.y:+.3f})')
                self.get_logger().info(f'  UKF Global pozisyon: ({pf.x:+.3f}, {pf.y:+.3f})')
                self.get_logger().info(f'  Fark: {diff:.3f}m')
            if self._last_msgs['odom_gps']:
                pg = self._last_msgs['odom_gps'].pose.pose.position
                self.get_logger().info(f'  GPS (map)  pozisyon: ({pg.x:+.3f}, {pg.y:+.3f})')
            if self._last_msgs['navsat']:
                ns = self._last_msgs['navsat']
                self.get_logger().info(f'  GPS fix: ({ns.latitude:.6f}, {ns.longitude:.6f})')

        # Sonuç
        self.get_logger().info('')
        all_ok = checks_passed >= 4 and tf_ok >= 2  # minimum: local + filtered + odom + imu + 2 TF
        if all_ok:
            self.get_logger().info(f'  ✅ DUAL-UKF DOĞRULAMA: BAŞARILI ({checks_passed}/{total_checks} topic, {tf_ok}/3 TF)')
        else:
            self.get_logger().warn(f'  ⚠️  DUAL-UKF DOĞRULAMA: EKSİK ({checks_passed}/{total_checks} topic, {tf_ok}/3 TF)')
        self.get_logger().info('=' * 65)

        return all_ok

    # ── /fromLL ile GPS→Map dönüşümü ───────────────────────────────────
    def _gps_to_map(self, lat, lon):
        if not self.fromll_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/fromLL servisi bulunamadı!')
            return None, None

        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        future = self.fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            return result.map_point.x, result.map_point.y
        else:
            self.get_logger().error('/fromLL çağrısı başarısız!')
            return None, None

    def _precompute_waypoints(self):
        """Tüm GPS waypoint'lerini navigasyona başlamadan önce map XY'ye çevir."""
        self.get_logger().info('')
        self.get_logger().info('  GPS waypoint → Map XY dönüşümleri:')
        for i, wp in enumerate(self.waypoints):
            x, y = self._gps_to_map(wp['lat'], wp['lon'])
            if x is None:
                self.get_logger().error(f'  WP{i+1} ({wp.get("name","")}) dönüşüm başarısız!')
                self._map_cache[i] = None
            else:
                self._map_cache[i] = (x, y)
                self.get_logger().info(
                    f'  WP{i+1}: ({wp["lat"]:.6f}, {wp["lon"]:.6f}) → ({x:+.2f}, {y:+.2f}) m')

    def _compute_yaw(self, current_idx):
        """Bir sonraki waypoint'e doğru yön hesapla."""
        if current_idx + 1 >= len(self.waypoints):
            return 0.0
        curr = self._map_cache.get(current_idx)
        nxt = self._map_cache.get(current_idx + 1)
        if curr is None or nxt is None:
            return 0.0
        return math.atan2(nxt[1] - curr[1], nxt[0] - curr[0])

    # ── Navigasyon ──────────────────────────────────────────────────────
    def start_navigation(self):
        """GPS fix, /fromLL, Nav2 hazır olunca 5 waypoint navigasyonunu başlat."""

        # GPS fix bekle
        self.get_logger().info('')
        self.get_logger().info('GPS fix bekleniyor...')
        while self._last_msgs['navsat'] is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        ns = self._last_msgs['navsat']
        self.get_logger().info(f'GPS fix alındı: ({ns.latitude:.6f}, {ns.longitude:.6f})')

        # /fromLL servis bekle
        self.get_logger().info('/fromLL servisi bekleniyor...')
        if not self.fromll_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('/fromLL servisi bulunamadı!')
            return

        # Nav2 action server bekle
        self.get_logger().info('Nav2 action server bekleniyor...')
        if not self.nav_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error('Nav2 action server hazır değil!')
            return

        # Tüm waypoint'leri önceden dönüştür (callback deadlock önleme)
        self._precompute_waypoints()

        # Navigasyonu başlat
        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('  5 WAYPOINT GPS NAVİGASYON TESTİ (Dual-UKF)')
        self.get_logger().info('=' * 65)
        for i, wp in enumerate(self.waypoints):
            cached = self._map_cache.get(i)
            if cached:
                self.get_logger().info(
                    f'  [{i+1}] {wp["name"]:25s} GPS({wp["lat"]:.6f},{wp["lon"]:.6f}) '
                    f'→ Map({cached[0]:+.2f},{cached[1]:+.2f})')
            else:
                self.get_logger().info(
                    f'  [{i+1}] {wp["name"]:25s} GPS({wp["lat"]:.6f},{wp["lon"]:.6f}) → HATA')
        self.get_logger().info('=' * 65)

        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            self._print_summary()
            rclpy.shutdown()
            return

        wp = self.waypoints[self.wp_index]
        name = wp.get('name', f'WP{self.wp_index+1}')
        cached = self._map_cache.get(self.wp_index)

        if cached is None:
            self.get_logger().error(f'  {name}: harita koordinatı yok, atlanıyor')
            self.wp_results.append({
                'wp': self.wp_index + 1, 'name': name,
                'status': 'ATLAN', 'time': 0
            })
            self.wp_index += 1
            self.send_next_waypoint()
            return

        x, y = cached
        yaw = self._compute_yaw(self.wp_index)

        self.get_logger().info('')
        self.get_logger().info(f'── [{self.wp_index+1}/{len(self.waypoints)}] {name} ──')
        self.get_logger().info(f'   GPS: ({wp["lat"]:.6f}, {wp["lon"]:.6f})')
        self.get_logger().info(f'   Map: ({x:+.2f}, {y:+.2f})  Yaw: {math.degrees(yaw):.1f}°')

        # UKF durumunu logla
        self._log_ukf_status()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.wp_start = time.time()
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _log_ukf_status(self):
        """Her waypoint öncesi Dual-UKF durumunu logla."""
        with self._count_lock:
            elapsed = time.time() - self._monitor_start
            l_rate = self._counts['odom_local'] / elapsed if elapsed > 0 else 0
            g_rate = self._counts['odom_filtered'] / elapsed if elapsed > 0 else 0
            gps_rate = self._counts['odom_gps'] / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f'   UKF Durum → Local:{l_rate:.1f}Hz  Global:{g_rate:.1f}Hz  GPS:{gps_rate:.1f}Hz')

        # TF: map→base_footprint
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p = t.transform.translation
            self.get_logger().info(f'   TF(map→base): ({p.x:+.2f}, {p.y:+.2f})')
        except Exception:
            self.get_logger().warn('   TF(map→base): bulunamadı!')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('   ❌ Hedef REDDEDİLDİ!')
            self.wp_results.append({
                'wp': self.wp_index + 1,
                'name': self.waypoints[self.wp_index].get('name', ''),
                'status': 'REDDEDİLDİ', 'time': 0
            })
            self.wp_index += 1
            self.send_next_waypoint()
            return
        self.get_logger().info('   ✔ Hedef kabul edildi, navigasyon başladı...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        pos = fb.current_pose.pose.position
        remaining = fb.distance_remaining

        # Dual-UKF pozisyon karşılaştırması
        extra = ''
        with self._count_lock:
            if self._last_msgs['odom_filtered']:
                pf = self._last_msgs['odom_filtered'].pose.pose.position
                extra = f'  UKF:({pf.x:+.1f},{pf.y:+.1f})'
            if self._last_msgs['navsat']:
                ns = self._last_msgs['navsat']
                extra += f'  GPS:({ns.latitude:.5f},{ns.longitude:.5f})'

        self.get_logger().info(
            f'   Nav:({pos.x:+.1f},{pos.y:+.1f}) Kalan:{remaining:.1f}m{extra}',
            throttle_duration_sec=4.0)

    def _result_cb(self, future):
        result = future.result()
        elapsed = round(time.time() - self.wp_start, 1)
        name = self.waypoints[self.wp_index].get('name', '')
        status_map = {4: 'BAŞARILI ✅', 5: 'CANCELED ⚠️', 6: 'İPTAL (ABORTED)'}
        status = status_map.get(result.status, f'DURUM={result.status}')

        self.wp_results.append({
            'wp': self.wp_index + 1, 'name': name,
            'status': status, 'time': elapsed
        })

        if result.status == 4:
            self.get_logger().info(f'   → {name} ULAŞILDI! ({elapsed}s)')
        else:
            self.get_logger().warn(f'   → {name}: {status} ({elapsed}s)')

        self.wp_index += 1
        self.send_next_waypoint()

    # ── Sonuç Raporu ────────────────────────────────────────────────────
    def _print_summary(self):
        total = round(time.time() - self.start_time, 1)
        elapsed = time.time() - self._monitor_start

        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('  DUAL-UKF GPS NAVİGASYON TEST SONUÇLARI')
        self.get_logger().info('=' * 65)

        # Waypoint sonuçları
        self.get_logger().info('  ── Waypoint Sonuçları ──')
        ok = 0
        for r in self.wp_results:
            self.get_logger().info(
                f'  WP{r["wp"]}: {r["status"]:15s}  {r["time"]}s  {r["name"]}')
            if 'BAŞARILI' in r['status']:
                ok += 1

        # Sensör füzyon istatistikleri
        self.get_logger().info('')
        self.get_logger().info('  ── Sensör Füzyon İstatistikleri ──')
        with self._count_lock:
            stats = [
                ('UKF Local  (/odometry/local)',    self._counts['odom_local']),
                ('UKF Global (/odometry/filtered)', self._counts['odom_filtered']),
                ('NavSat GPS (/odometry/gps)',       self._counts['odom_gps']),
                ('IMU        (/imu/data_raw)',       self._counts['imu']),
                ('Wheel Odom (/odom)',               self._counts['odom_raw']),
                ('GPS Fix    (/navsat)',              self._counts['navsat']),
            ]
            for label, cnt in stats:
                rate = cnt / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'  {label:40s}  {cnt:6d} msg  ({rate:.1f} Hz)')

        # Final TF
        self.get_logger().info('')
        self.get_logger().info('  ── Son TF Durumu ──')
        for parent, child in [('odom', 'base_footprint'), ('map', 'odom'), ('map', 'base_footprint')]:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                p = t.transform.translation
                self.get_logger().info(
                    f'  {parent}→{child:15s}  ({p.x:+.2f}, {p.y:+.2f}, {p.z:+.2f})')
            except Exception:
                self.get_logger().warn(f'  {parent}→{child:15s}  ❌ YOK')

        self.get_logger().info('')
        self.get_logger().info(f'  TOPLAM: {ok}/{len(self.wp_results)} başarılı  |  {total}s')
        self.get_logger().info(f'  MİMARİ: Dual-UKF (Local→Global) + NavSat + Nav2')
        self.get_logger().info('=' * 65)


def load_waypoints_from_yaml(filepath):
    """YAML dosyasından waypoint yükle."""
    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', [])
    except Exception as e:
        print(f'Waypoints dosyası okunamadı: {e}')
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Dual-UKF GPS Navigasyon Doğrulama Testi')
    parser.add_argument('--waypoints', '-w', type=str, default=None,
                        help='Waypoints YAML dosyası yolu')
    parser.add_argument('--skip-verify', action='store_true',
                        help='Dual-UKF doğrulama adımını atla')
    args, unknown = parser.parse_known_args()

    # Waypoints dosyası bul
    waypoints = None
    if args.waypoints:
        waypoints = load_waypoints_from_yaml(args.waypoints)
    else:
        # Varsayılan konumları dene
        candidates = [
            os.path.join(os.path.dirname(__file__),
                         'src', 'leo_simulator', 'leo_gz_bringup',
                         'config', 'waypoints.yaml'),
        ]
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('leo_gz_bringup')
            candidates.insert(0, os.path.join(pkg, 'config', 'waypoints.yaml'))
        except Exception:
            pass

        for path in candidates:
            if os.path.isfile(path):
                waypoints = load_waypoints_from_yaml(path)
                if waypoints:
                    print(f'Waypoints yüklendi: {path}')
                    break

    # YAML bulunamadıysa varsayılan GPS koordinatlarını kullan
    if not waypoints:
        print('YAML dosyası bulunamadı, varsayılan 5 GPS waypoint kullanılıyor.')
        waypoints = DEFAULT_WAYPOINTS

    rclpy.init()
    node = DualUKFTest(waypoints)

    # Kısa bekleme — subscriber'ların bağlanması için
    time.sleep(2)

    # 1. Dual-UKF doğrulama
    if not args.skip_verify:
        ukf_ok = node.verify_dual_ukf()
        if not ukf_ok:
            node.get_logger().warn(
                'Dual-UKF tam doğrulanamadı ama navigasyon denenecek...')

    # 2. GPS waypoint navigasyonu
    node.start_navigation()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == '__main__':
    main()
