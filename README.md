# Leo Rover ROS 2 Humble Simulation

Leo Rover'ın ROS 2 Humble + Gazebo Ignition (Fortress) simülasyon ortamı.  
GPS destekli **Dual-UKF** sensör füzyonu ve Nav2 ile **4 waypoint** otonom navigasyon içerir.

> **Sürüm Notları**
>
> | Sürüm | Durum | Açıklama |
> |-------|-------|----------|
> | **v0.1** | ✅ Kararlı — Test edildi | Dual-UKF + Nav2 GPS navigasyon. 4/4 waypoint başarılı. |
> | **v0.2** | ⚠️ Geliştirme — Test edilmedi | Hibrit GPS+SLAM navigasyon, tünel/kapı GPS spoof, RViz desteği. |
> | **v0.2.1** | 🔧 Mimari stabilizasyon | TF yarışı çözümü, mod geçiş koruması, waypoint düzeltme, install uyumluluk. |
>
> Kararlı sürümü kullanmak için: `git checkout v0.1`

## Özellikler

- 🌍 Gazebo simülasyonu (boş dünya + 14 engelli arazi)
- 🧭 Nav2 navigasyon (GPS waypoint, engel kaçınma, haritasız)
- 🔧 **Dual-UKF** sensör füzyonu (UKF Local + UKF Global + NavSat)
- 📡 LiDAR + GPS + IMU sensörleri
- 🕹️ Web tabanlı teleoperasyon
- 🧪 4 GPS waypoint otonom test scripti (4/4 başarılı ✅)

## Gereksinimler

- Docker & Docker Compose
- X11 (Gazebo GUI için)

## Hızlı Başlangıç

```bash
# ── v0.1: GPS-only test (Dual-UKF + Nav2 + 4 waypoint) ──
./run_dual_ukf_test.sh
./run_dual_ukf_test.sh --world empty
./run_dual_ukf_test.sh --timeout 300

# ── v0.2.1: Hibrit GPS+SLAM test (tünel + kapı + mod geçişi) ──
./run_hybrid_test.sh
./run_hybrid_test.sh --rviz
./run_hybrid_test.sh --world empty --timeout 600
```

## Proje Yapısı

```
├── Dockerfile                    # ROS 2 Humble Docker image
├── docker-compose.yml            # Container yapılandırması
├── sim_config.yaml               # Simülasyon parametreleri
├── run_dual_ukf_test.sh          # v0.1 GPS-only test başlatıcı
├── run_hybrid_test.sh            # v0.2 Hibrit GPS+SLAM test başlatıcı
├── test_dual_ukf.py              # Dual-UKF doğrulama + 4 waypoint test
├── src/
│   ├── teleop_web/               # Web teleoperasyon arayüzü
│   ├── leo_common/               # Leo Rover URDF & açıklama
│   ├── leo_robot/                # Leo Rover ROS 2 paketleri
│   └── leo_simulator/
│       └── leo_gz_bringup/
│           ├── launch/
│           │   ├── leo_gz.launch.py           # Gazebo başlatıcı
│           │   ├── navigation_ukf.launch.py   # Nav2 + Dual-UKF (v0.1, GPS-only)
│           │   ├── navigation_hybrid.launch.py# Nav2 + Hibrit GPS+SLAM (v0.2)
│           │   ├── navigation_ekf.launch.py   # Nav2 + EKF (alternatif)
│           │   ├── slam.launch.py             # SLAM harita oluşturma
│           │   └── spawn_robot.launch.py      # Robot spawn
│           ├── config/
│           │   ├── ukf_local.yaml             # UKF Local (odom+IMU → /odometry/local)
│           │   ├── ukf_global.yaml            # UKF Global (v0.1, map→odom)
│           │   ├── ukf_global_hybrid.yaml     # UKF Global (v0.2, map_gps→odom)
│           │   ├── slam_toolbox_params.yaml   # SLAM Toolbox (map_slam frame)
│           │   ├── navsat.yaml                # NavSat transform (GPS → /odometry/gps)
│           │   ├── hybrid_waypoints.yaml      # Faz tabanlı hibrit waypoint'ler
           │   ├── nav2_params_gps.yaml       # Nav2 haritasız GPS parametreleri (v0.1)
           │   ├── nav2_params_hybrid.yaml    # Nav2 hibrit parametreleri (v0.2.1)
│           │   └── nav2_params.yaml           # Nav2 standart parametreler
│           ├── scripts/
│           │   ├── gps_monitor.py             # GPS kalite izleme + mod geçişi
│           │   ├── gps_waypoint_nav.py        # GPS waypoint navigasyon
│           │   ├── tunnel_gps_spoofer.py      # Tünel GPS spoof
│           │   ├── tf_mode_relay.py           # TF frame seçici (map_gps/map_slam→map)
│           │   ├── waypoint_runner.py         # Paylaşılan waypoint kütüphanesi
│           │   ├── mission_manager.py         # Faz tabanlı görev yöneticisi
│           │   ├── test_gps_nav.py            # v0.1 GPS-only regresyon testi
│           │   └── test_hybrid_mission.py     # Hibrit görev testi
│           └── maps/
│               └── empty_50x50_map.*          # 50x50m boş harita
│       └── leo_gz_worlds/
│           └── worlds/
│               ├── leo_empty.sdf              # Boş dünya
│               └── leo_obstacles.sdf          # Engelli + engebeli test dünyası
```

## Sensör Füzyonu Mimarisi (Dual-UKF)

`robot_localization` paketi ile **iki katmanlı Unscented Kalman Filter**:

```
Gazebo DiffDrive  →  /odom  ─────────────────────────────────┐
Gazebo IMU        →  /imu/data_raw  ──────────────────────────┤
                                                              ▼
                                                    UKF Local (ukf_node)
                                                    world_frame: odom
                                                    → /odometry/local
                                                              │
                                              ┌───────────────┴──────────────┐
                                              ▼                              ▼
                                    UKF Global (ukf_node)         NavSat Transform
                                    world_frame: map               GPS → /odometry/gps
                                    → /odometry/filtered                     │
                                    → map→odom TF  ◄────────────────────────┘
                                              │
                                              ▼
                                           Nav2
                                  (navigate_to_pose, engel kaçınma)
```

| Filtre | Girdi | Çıktı | Amaç |
|--------|-------|-------|------|
| **UKF Local** | `/odom` + `/imu/data_raw` | `/odometry/local` | Kısa mesafe drift-free kestirimi |
| **UKF Global** | `/odometry/local` + `/odometry/gps` | `/odometry/filtered` + `map→odom` TF | GPS-ankrajlı global konum |
| **NavSat** | `/navsat` (GPS fix) | `/odometry/gps` + `/fromLL` servisi | GPS koordinatını ROS odometrisine çevirme |

## Hibrit GPS+SLAM Mimarisi (v0.2.1)

Tünel/kapalı alanda GPS kesildiğinde SLAM'e otomatik geçiş:

```
         Gazebo DiffDrive → /odom ───────┐
         Gazebo IMU → /imu/data_raw ─────┤
                                         ▼
                                  UKF Local (world_frame: odom)
                                  → /odometry/local
                                         │
                           ┌─────────────┴──────────────┐
                           ▼                            ▼
/navsat (Gazebo GPS)      UKF Global                NavSat Transform
        │              (world_frame: map_gps)      GPS → /odometry/gps
  tunnel_gps_spoofer      → map_gps→odom TF                │
        │                  → /odometry/filtered ◄───────────┘
  /navsat_filtered                │
        │                         │
  gps_monitor.py                  │
  → /nav_mode (GPS|SLAM)         │
        │                         │
  tf_mode_relay.py (50Hz)        │
  ┌─ GPS modu : map_gps→odom ◄──┘
  └─ SLAM modu: map_slam→odom ◄── SLAM Toolbox (LiDAR)
        │                         (world_frame: map_slam)
        ▼
    map→odom TF
        │
        ▼
      Nav2 (global_frame: map)
```

| Bileşen | Görev |
|---------|-------|
| **tunnel_gps_spoofer** | Tünelde GPS'i STATUS_NO_FIX'e çevirir |
| **gps_monitor** | GPS kalitesini izler, /nav_mode yayınlar |
| **tf_mode_relay** | Aktif TF kaynağını map→odom olarak relay eder |
| **UKF Global** | map_gps→odom: GPS-ankrajlı global konum |
| **SLAM Toolbox** | map_slam→odom: LiDAR tabanlı lokalizasyon |

## Test Waypoint'leri

Ankara koordinatlarına göre 4 güvenli hedef (datum: `39.925018°N, 32.836956°E`):  
Tüm waypoint'ler engellerden en az **2.9m** uzaklıkta.

| # | Ofset (m) | Yön | Engel Uzk. | Süre | Sonuç |
|---|-----------|-----|-----------|------|-------|
| 1 | (+3, +5) | Kuzeydoğu | 3.0m | 36.7s | ✅ |
| 2 | (-6, +11) | Kuzeybatı | 3.0m | 82.3s | ✅ |
| 3 | (+14, +1) | Doğu | 2.9m | 43.2s | ✅ |
| 4 | (-3, -5) | Güneybatı | 3.9m | 120.8s | ✅ |

**Toplam: 4/4 başarılı | 284.9s**

## v0.2.1 Değişiklikleri (Mimari Stabilizasyon)

**TF Frame Yarışı Çözümü:**
- UKF Global artık `map_gps→odom` yayınlıyor (yeni `ukf_global_hybrid.yaml`)
- SLAM Toolbox artık `map_slam→odom` yayınlıyor (`map_frame: map_slam`)
- `tf_mode_relay.py` aktif kaynağı seçerek `map→odom` olarak yayınlıyor (50Hz)
- Mod geçişinde fallback: yeni TF kaynağı hazır değilse önceki kaynak veya son bilinen transform kullanılır

**v0.1 Uyumluluk:**
- `spawn_robot.launch.py` artık `/navsat` remapping yapmıyor → `navigation_ukf.launch.py` sorunsuz çalışıyor
- Tüm eski GPS-only dosyalar korundu (`ukf_global.yaml`, `navigation_ukf.launch.py`, `waypoints.yaml`)

**Install Uyumluluk:**
- Launch dosyasındaki `executable` adları `.py` uzantısıyla eşleştirildi (CMake PROGRAMS install ile uyumlu)
- `run_hybrid_test.sh` install path'leri düzeltildi
- `nav2_params_hybrid.yaml` oluşturuldu (tünel/kapı için optimize Nav2 parametreleri)

**Görev Yönetimi:**
- `mission_manager.py`: Faz tabanlı waypoint orkestratörü — YAML'den faz/mod/waypoint okur
- `waypoint_runner.py`: Paylaşılan GPS→Map navigasyon kütüphanesi (DRY)
- `hybrid_waypoints.yaml`: 3 fazlı test senaryosu — waypoint'ler tünel/kapı bölgeleriyle uyumlu

**Waypoint Düzeltmeleri:**
- WP3/WP4 artık tünel içinde (x≈22, x≈28) — GPS spoofed NO_FIX bölgesinde
- WP5 artık tünel dışında (x≈5) — GPS geri gelir
- Waypoint açıklamaları ile gerçek konumları uyumlu

**Test Ayrımı:**
- `test_gps_nav.py`: v0.1 GPS-only regresyon testi (waypoint_runner tabanlı)
- `test_hybrid_mission.py`: Hibrit mod geçişi + TF doğrulama testi
- `run_hybrid_test.sh`: mission_manager + hybrid_waypoints.yaml ile çalışır

**Performans:**
- `gps_monitor.py`: 5Hz yayın, 1.5s histerezis (sim_config.yaml ile uyumlu)
- Mod geçiş kör penceresi: ~3.5s → ~1.7s

**Temizlik:**
- `wheel_odom_publisher.py` kaldırıldı (ölü kod)
- `gps_monitor.py`'den kullanılmayan topic'ler kaldırıldı
- `rclpy`, `nav_msgs` bağımlılıkları package.xml'e eklendi

## v0.2 Değişiklikleri

- `find_safe_wps.py` düzeltildi (bozuk dosya → tam grid-search waypoint bulucu)
- `rviz_hybrid.rviz` eklendi (hibrit navigasyon görselleştirme)
- `run_dual_ukf_test.sh` Docker container kontrolü eklendi
- Hibrit GPS+SLAM navigasyon launch dosyası (`navigation_hybrid.launch.py`)
- GPS kalite izleme (`gps_monitor.py`), tünel GPS spoof (`tunnel_gps_spoofer.py`), TF mod relay (`tf_mode_relay.py`)

> ⚠️ **Not:** v0.2 geliştirme aşamasındadır ve henüz tam test edilmemiştir.  
> Kararlı sürüm için `git checkout v0.1` kullanın.

## Lisans

Bu proje eğitim amaçlıdır.
