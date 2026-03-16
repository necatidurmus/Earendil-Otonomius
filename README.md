# Leo Rover — Hibrit GPS+SLAM Otonom Navigasyon

Leo Rover'ın ROS 2 Humble + Gazebo Ignition (Fortress) simülasyon ortamı.  
**Dual-UKF** sensör füzyonu, **SLAM Toolbox** ve Nav2 ile açık alan GPS navigasyonu + tünel SLAM navigasyonu birleştirir.

> **Sürüm Geçmişi**
>
> | Sürüm | Branch | Açıklama |
> |-------|--------|----------|
> | **v0.1** | `legacy/v0.1` | Dual-UKF GPS navigasyon. 4/4 waypoint ✅ |
> | **v0.2** | `main` | Hibrit GPS+SLAM. 9/9 waypoint ✅ (339.6s) |
>
> Eski sürüme dönmek için: `git checkout legacy/v0.1`

---

## Özellikler

- 🌍 **Gazebo Ignition** simülasyonu — tünel, duvarlar, bumps içeren `leo_obstacles.sdf` dünyası
- 🧭 **Nav2** navigasyon — haritasız, rolling costmap, lidar engel kaçınma
- 🔧 **Dual-UKF** sensör füzyonu — UKF Local (odom+IMU) + UKF Global (local+GPS)
- 🗺️ **SLAM Toolbox** — GPS-denied tünel içi harita oluşturma + lokalizasyon
- 📡 LiDAR + GPS + IMU sensörleri
- 🕹️ Web tabanlı teleoperasyon arayüzü
- 🧪 Tek komutla tam otonom test scripti (`run_full_test.sh`)

## Görev Mimarisi

```
Faz 1 — GPS (açık alan)    Faz 2 — SLAM (tünel)       Faz 3 — GPS (dönüş)
  WP1 → WP2 → WP3 → WP4  →  WP5 → WP6            →  WP7 → WP8 → WP9
  GPS navigasyon              SLAM navigasyon            GPS navigasyon
  Dual-UKF aktif              SLAM Toolbox aktif         Dual-UKF aktif
```

GPS kalitesi düşünce (tünel girişinde) `gps_monitor.py` otomatik SLAM moduna geçer.  
Tünelden çıkınca GPS moduna döner ve UKF Global sıfırlanır.

## Repo Yapısı

```
Earendil-Otonomius/
├── run_full_test.sh              # Tek komut tam test (Docker → Build → Gazebo → Nav2 → Mission)
├── run_hybrid_test.sh            # Gelişmiş test scripti (sim_config.yaml destekli)
├── ros_bridge_start.sh           # Manuel ROS-Gazebo bridge başlatma
├── sim_config.yaml               # Merkezi simulasyon konfigürasyonu (fizik, zamanlama, GPS eşikleri)
├── docker-compose.yml            # Docker ortamı (NVIDIA GPU, X11, host network)
├── Dockerfile                    # ROS 2 Humble + Nav2 + SLAM Toolbox imajı
└── src/
    ├── leo_common/               # Leo Rover temel paketleri (URDF, teleop, mesajlar)
    ├── leo_robot/                # Leo Rover donanım bringup ve firmware
    └── leo_simulator/
        └── leo_gz_bringup/
            ├── launch/
            │   ├── leo_gz.launch.py            # Gazebo simülatör başlatma
            │   ├── navigation_hybrid.launch.py # Ana launch: Dual-UKF + SLAM + Nav2
            │   └── spawn_robot.launch.py       # Robot spawning
            ├── config/
            │   ├── hybrid_waypoints.yaml       # 9 waypoint görev tanımı (3 faz)
            │   ├── nav2_params_hybrid.yaml     # Nav2 parametreleri (haritasız, lidar)
            │   ├── slam_toolbox_params.yaml    # SLAM Toolbox async modu
            │   ├── ukf_local.yaml              # UKF Local: odom + IMU füzyonu
            │   └── ukf_global_hybrid.yaml      # UKF Global: local + GPS füzyonu
            └── scripts/
                ├── mission_manager.py          # Faz tabanlı waypoint orkestratörü
                ├── gps_monitor.py              # GPS kalite izleme + mod geçişi
                ├── tf_mode_relay.py            # GPS↔SLAM TF çerçevesi geçişi
                └── tunnel_gps_spoofer.py       # Tünel bölgesi GPS sinyal kontrolü
```

## Gereksinimler

- Docker & Docker Compose
- NVIDIA GPU (CUDA sürücüsü)
- X11 display (Gazebo GUI için)

## Hızlı Başlangıç

```bash
## Hızlı Başlangıç

### 1. Docker Başlat

```bash
docker compose up -d --build
```

### 2. Tam Testi Çalıştır (Tek Komut)

```bash
./run_full_test.sh
```

Bu script otomatik olarak:
1. Docker container'ı başlatır (yoksa oluşturur)
2. Workspace'i build eder (`colcon build`)
3. Gazebo simülasyonunu başlatır (`leo_obstacles.sdf`)
4. Nav2 hibrit navigasyon stack'ini başlatır
5. Mission Manager ile 9 waypoint görevini çalıştırır
6. Sonucu raporlar (PASS / FAIL)

---

## Test Sonuçları (v0.2)

| Faz | Waypoint | Süre | Sonuç |
|-----|----------|------|-------|
| GPS (açık alan) | WP1 south_clear | 20.9s | ✅ |
| GPS (açık alan) | WP2 below_wall | 73.8s | ✅ |
| GPS (açık alan) | WP3 past_wall | 24.7s | ✅ |
| GPS (açık alan) | WP4 tunnel_entrance | 30.7s | ✅ |
| SLAM (tünel) | WP5 tunnel_middle | 15.7s | ✅ |
| SLAM (tünel) | WP6 tunnel_exit | 16.5s | ✅ |
| GPS (dönüş) | WP7 south_return | 79.4s | ✅ |
| GPS (dönüş) | WP8 west_clear | 24.9s | ✅ |
| GPS (dönüş) | WP9 return_home | 44.2s | ✅ |

**Toplam: 9/9 başarılı | 339.6s**

---

## Sensör Füzyonu Mimarisi

```
Gazebo DiffDrive  →  /odom ──────────────────────┐
Gazebo IMU        →  /imu/data_raw ───────────────┤
                                                  ▼
                                           UKF Local
                                         (world: odom)
                                       → /odometry/local
                                                  │
                                    ┌─────────────┴─────────────┐
                                    ▼                           ▼
                             UKF Global                  NavSat Transform
                           (world: map_gps)            GPS → /odometry/gps
                          → map_gps→odom TF
                         → /odometry/filtered

Tünel içinde:
  SLAM Toolbox (LiDAR) → map_slam→odom TF

tf_mode_relay.py → aktif kaynağı map→odom olarak yayınlar (50Hz)
```

---

## Simülasyon Konfigürasyonu

`sim_config.yaml` ile tüm parametreler merkezi olarak yönetilir:
- Gazebo fizik motoru (adım süresi, gerçek zaman oranı)
- Robot hız/ivme limitleri, tekerlek sürtünme katsayıları
- Başlatma gecikmeleri (Gazebo, UKF, NavSat, Nav2)
- GPS/SLAM mod geçiş eşikleri

---

## Lisans

Bu proje eğitim amaçlıdır.


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
│           │   ├── leo_gz.launch.py            # Gazebo başlatıcı
│           │   ├── navigation_hybrid.launch.py # Ana launch: Dual-UKF + SLAM + Nav2
│           │   ├── slam.launch.py              # Bağımsız SLAM harita oluşturma
│           │   └── spawn_robot.launch.py       # Robot spawning
│           ├── config/
│           │   ├── hybrid_waypoints.yaml       # 9 waypoint görev tanımı (3 faz)
│           │   ├── nav2_params_hybrid.yaml     # Nav2 parametreleri
│           │   ├── slam_toolbox_params.yaml    # SLAM Toolbox async config
│           │   ├── ukf_local.yaml              # UKF Local: odom + IMU
│           │   ├── ukf_global_hybrid.yaml      # UKF Global: local + GPS
│           │   └── navsat.yaml                 # NavSat transform
│           └── scripts/
│               ├── mission_manager.py          # Faz tabanlı waypoint orkestratörü ⭐
│               ├── gps_monitor.py              # GPS kalite izleme + mod geçişi
│               ├── tf_mode_relay.py            # GPS↔SLAM TF çerçevesi geçişi
│               └── tunnel_gps_spoofer.py       # Tünel bölgesi GPS sinyal kontrolü
└── leo_gz_worlds/
    └── worlds/
        ├── leo_empty.sdf                       # Boş dünya
        └── leo_obstacles.sdf                   # Tünel + engellli test dünyası ⭐
```
