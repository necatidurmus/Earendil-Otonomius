# Leo Rover ROS 2 Humble Simulation

Leo Rover'ın ROS 2 Humble + Gazebo Ignition (Fortress) simülasyon ortamı.  
GPS destekli **Dual-UKF** sensör füzyonu ve Nav2 ile 5 waypoint otonom navigasyon içerir.

## Özellikler

- 🌍 Gazebo simülasyonu (boş dünya + engelli/engebeli arazi)
- 🧭 Nav2 navigasyon (GPS waypoint, engel kaçınma, haritasız)
- 🔧 **Dual-UKF** sensör füzyonu (UKF Local + UKF Global + NavSat)
- 📡 LiDAR + GPS + IMU sensörleri
- 🕹️ Web tabanlı teleoperasyon
- 🧪 5 GPS waypoint otonom test scripti

## Gereksinimler

- Docker & Docker Compose
- X11 (Gazebo GUI için)

## Hızlı Başlangıç

```bash
# 1. Tam test (Gazebo GUI + Dual-UKF + Nav2 + 5 waypoint)
./run_dual_ukf_test.sh

# 2. Engelsiz dünyada test
./run_dual_ukf_test.sh --world empty

# 3. Özel timeout ile test
./run_dual_ukf_test.sh --world obstacles --timeout 200

# 4. Sadece simülasyon (Nav2 olmadan)
python3 start_simulation.py --no-nav
```

## Proje Yapısı

```
├── Dockerfile                    # ROS 2 Humble Docker image
├── docker-compose.yml            # Container yapılandırması
├── run_dual_ukf_test.sh          # Tek komutla tam test başlatıcı
├── test_dual_ukf.py              # Dual-UKF doğrulama + 5 waypoint test
├── start_simulation.py           # Simülasyon başlatıcı
├── src/
│   ├── teleop_web/               # Web teleoperasyon arayüzü
│   ├── leo_common/               # Leo Rover URDF & açıklama
│   ├── leo_robot/                # Leo Rover ROS 2 paketleri
│   └── leo_simulator/
│       └── leo_gz_bringup/
│           ├── launch/
│           │   ├── leo_gz.launch.py           # Gazebo başlatıcı
│           │   ├── navigation_ukf.launch.py   # Nav2 + Dual-UKF (aktif)
│           │   ├── navigation_ekf.launch.py   # Nav2 + EKF (alternatif)
│           │   ├── slam.launch.py             # SLAM harita oluşturma
│           │   └── spawn_robot.launch.py      # Robot spawn
│           ├── config/
│           │   ├── ukf_local.yaml             # UKF Local (odom+IMU → /odometry/local)
│           │   ├── ukf_global.yaml            # UKF Global (local+GPS → /odometry/filtered)
│           │   ├── navsat.yaml                # NavSat transform (GPS → /odometry/gps)
│           │   ├── nav2_params_gps.yaml       # Nav2 haritasız GPS parametreleri
│           │   └── nav2_params.yaml           # Nav2 standart parametreler
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

## Test Waypoint'leri

Ankara Keçiören koordinatlarına göre 5 hedef (datum: `39.93°N, 32.84°E`):

| # | Kuzey (m) | Doğu (m) | Açıklama |
|---|-----------|----------|----------|
| 1 | +10 | +10 | Kuzeydoğu |
| 2 | +20 | -5 | Kuzey |
| 3 | +5 | -15 | Kuzeybatı |
| 4 | -10 | +5 | Güney |
| 5 | 0 | 0 | Başlangıç |

## Lisans

Bu proje eğitim amaçlıdır.
