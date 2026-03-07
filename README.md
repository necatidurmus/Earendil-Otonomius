# Leo Rover ROS 2 Humble Simulation

Leo Rover'ın ROS 2 Humble + Gazebo Ignition (Fortress) simülasyon ortamı.  
GPS destekli **Dual-UKF** sensör füzyonu ve Nav2 ile **4 waypoint** otonom navigasyon içerir.

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
# 1. Tam test (Gazebo GUI + Dual-UKF + Nav2 + 4 waypoint)
./run_dual_ukf_test.sh

# 2. Engelsiz dünyada test
./run_dual_ukf_test.sh --world empty

# 3. Özel timeout ile test
./run_dual_ukf_test.sh --timeout 300
```

## Proje Yapısı

```
├── Dockerfile                    # ROS 2 Humble Docker image
├── docker-compose.yml            # Container yapılandırması
├── run_dual_ukf_test.sh          # Tek komutla tam test başlatıcı
├── test_dual_ukf.py              # Dual-UKF doğrulama + 4 waypoint test
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

Ankara koordinatlarına göre 4 güvenli hedef (datum: `39.925018°N, 32.836956°E`):  
Tüm waypoint'ler engellerden en az **2.9m** uzaklıkta.

| # | Ofset (m) | Yön | Engel Uzk. | Süre | Sonuç |
|---|-----------|-----|-----------|------|-------|
| 1 | (+3, +5) | Kuzeydoğu | 3.0m | 36.7s | ✅ |
| 2 | (-6, +11) | Kuzeybatı | 3.0m | 82.3s | ✅ |
| 3 | (+14, +1) | Doğu | 2.9m | 43.2s | ✅ |
| 4 | (-3, -5) | Güneybatı | 3.9m | 120.8s | ✅ |

**Toplam: 4/4 başarılı | 284.9s**

## Lisans

Bu proje eğitim amaçlıdır.
