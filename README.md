# Leo Rover ROS 2 Humble Simulation

Leo Rover'ın ROS 2 Humble + Gazebo Ignition (Fortress) simülasyon ortamı.

## Özellikler

- 🌍 Gazebo simülasyonu (boş dünya + marsyard)
- 🧭 Nav2 navigasyon (GPS waypoint, engel kaçınma)
- 🔧 EKF sensör füzyonu (Odometri + IMU + GPS)
- 📡 LiDAR + GPS + IMU sensörleri
- 🕹️ Web tabanlı teleoperasyon

## Gereksinimler

- Docker & Docker Compose
- X11 (Gazebo GUI için)

## Hızlı Başlangıç

```bash
# 1. Simülasyonu başlat (otomatik: Docker + Gazebo + engeller + Nav2)
python3 start_simulation.py

# 2. Mars yard dünyasında başlat
python3 start_simulation.py --world marsyard

# 3. GPS navigasyon testi ile başlat
python3 start_simulation.py --gps-test

# 4. Sadece Gazebo (Nav2 olmadan)
python3 start_simulation.py --no-nav
```

## Proje Yapısı

```
├── Dockerfile                    # ROS 2 Humble Docker image
├── docker-compose.yml            # Container yapılandırması
├── start_simulation.py           # Tek komutla başlatıcı
├── src/
│   ├── gps_waypoint_nav.py       # GPS waypoint navigasyon scripti
│   ├── teleop_web/               # Web teleoperasyon arayüzü
│   ├── leo_common/               # Leo Rover URDF & açıklama
│   ├── leo_robot/                # Leo Rover ROS 2 paketleri
│   └── leo_simulator/
│       └── leo_gz_bringup/
│           ├── launch/
│           │   ├── leo_gz.launch.py          # Gazebo başlatıcı
│           │   ├── navigation_ekf.launch.py  # Nav2 + EKF
│           │   ├── slam.launch.py            # SLAM harita oluşturma
│           │   └── spawn_robot.launch.py     # Robot spawn
│           ├── config/
│           │   ├── nav2_params.yaml          # Nav2 parametreleri
│           │   └── ekf.yaml                  # EKF sensör füzyon config
│           └── maps/
│               └── empty_50x50_map.*         # 50x50m boş harita
```

## Sensör Füzyonu (EKF)

`robot_localization` paketi ile Extended Kalman Filter:
- **Odometri** → kısa mesafe hız bilgisi
- **IMU** → yön ve açısal hız
- **GPS (NavSat)** → mutlak konum düzeltmesi

## Lisans

Bu proje eğitim amaçlıdır.
