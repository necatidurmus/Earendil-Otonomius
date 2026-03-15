# Earendil-Otonomius — Kernel Branch

Simülasyondan arındırılmış **otonom navigasyon çekirdeği**.  
Leo Rover üzerinde GPS+SLAM hibrit navigasyonu gerçek donanımda çalıştırmak için
gerekli olan minimum karar verme katmanı ve sensör füzyon konfigürasyonları.

## Mimari

```
                    ┌─────────────┐
   /navsat ───────►│ gps_monitor  │──► /nav_mode (GPS | SLAM)
                    └─────────────┘
                           │
                    ┌──────▼──────┐
                    │tf_mode_relay│──► map → map_slam TF
                    └─────────────┘
                           │
  ┌──────────┐      ┌──────▼──────┐      ┌──────────┐
  │UKF Local │─────►│ UKF Global  │─────►│  Nav2    │
  │(odom+IMU)│      │(local+GPS)  │      │(planner) │
  └──────────┘      └─────────────┘      └────┬─────┘
                                               │
  ┌──────────┐                          ┌──────▼──────┐
  │SLAM Tool │──► map_slam→odom TF      │mission_mgr  │
  │  box     │                          │(WP orkestra)│
  └──────────┘                          └─────────────┘
```

**TF Zinciri:** `map → map_slam → odom → base_footprint`

## Bileşenler

| Dosya | Rol |
|-------|-----|
| `scripts/mission_manager.py` | Faz tabanlı waypoint orkestratörü (GPS→SLAM→GPS) |
| `scripts/gps_monitor.py` | GPS kalite izleme + GPS/SLAM mod geçişi |
| `scripts/tf_mode_relay.py` | GPS↔SLAM TF frame geçişi |
| `config/hybrid_waypoints.yaml` | 9 waypoint görev tanımı |
| `config/nav2_params_hybrid.yaml` | Nav2 navigasyon parametreleri |
| `config/ukf_local.yaml` | UKF Local sensör füzyon config |
| `config/ukf_global_hybrid.yaml` | UKF Global (GPS+local) config |
| `config/navsat.yaml` | NavSat transform config |
| `config/slam_toolbox_params.yaml` | SLAM Toolbox config |
| `launch/navigation_hybrid.launch.py` | Ana launch dosyası |

## Docker ile Çalıştırma

```bash
# Build
docker compose build

# Container başlat
docker compose up -d

# Container içine gir
docker exec -it leo-kernel bash

# Workspace build
cd /home/ros/ws && colcon build --symlink-install
source install/setup.bash

# Navigasyonu başlat
ros2 launch leo_kernel navigation_hybrid.launch.py

# Görev başlat (ayrı terminalde)
ros2 run leo_kernel mission_manager.py
```

## Gereksinimler

- ROS 2 Humble
- robot_localization
- slam_toolbox
- navigation2 / nav2_bringup
- Gerçek robot: GPS (NavSatFix), IMU, 2D LiDAR, diferansiyel sürüş

## Branch Yapısı

| Branch | Açıklama |
|--------|----------|
| `main` | v0.2 — Simülasyon dahil tam sistem (9/9 PASS) |
| `kernal` | Simülasyondan arındırılmış otonom çekirdek |
| `legacy/v0.1` | Eski GPS-only navigasyon |
