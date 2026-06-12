# mltest — MATLAB-Controlled Test Scripts

Bu klasör, MATLAB arayüzünden waypoint gönderilerek çalışan test scriptlerini içerir.

## Kullanım

### 1. Simülasyonu Başlat (sadece ortam + navigasyon)
```bash
./mltest/run_leo.sh          # Leo Obstacles dünyası
./mltest/run_earendil.sh     # Earendil dünyası
./mltest/run_urban.sh        # Urban dünyası
./mltest/run_industrial.sh   # Industrial dünyası
```

### 2. MATLAB'dan Waypoint Gönder
MATLAB arayüzünden waypoint'ler ROS2 `/goal_pose` topic'ine publish edilir.
Sistem otomatik olarak waypoint'leri sırayla işler.

### 3. Waypoint Receiver (Opsiyonel)
Docker içinde waypoint receiver node'u başlatılabilir:
```bash
docker exec ros2-dev bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/waypoint_receiver.py
"
```

## Fark
- `run_*.sh` scriptleri: Gazebo + Nav2 başlatır, waypoint testi BAŞLATMAZ
- MATLAB GUI: Waypoint'leri sırayla `/goal_pose` topic'ine publish eder
- Waypoint receiver: Gelen waypoint'leri Nav2'ye goal olarak gönderir (opsiyonel)

## Harita Seçenekleri
| Script | Dünya | Navigasyon Modu |
|--------|-------|-----------------|
| `run_leo.sh` | leo_obstacles | Hybrid GPS+SLAM |
| `run_earendil.sh` | leo_earendil_env | Hybrid GPS+SLAM |
| `run_urban.sh` | leo_urban | GPS |
| `run_industrial.sh` | leo_industrial | SLAM |
