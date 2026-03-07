#!/bin/bash
# Dual-UKF Obstacle Test — tek komutla tam başlatma
# Kullanım: ./run_test.sh
# Durdurmak: Ctrl+C

set -e

CONTAINER="ros2-dev"
SETUP="source /home/ros/ws/install/setup.bash"
WORLD="/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/leo_obstacles.sdf"

# ── 0. Eski süreçleri temizle ─────────────────────────────────────────────────
echo "[1/4] Eski ROS/Gazebo süreçleri temizleniyor..."
docker exec $CONTAINER bash -c "
  pkill -9 -f 'ign gazebo|gz_sim|ukf_node|navsat_transform|bt_navigator|controller_server|planner_server|lifecycle_manager' 2>/dev/null || true
  sleep 2
"

# ── 1. Build ──────────────────────────────────────────────────────────────────
echo "[2/4] Paketler derleniyor..."
docker exec $CONTAINER bash -c "
  cd /home/ros/ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select leo_gz_bringup leo_gz_worlds --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
"

# ── 2. Gazebo + Robot spawn ───────────────────────────────────────────────────
echo "[3/4] Gazebo başlatılıyor (engelli dünya)..."
docker exec $CONTAINER bash -c "$SETUP && ros2 launch leo_gz_bringup leo_gz.launch.py sim_world:=$WORLD" &
GZ_PID=$!

echo "  Gazebo yükleniyor (25 saniye)..."
sleep 25

# TF kontrol
echo "  TF kontrol ediliyor..."
TF_OK=$(docker exec $CONTAINER bash -c "$SETUP && timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1" | grep -c "Translation" || true)
if [ "$TF_OK" -eq 0 ]; then
  echo "  [UYARI] odom→base_footprint TF henüz yok, 10s daha bekleniyor..."
  sleep 10
fi

# ── 3. Navigation stack ───────────────────────────────────────────────────────
echo "[4/4] Dual-UKF + Nav2 başlatılıyor..."
docker exec $CONTAINER bash -c "$SETUP && ros2 launch leo_gz_bringup navigation_ukf.launch.py" &
NAV_PID=$!

echo ""
echo "  Başlatma sırası: UKF Local(0s) → NavSat(3s) → UKF Global(6s) → Nav2(20s)"
echo "  Nav2'nin aktif olması için 30s bekleniyor..."
sleep 30

# ── 4. Durum raporu ───────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "DURUM RAPORU"
echo "════════════════════════════════════════"

docker exec $CONTAINER bash -c "
  $SETUP
  echo -n '  UKF Local  (/odometry/local):    '
  timeout 3 ros2 topic hz /odometry/local 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  UKF Global (/odometry/filtered): '
  timeout 3 ros2 topic hz /odometry/filtered 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  NavSat     (/odometry/gps):      '
  timeout 3 ros2 topic hz /odometry/gps 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  Lidar      (/scan):              '
  timeout 3 ros2 topic hz /scan 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo ''
  echo '  map→base_footprint TF:'
  timeout 4 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep 'Translation' | head -1 || echo '  TF yok'
"

# ── 5. 5-Waypoint GPS Navigasyon Testi ───────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "5-WAYPOINT GPS NAVİGASYON TESTİ"
echo "════════════════════════════════════════"

WAYPOINTS="$HOME/ros-humble-sim/src/leo_simulator/leo_gz_bringup/config/waypoints.yaml"

# Nav2 action server hazır mı kontrol et
echo "  Nav2 action server bekleniyor (max 30s)..."
NAV2_OK=false
for i in $(seq 1 15); do
  RESULT=$(docker exec $CONTAINER bash -c "$SETUP && timeout 2 ros2 action list 2>/dev/null | grep navigate_to_pose" || true)
  if [ -n "$RESULT" ]; then
    NAV2_OK=true
    echo "  Nav2 hazır ✓"
    break
  fi
  echo "  ...bekleniyor ($i/15)"
  sleep 2
done

if [ "$NAV2_OK" = false ]; then
  echo "  [HATA] Nav2 action server başlatılamadı — navigasyon testi atlanıyor."
  echo "  Stack loglarını kontrol edin."
else
  echo ""
  echo "  Waypoints dosyası: $WAYPOINTS"
  echo "  5 nokta sırayla gönderiliyor..."
  echo ""

  # Waypoints dosyasını container içine kopyala
  docker cp "$WAYPOINTS" $CONTAINER:/tmp/waypoints.yaml

  # gps_waypoint_nav.py'yi container içinde çalıştır
  docker exec $CONTAINER bash -c "
    $SETUP
    python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/gps_waypoint_nav.py \
      --waypoints /tmp/waypoints.yaml
  " &
  NAV_TEST_PID=$!

  echo "  Navigasyon testi PID: $NAV_TEST_PID"
  echo "  (Ctrl+C ile durdurabilirsiniz)"
  wait $NAV_TEST_PID
  TEST_EXIT=$?

  echo ""
  if [ "$TEST_EXIT" -eq 0 ]; then
    echo "  ✅ Waypoint testi tamamlandı."
  else
    echo "  ⚠️  Waypoint testi çıkış kodu: $TEST_EXIT"
  fi
fi

echo ""
echo "════════════════════════════════════════"
echo "Durdurmak için: Ctrl+C"
echo "════════════════════════════════════════"

# Arka plan süreçlerini bekle
wait $GZ_PID $NAV_PID
