#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
# Dual-UKF GPS Navigasyon Tam Test Scripti
# ═══════════════════════════════════════════════════════════════════════
# Sırayla:
#   1. Eski süreçleri temizle
#   2. Build
#   3. Gazebo + Robot spawn (engelli dünya)
#   4. Dual-UKF + NavSat + Nav2 başlat
#   5. UKF doğrulama + 4 GPS waypoint navigasyonu
#
# Kullanım:
#   ./run_dual_ukf_test.sh                         # GUI + obstacles
#   ./run_dual_ukf_test.sh --world empty           # engelsiz dünya
#   ./run_dual_ukf_test.sh --timeout 300           # 300s zaman aşımı
#
# Durdurmak: Ctrl+C
# ═══════════════════════════════════════════════════════════════════════

set -e

CONTAINER="ros2-dev"
SETUP="source /home/ros/ws/install/setup.bash"
WORLD_TYPE="obstacles"  # obstacles veya empty
TIMEOUT="200"

# Arguman parse: --world <empty|obstacles> --timeout <sn>
while [ $# -gt 0 ]; do
    case "$1" in
        --world)
            WORLD_TYPE="${2:-obstacles}"
            shift 2
            ;;
        --timeout)
            TIMEOUT="${2:-200}"
            shift 2
            ;;
        empty|obstacles)
            WORLD_TYPE="$1"
            shift
            ;;
        *)
            echo "Bilinmeyen arguman: $1"
            echo "Kullanim: $0 [--world empty|obstacles] [--timeout saniye]"
            exit 1
            ;;
    esac
done

# Dünya dosyasını seç
if [ "$WORLD_TYPE" = "empty" ]; then
    WORLD="/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/leo_empty.sdf"
    WORLD_NAME="Boş Dünya"
    GZ_WORLD_NAME="leo_empty"
else
    WORLD="/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/leo_obstacles.sdf"
    WORLD_NAME="Engelli Dünya"
    GZ_WORLD_NAME="leo_obstacles"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  DUAL-UKF GPS NAVİGASYON TAM TESTİ"
echo "═══════════════════════════════════════════════════════════"
echo "  Dünya:    $WORLD_NAME"
echo "  Mimari:   UKF Local → UKF Global + NavSat → Nav2"
echo "  Harita:   empty_50x50_map (proje haritası)"
echo "  Timeout:  ${TIMEOUT}s"
echo "═══════════════════════════════════════════════════════════"
echo ""

# ── 1. Temizlik ───────────────────────────────────────────────────────
echo "[1/5] Eski süreçler temizleniyor..."
docker exec $CONTAINER bash -c "
    # Önce launch süreçlerini durdur (yeni node spawn etmesinler)
    pkill -9 -f 'ros2 launch' 2>/dev/null || true
    sleep 1
    # Gazebo (killall ile kesin öldür)
    killall -9 'ign gazebo' ruby 2>/dev/null || true
    pkill -9 -x gz 2>/dev/null || true
    # UKF / NavSat
    killall -9 ukf_node navsat_transform_node static_transform_publisher 2>/dev/null || true
    # Nav2 node'ları
    killall -9 bt_navigator lifecycle_manager velocity_smoother waypoint_follower \
      controller_server planner_server smoother_server behavior_server \
      component_container_isolated 2>/dev/null || true
    # Köprüler
    killall -9 robot_state_publisher parameter_bridge image_bridge 2>/dev/null || true
    # Test script
    pkill -9 -f test_dual_ukf 2>/dev/null || true
    sleep 3
    echo 'Temizlik tamamlandı'
" || true

# ── 2. Build ──────────────────────────────────────────────────────────
echo "[2/5] Paketler derleniyor..."
docker exec $CONTAINER bash -c "
  cd /home/ros/ws &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select leo_gz_bringup leo_gz_worlds --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
"

# ── 3. Gazebo + Robot ────────────────────────────────────────────────
echo "[3/5] Gazebo başlatılıyor ($WORLD_NAME)..."
GZ_ARGS="-r $WORLD"
docker exec $CONTAINER bash -c "$SETUP && ros2 launch leo_gz_bringup leo_gz.launch.py sim_world:='$GZ_ARGS' world_name:=$GZ_WORLD_NAME" &
GZ_PID=$!

echo "  Gazebo yükleniyor (30s)..."
sleep 30

# TF kontrol
echo "  TF kontrol ediliyor (odom→base_footprint)..."
TF_OK=$(docker exec $CONTAINER bash -c "$SETUP && timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1" | grep -c "Translation" || true)
if [ "$TF_OK" -eq 0 ]; then
    echo "  ⚠️ odom→base_footprint TF yok, 10s daha bekleniyor..."
    sleep 10
fi

# ── 4. Dual-UKF + Nav2 ───────────────────────────────────────────────
echo "[4/5] Dual-UKF + NavSat + Nav2 başlatılıyor..."
docker exec $CONTAINER bash -c "$SETUP && ros2 launch leo_gz_bringup navigation_ukf.launch.py" &
NAV_PID=$!

echo ""
echo "  Başlatma sırası:"
echo "    t=0s   UKF Local  (odom + IMU → /odometry/local)"
echo "    t=4s   UKF Global (/odometry/local + /odometry/gps → /odometry/filtered)"
echo "    t=8s   NavSat     (GPS → /odometry/gps + /fromLL)"
echo "    t=40s  Nav2       (navigate_to_pose action)"
echo ""
echo "  Nav2'nin tam hazır olması için 70s bekleniyor..."
sleep 70

# Nav2 lifecycle kontrol — eğer autostart başarılıysa ekstra müdahale gereksiz
echo "  Nav2 lifecycle durumu kontrol ediliyor..."
NAV2_ACTIVE=$(docker exec $CONTAINER bash -c "$SETUP && timeout 5 ros2 action list 2>/dev/null | grep -c navigate_to_pose" 2>/dev/null || echo 0)
if [ "${NAV2_ACTIVE:-0}" -gt 0 ]; then
    echo "  ✅ Nav2 lifecycle başarılı — tüm node'lar aktif"
else
    echo "  ⚠️  Nav2 autostart başarısız, lifecycle recovery deneniyor..."
    # Reset → Startup
    docker exec $CONTAINER bash -c "$SETUP && \
        timeout 10 ros2 service call /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes '{command: 3}'" >/dev/null 2>&1 || true
    sleep 3
    docker exec $CONTAINER bash -c "$SETUP && \
        timeout 30 ros2 service call /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'" >/dev/null 2>&1 || true
    sleep 5
fi

# ── 4a. Durum Raporu ─────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "  PIPELINE DURUM RAPORU"
echo "════════════════════════════════════════"

docker exec $CONTAINER bash -c "
  $SETUP
  echo ''
  echo '  ── Topic Hızları ──'
  echo -n '  /odom              (DiffDrive):    '
  timeout 3 ros2 topic hz /odom 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  /imu/data_raw      (IMU):          '
  timeout 3 ros2 topic hz /imu/data_raw 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  /navsat            (GPS fix):      '
  timeout 3 ros2 topic hz /navsat 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  /odometry/local    (UKF Local):    '
  timeout 3 ros2 topic hz /odometry/local 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  /odometry/gps      (NavSat):       '
  timeout 3 ros2 topic hz /odometry/gps 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo -n '  /odometry/filtered (UKF Global):   '
  timeout 3 ros2 topic hz /odometry/filtered 2>/dev/null | grep -m1 'average rate' || echo 'veri yok'
  echo ''
  echo '  ── TF Zinciri ──'
  echo -n '  odom→base_footprint: '
  timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep 'Translation' | head -1 || echo 'yok'
  echo -n '  map→odom:            '
  timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep 'Translation' | head -1 || echo 'yok'
  echo -n '  map→base_footprint:  '
  timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep 'Translation' | head -1 || echo 'yok'
"

# Nav2 action server kontrol
echo ""
echo "  ── Nav2 Action Server ──"
NAV2_OK=false
for i in $(seq 1 15); do
    RESULT=$(docker exec $CONTAINER bash -c "$SETUP && timeout 2 ros2 action list 2>/dev/null | grep navigate_to_pose" || true)
    if [ -n "$RESULT" ]; then
        NAV2_OK=true
        echo "  navigate_to_pose: ✅ Hazır"
        break
    fi
    echo "  Bekleniyor... ($i/15)"
    sleep 2
done

if [ "$NAV2_OK" = false ]; then
    echo "  ❌ Nav2 action server başlatılamadı!"
    echo "  Logları kontrol edin."
    wait $GZ_PID $NAV_PID
    exit 1
fi

# ── 5. Test ───────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════"
echo "  4 GPS WAYPOINT NAVİGASYON TESTİ"
echo "════════════════════════════════════════"

# Waypoints dosyasını container'a kopyala
WAYPOINTS="$HOME/ros-humble-sim/src/leo_simulator/leo_gz_bringup/config/waypoints.yaml"
docker cp "$WAYPOINTS" $CONTAINER:/tmp/waypoints.yaml 2>/dev/null || true

# Test scriptini container'a kopyala
docker cp "$HOME/ros-humble-sim/test_dual_ukf.py" $CONTAINER:/tmp/test_dual_ukf.py

echo "  Dual-UKF doğrulama + 4 waypoint navigasyonu başlıyor..."
echo ""

# Timeout ile testi çalıştır
timeout $TIMEOUT docker exec $CONTAINER bash -c "
  $SETUP
  python3 /tmp/test_dual_ukf.py --waypoints /tmp/waypoints.yaml
" &
TEST_PID=$!

echo "  Test PID: $TEST_PID  (Timeout: ${TIMEOUT}s)"
echo "  Durdurmak: Ctrl+C"
echo ""

wait $TEST_PID
TEST_EXIT=$?

echo ""
if [ "$TEST_EXIT" -eq 0 ]; then
    echo "  ✅ Dual-UKF GPS navigasyon testi tamamlandı."
elif [ "$TEST_EXIT" -eq 124 ]; then
    echo "  ⏰ Zaman aşımı (${TIMEOUT}s) — test tamamlanamadı."
else
    echo "  ⚠️ Test çıkış kodu: $TEST_EXIT"
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  Arka plan süreçleri çalışmaya devam ediyor."
echo "  Durdurmak: Ctrl+C"
echo "═══════════════════════════════════════════════════════════"

# Arka plan süreçlerini bekle
wait $GZ_PID $NAV_PID
