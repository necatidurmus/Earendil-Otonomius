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
SETUP="source /opt/ros/humble/setup.bash && source /home/ros/ws/install/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$SCRIPT_DIR/sim_config.yaml"

# ── Docker container kontrolü ─────────────────────────────────────────────
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "❌ Hata: '${CONTAINER}' container'ı çalışmıyor!"
    echo "   Önce başlatın:  docker-compose up -d"
    exit 1
fi

# ── sim_config.yaml oku ───────────────────────────────────────────────────
if [ -f "$CONFIG" ]; then
  _cfg() { python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c$1)"; }
  WORLD_TYPE=$(_cfg "['test']['world']")
  TIMEOUT=$(_cfg "['test']['timeout']")
  CFG_GAZEBO_WAIT=$(_cfg "['timing']['gazebo_startup']")
  CFG_NAV_WAIT=$(_cfg "['timing']['nav_stack_wait']")
  CFG_UKF_GLOBAL=$(_cfg "['timing']['ukf_global_start']")
  CFG_NAVSAT=$(_cfg "['timing']['navsat_start']")
  CFG_NAV2=$(_cfg "['timing']['nav2_start']")
  CFG_GRAVITY=$(_cfg ".get('gravity',{}).get('z',-9.81)")
  CFG_RTF=$(_cfg ".get('physics',{}).get('real_time_factor',1.0)")
  CFG_GROUND_MU=$(_cfg ".get('ground',{}).get('mu',0.8)")
  CFG_WHEEL_MU=$(_cfg ".get('robot',{}).get('wheel_mu',3.0)")
  CFG_MAX_SPEED=$(_cfg ".get('robot',{}).get('max_speed_ms',0.4)")
else
  echo "  sim_config.yaml bulunamadi, varsayilan degerler kullaniliyor"
  WORLD_TYPE="obstacles"; TIMEOUT=200
  CFG_GAZEBO_WAIT=25; CFG_NAV_WAIT=50
  CFG_UKF_GLOBAL=4; CFG_NAVSAT=8; CFG_NAV2=40
  CFG_GRAVITY=-9.81; CFG_RTF=1.0; CFG_GROUND_MU=0.8
  CFG_WHEEL_MU=3.0; CFG_MAX_SPEED=0.4
fi

# Arguman parse: --world <empty|obstacles> --timeout <sn> (config'i override eder)
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
echo "  Config:   $CONFIG"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Zamanlama (sim_config.yaml):"
echo "  Gazebo startup : ${CFG_GAZEBO_WAIT}s"
echo "  UKF Global     : +${CFG_UKF_GLOBAL}s"
echo "  NavSat         : +${CFG_NAVSAT}s"
echo "  Nav2           : +${CFG_NAV2}s"
echo "  Nav stack wait : ${CFG_NAV_WAIT}s"
echo ""
echo "Fizik (sim_config.yaml):"
echo "  Yerçekimi      : ${CFG_GRAVITY} m/s²"
echo "  Gerçek zaman   : ${CFG_RTF}x"
echo "  Zemin mu       : ${CFG_GROUND_MU}"
echo "  Tekerlek mu    : ${CFG_WHEEL_MU}"
echo "  Max hız        : ${CFG_MAX_SPEED} m/s"
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
  source /opt/ros/humble/setup.bash &&
  cd /home/ros/ws &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -12
"

# ── 3. Gazebo + Robot ────────────────────────────────────────────────
echo "[3/5] Gazebo başlatılıyor ($WORLD_NAME)..."
GZ_ARGS="-r $WORLD"
docker exec $CONTAINER bash -c "$SETUP && ros2 launch leo_gz_bringup leo_gz.launch.py sim_world:='$GZ_ARGS' world_name:=$GZ_WORLD_NAME" &
GZ_PID=$!

echo "  Gazebo yükleniyor (${CFG_GAZEBO_WAIT}s)..."
sleep $CFG_GAZEBO_WAIT

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
echo "  Başlatma sırası (sim_config.yaml):"
echo "    t=0s    UKF Local  (odom + IMU → /odometry/local)"
echo "    t=+${CFG_UKF_GLOBAL}s  UKF Global (/odometry/local + /odometry/gps → /odometry/filtered)"
echo "    t=+${CFG_NAVSAT}s  NavSat     (GPS → /odometry/gps + /fromLL)"
echo "    t=+${CFG_NAV2}s Nav2       (navigate_to_pose action)"
echo ""
echo "  Nav2'nin tam hazır olması için ${CFG_NAV_WAIT}s bekleniyor..."
sleep $CFG_NAV_WAIT

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
WAYPOINTS="$SCRIPT_DIR/src/leo_simulator/leo_gz_bringup/config/waypoints.yaml"
if [ ! -f "$WAYPOINTS" ]; then
  echo "⚠️  Waypoints dosyası bulunamadı: $WAYPOINTS"
  echo "   Varsayılan waypoints kullanılacak."
fi
docker cp "$WAYPOINTS" $CONTAINER:/tmp/waypoints.yaml 2>/dev/null || true

# Test scriptini container'a kopyala
docker cp "$SCRIPT_DIR/test_dual_ukf.py" $CONTAINER:/tmp/test_dual_ukf.py

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
