#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_hybrid_test.sh  —  Hibrit GPS+SLAM Navigasyon Test Başlatıcı
#  Leo Rover: Dual-UKF (açık alan) + SLAM Toolbox (tünel/kapı)
#
#  Tüm ayarlar sim_config.yaml'dan okunur.
#  Argümanlar config'i override eder.
#
#  Kullanım:
#    ./run_hybrid_test.sh                    # Engelli dünya + tünel + kapı
#    ./run_hybrid_test.sh --world empty      # Boş dünya (SLAM testi yok)
#    ./run_hybrid_test.sh --rviz             # RViz ile görselleştir
#    ./run_hybrid_test.sh --timeout 600      # 10 dakika timeout
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$SCRIPT_DIR/sim_config.yaml"
CONTAINER="ros2-dev"

# ── sim_config.yaml oku ───────────────────────────────────────────────────
if [[ -f "$CONFIG" ]]; then
  _cfg() { python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c$1)"; }
  WORLD=$(_cfg "['test']['world']")
  TIMEOUT=$(_cfg "['test']['timeout']")
  LAUNCH_RVIZ=$(_cfg "['test']['launch_rviz']")
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
  WORLD="obstacles"; TIMEOUT=600; LAUNCH_RVIZ=false
  CFG_GAZEBO_WAIT=25; CFG_NAV_WAIT=50
  CFG_UKF_GLOBAL=4; CFG_NAVSAT=8; CFG_NAV2=40
  CFG_GRAVITY=-9.81; CFG_RTF=1.0; CFG_GROUND_MU=0.8
  CFG_WHEEL_MU=3.0; CFG_MAX_SPEED=0.4
fi

# ── Argüman ayrıştırma (config'i override eder) ───────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --world)    WORLD="$2";   shift 2 ;;
    --timeout)  TIMEOUT="$2"; shift 2 ;;
    --rviz)     LAUNCH_RVIZ=true; shift ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//'
      exit 0 ;;
    *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

# ── Dünya seçimi ──────────────────────────────────────────────────────────
if [[ "$WORLD" == "empty" ]]; then
  WORLD_SDF="leo_empty.sdf"
  WORLD_NAME="leo_empty"
else
  WORLD_SDF="leo_obstacles.sdf"
  WORLD_NAME="leo_obstacles"
fi

# ── Renk kodları ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo ""
echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║      LEO ROVER — HİBRİT GPS+SLAM NAVIGASYON TESTİ       ║${NC}"
echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Dünya   : ${YELLOW}$WORLD_SDF${NC}"
echo -e "  Timeout : ${YELLOW}${TIMEOUT}s${NC}"
echo -e "  RViz    : ${YELLOW}$LAUNCH_RVIZ${NC}"
echo -e "  Config  : ${YELLOW}$CONFIG${NC}"
echo ""
echo -e "${CYAN}Zamanlama (sim_config.yaml):${NC}"
echo "  Gazebo startup : ${CFG_GAZEBO_WAIT}s"
echo "  UKF Global     : +${CFG_UKF_GLOBAL}s"
echo "  NavSat         : +${CFG_NAVSAT}s"
echo "  Nav2           : +${CFG_NAV2}s"
echo "  Nav stack wait : ${CFG_NAV_WAIT}s"
echo ""
echo -e "${CYAN}Fizik (sim_config.yaml):${NC}"
echo "  Yercekimi      : ${CFG_GRAVITY} m/s^2"
echo "  Gercek zaman   : ${CFG_RTF}x"
echo "  Zemin mu       : ${CFG_GROUND_MU}"
echo "  Tekerlek mu    : ${CFG_WHEEL_MU}"
echo "  Max hiz        : ${CFG_MAX_SPEED} m/s"
echo ""

# ── Docker container kontrolü ─────────────────────────────────────────────
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo -e "${RED}Hata: '${CONTAINER}' container'ı çalışmıyor!${NC}"
  echo "  docker-compose up -d  ile başlatın"
  exit 1
fi

echo -e "${GREEN}✓ Container '$CONTAINER' hazır${NC}"

# ── X11 erişimi ───────────────────────────────────────────────────────────
xhost +local:docker 2>/dev/null || true

# ── Paket build (Docker içinde) ───────────────────────────────────────────
echo ""
echo "🔨 Paketler build ediliyor..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  cd /home/ros/ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -12
"
echo -e "${GREEN}✓ Build tamamlandı${NC}"

# ── Önceki processleri temizle ────────────────────────────────────────────
echo ""
echo "🧹 Önceki session temizleniyor..."
docker exec "$CONTAINER" bash -c "
  pkill -f 'ros2 launch' 2>/dev/null || true
  pkill -f 'gz sim'      2>/dev/null || true
  pkill -f 'rviz2'       2>/dev/null || true
  sleep 2
" 2>/dev/null || true

# ── 1. Gazebo başlat ──────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}[1/3] Gazebo simülasyonu başlatılıyor...${NC}"
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup leo_gz.launch.py \
    sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/$WORLD_SDF \
    world_name:=$WORLD_NAME \
    2>&1 | tee /tmp/gazebo.log
"
echo "  Gazebo başlıyor (${CFG_GAZEBO_WAIT}s bekleniyor)..."
sleep "$CFG_GAZEBO_WAIT"

# ── 2. Hibrit navigasyon başlat ───────────────────────────────────────────
echo ""
echo -e "${BOLD}[2/3] Hibrit navigasyon başlatılıyor...${NC}"
echo "  t=0s    → UKF Local + SLAM Toolbox + GPS Monitor + Spoofer"
echo "  t=+${CFG_UKF_GLOBAL}s   → UKF Global"
echo "  t=+${CFG_NAVSAT}s   → NavSat Transform"
echo "  t=+${CFG_NAV2}s  → Nav2"

RVIZ_ARG="false"
[[ "$LAUNCH_RVIZ" == "true" ]] && RVIZ_ARG="true"

docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup navigation_hybrid.launch.py \
    use_sim_time:=true \
    launch_rviz:=$RVIZ_ARG \
    2>&1 | tee /tmp/navigation.log
"
echo "  Navigasyon stack başlıyor (${CFG_NAV_WAIT}s bekleniyor)..."
sleep "$CFG_NAV_WAIT"

# ── GPS Monitor durum kontrolü ────────────────────────────────────────────
echo ""
echo "🔍 GPS Monitor modu:"
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  timeout 5 ros2 topic echo /nav_mode --once 2>/dev/null || echo '  (henüz hazır değil)'
" 2>/dev/null || true

# ── 3. Waypoint testi ─────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}[3/3] Waypoint navigasyon testi başlıyor...${NC}"
echo ""
echo "  Waypoint planı:"
echo "    WP1   : Dış alan GPS (+5,+3)"
echo "    WP2   : Tünele yaklaşma (+12,0) → GPS hâlâ iyi"
echo "    WP3   : Tünel içi (+22,0) → GPS→SLAM geçişi"
echo "    WP4   : Tünel derinliği (+28,0) → SLAM modu"
echo "    WP5   : Tünelden çıkış (+5,-3) → SLAM→GPS geçişi"
echo "    WP6   : Başlangıca dönüş (-3,-5) → GPS modu"
echo ""

docker exec -it "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash

  python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/mission_manager.py \
    --mission /home/ros/ws/install/leo_gz_bringup/share/leo_gz_bringup/config/hybrid_waypoints.yaml \
    2>&1 | tee /tmp/waypoint_test.log
" || true

# ── Sonuç özeti ───────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${CYAN}═══════════════════════ TEST SONUÇLARI ═══════════════════════${NC}"
docker exec "$CONTAINER" bash -c "
  grep -E '(✅|⚠️|SUCCEEDED|ABORTED|REJECTED|Total|Toplam|MISSION RESULTS|FAZ)' \
    /tmp/waypoint_test.log 2>/dev/null || echo '  Log hazır değil'
" 2>/dev/null || true

echo ""
echo -e "${BOLD}GPS Monitor mod geçişleri:${NC}"
docker exec "$CONTAINER" bash -c "
  grep 'MOD DEGISIKLIGI' /tmp/navigation.log 2>/dev/null || echo '  (gecis olmadi)'
" 2>/dev/null || true

echo ""
echo -e "${GREEN}✓ Test tamamlandı!${NC}"
echo ""
echo "  Loglar:"
echo "    Gazebo     : docker exec $CONTAINER cat /tmp/gazebo.log"
echo "    Navigasyon : docker exec $CONTAINER cat /tmp/navigation.log"
echo "    Waypoint   : docker exec $CONTAINER cat /tmp/waypoint_test.log"
