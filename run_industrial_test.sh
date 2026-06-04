#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_industrial_test.sh  —  Endüstriyel Tesis SLAM Test Başlatıcı
#  Leo Rover: SLAM navigasyon testi (fabrika/depo ortamı, GPS-denied)
#
#  Tüm ayarlar sim_config.yaml'dan okunur.
#  Argümanlar config'i override eder.
#
#  Kullanım:
#    ./run_industrial_test.sh                    # Endüstriyel dünyası
#    ./run_industrial_test.sh --rviz             # RViz ile görselleştir
#    ./run_industrial_test.sh --timeout 600      # 10 dakika timeout
#    ./run_industrial_test.sh --no-restart       # Container restart atla
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$SCRIPT_DIR/sim_config.yaml"
CONTAINER="ros2-dev"
SKIP_RESTART=false

# ── sim_config.yaml oku ───────────────────────────────────────────────────
if [[ -f "$CONFIG" ]]; then
  _cfg() { python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c$1)"; }
  TIMEOUT=$(_cfg "['test']['timeout']")
  LAUNCH_RVIZ=$(_cfg "['test']['launch_rviz']")
  CFG_GAZEBO_WAIT=$(_cfg "['timing']['gazebo_startup']")
  CFG_NAV_WAIT=$(_cfg "['timing']['nav_stack_wait']")
  CFG_UKF_GLOBAL=$(_cfg "['timing']['ukf_global_start']")
  CFG_NAVSAT=$(_cfg "['timing']['navsat_start']")
  CFG_NAV2=$(_cfg "['timing']['nav2_start']")
else
  echo "  sim_config.yaml bulunamadi, varsayilan degerler kullaniliyor"
  TIMEOUT=600; LAUNCH_RVIZ=false
  CFG_GAZEBO_WAIT=25; CFG_NAV_WAIT=50
  CFG_UKF_GLOBAL=4; CFG_NAVSAT=8; CFG_NAV2=40
fi

# ── Argüman ayrıştırma (config'i override eder) ───────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)      TIMEOUT="$2"; shift 2 ;;
    --rviz)         LAUNCH_RVIZ=true; shift ;;
    --no-restart)   SKIP_RESTART=true; shift ;;
    --no-mission)   SKIP_MISSION=true; shift ;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//'
      exit 0 ;;
    *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

# ── Dünya seçimi ──────────────────────────────────────────────────────────
WORLD_SDF="leo_industrial.sdf"
WORLD_NAME="leo_industrial"
WAYPOINTS="industrial_waypoints.yaml"

# ── Renk kodları ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo ""
echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║      LEO ROVER — ENDÜSTRİYEL TESİS SLAM TESTİ          ║${NC}"
echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Dünya   : ${YELLOW}$WORLD_SDF${NC}"
echo -e "  Timeout : ${YELLOW}${TIMEOUT}s${NC}"
echo -e "  RViz    : ${YELLOW}$LAUNCH_RVIZ${NC}"
echo -e "  Config  : ${YELLOW}$CONFIG${NC}"
echo ""
echo -e "${CYAN}Not: Bu test GPS-denied ortamda SLAM navigasyonu kullanır.${NC}"
echo ""

# ── X11 erişimi ───────────────────────────────────────────────────────────
xhost +local:docker 2>/dev/null || true

# ── Container temiz başlangıç ─────────────────────────────────────────────
if [[ "$SKIP_RESTART" == "false" ]]; then
  echo "🔄 Container temiz başlangıç için yeniden başlatılıyor..."
  docker restart "$CONTAINER" >/dev/null 2>&1
  for i in $(seq 1 15); do
    if docker exec "$CONTAINER" true 2>/dev/null; then
      break
    fi
    sleep 1
  done
  echo -e "${GREEN}✓ Container '$CONTAINER' temiz olarak hazır${NC}"
else
  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo -e "${RED}Hata: '${CONTAINER}' container'ı çalışmıyor!${NC}"
    echo "  docker-compose up -d  ile başlatın"
    exit 1
  fi
  echo -e "${GREEN}✓ Container '$CONTAINER' hazır (restart atlandı)${NC}"
fi

# ── Eski logları temizle ──────────────────────────────────────────────────
docker exec "$CONTAINER" bash -c "
  rm -f /tmp/gazebo.log /tmp/navigation.log /tmp/waypoint_test.log /tmp/mission.log
" 2>/dev/null || true

# ── Paket build (Docker içinde) ───────────────────────────────────────────
echo ""
echo "🔨 Paketler build ediliyor..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  cd /home/ros/ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -12
"
echo -e "${GREEN}✓ Build tamamlandı${NC}"

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

# Gazebo process kontrolü
if docker exec "$CONTAINER" bash -c "pgrep -f 'ign gazebo' >/dev/null 2>&1"; then
  echo -e "  ${GREEN}✓ Gazebo çalışıyor${NC}"
else
  echo -e "  ${RED}✗ Gazebo başlatılamadı! Log:${NC}"
  docker exec "$CONTAINER" bash -c "tail -20 /tmp/gazebo.log 2>/dev/null" || true
  exit 1
fi

# ── 2. SLAM navigasyon başlat ─────────────────────────────────────────────
echo ""
echo -e "${BOLD}[2/3] SLAM navigasyon başlatılıyor...${NC}"
echo -e "  ${CYAN}GPS-denied ortam: SLAM Toolbox kullanılacak${NC}"

RVIZ_ARG="false"
[[ "$LAUNCH_RVIZ" == "true" || "$LAUNCH_RVIZ" == "True" ]] && RVIZ_ARG="true"

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

# Nav2 hazırlık kontrolü
echo ""
echo "🔍 Nav2 hazırlık kontrolü..."
NAV2_READY=false
for i in $(seq 1 10); do
  if docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    ros2 action list 2>/dev/null | grep -q navigate_to_pose
  " 2>/dev/null; then
    NAV2_READY=true
    break
  fi
  echo "  Nav2 bekleniyor... (${i}/10)"
  sleep 5
done

if [[ "$NAV2_READY" == "true" ]]; then
  echo -e "  ${GREEN}✓ Nav2 action server hazır${NC}"
else
  echo -e "  ${YELLOW}⚠ Nav2 henüz hazır olmayabilir, devam ediliyor...${NC}"
fi

# ── 3. Waypoint testi ─────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}[3/3] Endüstriyel tesis SLAM testi başlıyor...${NC}"
echo ""
echo "  Misyon planı (SLAM navigasyon - raf koridorları):"
echo "    WP1: Başlangıç noktası"
echo "    WP2: Raf koridoru 1"
echo "    WP3: Raf koridoru 2"
echo "    WP4: Yükleme rampası"
echo "    WP5: Ofis alanı"
echo "    WP6: Başlangıca dönüş"
echo ""

if [[ "${SKIP_MISSION:-false}" == "true" ]]; then
  echo -e "${YELLOW}⚠ Görev atlandı (--no-mission). Kontrol MATLAB GUI'sine devredildi.${NC}"
else
  docker exec -i "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash

  python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/mission_manager.py \
    --mission /home/ros/ws/install/leo_gz_bringup/share/leo_gz_bringup/config/$WAYPOINTS \
    2>&1 | tee /tmp/waypoint_test.log
" || true
fi

# ── Sonuç özeti ───────────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}${CYAN}═══════════════════════ TEST SONUÇLARI ═══════════════════════${NC}"
docker exec "$CONTAINER" bash -c "
  grep -E '(✅|⚠️|SUCCEEDED|ABORTED|REJECTED|Total|Toplam|MISSION RESULTS|FAZ|indoor_slam)' \
    /tmp/waypoint_test.log 2>/dev/null || echo '  Log hazır değil'
" 2>/dev/null || true

echo ""
echo -e "${GREEN}✓ Test tamamlandı!${NC}"
echo ""
echo "  Loglar:"
echo "    Gazebo     : docker exec $CONTAINER cat /tmp/gazebo.log"
echo "    Navigasyon : docker exec $CONTAINER cat /tmp/navigation.log"
echo "    Waypoint   : docker exec $CONTAINER cat /tmp/waypoint_test.log"
echo ""
echo -e "  ${CYAN}Tekrar çalıştırma: ./run_industrial_test.sh${NC}"
echo -e "  ${CYAN}Restart atlayarak : ./run_industrial_test.sh --no-restart${NC}"
