#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  mltest/run_industrial.sh — Industrial Tesis Başlatıcı (MATLAB-Ready)
#
#  Gazebo + Nav2 başlatır, waypoint testi BAŞLATMAZ.
#  MATLAB arayüzünden waypoint gönderilmesini bekler.
#  GPS-denied ortam: SLAM navigasyonu kullanır.
#
#  Kullanım:
#    ./mltest/run_industrial.sh
#    ./mltest/run_industrial.sh --rviz
#    ./mltest/run_industrial.sh --no-restart
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG="$ROOT_DIR/sim_config.yaml"
CONTAINER="ros2-dev"
SKIP_RESTART=false

if [[ -f "$CONFIG" ]]; then
  _cfg() { python3 -c "import yaml; c=yaml.safe_load(open('$CONFIG')); print(c$1)"; }
  LAUNCH_RVIZ=$(_cfg "['test']['launch_rviz']")
  CFG_GAZEBO_WAIT=$(_cfg "['timing']['gazebo_startup']")
  CFG_NAV_WAIT=$(_cfg "['timing']['nav_stack_wait']")
else
  LAUNCH_RVIZ=false
  CFG_GAZEBO_WAIT=30; CFG_NAV_WAIT=60
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rviz)         LAUNCH_RVIZ=true; shift ;;
    --no-restart)   SKIP_RESTART=true; shift ;;
    -h|--help)      grep '^#' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

WORLD_SDF="leo_industrial.sdf"
WORLD_NAME="leo_industrial"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo ""
echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║   INDUSTRIAL — MATLAB-CONTROLLED TEST (mltest)          ║${NC}"
echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Dünya   : ${YELLOW}$WORLD_SDF${NC}"
echo -e "  RViz    : ${YELLOW}$LAUNCH_RVIZ${NC}"
echo -e "  Mod     : ${YELLOW}MATLAB waypoint control (SLAM)${NC}"
echo ""
echo -e "${CYAN}Not: GPS-denied ortam — SLAM navigasyonu${NC}"
echo ""

xhost +local:docker 2>/dev/null || true

if [[ "$SKIP_RESTART" == "false" ]]; then
  echo "🔄 Container yeniden başlatılıyor..."
  docker restart "$CONTAINER" >/dev/null 2>&1
  for i in $(seq 1 15); do
    if docker exec "$CONTAINER" true 2>/dev/null; then break; fi
    sleep 1
  done
  echo -e "${GREEN}✓ Container hazır${NC}"
else
  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo -e "${RED}Hata: '${CONTAINER}' çalışmıyor!${NC}"; exit 1
  fi
  echo -e "${GREEN}✓ Container hazır${NC}"
fi

# ── ROS2 ağ ayarları (dış PC'den erişim için) ─────────────────────────────
docker exec "$CONTAINER" bash -c "
  echo 'export ROS_LOCALHOST_ONLY=0' >> /home/ros/.bashrc
  echo 'export ROS_DOMAIN_ID=0' >> /home/ros/.bashrc
" 2>/dev/null || true

docker exec "$CONTAINER" bash -c "rm -f /tmp/gazebo.log /tmp/navigation.log /tmp/ml_nav.log" 2>/dev/null || true

echo ""
echo "🔨 Build ediliyor..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  cd /home/ros/ws
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -8
"
echo -e "${GREEN}✓ Build tamamlandı${NC}"

echo ""
echo -e "${BOLD}[1/2] Gazebo başlatılıyor...${NC}"
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup leo_gz.launch.py \
    sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/$WORLD_SDF \
    world_name:=$WORLD_NAME \
    2>&1 | tee /tmp/gazebo.log
"
echo "  Gazebo (${CFG_GAZEBO_WAIT}s)..."
sleep "$CFG_GAZEBO_WAIT"

if docker exec "$CONTAINER" bash -c "pgrep -f 'ign gazebo' >/dev/null 2>&1"; then
  echo -e "  ${GREEN}✓ Gazebo çalışıyor${NC}"
else
  echo -e "  ${RED}✗ Gazebo başlatılamadı!${NC}"; exit 1
fi

echo ""
echo -e "${BOLD}[2/2] SLAM navigasyon başlatılıyor...${NC}"
RVIZ_ARG="false"
[[ "$LAUNCH_RVIZ" == "true" || "$LAUNCH_RVIZ" == "True" ]] && RVIZ_ARG="true"

docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup navigation_hybrid.launch.py \
    use_sim_time:=true \
    launch_rviz:=$RVIZ_ARG \
    2>&1 | tee /tmp/ml_nav.log
"
echo "  Nav2 (${CFG_NAV_WAIT}s)..."
sleep "$CFG_NAV_WAIT"

echo ""
echo "🔍 Nav2 kontrol..."
NAV2_READY=false
for i in $(seq 1 15); do
  if docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && ros2 action list 2>/dev/null | grep -q navigate_to_pose" 2>/dev/null; then
    NAV2_READY=true; break
  fi
  echo "  Bekleniyor... (${i}/15)"; sleep 5
done
[[ "$NAV2_READY" == "true" ]] && echo -e "  ${GREEN}✓ Nav2 hazır${NC}" || echo -e "  ${YELLOW}⚠ Nav2 bekleniyor${NC}"

echo ""
echo -e "${BOLD}${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo -e "${BOLD}${GREEN}  Sistem hazır! MATLAB'dan waypoint bekleniyor...${NC}"
echo -e "${BOLD}${GREEN}═══════════════════════════════════════════════════════════${NC}"
echo ""
