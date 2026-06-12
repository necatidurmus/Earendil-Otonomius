#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_clearpath_test.sh  —  Clearpath Dünya Test Başlatıcı
#  Leo Rover: GPS navigasyon testi (Clearpath dünyaları)
#
#  Tüm ayarlar sim_config.yaml'dan okunur.
#  Argümanlar config'i override eder.
#
#  Kullanım:
#    ./run_clearpath_test.sh warehouse           # Depo dünyası
#    ./run_clearpath_test.sh office --rviz       # Ofis dünyası + RViz
#    ./run_clearpath_test.sh pipeline --timeout 600
#    ./run_clearpath_test.sh orchard --no-restart
#
#  Desteklenen dünyalar: warehouse, office, orchard, pipeline, solar_farm, construction
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

# ── Dünya adı argümanı ────────────────────────────────────────────────────
if [[ $# -lt 1 ]]; then
  echo "Kullanım: $0 <world_name> [options]"
  echo "Desteklenen dünyalar: warehouse, office, orchard, pipeline, solar_farm, construction"
  exit 1
fi

CLEARPATH_WORLD="$1"
shift

# Dünya doğrulama
VALID_WORLDS=("warehouse" "office" "orchard" "pipeline" "solar_farm" "construction")
if [[ ! " ${VALID_WORLDS[@]} " =~ " ${CLEARPATH_WORLD} " ]]; then
  echo "Hata: Geçersiz dünya adı: $CLEARPATH_WORLD"
  echo "Desteklenen dünyalar: ${VALID_WORLDS[*]}"
  exit 1
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
# Basitleştirilmiş dünyalar kullan (Fuel indirme gerektirmez)
if [[ "$CLEARPATH_WORLD" == "warehouse" ]]; then
  WORLD_SDF="clearpath_warehouse_simple.sdf"
  WORLD_NAME="leo_warehouse"
else
  WORLD_SDF="clearpath_${CLEARPATH_WORLD}.sdf"
  WORLD_NAME="clearpath_${CLEARPATH_WORLD}"
fi
WAYPOINTS="${CLEARPATH_WORLD}_waypoints.yaml"

# ── Renk kodları ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo ""
echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║      LEO ROVER — CLEARPATH DÜNYA TESTİ                 ║${NC}"
echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "  Dünya   : ${YELLOW}$CLEARPATH_WORLD${NC}"
echo -e "  SDF     : ${YELLOW}$WORLD_SDF${NC}"
echo -e "  Timeout : ${YELLOW}${TIMEOUT}s${NC}"
echo -e "  RViz    : ${YELLOW}$LAUNCH_RVIZ${NC}"
echo -e "  Config  : ${YELLOW}$CONFIG${NC}"
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

# ── Eski Gazebo ve navigation süreçlerini temizle ────────────────────────
echo "🧹 Eski süreçler temizleniyor..."
docker exec "$CONTAINER" bash -c "
  pkill -9 -f 'ign gazebo' 2>/dev/null || true
  pkill -9 -f 'ros2 launch' 2>/dev/null || true
  pkill -9 -f 'mission_manager' 2>/dev/null || true
  pkill -9 -f 'slam_toolbox' 2>/dev/null || true
  pkill -9 -f 'nav2' 2>/dev/null || true
  pkill -9 -f 'parameter_bridge' 2>/dev/null || true
  pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
  pkill -9 -f 'ukf_node' 2>/dev/null || true
  pkill -9 -f 'navsat_transform' 2>/dev/null || true
  pkill -9 -f 'gps_monitor' 2>/dev/null || true
  pkill -9 -f 'tunnel_gps_spoofer' 2>/dev/null || true
  pkill -9 -f 'tf_mode_relay' 2>/dev/null || true
  sleep 3
" 2>/dev/null || true

# Gazebo'nun tamamen kapandığını doğrula
for i in $(seq 1 10); do
  if ! docker exec "$CONTAINER" bash -c "pgrep -f 'ign gazebo' >/dev/null 2>&1" 2>/dev/null; then
    break
  fi
  echo "  Gazebo kapatılıyor... (${i}/10)"
  sleep 1
done
echo -e "${GREEN}✓ Eski süreçler temizlendi${NC}"

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
    sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/clearpath/$WORLD_SDF \
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

# ── 2. Navigasyon başlat ──────────────────────────────────────────────────
echo ""
echo -e "${BOLD}[2/3] Navigasyon başlatılıyor...${NC}"

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
for i in $(seq 1 15); do
  if docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    ros2 action list 2>/dev/null | grep -q navigate_to_pose
  " 2>/dev/null; then
    NAV2_READY=true
    break
  fi
  echo "  Nav2 bekleniyor... (${i}/15)"
  sleep 5
done

if [[ "$NAV2_READY" == "true" ]]; then
  echo -e "  ${GREEN}✓ Nav2 action server hazır${NC}"
else
  echo -e "  ${YELLOW}⚠ Nav2 henüz hazır olmayabilir, devam ediliyor...${NC}"
fi

# /fromLL servisi kontrolü (GPS navigasyon için kritik)
echo ""
echo "🔍 /fromLL servisi kontrolü..."
FROMLL_READY=false
for i in $(seq 1 10); do
  if docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    ros2 service list 2>/dev/null | grep -q fromLL
  " 2>/dev/null; then
    FROMLL_READY=true
    break
  fi
  echo "  /fromLL servisi bekleniyor... (${i}/10)"
  sleep 3
done

if [[ "$FROMLL_READY" == "true" ]]; then
  echo -e "  ${GREEN}✓ /fromLL servisi hazır${NC}"
else
  echo -e "  ${RED}✗ /fromLL servisi bulunamadı! GPS koordinatları çevrilemeyecek.${NC}"
  echo "  Devam ediliyor ancak waypoint'lar başarısız olabilir..."
fi

# ── 3. Waypoint testi ─────────────────────────────────────────────────────
echo ""
echo -e "${BOLD}[3/3] $CLEARPATH_WORLD waypoint testi başlıyor...${NC}"

# Costmap ve TF'in hazır olması için ek bekleme
echo "  Nav2 costmap ve TF hazırlanması bekleniyor (30s)..."
sleep 30

# Costmap servislerinin hazır olduğunu doğrula
echo "  Costmap servisleri kontrol ediliyor..."
COSTMAP_READY=false
for i in $(seq 1 10); do
  if docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    ros2 service list 2>/dev/null | grep -q get_costmap
  " 2>/dev/null; then
    COSTMAP_READY=true
    break
  fi
  echo "  Costmap bekleniyor... (${i}/10)"
  sleep 3
done

if [[ "$COSTMAP_READY" == "true" ]]; then
  echo -e "  ${GREEN}✓ Costmap servisleri hazır${NC}"
else
  echo -e "  ${YELLOW}⚠ Costmap henüz hazır olmayabilir${NC}"
fi

echo ""
echo "  Misyon planı (GPS navigasyon - $CLEARPATH_WORLD):"
echo "    WP1: Başlangıç noktası"
echo "    WP2-5: Dünya içi navigasyon noktaları"
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
  grep -E '(✅|⚠️|SUCCEEDED|ABORTED|REJECTED|Total|Toplam|MISSION RESULTS|FAZ|${CLEARPATH_WORLD})' \
    /tmp/waypoint_test.log 2>/dev/null || echo '  Log hazır değil'
" 2>/dev/null || true

echo ""
echo -e "${GREEN}✓ Test tamamlandı!${NC}"
echo ""

# ═══ LOG KAYDET ═══
if [[ -f "$SCRIPT_DIR/scripts/save_logs.sh" ]]; then
  "$SCRIPT_DIR/scripts/save_logs.sh" "$CONTAINER" "clearpath_${CLEARPATH_WORLD}"
fi

echo ""
echo "  Loglar (container):"
echo "    Gazebo     : docker exec $CONTAINER cat /tmp/gazebo.log"
echo "    Navigasyon : docker exec $CONTAINER cat /tmp/navigation.log"
echo "    Waypoint   : docker exec $CONTAINER cat /tmp/waypoint_test.log"
echo ""
echo -e "  ${CYAN}Tekrar çalıştırma: ./run_clearpath_test.sh $CLEARPATH_WORLD${NC}"
echo -e "  ${CYAN}Restart atlayarak : ./run_clearpath_test.sh $CLEARPATH_WORLD --no-restart${NC}"
