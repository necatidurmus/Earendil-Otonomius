#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_full_test.sh  —  Tek Komutla Hibrit GPS+SLAM Tam Test
#
#  Docker container'ı başlatır, workspace'i build eder,
#  Gazebo + Nav2 + Mission Manager çalıştırır ve sonucu raporlar.
#
#  Kullanım:
#    ./run_full_test.sh              # Tam test
#    ./run_full_test.sh --no-build   # Build atla (zaten build edilmişse)
#    ./run_full_test.sh --timeout 600
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONTAINER="ros2-dev"
WORLD_SDF="leo_obstacles.sdf"
WORLD_NAME="leo_obstacles"
TIMEOUT=480
DO_BUILD=true
GAZEBO_WAIT=25
NAV_ACTIVE_TIMEOUT=120
MISSION_TIMEOUT=360

# ── Argümanlar ────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build)  DO_BUILD=false; shift ;;
    --timeout)   TIMEOUT="$2"; shift 2 ;;
    -h|--help)
      echo "Kullanım: $0 [--no-build] [--timeout SECONDS]"
      exit 0 ;;
    *) echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

# ── Renk kodları ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] ✓ $*${NC}"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] ✗ $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] ⚠ $*${NC}"; }

echo ""
echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${CYAN}║   LEO ROVER — TEK KOMUT HİBRİT GPS+SLAM TAM TEST v0.3     ║${NC}"
echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

cleanup() {
  log "Temizlik yapılıyor (container restart)..."
  docker restart "$CONTAINER" >/dev/null 2>&1 || true
  sleep 3
}

# ── 0. Docker container başlat ────────────────────────────────────────────
log "Docker container kontrol ediliyor..."

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  ok "Container '$CONTAINER' mevcut"
  # Container restart — zombie processleri ve eski session'ları temizler
  log "Temiz başlangıç için container yeniden başlatılıyor..."
  docker restart "$CONTAINER" >/dev/null 2>&1
  for i in $(seq 1 15); do
    if docker exec "$CONTAINER" true 2>/dev/null; then break; fi
    sleep 1
  done
  ok "Container '$CONTAINER' temiz olarak hazır"
else
  if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    log "Container durdurulmuş, kaldırılıyor..."
    docker rm -f "$CONTAINER" >/dev/null 2>&1
  fi

  log "Docker container başlatılıyor (docker-compose)..."
  cd "$SCRIPT_DIR"
  xhost +local:docker 2>/dev/null || true
  docker compose up -d --build 2>&1 | tail -5
  
  for i in $(seq 1 30); do
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then break; fi
    sleep 1
  done

  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    err "Container başlatılamadı!"
    exit 1
  fi
  ok "Container '$CONTAINER' başlatıldı"
fi

# X11 erişimi
xhost +local:docker 2>/dev/null || true

# Eski logları temizle
docker exec "$CONTAINER" bash -c "
  rm -f /tmp/gazebo.log /tmp/navigation.log /tmp/mission.log /tmp/nav.log /tmp/waypoint_test.log
" 2>/dev/null || true

# ── 1. Build ──────────────────────────────────────────────────────────────
if $DO_BUILD; then
  log "Workspace build ediliyor..."
  docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    cd /home/ros/ws
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -15
  "
  ok "Build tamamlandı"
else
  warn "Build atlandı (--no-build)"
fi

# ── 2. Gazebo başlat ─────────────────────────────────────────────────────
log "[1/3] Gazebo simülasyonu başlatılıyor ($WORLD_SDF)..."
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup leo_gz.launch.py \
    sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/$WORLD_SDF \
    world_name:=$WORLD_NAME \
    2>&1 | tee /tmp/gazebo.log
"

log "Gazebo başlıyor (${GAZEBO_WAIT}s bekleniyor)..."
sleep "$GAZEBO_WAIT"

# Gazebo topic kontrolü
if docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  timeout 5 ros2 topic list 2>/dev/null | grep -q '/clock'
"; then
  ok "Gazebo çalışıyor (/clock topic mevcut)"
else
  err "Gazebo başlatılamadı!"
  exit 1
fi

# ── 3. Hibrit navigasyon başlat ──────────────────────────────────────────
log "[2/3] Hibrit navigasyon stack başlatılıyor..."
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  ros2 launch leo_gz_bringup navigation_hybrid.launch.py \
    use_sim_time:=true \
    launch_rviz:=false \
    2>&1 | tee /tmp/navigation.log
"

# Nav2 lifecycle active bekle
log "Nav2 lifecycle active bekleniyor (max ${NAV_ACTIVE_TIMEOUT}s)..."
NAV_READY=false
for i in $(seq 1 "$NAV_ACTIVE_TIMEOUT"); do
  if docker exec "$CONTAINER" bash -c "
    grep -q 'Managed nodes are active' /tmp/navigation.log 2>/dev/null
  " 2>/dev/null; then
    NAV_READY=true
    break
  fi
  # Her 10 saniyede bir durum bildir
  if (( i % 10 == 0 )); then
    log "  ...${i}s beklendi"
  fi
  sleep 1
done

if $NAV_READY; then
  ok "Nav2 lifecycle active (${i}s)"
else
  warn "Nav2 active mesajı bulunamadı (${NAV_ACTIVE_TIMEOUT}s). Devam ediliyor..."
fi

# GPS Monitor kontrolü
log "GPS Monitor modu kontrol ediliyor..."
docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  timeout 5 ros2 topic echo /nav_mode --once 2>/dev/null || echo '  (henüz hazır değil)'
" 2>/dev/null || true

# ── 4. Mission Manager ───────────────────────────────────────────────────
echo ""
log "[3/3] Mission Manager başlatılıyor..."
echo -e "${BOLD}${CYAN}──────────────────── GÖREV BAŞLADI ────────────────────${NC}"
echo ""

# Mission Manager'ı arka planda başlat, log'u yakala
docker exec -d "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/mission_manager.py \
    --mission /home/ros/ws/install/leo_gz_bringup/share/leo_gz_bringup/config/hybrid_waypoints.yaml \
    2>&1 | tee /tmp/mission.log
"

# Mission tamamlanmasını bekle
log "Görev çalışıyor (max ${MISSION_TIMEOUT}s)..."
MISSION_DONE=false
for i in $(seq 1 "$MISSION_TIMEOUT"); do
  if docker exec "$CONTAINER" bash -c "
    grep -q 'Toplam:' /tmp/mission.log 2>/dev/null
  " 2>/dev/null; then
    MISSION_DONE=true
    break
  fi
  # Her 30 saniyede durum
  if (( i % 30 == 0 )); then
    # Son WP durumunu göster
    LAST_WP=$(docker exec "$CONTAINER" bash -c "
      grep -E '(WP[0-9]+|FAZ|Phase)' /tmp/mission.log 2>/dev/null | tail -1
    " 2>/dev/null || echo "bekleniyor...")
    log "  ...${i}s — $LAST_WP"
  fi
  sleep 1
done

echo ""
echo -e "${BOLD}${CYAN}═══════════════════ TEST SONUÇLARI ═══════════════════${NC}"
echo ""

if ! $MISSION_DONE; then
  err "Mission ${MISSION_TIMEOUT}s içinde tamamlanamadı!"
  docker exec "$CONTAINER" bash -c "cat /tmp/mission.log 2>/dev/null" || true
  cleanup
  exit 1
fi

# Sonuçları göster
docker exec "$CONTAINER" bash -c "
  grep -E '(═|──|MISSION|WP[0-9]|Toplam|FAZ|Phase)' /tmp/mission.log 2>/dev/null
" 2>/dev/null || true

echo ""

# Başarı kontrolü — "Toplam: X/Y başarılı" satırını parse et
RESULT_LINE=$(docker exec "$CONTAINER" bash -c "
  grep 'Toplam:' /tmp/mission.log 2>/dev/null | tail -1
" 2>/dev/null || echo "")

if echo "$RESULT_LINE" | grep -qP 'Toplam: (\d+)/\1 başarılı'; then
  echo ""
  echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════════════╗${NC}"
  echo -e "${BOLD}${GREEN}║              ✓ TEST BAŞARILI — FULL PASS             ║${NC}"
  echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════════════╝${NC}"
  echo ""
  EXIT_CODE=0
else
  echo ""
  echo -e "${BOLD}${RED}╔══════════════════════════════════════════════════════╗${NC}"
  echo -e "${BOLD}${RED}║              ✗ TEST BAŞARISIZ                        ║${NC}"
  echo -e "${BOLD}${RED}╚══════════════════════════════════════════════════════╝${NC}"
  echo -e "  Sonuç: ${RESULT_LINE}"
  echo ""
  EXIT_CODE=1
fi

# GPS mod geçişleri
echo -e "${BOLD}GPS Monitor mod geçişleri:${NC}"
docker exec "$CONTAINER" bash -c "
  grep 'MOD DEGISIKLIGI' /tmp/navigation.log 2>/dev/null | head -10 || echo '  (geçiş olmadı)'
" 2>/dev/null || true

echo ""
echo "Loglar:"
echo "  Gazebo     : docker exec $CONTAINER cat /tmp/gazebo.log"
echo "  Navigasyon : docker exec $CONTAINER cat /tmp/navigation.log"
echo "  Mission    : docker exec $CONTAINER cat /tmp/mission.log"
echo ""

# Temizlik
cleanup

exit $EXIT_CODE
