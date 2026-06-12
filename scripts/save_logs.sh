#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  save_logs.sh — Test loglarını tarih/saat ile kaydet
#
#  Container'dan tüm logları çeker, test_logs/ klasörüne kaydeder.
#  Özet rapor oluşturur (başarı/başarısızlık, mod geçişleri, hatalar).
#
#  Kullanım:
#    ./scripts/save_logs.sh                    # Varsayılan: ros2-dev container
#    ./scripts/save_logs.sh my-container       # Özel container adı
#    ./scripts/save_logs.sh ros2-dev mytest    # Özel test adı (prefix)
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/test_logs"
CONTAINER="${1:-ros2-dev}"
TEST_NAME="${2:-test}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

mkdir -p "$LOG_DIR"

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  LOG KAYDETME: $TEST_NAME @ $TIMESTAMP"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Log dosyaları
MISSION_LOG="$LOG_DIR/${TEST_NAME}_mission_${TIMESTAMP}.log"
WAYPOINT_LOG="$LOG_DIR/${TEST_NAME}_waypoint_${TIMESTAMP}.log"
NAVIGATION_LOG="$LOG_DIR/${TEST_NAME}_navigation_${TIMESTAMP}.log"
GAZEBO_LOG="$LOG_DIR/${TEST_NAME}_gazebo_${TIMESTAMP}.log"
SUMMARY_LOG="$LOG_DIR/${TEST_NAME}_summary_${TIMESTAMP}.txt"

# Container'dan logları çek
echo "  Container: $CONTAINER"
echo "  Hedef: $LOG_DIR"
echo ""

docker exec "$CONTAINER" cat /tmp/mission.log > "$MISSION_LOG" 2>/dev/null || true
docker exec "$CONTAINER" cat /tmp/waypoint_test.log > "$WAYPOINT_LOG" 2>/dev/null || true
docker exec "$CONTAINER" cat /tmp/navigation.log > "$NAVIGATION_LOG" 2>/dev/null || true
docker exec "$CONTAINER" cat /tmp/gazebo.log > "$GAZEBO_LOG" 2>/dev/null || true

# mission.log boşsa waypoint_test.log'u kullan (hybrid/clearpath testleri)
if [[ ! -s "$MISSION_LOG" && -s "$WAYPOINT_LOG" ]]; then
  cp "$WAYPOINT_LOG" "$MISSION_LOG"
fi

# Dosya boyutları
MISSION_SIZE=$(wc -c < "$MISSION_LOG" 2>/dev/null || echo 0)
WAYPOINT_SIZE=$(wc -c < "$WAYPOINT_LOG" 2>/dev/null || echo 0)
NAVIGATION_SIZE=$(wc -c < "$NAVIGATION_LOG" 2>/dev/null || echo 0)
GAZEBO_SIZE=$(wc -c < "$GAZEBO_LOG" 2>/dev/null || echo 0)

echo "  ✓ mission.log      → $(basename "$MISSION_LOG") ($MISSION_SIZE bytes)"
echo "  ✓ waypoint_test.log → $(basename "$WAYPOINT_LOG") ($WAYPOINT_SIZE bytes)"
echo "  ✓ navigation.log   → $(basename "$NAVIGATION_LOG") ($NAVIGATION_SIZE bytes)"
echo "  ✓ gazebo.log       → $(basename "$GAZEBO_LOG") ($GAZEBO_SIZE bytes)"
echo ""

# ═══ ÖZET RAPOR ═══
{
  echo "═══════════════════════════════════════════════════════════"
  echo "  TEST SUMMARY: $TEST_NAME"
  echo "  Timestamp: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "  Container: $CONTAINER"
  echo "═══════════════════════════════════════════════════════════"
  echo ""

  # Mission sonuçları
  if [[ -s "$MISSION_LOG" ]]; then
    echo "── MISSION RESULTS ──"
    grep -E '(Toplam:|SUCCEEDED|REJECTED|ABORTED|MODE_TIMEOUT)' "$MISSION_LOG" 2>/dev/null | tail -20 || echo "  (sonuç bulunamadı)"
    echo ""
  fi

  # Mod geçişleri
  if [[ -s "$NAVIGATION_LOG" ]]; then
    echo "── MODE TRANSITIONS ──"
    grep -E 'MOD DEGISIKLIGI' "$NAVIGATION_LOG" 2>/dev/null || echo "  (mod geçişi yok)"
    echo ""

    # GPS override değişiklikleri
    echo "── GPS OVERRIDE ──"
    grep -E 'GPS override degisti' "$NAVIGATION_LOG" 2>/dev/null || echo "  (override değişikliği yok)"
    echo ""

    # Tünel latch
    echo "── TUNNEL LATCH ──"
    grep -E 'TUNEL LATCH' "$NAVIGATION_LOG" 2>/dev/null || echo "  (tünel latch yok)"
    echo ""

    # UKF reset
    echo "── UKF RESET ──"
    grep -E 'UKF.*reset|sıfırlandı|sıfırlama' "$NAVIGATION_LOG" 2>/dev/null || echo "  (UKF reset yok)"
    echo ""

    # Hatalar
    echo "── ERRORS ──"
    ERROR_COUNT=$(grep -c 'ERROR' "$NAVIGATION_LOG" 2>/dev/null || echo 0)
    CRASH_COUNT=$(grep -c 'process has died' "$NAVIGATION_LOG" 2>/dev/null || echo 0)
    FRAME_ERROR_COUNT=$(grep -c 'frame does not exist' "$NAVIGATION_LOG" 2>/dev/null || echo 0)
    echo "  Total ERROR lines: $ERROR_COUNT"
    echo "  Process crashes: $CRASH_COUNT"
    echo "  Frame missing errors: $FRAME_ERROR_COUNT"
    echo ""

    # TF mode relay versiyonu
    echo "── NODE VERSIONS ──"
    grep -E 'TF Mode Relay v[0-9.]+' "$NAVIGATION_LOG" 2>/dev/null | head -1 || echo "  (tf_mode_relay versiyonu bulunamadı)"
    grep -E 'MISSION MANAGER v[0-9.]+' "$MISSION_LOG" 2>/dev/null | head -1 || echo "  (mission_manager versiyonu bulunamadı)"
    echo ""
  fi

  echo "═══════════════════════════════════════════════════════════"
  echo "  Log files saved to: $LOG_DIR"
  echo "═══════════════════════════════════════════════════════════"

} > "$SUMMARY_LOG"

cat "$SUMMARY_LOG"
echo ""
echo "✓ Özet rapor: $(basename "$SUMMARY_LOG")"
echo ""
