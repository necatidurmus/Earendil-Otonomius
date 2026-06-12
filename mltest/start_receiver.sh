#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  mltest/start_receiver.sh — Waypoint Receiver Node Başlatıcı
#
#  Docker içinde waypoint_receiver.py node'unu başlatır.
#  MATLAB'dan gelen waypoint'leri Nav2'ye iletir.
#
#  Kullanım:
#    ./mltest/start_receiver.sh              # map frame
#    ./mltest/start_receiver.sh --frame odom # odom frame
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

CONTAINER="ros2-dev"
FRAME="map"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frame)  FRAME="$2"; shift 2 ;;
    *)        echo "Bilinmeyen argüman: $1"; exit 1 ;;
  esac
done

echo "🔄 Waypoint Receiver başlatılıyor (frame=$FRAME)..."

docker exec -i "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  source /home/ros/ws/install/setup.bash
  python3 /home/ros/ws/install/leo_gz_bringup/lib/leo_gz_bringup/waypoint_receiver.py \
    --frame $FRAME
"
