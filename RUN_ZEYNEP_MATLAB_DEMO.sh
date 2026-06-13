#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  RUN_ZEYNEP_MATLAB_DEMO.sh
#  Gazebo'yu ve Necati'nin otonom altyapısını başlatır, ardından kontrolü 
#  senin yazdığın MATLAB arayüzüne (mission_control_v2_GPS_FIX) devreder.
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo -e "\033[1;36m=============================================================\033[0m"
echo -e "\033[1;36m           ZEYNEP MISSION CONTROL (MATLAB) BAŞLATICI         \033[0m"
echo -e "\033[1;36m=============================================================\033[0m"
echo ""

echo "🔄 [Adım 1/3] Eski süreçler ve açık kalan MATLAB'lar temizleniyor..."
pkill -f "run_hybrid_test" || true
pkill -f "tunnel_gps_spoofer" || true
pkill -f "tf_mode_relay" || true
pkill -f "mission_manager" || true
pkill -f "ign gazebo" || true
pkill -f "matlab -desktop" || true
pkill -f "ros2 run" || true

cd "$SCRIPT_DIR"

# Docker container'ı ismen zorla durdur ve sil (Çakışmayı önlemek için)
docker stop ros2-dev 2>/dev/null || true
docker rm ros2-dev 2>/dev/null || true

# Docker'ı sıfırdan temiz başlat
docker compose down 2>/dev/null || true
docker compose up -d

echo "🚀 [Adım 2/3] ROS 2 ve Otonom Sistem başlatılıyor..."
echo "Lütfen bekleyin, Gazebo ve Nav2 altyapısının ayağa kalkması 1-2 dakika sürebilir."
# --no-mission ile Necati'nin Python görev yöneticisi iptal ediliyor.
# Sonunda '&' YOK, yani sistem tamamen hazır olana kadar bekleyecek.
./run_hybrid_test.sh --no-mission

echo "📈 [Adım 3/3] MATLAB Mission Control arayüzü açılıyor..."
echo "Lütfen MATLAB'ın açılmasını bekleyin..."

matlab -desktop -r "addpath('$SCRIPT_DIR/matlab_interface'); clear classes; mission_control_v2_GPS_FIX" &

echo ""
echo -e "\033[1;32m✓ Sistem başarıyla tetiklendi!\033[0m"
echo "  1. Gazebo ekranı otomatik olarak açılacak."
echo "  2. MATLAB arayüzün (GPS_FIX sürümü) açılacak."
echo "  3. MATLAB'daki 'ROS2' ışığı yeşil yandığında haritadan hedef seçebilirsin."
echo ""
