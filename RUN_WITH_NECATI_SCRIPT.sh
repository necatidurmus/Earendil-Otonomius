#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  RUN_WITH_NECATI_SCRIPT.sh
#  Bu script, MATLAB arayüzünü devre dışı bırakır ve otonom görevi tamamen 
#  Necati'nin yazdığı "mission_manager.py" (YAML okuyucu) üzerinden başlatır.
#  Bu sayede terminal akmaya devam eder ve görev loglarını anlık görürsün.
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo -e "\033[1;36m=============================================================\033[0m"
echo -e "\033[1;36m           NECATİ'NİN SCRIPTİ İLE TEST BAŞLATICI             \033[0m"
echo -e "\033[1;36m=============================================================\033[0m"
echo ""

echo "🔄 [Adım 1/2] Eski süreçler ve Docker temizleniyor..."
pkill -f "run_hybrid_test" || true
pkill -f "tunnel_gps_spoofer" || true
pkill -f "tf_mode_relay" || true
pkill -f "mission_manager" || true
pkill -f gz || true

cd "$SCRIPT_DIR"

# Docker'ı sıfırdan temiz başlat
docker compose down 2>/dev/null || true
docker compose up -d

echo ""
echo "🚀 [Adım 2/2] Gazebo ve Necati'nin Mission Manager'ı başlatılıyor..."
echo "Lütfen bekleyin, terminal birazdan akmaya başlayacak..."
echo ""

# DİKKAT: Burada --no-mission YOK! Bu yüzden Necati'nin scripti çalışacak
# ve terminalde "✅ WP1 -> 10.5s" gibi loglar akmaya devam edecek.
./run_hybrid_test.sh

echo ""
echo -e "\033[1;32m✓ Test tamamlandı!\033[0m"
