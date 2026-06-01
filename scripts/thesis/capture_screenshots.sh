#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  capture_screenshots.sh  —  Otomatik Screenshot Yakalama
#
#  Sırayla birden fazla screenshot yöntemini dener.
#  Başarısız olursa manuel screenshot checklist üretir.
#
#  Kullanım:
#    bash scripts/thesis/capture_screenshots.sh <output_dir>
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

OUTPUT_DIR="${1:-.}"
mkdir -p "$OUTPUT_DIR"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] ✓ $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] ⚠ $*${NC}"; }

# ── Screenshot aracı bul ──────────────────────────────────────────────────
find_screenshot_tool() {
    if command -v gnome-screenshot &>/dev/null; then
        echo "gnome-screenshot"
    elif command -v spectacle &>/dev/null; then
        echo "spectacle"
    elif command -v grim &>/dev/null; then
        echo "grim"
    elif command -v scrot &>/dev/null; then
        echo "scrot"
    elif command -v import &>/dev/null; then
        echo "import"
    else
        echo ""
    fi
}

# ── Tek screenshot al ─────────────────────────────────────────────────────
take_screenshot() {
    local tool="$1"
    local output_file="$2"
    local delay="${3:-2}"

    case "$tool" in
        gnome-screenshot)
            gnome-screenshot -f "$output_file" -d "$delay" 2>/dev/null
            ;;
        spectacle)
            spectacle --background --nonotify --delay "$delay" -o "$output_file" 2>/dev/null
            ;;
        grim)
            sleep "$delay"
            grim "$output_file" 2>/dev/null
            ;;
        scrot)
            sleep "$delay"
            scrot "$output_file" 2>/dev/null
            ;;
        import)
            sleep "$delay"
            import -window root "$output_file" 2>/dev/null
            ;;
        *)
            return 1
            ;;
    esac
}

# ── Ana akış ──────────────────────────────────────────────────────────────
main() {
    local tool
    tool=$(find_screenshot_tool)

    if [[ -z "$tool" ]]; then
        warn "Otomatik screenshot aracı bulunamadı!"
        warn "Desteklenen araçlar: gnome-screenshot, spectacle, grim, scrot, import"
        generate_manual_checklist
        return 0
    fi

    log "Screenshot aracı: $tool"

    # Screenshot listesi
    local screenshots=(
        "01_gazebo_world|Gazebo simülasyon penceresi (dünya + robot)"
        "02_rviz_nav2|RViz: Nav2 path + costmap + robot model"
        "03_rviz_costmap|RViz: Local + global costmap detayı"
        "04_slam_map|RViz: SLAM Toolbox harita çıktısı"
        "05_terminal_log|Terminal: test çıktı logu"
        "06_tf_tree|TF tree (frames.pdf varsa)"
    )

    local captured=0
    local total=${#screenshots[@]}

    for entry in "${screenshots[@]}"; do
        local name="${entry%%|*}"
        local desc="${entry#*|}"
        local output_file="$OUTPUT_DIR/${name}.png"

        log "Screenshot: $desc"
        echo "  → 3 saniye bekleme (pencereyi ayarlayın)..."

        if take_screenshot "$tool" "$output_file" 3; then
            if [[ -f "$output_file" ]]; then
                ok "Kaydedildi: ${name}.png"
                ((captured++))
            else
                warn "Dosya oluşmadı: ${name}.png"
            fi
        else
            warn "Screenshot alınamadı: ${name}.png"
        fi

        # Kullanıcıya zaman ver
        sleep 1
    done

    echo ""
    log "Toplam: $captured/$total screenshot alındı"

    # Manuel checklist her zaman üret (eksik olanlar için)
    generate_manual_checklist

    # frames.pdf varsa kopyala
    if [[ -f "$OUTPUT_DIR/../system_info/frames.pdf" ]]; then
        cp "$OUTPUT_DIR/../system_info/frames.pdf" "$OUTPUT_DIR/06_tf_tree.pdf" 2>/dev/null || true
        ok "TF tree PDF kopyalandı"
    fi
}

# ── Manuel screenshot checklist ───────────────────────────────────────────
generate_manual_checklist() {
    cat > "$OUTPUT_DIR/MANUAL_SCREENSHOT_CHECKLIST.md" <<'CHECKLIST'
# Manuel Screenshot Checklist

Eğer otomatik screenshot alınamadıysa, aşağıdaki ekran görüntülerini manuel olarak alın
ve bu klasöre kaydedin.

## Gerekli Screenshot'lar

| # | Dosya Adı | Açıklama | Ne Zaman |
|---|-----------|----------|----------|
| 1 | `01_gazebo_world.png` | Gazebo simülasyon penceresi, dünya ve robot görünür | Test başlangıcında |
| 2 | `02_rviz_nav2.png` | RViz: Nav2 planlanan yol, costmap, robot modeli | Navigasyon sırasında |
| 3 | `03_rviz_costmap.png` | RViz: Local ve global costmap detayı | Engel yakınında |
| 4 | `04_slam_map.png` | RViz: SLAM Toolbox harita çıktısı | SLAM fazında veya sonrasında |
| 5 | `05_terminal_log.png` | Terminal: test çıktı logu (PASS/FAIL banner) | Test bitiminde |
| 6 | `06_tf_tree.png` | `ros2 run tf2_tools view_frames` çıktısı veya PDF | Herhangi bir zamanda |

## Opsiyonel Screenshot'lar

| # | Dosya Adı | Açıklama |
|---|-----------|----------|
| 7 | `07_matlab_interface.png` | MATLAB App Designer arayüzü (varsa) |
| 8 | `08_rviz_slam_mapping.png` | SLAM harita oluşturma süreci |
| 9 | `09_gps_quality_plot.png` | GPS kalite grafiği (rqt_plot veya plot) |
| 10 | `10_mode_transition.png` | GPS→SLAM geçiş anı RViz görüntüsü |

## İpuçları

- **Wayland**: `grim` veya `spectacle` kullanın
- **X11**: `gnome-screenshot`, `scrot` veya `import` kullanın
- **Docker**: Container içinde GUI varsa `xhost +local:docker` çalıştırın
- **RViz**: `rviz2` başlatmak için launch'a `launch_rviz:=true` ekleyin

## Kaydetme

Screenshot'ları bu klasöre kaydedin:
```
screenshots/
├── 01_gazebo_world.png
├── 02_rviz_nav2.png
├── 03_rviz_costmap.png
├── 04_slam_map.png
├── 05_terminal_log.png
├── 06_tf_tree.png
└── MANUAL_SCREENSHOT_CHECKLIST.md
```
CHECKLIST

    ok "Manuel checklist üretildi: MANUAL_SCREENSHOT_CHECKLIST.md"
}

main "$@"
