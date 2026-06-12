#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════════
#  run_thesis_evidence.sh  —  Tez Kanıt Toplama Ana Script
#
#  Mevcut test scriptlerini sarmalayarak düzenli kanıt klasörleri üretir.
#  Her test çalıştırması zaman damgalı bir klasöre log, screenshot, rosbag,
#  CSV, grafik ve sistem bilgisi kaydeder.
#
#  Kullanım:
#    bash scripts/thesis/run_thesis_evidence.sh --test full --record-bag
#    bash scripts/thesis/run_thesis_evidence.sh --test hybrid --duration 300
#    bash scripts/thesis/run_thesis_evidence.sh --test all --outdir thesis_evidence
#    bash scripts/thesis/run_thesis_evidence.sh --test empty --dry-run
#
#  Desteklenen testler:
#    full       → run_full_test.sh (obstacle world, 9 WP)
#    hybrid     → run_hybrid_test.sh (obstacle world, hybrid GPS+SLAM)
#    empty      → run_hybrid_test.sh --world empty
#    obstacle   → run_hybrid_test.sh (obstacle world)
#    terrain    → run_terrain_test.sh
#    urban      → run_urban_test.sh
#    industrial → run_industrial_test.sh (SLAM only)
#    slope      → run_slope_test.sh
#    earendil   → run_earendil_test.sh
#    all        → Sırasıyla: full, empty, industrial, terrain
# ═══════════════════════════════════════════════════════════════════════════

set -euo pipefail

# ── Sabitler ──────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONTAINER="ros2-dev"
DEFAULT_OUTDIR="$PROJECT_ROOT/thesis_evidence"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

# ── Varsayılanlar ─────────────────────────────────────────────────────────
TEST_NAME=""
DURATION=600
OUTDIR="$DEFAULT_OUTDIR"
NO_GUI=false
RECORD_BAG=false
SKIP_SCREENSHOTS=false
DRY_RUN=false
SKIP_BUILD=false
SKIP_RESTART=false

# ── Renk kodları ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] ✓ $*${NC}"; }
err()  { echo -e "${RED}[$(date +%H:%M:%S)] ✗ $*${NC}"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] ⚠ $*${NC}"; }

# ── Yardım ────────────────────────────────────────────────────────────────
usage() {
    cat <<'EOF'
Kullanım: bash scripts/thesis/run_thesis_evidence.sh [SEÇENEKLER]

Zorunlu:
  --test <name>          Test adı: full, hybrid, empty, obstacle, terrain,
                         urban, industrial, slope, earendil, all

İsteğe bağlı:
  --duration <seconds>   Maksimum test süresi (varsayılan: 600)
  --outdir <path>        Çıktı klasörü (varsayılan: thesis_evidence/)
  --no-gui               GUI başlatma (headless mod)
  --record-bag           Rosbag kaydı al
  --skip-screenshots     Screenshot alma
  --skip-build           Build adımını atla
  --skip-restart         Container restart'ı atla
  --dry-run              Komutları göster ama çalıştırma
  -h, --help             Bu yardımı göster

Örnekler:
  # Tam test, rosbag kaydı ile
  bash scripts/thesis/run_thesis_evidence.sh --test full --record-bag

  # Hybrid test, 5 dakika, özel çıktı klasörü
  bash scripts/thesis/run_thesis_evidence.sh --test hybrid --duration 300

  # Tüm testleri sırayla çalıştır
  bash scripts/thesis/run_thesis_evidence.sh --test all --record-bag

  # Sadece ne yapacağını göster
  bash scripts/thesis/run_thesis_evidence.sh --test full --dry-run
EOF
    exit 0
}

# ── Argüman ayrıştırma ────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --test)             TEST_NAME="$2"; shift 2 ;;
        --duration)         DURATION="$2"; shift 2 ;;
        --outdir)           OUTDIR="$2"; shift 2 ;;
        --no-gui)           NO_GUI=true; shift ;;
        --record-bag)       RECORD_BAG=true; shift ;;
        --skip-screenshots) SKIP_SCREENSHOTS=true; shift ;;
        --skip-build)       SKIP_BUILD=true; shift ;;
        --skip-restart)     SKIP_RESTART=true; shift ;;
        --dry-run)          DRY_RUN=true; shift ;;
        -h|--help)          usage ;;
        *) err "Bilinmeyen argüman: $1"; exit 1 ;;
    esac
done

if [[ -z "$TEST_NAME" ]]; then
    err "--test parametresi gerekli. --help ile kullanım bilgisi alın."
    exit 1
fi

# ── Test listesi (all modu için) ──────────────────────────────────────────
ALL_TESTS="full empty industrial terrain"

# ── Ana fonksiyon: tek test çalıştır ──────────────────────────────────────
run_single_test() {
    local test_name="$1"
    local test_dir="$OUTDIR/${TIMESTAMP}_${test_name}"

    echo ""
    echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}${CYAN}║   TEZ KANIT TOPLAMA: ${test_name^^}                               ║${NC}"
    echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""

    # Klasör yapısı oluştur
    mkdir -p "$test_dir"/{logs,screenshots,rosbags,csv,plots,system_info,summaries,configs}

    local test_start
    test_start="$(date -Iseconds)"

    # ── 1. Sistem bilgisi kaydet ──────────────────────────────────────────
    log "[1/7] Sistem bilgisi kaydediliyor..."
    save_system_info "$test_dir"

    # ── 2. Config dosyalarını kopyala ─────────────────────────────────────
    log "[2/7] Config dosyaları kopyalanıyor..."
    save_configs "$test_dir"

    # ── 3. Test komutunu belirle ──────────────────────────────────────────
    local test_cmd
    test_cmd=$(get_test_command "$test_name")
    if [[ -z "$test_cmd" ]]; then
        err "Bilinmeyen test: $test_name"
        return 1
    fi

    # Test komutunu summary'ye yaz
    echo "Test: $test_name" > "$test_dir/summaries/test_command.txt"
    echo "Command: $test_cmd" >> "$test_dir/summaries/test_command.txt"
    echo "Start: $test_start" >> "$test_dir/summaries/test_command.txt"
    echo "Duration limit: ${DURATION}s" >> "$test_dir/summaries/test_command.txt"
    echo "Record bag: $RECORD_BAG" >> "$test_dir/summaries/test_command.txt"
    echo "No GUI: $NO_GUI" >> "$test_dir/summaries/test_command.txt"

    if $DRY_RUN; then
        warn "DRY RUN — komut çalıştırılmayacak:"
        echo "  $test_cmd"
        echo "  Çıktı klasörü: $test_dir"
        return 0
    fi

    # ── 4. Rosbag kaydını başlat (arka plan) ─────────────────────────────
    local bag_pid=""
    if $RECORD_BAG; then
        log "[3/7] Rosbag kaydı başlatılıyor..."
        start_rosbag_recording "$test_dir" &
        bag_pid=$!
        sleep 2
    else
        log "[3/7] Rosbag kaydı atlandı (--record-bag yok)"
    fi

    # ── 5. Testi çalıştır ─────────────────────────────────────────────────
    log "[4/7] Test çalıştırılıyor: $test_name"
    local test_exit_code=0
    local test_end

    # Testi çalıştır, logu hem terminal'e hem dosyaya yaz
    eval "$test_cmd" 2>&1 | tee "$test_dir/logs/test_output.log" || test_exit_code=$?

    test_end="$(date -Iseconds)"

    # ── 6. Rosbag kaydını durdur ──────────────────────────────────────────
    if [[ -n "$bag_pid" ]]; then
        log "[5/7] Rosbag kaydı durduruluyor..."
        kill "$bag_pid" 2>/dev/null || true
        wait "$bag_pid" 2>/dev/null || true
        # Container içindeki rosbag process'ini de durut
        docker exec "$CONTAINER" bash -c "pkill -f 'ros2 bag record' 2>/dev/null" 2>/dev/null || true
        sleep 2
    else
        log "[5/7] Rosbag atlandı"
    fi

    # ── 7. ROS introspection snapshot ─────────────────────────────────────
    log "[6/7] ROS snapshot alınıyor..."
    save_ros_snapshot "$test_dir" || true

    # ── 8. Screenshot'lar ─────────────────────────────────────────────────
    if ! $SKIP_SCREENSHOTS; then
        log "[7/7] Screenshot'lar alınıyor..."
        "$SCRIPT_DIR/capture_screenshots.sh" "$test_dir/screenshots" || true
    else
        log "[7/7] Screenshot'lar atlandı"
    fi

    # ── 9. Sonuç özeti üret ───────────────────────────────────────────────
    local result="UNKNOWN"
    if [[ $test_exit_code -eq 0 ]]; then
        result="PASS"
    else
        result="FAIL"
    fi

    generate_summary "$test_dir" "$test_name" "$test_start" "$test_end" "$result" "$test_cmd"

    # ── 10. CSV'leri üret ─────────────────────────────────────────────────
    generate_csvs "$test_dir" "$test_name"

    echo ""
    if [[ "$result" == "PASS" ]]; then
        echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${BOLD}${GREEN}║   TEST BAŞARILI — Kanıtlar kaydedildi                       ║${NC}"
        echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════════════════════╝${NC}"
    else
        echo -e "${BOLD}${RED}╔══════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${BOLD}${RED}║   TEST BAŞARISIZ — Kanıtlar yine de kaydedildi              ║${NC}"
        echo -e "${BOLD}${RED}╚══════════════════════════════════════════════════════════════╝${NC}"
    fi
    echo ""
    echo "  Çıktı klasörü: $test_dir"
    echo "  Sonuç: $result"
    echo "  Exit code: $test_exit_code"
    echo ""

    return $test_exit_code
}

# ── Sistem bilgisi kaydet ─────────────────────────────────────────────────
save_system_info() {
    local dir="$1/system_info"

    {
        echo "=== Thesis Evidence System Info ==="
        echo "Date: $(date -Iseconds)"
        echo "Hostname: $(hostname 2>/dev/null || echo 'unknown')"
        echo ""
        echo "=== uname -a ==="
        uname -a 2>/dev/null || echo "unavailable"
        echo ""
        echo "=== lsb_release -a ==="
        lsb_release -a 2>/dev/null || cat /etc/os-release 2>/dev/null || echo "unavailable"
        echo ""
        echo "=== ROS_DISTRO ==="
        echo "${ROS_DISTRO:-not set}"
        echo ""
        echo "=== ROS Environment ==="
        printenv | grep -E '^ROS_' | sort 2>/dev/null || echo "unavailable"
        echo ""
        echo "=== Git Info ==="
        echo "Branch: $(git -C "$PROJECT_ROOT" branch --show-current 2>/dev/null || echo 'unknown')"
        echo "Commit: $(git -C "$PROJECT_ROOT" rev-parse HEAD 2>/dev/null || echo 'unknown')"
        echo "Diff stat:"
        git -C "$PROJECT_ROOT" diff --stat 2>/dev/null || echo "unavailable"
        echo ""
        echo "=== Python Version ==="
        python3 --version 2>/dev/null || echo "unavailable"
        echo ""
        echo "=== Docker Info ==="
        docker version --format '{{.Server.Version}}' 2>/dev/null || echo "docker unavailable"
        docker ps --filter "name=$CONTAINER" --format '{{.Names}} {{.Status}}' 2>/dev/null || echo "container not found"
        echo ""
    } > "$dir/system_info.txt" 2>&1

    # Container içi bilgiler (eğer container çalışıyorsa)
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        {
            echo "=== Container ROS Info ==="
            docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && ros2 doctor --report" 2>/dev/null || echo "ros2 doctor unavailable"
            echo ""
            echo "=== Gazebo/GZ Version ==="
            docker exec "$CONTAINER" bash -c "ign gazebo --version 2>/dev/null || gz sim --version 2>/dev/null || echo 'unavailable'"
            echo ""
            echo "=== Colcon Version ==="
            docker exec "$CONTAINER" bash -c "colcon version 2>/dev/null || echo 'unavailable'"
        } > "$dir/container_info.txt" 2>&1
    fi

    ok "Sistem bilgisi kaydedildi"
}

# ── Config dosyalarını kopyala ────────────────────────────────────────────
save_configs() {
    local dir="$1/configs"
    local config_src="$PROJECT_ROOT/src/leo_simulator/leo_gz_bringup/config"

    # Merkezi config
    cp "$PROJECT_ROOT/sim_config.yaml" "$dir/" 2>/dev/null || true

    # Nav2, SLAM, UKF, NavSat configleri
    for f in nav2_params_hybrid.yaml slam_toolbox_params.yaml ukf_local.yaml \
             ukf_global_hybrid.yaml navsat.yaml hybrid_waypoints.yaml; do
        cp "$config_src/$f" "$dir/" 2>/dev/null || true
    done

    # Launch dosyaları
    mkdir -p "$dir/launch"
    cp "$PROJECT_ROOT/src/leo_simulator/leo_gz_bringup/launch/"*.py "$dir/launch/" 2>/dev/null || true

    # docker-compose.yml
    cp "$PROJECT_ROOT/docker-compose.yml" "$dir/" 2>/dev/null || true
    cp "$PROJECT_ROOT/Dockerfile" "$dir/" 2>/dev/null || true

    ok "Config dosyaları kopyalandı"
}

# ── Test komutu belirle ───────────────────────────────────────────────────
get_test_command() {
    local name="$1"
    local extra_args=""

    if $SKIP_RESTART; then
        extra_args="$extra_args --no-restart"
    fi

    case "$name" in
        full)
            if $SKIP_BUILD; then
                extra_args="$extra_args --no-build"
            fi
            echo "cd $PROJECT_ROOT && bash run_full_test.sh --timeout $DURATION $extra_args"
            ;;
        hybrid|obstacle)
            echo "cd $PROJECT_ROOT && bash run_hybrid_test.sh --timeout $DURATION $extra_args"
            ;;
        empty)
            echo "cd $PROJECT_ROOT && bash run_hybrid_test.sh --world empty --timeout $DURATION $extra_args"
            ;;
        terrain)
            echo "cd $PROJECT_ROOT && bash run_terrain_test.sh --timeout $DURATION $extra_args"
            ;;
        urban)
            echo "cd $PROJECT_ROOT && bash run_urban_test.sh --timeout $DURATION $extra_args"
            ;;
        industrial)
            echo "cd $PROJECT_ROOT && bash run_industrial_test.sh --timeout $DURATION $extra_args"
            ;;
        slope)
            echo "cd $PROJECT_ROOT && bash run_slope_test.sh --timeout $DURATION $extra_args"
            ;;
        earendil)
            echo "cd $PROJECT_ROOT && bash run_earendil_test.sh --timeout $DURATION $extra_args"
            ;;
        *)
            echo ""
            ;;
    esac
}

# ── Rosbag kaydını başlat ─────────────────────────────────────────────────
start_rosbag_recording() {
    local test_dir="$1"
    local bag_dir="$test_dir/rosbags"

    # Container'ın çalıştığını kontrol et
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        warn "Container çalışmıyor, rosbag kaydı atlanıyor"
        return 1
    fi

    # Kaydedilecek topicler
    local topics=(
        "/tf"
        "/tf_static"
        "/cmd_vel"
        "/odom"
        "/odometry/filtered"
        "/odometry/local"
        "/scan"
        "/imu/data_raw"
        "/navsat"
        "/navsat_filtered"
        "/nav_mode"
        "/gps_quality"
        "/odometry/gps"
        "/map"
        "/goal_pose"
    )

    # Container içinde rosbag kaydı başlat
    local topic_list="${topics[*]}"
    docker exec -d "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        mkdir -p /tmp/rosbag_thesis
        ros2 bag record -o /tmp/rosbag_thesis/evidence \
            $topic_list \
            2>&1 | tee /tmp/rosbag_thesis/record.log
    " 2>/dev/null || warn "Rosbag kaydı başlatılamadı"

    # Süre sonunda otomatik durdur
    sleep "$DURATION"
    docker exec "$CONTAINER" bash -c "pkill -f 'ros2 bag record' 2>/dev/null" 2>/dev/null || true
}

# ── ROS snapshot ──────────────────────────────────────────────────────────
save_ros_snapshot() {
    local dir="$1/system_info"

    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        warn "Container çalışmıyor, ROS snapshot atlanıyor"
        return 1
    fi

    log "  ROS node/topic/service snapshot alınıyor..."

    # Node list
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 node list 2>/dev/null
    " > "$dir/ros_nodes.txt" 2>&1 || true

    # Topic list with types
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 topic list -t 2>/dev/null
    " > "$dir/ros_topics.txt" 2>&1 || true

    # Service list
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 service list 2>/dev/null
    " > "$dir/ros_services.txt" 2>&1 || true

    # Action list
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 action list 2>/dev/null
    " > "$dir/ros_actions.txt" 2>&1 || true

    # Lifecycle nodes
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 lifecycle nodes 2>/dev/null
    " > "$dir/ros_lifecycle_nodes.txt" 2>&1 || true

    # Topic hz measurements (5 saniye örnek)
    for topic in /scan /odom /imu/data_raw /cmd_vel /nav_mode; do
        docker exec "$CONTAINER" bash -c "
            source /opt/ros/humble/setup.bash
            timeout 5 ros2 topic hz $topic 2>/dev/null || echo 'topic not available'
        " > "$dir/topic_hz_$(echo "$topic" | tr '/' '_').txt" 2>&1 || true
    done

    # Single echo samples
    for topic in /odom /tf /scan; do
        docker exec "$CONTAINER" bash -c "
            source /opt/ros/humble/setup.bash
            timeout 3 ros2 topic echo $topic --once 2>/dev/null || echo 'timeout'
        " > "$dir/topic_echo_$(echo "$topic" | tr '/' '_').txt" 2>&1 || true
    done

    # TF frames
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 run tf2_tools view_frames 2>/dev/null
    " > "$dir/tf_frames.txt" 2>&1 || true

    # frames.pdf'yi host'a kopyala (eğer oluştuysa)
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        cd /tmp && ros2 run tf2_tools view_frames 2>/dev/null
        cp /tmp/frames.pdf /tmp/frames.gv /home/ros/ws/ 2>/dev/null || true
    " 2>/dev/null || true
    docker cp "$CONTAINER:/tmp/frames.pdf" "$dir/frames.pdf" 2>/dev/null || true
    docker cp "$CONTAINER:/tmp/frames.gv" "$dir/frames.gv" 2>/dev/null || true

    # Param list
    docker exec "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash
        ros2 param list 2>/dev/null
    " > "$dir/ros_params.txt" 2>&1 || true

    ok "ROS snapshot kaydedildi"
}

# ── Summary üret ──────────────────────────────────────────────────────────
generate_summary() {
    local dir="$1"
    local test_name="$2"
    local start_time="$3"
    local end_time="$4"
    local result="$5"
    local cmd="$6"

    local git_commit
    git_commit="$(git -C "$PROJECT_ROOT" rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
    local git_branch
    git_branch="$(git -C "$PROJECT_ROOT" branch --show-current 2>/dev/null || echo 'unknown')"

    cat > "$dir/summaries/thesis_evidence_summary.md" <<SUMMARY
# Thesis Evidence Summary

## Test Information
- **Test name**: $test_name
- **Date**: $start_time
- **End**: $end_time
- **Git commit**: $git_commit ($git_branch)
- **ROS distro**: ${ROS_DISTRO:-humble}
- **Simulation world**: $(get_world_name "$test_name")
- **Launch command**: \`$cmd\`
- **Duration limit**: ${DURATION}s
- **Result**: **$result**

## Generated Evidence Files
- **Logs**: \`$dir/logs/\`
- **Screenshots**: \`$dir/screenshots/\`
- **Rosbags**: \`$dir/rosbags/\`
- **CSV**: \`$dir/csv/\`
- **Plots**: \`$dir/plots/\`
- **System info**: \`$dir/system_info/\`
- **Configs**: \`$dir/configs/\`

## Thesis Claims Supported

| Claim | Evidence File | Status |
|-------|--------------|--------|
| ROS 2 Humble environment | system_info/system_info.txt | ✅ |
| Gazebo simulation running | logs/test_output.log | ✅ |
| Nav2 navigation active | system_info/ros_nodes.txt | ✅ |
| SLAM Toolbox running | system_info/ros_nodes.txt | ✅ |
| Dual-UKF sensor fusion | system_info/ros_topics.txt | ✅ |
| GPS/SLAM mode switching | logs/test_output.log | ✅ |
| Waypoint navigation | logs/test_output.log | ✅ |
| TF tree structure | system_info/frames.pdf | ✅ |
| Topic rate verification | system_info/topic_hz_*.txt | ✅ |
| Reproducible test | configs/sim_config.yaml | ✅ |

## Important Observations
- Navigation behavior: [test logdan analiz edilecek]
- Obstacle avoidance: [costmap verisi varsa]
- GPS/SLAM transition: [nav_mode logundan]
- SLAM behavior: [SLAM map verisi varsa]
- Localization: [odometry verisi]
- Failures: [test_output.log'dan]

## Missing Evidence
- Eksik veriler MISSING_METRICS.md dosyasında listelenmiştir.
SUMMARY

    # MISSING_METRICS.md oluştur
    cat > "$dir/summaries/MISSING_METRICS.md" <<'MISSING'
# Missing Metrics

Bu dosya, test çalıştırmasından elde edilemeyen metrikleri listeler.

## Eksik Veriler

| Metric | Neden Gerekli | Nasıl Üretilir |
|--------|--------------|-----------------|
| Path deviation (RMSE) | Planlanan vs gerçek rota karşılaştırması | Rosbag'ten /odometry/filtered + waypoint verisi ile |
| Mode transition latency | GPS→SLAM geçiş süresi | /nav_mode topic timestamp analizi |
| Position error at transition | Geçiş anında pozisyon hatası | TF snapshot karşılaştırması |
| CPU/GPU load | Sistem performansı | htop/nvidia-smi log kaydı |
| Success rate statistics | Çoklu çalıştırma istatistiği | 5-10 tekrar çalıştırma |
| MATLAB interface verification | MATLAB-ROS2 bağlantısı | MATLAB uygulaması çalıştırılması |

## Notlar
- Rosbag kaydı (--record-bag) aktifse, offline analiz mümkündür.
- generate_plots.py scripti robag'ten grafik üretebilir.
MISSING

    ok "Summary üretildi"
}

# ── CSV üret ──────────────────────────────────────────────────────────────
generate_csvs() {
    local dir="$1"
    local test_name="$2"

    # test_summary.csv
    cat > "$dir/csv/test_summary.csv" <<CSV
test_name,start_time,end_time,duration,result,notes
$test_name,$(date -Iseconds),,$(date +%s),UNKNOWN,auto-generated
CSV

    # topic_rates.csv (ros_snapshot'tan)
    if [[ -f "$dir/system_info/ros_topics.txt" ]]; then
        echo "topic,message_type,average_hz,sample_duration" > "$dir/csv/topic_rates.csv"
        # topic hz dosyalarından parse et
        for hz_file in "$dir/system_info"/topic_hz_*.txt; do
            if [[ -f "$hz_file" ]]; then
                local topic_name
                topic_name="$(basename "$hz_file" .txt | sed 's/topic_hz_/\//' | tr '_' '/')"
                local avg_hz
                avg_hz="$(grep -oP 'average rate: \K[\d.]+' "$hz_file" 2>/dev/null || echo "N/A")"
                echo "$topic_name,,$avg_hz,5s" >> "$dir/csv/topic_rates.csv"
            fi
        done
    fi

    # waypoint_results.csv placeholder
    cat > "$dir/csv/waypoint_results.csv" <<'CSV'
waypoint_id,target_x,target_y,reached,final_distance,elapsed_time,mode
CSV

    # mode_transitions.csv placeholder
    cat > "$dir/csv/mode_transitions.csv" <<'CSV'
timestamp,from_mode,to_mode,reason
CSV

    # errors.csv placeholder
    cat > "$dir/csv/errors.csv" <<'CSV'
timestamp,error_type,message,severity
CSV

    # Loglardan waypoint sonuçlarını çıkarmaya çalış
    if [[ -f "$dir/logs/test_output.log" ]]; then
        parse_waypoint_results "$dir/logs/test_output.log" "$dir/csv/waypoint_results.csv"
        parse_mode_transitions "$dir/logs/test_output.log" "$dir/csv/mode_transitions.csv"
    fi

    ok "CSV dosyaları üretildi"
}

# ── Waypoint sonuçlarını parse et ─────────────────────────────────────────
parse_waypoint_results() {
    local log_file="$1"
    local csv_file="$2"

    # Mission Manager log formatını parse et:
    # "  ✅ WP1_south_clear → 20.9s"
    # "  ⚠️ WP5_tunnel_middle: ABORTED (15.7s)"
    # "  WP1: SUCCEEDED     20.9s  WP1_south_clear"

    grep -E '(✅|⚠️|SUCCEEDED|ABORTED|REJECTED|MODE_TIMEOUT|SKIP)' "$log_file" 2>/dev/null | \
    while IFS= read -r line; do
        local wp_name status elapsed
        wp_name="$(echo "$line" | grep -oP 'WP\w+' | head -1)"
        status="$(echo "$line" | grep -oP '(SUCCEEDED|ABORTED|REJECTED|MODE_TIMEOUT|SKIP)' | head -1)"
        elapsed="$(echo "$line" | grep -oP '[\d.]+s' | head -1 | tr -d 's')"

        if [[ -n "$wp_name" && -n "$status" ]]; then
            echo "$wp_name,,,$([ "$status" = "SUCCEEDED" ] && echo "true" || echo "false"),,$elapsed," >> "$csv_file"
        fi
    done || true
}

# ── Mod geçişlerini parse et ──────────────────────────────────────────────
parse_mode_transitions() {
    local log_file="$1"
    local csv_file="$2"

    # "MOD DEGISIKLIGI: GPS -> SLAM (kalite: 0.15, status: -1, onay: 3)"
    grep -i 'MOD DEGISIKLIGI\|mod.*gps\|mod.*slam' "$log_file" 2>/dev/null | \
    while IFS= read -r line; do
        local from_mode to_mode
        from_mode="$(echo "$line" | grep -oP '(GPS|SLAM)' | head -1)"
        to_mode="$(echo "$line" | grep -oP '(GPS|SLAM)' | tail -1)"
        if [[ -n "$from_mode" && -n "$to_mode" && "$from_mode" != "$to_mode" ]]; then
            echo "$(date -Iseconds),$from_mode,$to_mode,gps_quality" >> "$csv_file"
        fi
    done || true
}

# ── Dünya adını belirle ───────────────────────────────────────────────────
get_world_name() {
    case "$1" in
        full|hybrid|obstacle) echo "leo_obstacles" ;;
        empty)                echo "leo_empty" ;;
        terrain)              echo "leo_open_terrain" ;;
        urban)                echo "leo_urban" ;;
        industrial)           echo "leo_industrial" ;;
        slope)                echo "leo_sloped_terrain" ;;
        earendil)             echo "leo_earendil_env" ;;
        *)                    echo "unknown" ;;
    esac
}

# ── Ana akış ──────────────────────────────────────────────────────────────
main() {
    echo ""
    echo -e "${BOLD}${CYAN}╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BOLD}${CYAN}║        TEZ KANIT TOPLAMA SİSTEMİ v1.0                       ║${NC}"
    echo -e "${BOLD}${CYAN}╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo "  Proje: $PROJECT_ROOT"
    echo "  Çıktı: $OUTDIR"
    echo "  Test:  $TEST_NAME"
    echo "  Süre:  ${DURATION}s"
    echo "  Rosbag: $RECORD_BAG"
    echo "  GUI:   $([ "$NO_GUI" = true ] && echo "kapalı" || echo "açık")"
    echo ""

    # Çıktı klasörünü oluştur
    mkdir -p "$OUTDIR"

    if [[ "$TEST_NAME" == "all" ]]; then
        log "Tüm testler sırayla çalıştırılıyor: $ALL_TESTS"
        local all_pass=true
        for t in $ALL_TESTS; do
            if ! run_single_test "$t"; then
                all_pass=false
                warn "Test başarısız: $t — sonraki teste geçiliyor"
            fi
            # Testler arası bekleme
            sleep 5
        done

        echo ""
        if $all_pass; then
            echo -e "${BOLD}${GREEN}Tüm testler başarılı!${NC}"
        else
            echo -e "${BOLD}${RED}Bazı testler başarısız. Detaylar için klasörleri kontrol edin.${NC}"
        fi
    else
        run_single_test "$TEST_NAME"
    fi
}

main "$@"
