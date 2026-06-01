# Tez Kanıt Toplama Sistemi — README

## Genel Bakış

Bu sistem, Earendil-Otonomius projesindeki ROS 2 + Gazebo + Nav2 testlerini
çalıştırarak tezde kullanılabilir formatta kanıt dosyaları üretir.

## Hızlı Başlangıç

### 1. Tek Test Çalıştırma

```bash
# Full test (obstacle world, 9 waypoint, rosbag kaydı ile)
bash scripts/thesis/run_thesis_evidence.sh --test full --record-bag

# Hybrid test, 5 dakika süre limiti
bash scripts/thesis/run_thesis_evidence.sh --test hybrid --duration 300

# Boş dünya testi
bash scripts/thesis/run_thesis_evidence.sh --test empty

# Endüstriyel tesis (SLAM only)
bash scripts/thesis/run_thesis_evidence.sh --test industrial
```

### 2. Tüm Testleri Çalıştırma

```bash
# Sırasıyla: full, empty, industrial, terrain
bash scripts/thesis/run_thesis_evidence.sh --test all --record-bag
```

### 3. Dry Run (Ne yapacağını göster)

```bash
bash scripts/thesis/run_thesis_evidence.sh --test full --dry-run
```

## Parametreler

| Parametre | Açıklama | Varsayılan |
|-----------|----------|------------|
| `--test <name>` | Test adı (zorunlu) | — |
| `--duration <s>` | Maksimum süre (saniye) | 600 |
| `--outdir <path>` | Çıktı klasörü | `thesis_evidence/` |
| `--no-gui` | GUI başlatma | false |
| `--record-bag` | Rosbag kaydı | false |
| `--skip-screenshots` | Screenshot alma | false |
| `--skip-build` | Build atla | false |
| `--skip-restart` | Container restart atla | false |
| `--dry-run` | Sadece göster | false |

## Desteklenen Testler

| Test | Script | Dünya | Açıklama |
|------|--------|-------|----------|
| `full` | `run_full_test.sh` | leo_obstacles | Tam hibrit test, 9 WP |
| `hybrid` | `run_hybrid_test.sh` | leo_obstacles | Hibrit GPS+SLAM |
| `empty` | `run_hybrid_test.sh --world empty` | leo_empty | Boş dünya, GPS |
| `obstacle` | `run_hybrid_test.sh` | leo_obstacles | Engelli dünya |
| `terrain` | `run_terrain_test.sh` | leo_open_terrain | Açık arazi |
| `urban` | `run_urban_test.sh` | leo_urban | Şehir ortamı |
| `industrial` | `run_industrial_test.sh` | leo_industrial | SLAM only |
| `slope` | `run_slope_test.sh` | leo_sloped_terrain | Eğim travers |
| `earendil` | `run_earendil_test.sh` | leo_earendil_env | Earendil ortamı |
| `all` | — | — | Sırasıyla: full, empty, industrial, terrain |

## Çıktı Klasör Yapısı

```
thesis_evidence/
└── 20250529_120000_full/
    ├── logs/
    │   └── test_output.log              # Tam test logu
    ├── screenshots/
    │   ├── 01_gazebo_world.png          # Gazebo penceresi
    │   ├── 02_rviz_nav2.png             # RViz Nav2
    │   ├── 03_rviz_costmap.png          # Costmap detayı
    │   ├── 04_slam_map.png              # SLAM haritası
    │   ├── 05_terminal_log.png          # Terminal logu
    │   ├── 06_tf_tree.png               # TF ağacı
    │   └── MANUAL_SCREENSHOT_CHECKLIST.md
    ├── rosbags/
    │   └── evidence/                    # rosbag2 dosyaları
    ├── csv/
    │   ├── test_summary.csv             # Test özeti
    │   ├── waypoint_results.csv         # WP sonuçları
    │   ├── topic_rates.csv              # Topic hızları
    │   ├── mode_transitions.csv         # Mod geçişleri
    │   └── errors.csv                   # Hatalar
    ├── plots/
    │   ├── waypoint_completion_bar.png  # WP bar chart
    │   ├── topic_rates_bar.png          # Topic hızları
    │   ├── test_summary_table.png       # Özet tablo
    │   ├── trajectory_plot.png          # Yörünge (rosbag varsa)
    │   └── cmd_vel_plot.png             # Hiz komutları (rosbag varsa)
    ├── system_info/
    │   ├── system_info.txt              # Host bilgisi
    │   ├── container_info.txt           # Container bilgisi
    │   ├── ros_nodes.txt                # Node listesi
    │   ├── ros_topics.txt               # Topic listesi
    │   ├── ros_services.txt             # Service listesi
    │   ├── ros_actions.txt              # Action listesi
    │   ├── ros_lifecycle_nodes.txt      # Lifecycle nodes
    │   ├── ros_params.txt               # Param listesi
    │   ├── frames.pdf                   # TF tree PDF
    │   ├── frames.gv                    # TF tree GraphViz
    │   └── topic_hz_*.txt               # Topic hız ölçümleri
    ├── configs/
    │   ├── sim_config.yaml              # Merkezi config
    │   ├── nav2_params_hybrid.yaml      # Nav2 parametreleri
    │   ├── slam_toolbox_params.yaml     # SLAM config
    │   ├── ukf_local.yaml               # UKF Local
    │   ├── ukf_global_hybrid.yaml       # UKF Global
    │   ├── navsat.yaml                  # NavSat config
    │   ├── hybrid_waypoints.yaml        # Waypoint tanımı
    │   ├── docker-compose.yml           # Docker config
    │   ├── Dockerfile                   # Docker image
    │   └── launch/                      # Launch dosyaları
    └── summaries/
        ├── thesis_evidence_summary.md   # Tez kanıt özeti
        ├── test_command.txt             # Çalıştırılan komut
        └── MISSING_METRICS.md           # Eksik metrikler
```

## Grafik Üretimi

Rosbag veya CSV verilerinden ek grafik üretmek için:

```bash
# Evidence klasöründen otomatik
python3 scripts/thesis/generate_plots.py --input thesis_evidence/20250529_120000_full/

# Rosbag'ten doğrudan
python3 scripts/thesis/generate_plots.py --bag thesis_evidence/.../rosbags/evidence/

# CSV'den doğrudan
python3 scripts/thesis/generate_plots.py --csv thesis_evidence/.../csv/ --output plots/
```

## Screenshot'lar

Otomatik screenshot araçları: `gnome-screenshot`, `spectacle`, `grim`, `scrot`, `import`

Eğer otomatik çalışmazsa `MANUAL_SCREENSHOT_CHECKLIST.md` dosyası oluşturulur.
Bu dosyada hangi ekran görüntülerinin manuel alınması gerektiği belirtilir.

## Rosbag İnceleme

```bash
# Rosbag bilgisi
ros2 bag info thesis_evidence/.../rosbags/evidence/

# Rosbag oynatma
ros2 bag play thesis_evidence/.../rosbags/evidence/

# RViz ile görselleştirme
rviz2  # ve /odom, /scan, /tf topic'lerini ekle
```

## Tezde Kullanım

| Çıktı | Tez Bölümü | Kullanım |
|-------|-----------|----------|
| `system_info.txt` | Yöntem | Sistem tanımı |
| `configs/` | Yöntem | Konfigürasyon detayları |
| `ros_nodes.txt` | Yöntem | Mimari tanım |
| `frames.pdf` | Yöntem | TF ağacı |
| `01_gazebo_world.png` | Deney | Simülasyon ortamı |
| `02_rviz_nav2.png` | Deney | Navigasyon görselleştirme |
| `04_slam_map.png` | Deney | SLAM harita çıktısı |
| `waypoint_completion_bar.png` | Sonuçlar | Performans analizi |
| `trajectory_plot.png` | Sonuçlar | Yörünge analizi |
| `test_summary_table.png` | Sonuçlar | Genel sonuç özeti |
| `thesis_evidence_summary.md` | Ek | Kanıt dokümantasyonu |

## Hata Ayıklama

### Container çalışmıyor
```bash
docker compose up -d --build
docker ps  # ros2-dev kontrol
```

### Gazebo başlamıyor
```bash
docker exec ros2-dev cat /tmp/gazebo.log
```

### Nav2 hazır değil
```bash
docker exec ros2-dev bash -c "source /opt/ros/humble/setup.bash && ros2 action list"
```

### Rosbag kaydı başarısız
```bash
docker exec ros2-dev bash -c "source /opt/ros/humble/setup.bash && which ros2"
```

## Dosya Listesi

| Dosya | Açıklama |
|-------|----------|
| `scripts/thesis/run_thesis_evidence.sh` | Ana kanıt toplama scripti |
| `scripts/thesis/capture_screenshots.sh` | Screenshot yakalama yardımcısı |
| `scripts/thesis/generate_plots.py` | Grafik üretim scripti |
| `scripts/thesis/README_THESIS_TESTS.md` | Bu dosya |
