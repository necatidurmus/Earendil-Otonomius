#!/usr/bin/env python3
"""
generate_plots.py — Tez Grafik Üretim Scripti
================================================
Rosbag, CSV veya log dosyalarından tezde kullanılacak grafikler üretir.

Kullanım:
    python3 scripts/thesis/generate_plots.py --input thesis_evidence/20250529_120000_full/
    python3 scripts/thesis/generate_plots.py --bag /path/to/rosbag --output plots/
    python3 scripts/thesis/generate_plots.py --csv /path/to/csv_dir --output plots/

Üretilen grafikler:
    - waypoint_completion_bar.png   : Waypoint tamamlama süresi bar chart
    - topic_rates_bar.png           : Topic yayın hızları
    - test_summary_table.png        : Test sonuç özeti tablosu
    - trajectory_plot.png           : Robot yörüngesi (rosbag varsa)
    - cmd_vel_plot.png              : Hiz komutları zaman serisi (rosbag varsa)
"""

import argparse
import os
import sys
import csv
import glob


def check_matplotlib():
    """Matplotlib mevcut mu kontrol et."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        return True
    except ImportError:
        print("UYARI: matplotlib bulunamadı. Grafik üretilemiyor.")
        print("  Kurulum: pip3 install matplotlib")
        return False


def check_rosbag():
    """rosbag2 Python API mevcut mu kontrol et."""
    try:
        from rosbags.rosbag2 import Reader
        return True
    except ImportError:
        try:
            import rosbag
            return True
        except ImportError:
            return False


def plot_waypoint_completion(csv_path, output_dir):
    """Waypoint tamamlama süresi bar chart."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not os.path.isfile(csv_path):
        print(f"  CSV bulunamadı: {csv_path}")
        return False

    names = []
    times = []
    statuses = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row.get('waypoint_id', row.get('name', ''))
            elapsed = row.get('elapsed_time', row.get('time', '0'))
            status = row.get('reached', row.get('status', ''))
            if name:
                names.append(name)
                try:
                    times.append(float(elapsed))
                except (ValueError, TypeError):
                    times.append(0.0)
                statuses.append(status)

    if not names:
        print("  Waypoint verisi boş")
        return False

    colors = ['green' if s in ('true', 'SUCCEEDED') else 'red' for s in statuses]

    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(range(len(names)), times, color=colors, edgecolor='black', linewidth=0.5)
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=9)
    ax.set_ylabel('Süre (saniye)')
    ax.set_xlabel('Waypoint')
    ax.set_title('Waypoint Tamamlama Süreleri')
    ax.grid(axis='y', alpha=0.3)

    # Değerleri bar üzerine yaz
    for bar, t in zip(bars, times):
        if t > 0:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                    f'{t:.1f}s', ha='center', va='bottom', fontsize=8)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'waypoint_completion_bar.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {output_path}")
    return True


def plot_topic_rates(csv_path, output_dir):
    """Topic yayın hızları bar chart."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not os.path.isfile(csv_path):
        print(f"  CSV bulunamadı: {csv_path}")
        return False

    topics = []
    rates = []

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            topic = row.get('topic', '')
            rate = row.get('average_hz', '0')
            if topic:
                topics.append(topic)
                try:
                    rates.append(float(rate))
                except (ValueError, TypeError):
                    rates.append(0.0)

    if not topics:
        print("  Topic rate verisi boş")
        return False

    fig, ax = plt.subplots(figsize=(12, 6))
    colors = ['#2196F3' if r > 0 else '#FF9800' for r in rates]
    bars = ax.barh(range(len(topics)), rates, color=colors, edgecolor='black', linewidth=0.5)
    ax.set_yticks(range(len(topics)))
    ax.set_yticklabels(topics, fontsize=9)
    ax.set_xlabel('Ortalama Hz')
    ax.set_title('ROS 2 Topic Yayın Hızları')
    ax.grid(axis='x', alpha=0.3)

    # Değerleri bar üzerine yaz
    for bar, r in zip(bars, rates):
        if r > 0:
            ax.text(bar.get_width() + 0.2, bar.get_y() + bar.get_height() / 2,
                    f'{r:.1f} Hz', ha='left', va='center', fontsize=8)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'topic_rates_bar.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {output_path}")
    return True


def plot_test_summary_table(csv_path, output_dir):
    """Test sonuç özeti tablosu."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not os.path.isfile(csv_path):
        print(f"  CSV bulunamadı: {csv_path}")
        return False

    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        print("  Test summary verisi boş")
        return False

    fig, ax = plt.subplots(figsize=(10, max(2, len(rows) * 0.5 + 1)))
    ax.axis('off')

    col_labels = ['Test', 'Sonuç', 'Notlar']
    table_data = []
    for row in rows:
        table_data.append([
            row.get('test_name', ''),
            row.get('result', ''),
            row.get('notes', '')
        ])

    table = ax.table(cellText=table_data, colLabels=col_labels,
                     loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.5)

    # Başlık renkleri
    for j, label in enumerate(col_labels):
        table[0, j].set_facecolor('#2196F3')
        table[0, j].set_text_props(color='white', fontweight='bold')

    # Sonuç renkleri
    for i, row in enumerate(rows):
        result = row.get('result', '')
        if result == 'PASS':
            table[i + 1, 1].set_facecolor('#C8E6C9')
        elif result == 'FAIL':
            table[i + 1, 1].set_facecolor('#FFCDD2')

    ax.set_title('Test Sonuç Özeti', fontsize=14, fontweight='bold', pad=20)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'test_summary_table.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {output_path}")
    return True


def plot_trajectory_from_rosbag(bag_path, output_dir):
    """Rosbag'ten robot yörüngesi çiz."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # rosbags API dene
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr
    except ImportError:
        try:
            import rosbag
        except ImportError:
            print("  rosbag kütüphanesi bulunamadı, trajectory plot atlanıyor")
            return False

    xs = []
    ys = []

    try:
        # rosbags API
        from rosbags.rosbag2 import Reader
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/odometry/filtered':
                    from rosbags.serde import deserialize_cdr
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    xs.append(msg.pose.pose.position.x)
                    ys.append(msg.pose.pose.position.y)
    except Exception:
        try:
            # eski rosbag API
            import rosbag
            with rosbag.Bag(bag_path, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=['/odometry/filtered']):
                    xs.append(msg.pose.pose.position.x)
                    ys.append(msg.pose.pose.position.y)
        except Exception as e:
            print(f"  Rosbag okuma hatası: {e}")
            return False

    if not xs:
        print("  Yörünge verisi bulunamadı")
        return False

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(xs, ys, 'b-', linewidth=1.5, label='Robot yörüngesi')
    ax.plot(xs[0], ys[0], 'go', markersize=10, label='Başlangıç')
    ax.plot(xs[-1], ys[-1], 'r*', markersize=15, label='Bitiş')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot Yörüngesi (/odometry/filtered)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'trajectory_plot.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {output_path}")
    return True


def plot_cmd_vel_from_rosbag(bag_path, output_dir):
    """Rosbag'ten cmd_vel zaman serisi çiz."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    try:
        from rosbags.rosbag2 import Reader
        from rosbags.serde import deserialize_cdr
    except ImportError:
        try:
            import rosbag
        except ImportError:
            print("  rosbag kütüphanesi bulunamadı, cmd_vel plot atlanıyor")
            return False

    timestamps = []
    linear_vels = []
    angular_vels = []

    try:
        from rosbags.rosbag2 import Reader
        with Reader(bag_path) as reader:
            start_time = None
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == '/cmd_vel':
                    if start_time is None:
                        start_time = timestamp
                    t = (timestamp - start_time) / 1e9  # nanoseconds to seconds
                    from rosbags.serde import deserialize_cdr
                    msg = deserialize_cdr(rawdata, connection.msgtype)
                    timestamps.append(t)
                    linear_vels.append(msg.linear.x)
                    angular_vels.append(msg.angular.z)
    except Exception:
        try:
            import rosbag
            with rosbag.Bag(bag_path, 'r') as bag:
                start_time = None
                for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
                    if start_time is None:
                        start_time = t.to_sec()
                    timestamps.append(t.to_sec() - start_time)
                    linear_vels.append(msg.linear.x)
                    angular_vels.append(msg.angular.z)
        except Exception as e:
            print(f"  Rosbag okuma hatası: {e}")
            return False

    if not timestamps:
        print("  cmd_vel verisi bulunamadı")
        return False

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1.plot(timestamps, linear_vels, 'b-', linewidth=1)
    ax1.set_ylabel('Linear Hız (m/s)')
    ax1.set_title('cmd_vel Zaman Serisi')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    ax2.plot(timestamps, angular_vels, 'r-', linewidth=1)
    ax2.set_ylabel('Angular Hız (rad/s)')
    ax2.set_xlabel('Zaman (s)')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.5)

    plt.tight_layout()
    output_path = os.path.join(output_dir, 'cmd_vel_plot.png')
    fig.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  ✓ {output_path}")
    return True


def generate_plots_from_evidence_dir(evidence_dir):
    """Test evidence klasöründen tüm grafikleri üret."""
    output_dir = os.path.join(evidence_dir, 'plots')
    os.makedirs(output_dir, exist_ok=True)

    print(f"\nGrafik üretimi: {evidence_dir}")
    print(f"Çıktı: {output_dir}\n")

    generated = 0

    # Waypoint completion
    wp_csv = os.path.join(evidence_dir, 'csv', 'waypoint_results.csv')
    if os.path.isfile(wp_csv):
        if plot_waypoint_completion(wp_csv, output_dir):
            generated += 1

    # Topic rates
    rates_csv = os.path.join(evidence_dir, 'csv', 'topic_rates.csv')
    if os.path.isfile(rates_csv):
        if plot_topic_rates(rates_csv, output_dir):
            generated += 1

    # Test summary table
    summary_csv = os.path.join(evidence_dir, 'csv', 'test_summary.csv')
    if os.path.isfile(summary_csv):
        if plot_test_summary_table(summary_csv, output_dir):
            generated += 1

    # Rosbag grafikleri
    bag_dirs = glob.glob(os.path.join(evidence_dir, 'rosbags', '*'))
    for bag_path in bag_dirs:
        if os.path.isdir(bag_path):
            if plot_trajectory_from_rosbag(bag_path, output_dir):
                generated += 1
            if plot_cmd_vel_from_rosbag(bag_path, output_dir):
                generated += 1
            break  # Sadece ilk rosbag

    print(f"\nToplam {generated} grafik üretildi.")
    return generated


def main():
    parser = argparse.ArgumentParser(
        description='Tez grafik üretim scripti')
    parser.add_argument('--input', '-i', type=str,
                        help='Test evidence klasörü')
    parser.add_argument('--bag', '-b', type=str,
                        help='Rosbag dosya yolu')
    parser.add_argument('--csv', '-c', type=str,
                        help='CSV klasörü')
    parser.add_argument('--output', '-o', type=str, default='plots',
                        help='Çıktı klasörü')

    args = parser.parse_args()

    if not check_matplotlib():
        print("\nMatplotlib olmadan grafik üretilemiyor.")
        print("Kurulum: pip3 install matplotlib")
        sys.exit(1)

    if args.input:
        generate_plots_from_evidence_dir(args.input)
    elif args.bag or args.csv:
        output_dir = args.output
        os.makedirs(output_dir, exist_ok=True)
        generated = 0

        if args.bag:
            if plot_trajectory_from_rosbag(args.bag, output_dir):
                generated += 1
            if plot_cmd_vel_from_rosbag(args.bag, output_dir):
                generated += 1

        if args.csv:
            wp_csv = os.path.join(args.csv, 'waypoint_results.csv')
            if plot_waypoint_completion(wp_csv, output_dir):
                generated += 1

            rates_csv = os.path.join(args.csv, 'topic_rates.csv')
            if plot_topic_rates(rates_csv, output_dir):
                generated += 1

        print(f"\nToplam {generated} grafik üretildi.")
    else:
        print("Kullanım:")
        print("  python3 generate_plots.py --input thesis_evidence/20250529_120000_full/")
        print("  python3 generate_plots.py --bag /path/to/rosbag --output plots/")
        print("  python3 generate_plots.py --csv /path/to/csv_dir --output plots/")
        sys.exit(1)


if __name__ == '__main__':
    main()
