#!/usr/bin/env python3
"""
find_safe_wps.py — Leo Rover Engel Haritasında Güvenli Waypoint Bulucu
======================================================================
leo_obstacles.sdf dünyasındaki engelleri (kutu, duvar, tünel, kapı) dikkate
alarak robotun güvenle ulaşabileceği GPS waypoint adaylarını grid search ile
bulur.

Kullanım:
    python3 find_safe_wps.py                    # Varsayılan 2.5m güvenlik mesafesi
    python3 find_safe_wps.py --margin 3.0       # 3m güvenlik mesafesi
    python3 find_safe_wps.py --grid-step 0.5    # 0.5m adım (daha yoğun arama)
    python3 find_safe_wps.py --count 6          # En iyi 6 noktayı göster

Çıktı:
    Her aday waypoint için GPS koordinatı (lat, lon) ve en yakın engele mesafe.
"""

import argparse
import math
import os
import yaml


# ── GPS datum: sim_config.yaml'dan oku, yoksa varsayılan kullan ───────────
def _load_datum():
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'sim_config.yaml')
    try:
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        datum = cfg.get('gps_datum', {})
        return datum.get('lat', 39.925018), datum.get('lon', 32.836956)
    except (FileNotFoundError, yaml.YAMLError):
        return 39.925018, 32.836956


DATUM_LAT, DATUM_LON = _load_datum()

# ── Metre → derece dönüşüm sabitleri ────────────────────────────────────
METERS_PER_DEG_LAT = 111320.0
METERS_PER_DEG_LON = 111320.0 * math.cos(math.radians(DATUM_LAT))

# ── Engeller (x, y, half_size_x, half_size_y) ───────────────────────────
# leo_obstacles.sdf'den çıkarılmıştır.
OBSTACLES = [
    # 14 kutu engel — her biri 2×2m (half = 1.0)
    (6, 0, 1.0, 1.0),
    (-7, 4, 1.0, 1.0),
    (10, -3, 1.0, 1.0),
    (-10, -5, 1.0, 1.0),
    (3, -8, 1.0, 1.0),
    (9, 10, 1.0, 1.0),
    (-5, -12, 1.0, 1.0),
    (13, 7, 1.0, 1.0),
    (-12, 10, 1.0, 1.0),
    (0, 10, 1.0, 1.0),
    (-3, 6, 1.0, 1.0),
    (6, -12, 1.0, 1.0),
    (-14, 0, 1.0, 1.0),
    (14, -8, 1.0, 1.0),
    # wall_north: merkez(0,17), 20×0.4 → half(10, 0.2)
    (0, 17, 10.0, 0.2),
    # wall_east: merkez(17,0), döndürülmüş → half(0.2, 8.0)
    (17, 0, 0.2, 8.0),
    # Tünel duvarları
    (25, 1.5, 5.0, 0.15),
    (25, -1.5, 5.0, 0.15),
    # Kapı çerçevesi
    (35, 3.0, 0.15, 1.5),
    (35, -3.0, 0.15, 1.5),
]


def min_distance_to_obstacles(x, y):
    """(x, y) noktasının en yakın engel yüzeyine mesafesini hesapla."""
    min_dist = float('inf')
    for ox, oy, hx, hy in OBSTACLES:
        # Engel dikdörtgeninin kenarına olan mesafe
        dx = max(0.0, abs(x - ox) - hx)
        dy = max(0.0, abs(y - oy) - hy)
        dist = math.sqrt(dx * dx + dy * dy)
        min_dist = min(min_dist, dist)
    return min_dist


def local_to_gps(x, y):
    """Lokal ENU (x=doğu, y=kuzey) koordinatını GPS'e çevir."""
    lat = DATUM_LAT + y / METERS_PER_DEG_LAT
    lon = DATUM_LON + x / METERS_PER_DEG_LON
    return lat, lon


def find_safe_waypoints(margin=2.5, grid_step=1.0, x_range=(-16, 16),
                        y_range=(-14, 14), count=10):
    """Grid search ile güvenli waypoint adaylarını bul."""
    candidates = []

    x = x_range[0]
    while x <= x_range[1]:
        y = y_range[0]
        while y <= y_range[1]:
            dist = min_distance_to_obstacles(x, y)
            if dist >= margin:
                lat, lon = local_to_gps(x, y)
                candidates.append({
                    'x': x, 'y': y,
                    'lat': lat, 'lon': lon,
                    'min_dist': round(dist, 2),
                })
            y += grid_step
        x += grid_step

    # En uzak mesafeye göre sırala, çeşitlilik için farklı kadranlardan seç
    candidates.sort(key=lambda c: c['min_dist'], reverse=True)
    return candidates[:count]


def main():
    parser = argparse.ArgumentParser(
        description='Leo Rover engelli dünyada güvenli waypoint bulucu')
    parser.add_argument('--margin', type=float, default=2.5,
                        help='Engellerden minimum güvenlik mesafesi (m)')
    parser.add_argument('--grid-step', type=float, default=1.0,
                        help='Grid arama adım boyutu (m)')
    parser.add_argument('--count', type=int, default=10,
                        help='Gösterilecek maksimum aday sayısı')
    parser.add_argument('--x-min', type=float, default=-16.0)
    parser.add_argument('--x-max', type=float, default=16.0)
    parser.add_argument('--y-min', type=float, default=-14.0)
    parser.add_argument('--y-max', type=float, default=14.0)
    args = parser.parse_args()

    print('=' * 65)
    print('  LEO ROVER — GÜVENLİ WAYPOINT BULUCU')
    print('=' * 65)
    print(f'  Datum    : {DATUM_LAT:.6f}°N, {DATUM_LON:.6f}°E')
    print(f'  Margin   : {args.margin}m')
    print(f'  Grid     : {args.grid_step}m adım')
    print(f'  Alan     : x=[{args.x_min},{args.x_max}], y=[{args.y_min},{args.y_max}]')
    print(f'  Engeller : {len(OBSTACLES)} adet')
    print('=' * 65)

    results = find_safe_waypoints(
        margin=args.margin,
        grid_step=args.grid_step,
        x_range=(args.x_min, args.x_max),
        y_range=(args.y_min, args.y_max),
        count=args.count,
    )

    if not results:
        print('\n  Uygun waypoint bulunamadı! Margin değerini düşürün.')
        return

    print(f'\n  En güvenli {len(results)} waypoint:\n')
    print(f'  {"#":>3}  {"X":>6}  {"Y":>6}  {"Mesafe":>7}  {"Lat":>12}  {"Lon":>12}')
    print(f'  {"─"*3}  {"─"*6}  {"─"*6}  {"─"*7}  {"─"*12}  {"─"*12}')

    for i, wp in enumerate(results, 1):
        print(f'  {i:3d}  {wp["x"]:+6.1f}  {wp["y"]:+6.1f}  '
              f'{wp["min_dist"]:6.2f}m  {wp["lat"]:.6f}  {wp["lon"]:.6f}')

    # YAML formatında çıktı
    print('\n  ── waypoints.yaml formatı ──')
    print('  waypoints:')
    for i, wp in enumerate(results[:4], 1):
        print(f'    - lat: {wp["lat"]:.6f}')
        print(f'      lon: {wp["lon"]:.6f}')
        print(f'      name: "WP{i}: ({wp["x"]:+.0f},{wp["y"]:+.0f}) uzk={wp["min_dist"]:.1f}m"')

    print()


if __name__ == '__main__':
    main()