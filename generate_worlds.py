#!/usr/bin/env python3
"""
SDF World Generator — sim_config.yaml → SDF Fizik/Yerçekimi/Zemin Güncelleyici
===============================================================================
sim_config.yaml'daki physics, gravity ve ground değerlerini
leo_obstacles.sdf ve leo_empty.sdf dosyalarına uygular.

Kullanım:
    python3 generate_worlds.py                  # Varsayılan konumlar
    python3 generate_worlds.py --config sim_config.yaml --dry-run

Bu script SDF dosyalarını doğrudan düzenler (XML parse).
Yalnızca physics, gravity ve ground_plane friction değerlerini günceller;
engeller, tünel, kapı ve diğer içerik korunur.
"""

import os
import re
import sys
import yaml
import argparse
import xml.etree.ElementTree as ET


def load_config(config_path):
    with open(config_path) as f:
        return yaml.safe_load(f)


def patch_physics(root, cfg):
    """<physics> bloğundaki max_step_size ve real_time_factor'ü güncelle."""
    physics_el = root.find('.//physics')
    if physics_el is None:
        return

    step = physics_el.find('max_step_size')
    if step is not None:
        step.text = str(cfg.get('physics', {}).get('max_step_size', 0.001))

    rtf = physics_el.find('real_time_factor')
    if rtf is not None:
        rtf.text = str(cfg.get('physics', {}).get('real_time_factor', 1.0))


def patch_gravity(root, cfg):
    """<gravity> elementini ekle/güncelle."""
    gravity_cfg = cfg.get('gravity', {})
    gx = str(gravity_cfg.get('x', 0.0))
    gy = str(gravity_cfg.get('y', 0.0))
    gz = str(gravity_cfg.get('z', -9.81))

    world = root.find('.//world')
    if world is None:
        return

    gravity_el = world.find('gravity')
    if gravity_el is not None:
        gravity_el.text = f'{gx} {gy} {gz}'
    else:
        # <gravity> yoksa physics'ten sonra ekle
        gravity_el = ET.SubElement(world, 'gravity')
        gravity_el.text = f'{gx} {gy} {gz}'


def patch_ground_friction(root, cfg):
    """ground_plane modelinin zemin sürtünme değerlerini güncelle."""
    ground_mu = cfg.get('ground', {}).get('mu', 0.8)
    ground_mu2 = cfg.get('ground', {}).get('mu2', 0.8)

    # ground_plane modeli bul
    for model in root.iter('model'):
        if model.get('name') == 'ground_plane':
            # ODE friction güncelle
            for ode in model.iter('ode'):
                mu = ode.find('mu')
                if mu is not None:
                    mu.text = str(ground_mu)
                mu2 = ode.find('mu2')
                if mu2 is not None:
                    mu2.text = str(ground_mu2)
            break


def patch_sdf(sdf_path, cfg, dry_run=False):
    """Tek bir SDF dosyasını config ile güncelle."""
    try:
        tree = ET.parse(sdf_path)
    except ET.ParseError as e:
        print(f'  ❌ XML parse hatası: {sdf_path}: {e}')
        return
    root = tree.getroot()

    patch_physics(root, cfg)
    patch_gravity(root, cfg)
    patch_ground_friction(root, cfg)

    if dry_run:
        print(f'  [DRY-RUN] {sdf_path} güncellenmedi')
        return

    # XML declaration ile yaz
    tree.write(sdf_path, encoding='unicode', xml_declaration=True)
    # ElementTree XML declaration'ı version="1.0" olarak yazar, SDF formatını koru
    print(f'  ✅ {sdf_path} güncellendi')


def main():
    parser = argparse.ArgumentParser(
        description='sim_config.yaml → SDF world dosyası güncelleyici')
    parser.add_argument('--config', '-c', default=None,
                        help='sim_config.yaml yolu')
    parser.add_argument('--worlds-dir', '-w', default=None,
                        help='SDF dosyalarının bulunduğu dizin')
    parser.add_argument('--dry-run', action='store_true',
                        help='Değişiklik yapmadan sadece göster')
    args = parser.parse_args()

    # Config dosyasını bul
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = args.config or os.path.join(script_dir, 'sim_config.yaml')

    if not os.path.isfile(config_path):
        print(f'Hata: Config dosyası bulunamadı: {config_path}')
        sys.exit(1)

    cfg = load_config(config_path)
    print(f'Config: {config_path}')
    print(f'  physics.max_step_size  = {cfg.get("physics", {}).get("max_step_size", "?")}')
    print(f'  physics.real_time_factor = {cfg.get("physics", {}).get("real_time_factor", "?")}')
    print(f'  gravity.z              = {cfg.get("gravity", {}).get("z", "?")}')
    print(f'  ground.mu              = {cfg.get("ground", {}).get("mu", "?")}')
    print()

    # Worlds dizinini bul
    worlds_dir = args.worlds_dir or os.path.join(
        script_dir, 'src', 'leo_simulator', 'leo_gz_worlds', 'worlds')

    sdf_files = ['leo_obstacles.sdf', 'leo_empty.sdf']
    updated = 0

    for sdf_name in sdf_files:
        sdf_path = os.path.join(worlds_dir, sdf_name)
        if os.path.isfile(sdf_path):
            patch_sdf(sdf_path, cfg, dry_run=args.dry_run)
            updated += 1
        else:
            print(f'  ⚠️ {sdf_path} bulunamadı, atlanıyor')

    print(f'\n{updated} SDF dosyası {"kontrol edildi (dry-run)" if args.dry_run else "güncellendi"}')


if __name__ == '__main__':
    main()
