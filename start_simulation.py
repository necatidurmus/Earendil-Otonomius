#!/usr/bin/env python3
"""
Leo Rover Simulation Launcher
Tek komutla Gazebo + Nav2 + EKF simülasyonunu başlatır.

Kullanım:
  python3 start_simulation.py                    # Varsayılan: leo_empty + engeller + Nav2 EKF
  python3 start_simulation.py --world marsyard   # Mars yard dünyası
  python3 start_simulation.py --no-nav           # Sadece Gazebo (Nav2 yok)
  python3 start_simulation.py --gps-test         # GPS waypoint testi de çalıştır
"""

import subprocess
import sys
import time
import argparse


CONTAINER = "ros2-dev"
ROS_SETUP = "source /opt/ros/humble/setup.bash && source /home/ros/ws/install/setup.bash"

WORLDS = {
    "empty": "leo_empty",
    "marsyard": "marsyard2020",
}

OBSTACLES = [
    # (x, y, z_center, w, d, h, r, g, b, name, shape)
    (4, 1, 0.75, 0.3, 3.0, 1.5, 0.9, 0.1, 0.1, "wall_red", "box"),
    (-3, -2, 0.75, 0.3, 2.5, 1.5, 0.1, 0.1, 0.9, "wall_blue", "box"),
    (6, -2, 0.5, 1.0, 1.0, 1.0, 0.1, 0.8, 0.1, "box_green", "box"),
    (-5, 4, 0.75, 0.4, 0.4, 1.5, 1.0, 1.0, 0.0, "pylon_yellow", "cylinder"),
]


def run(cmd, check=True, capture=False):
    """Run a shell command."""
    result = subprocess.run(cmd, shell=True, capture_output=capture, text=True)
    if check and result.returncode != 0:
        print(f"  ⚠️  Komut başarısız: {cmd}")
    return result


def docker_exec(cmd, detach=False, timeout=None):
    """Execute a command inside the Docker container."""
    d = "-d " if detach else ""
    t = f"timeout {timeout} " if timeout else ""
    return run(f'docker exec {d}{CONTAINER} bash -c "{ROS_SETUP} && {t}{cmd}"', check=False, capture=True)


def wait_for_topic(topic, max_wait=20):
    """Wait for a ROS 2 topic to become available."""
    print(f"  ⏳ {topic} bekleniyor...", end="", flush=True)
    for i in range(max_wait):
        result = docker_exec(f"ros2 topic list | grep '{topic}' 2>/dev/null")
        if topic in result.stdout:
            print(" ✅")
            return True
        time.sleep(1)
        print(".", end="", flush=True)
    print(" ❌ timeout")
    return False


def check_container():
    """Check if the Docker container is running."""
    result = run(f"docker ps --filter name={CONTAINER} --format '{{{{.Status}}}}'", capture=True)
    if not result.stdout.strip():
        print("🐳 Container başlatılıyor...")
        run("docker compose up -d", check=True)
        time.sleep(3)
    else:
        print(f"🐳 Container çalışıyor: {result.stdout.strip()}")


def build_workspace():
    """Build the ROS 2 workspace."""
    print("🔨 Workspace build ediliyor...")
    docker_exec("cd /home/ros/ws && colcon build --packages-select leo_gz_worlds leo_gz_bringup --symlink-install 2>&1 | tail -1")


def start_gazebo(world_name):
    """Start Gazebo simulation."""
    world_file = WORLDS.get(world_name, world_name)
    print(f"🌍 Gazebo başlatılıyor ({world_file})...")
    run(f"xhost +local:docker 2>/dev/null", check=False)

    docker_exec(
        f"ros2 launch leo_gz_bringup leo_gz.launch.py "
        f"sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/{world_file}.sdf "
        f"> /tmp/gazebo.log 2>&1",
        detach=True,
    )

    if not wait_for_topic("/odom", 20):
        print("❌ Gazebo başlatılamadı!")
        sys.exit(1)
    print("  🌍 Gazebo hazır!")


def spawn_obstacles(world_name):
    """Spawn obstacles in the simulation world."""
    gz_world = "leo_empty" if world_name == "empty" else "leo_marsyard"
    print(f"🧱 Engeller ekleniyor ({len(OBSTACLES)} adet)...")

    for x, y, z, w, d, h, r, g, b, name, shape in OBSTACLES:
        if shape == "box":
            geo = f"<box><size>{w} {d} {h}</size></box>"
        else:
            geo = f"<cylinder><radius>{w}</radius><length>{h}</length></cylinder>"

        sdf = (
            f'<sdf version=\\"1.6\\"><model name=\\"{name}\\"><static>true</static>'
            f'<pose>{x} {y} {z} 0 0 0</pose><link name=\\"l\\">'
            f'<collision name=\\"c\\"><geometry>{geo}</geometry></collision>'
            f'<visual name=\\"v\\"><geometry>{geo}</geometry>'
            f'<material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual>'
            f'</link></model></sdf>'
        )

        result = docker_exec(
            f"ign service -s /world/{gz_world}/create "
            f"--reqtype ignition.msgs.EntityFactory "
            f"--reptype ignition.msgs.Boolean "
            f"--timeout 3000 "
            f"--req 'sdf: \\\"{sdf}\\\"'"
        )
        status = "✅" if "true" in (result.stdout or "") else "❌"
        print(f"  {status} {name} ({x},{y})")


def start_nav2():
    """Start Nav2 with EKF sensor fusion."""
    print("🧭 Nav2 + EKF başlatılıyor...")
    docker_exec(
        "ros2 launch leo_gz_bringup navigation_ekf.launch.py > /tmp/nav2_ekf.log 2>&1",
        detach=True,
    )

    # Wait for Nav2 to become active
    print("  ⏳ Nav2 aktif olması bekleniyor...", end="", flush=True)
    for i in range(40):
        result = docker_exec("strings /tmp/nav2_ekf.log 2>/dev/null | grep 'Managed nodes are active'")
        if "Managed nodes are active" in (result.stdout or ""):
            print(" ✅")
            return True
        time.sleep(1)
        print(".", end="", flush=True)
    print(" ❌ timeout")
    return False


def run_gps_test():
    """Run GPS waypoint navigation test."""
    print("📍 GPS waypoint navigasyon testi başlatılıyor...")
    result = docker_exec(
        "python3 /home/ros/ws/src/gps_waypoint_nav.py",
        timeout=200,
    )
    print(result.stdout or "")
    if result.stderr:
        print(result.stderr)


def main():
    parser = argparse.ArgumentParser(description="Leo Rover Simülasyon Başlatıcı")
    parser.add_argument("--world", choices=["empty", "marsyard"], default="empty",
                        help="Simülasyon dünyası (varsayılan: empty)")
    parser.add_argument("--no-nav", action="store_true",
                        help="Nav2 başlatma (sadece Gazebo)")
    parser.add_argument("--no-obstacles", action="store_true",
                        help="Engel ekleme")
    parser.add_argument("--gps-test", action="store_true",
                        help="GPS waypoint navigasyon testini çalıştır")
    args = parser.parse_args()

    print("=" * 50)
    print("  🤖 LEO ROVER SİMÜLASYON BAŞLATICI")
    print("=" * 50)

    check_container()
    build_workspace()
    start_gazebo(args.world)

    if not args.no_obstacles:
        spawn_obstacles(args.world)

    if not args.no_nav:
        start_nav2()

    if args.gps_test:
        run_gps_test()

    print("\n" + "=" * 50)
    print("  ✅ Simülasyon hazır!")
    print("=" * 50)
    print("\nKullanışlı komutlar:")
    print("  GPS test:   docker exec ros2-dev bash -c '... python3 /home/ros/ws/src/gps_waypoint_nav.py'")
    print("  Teleop web: cd src/teleop_web && python3 teleop_server.py")
    print("  Durdur:     docker compose stop")


if __name__ == "__main__":
    main()
