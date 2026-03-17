#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${1:-$HOME/Earendil-Otonomius}"
CONTAINER="ros2-dev"
ROS_DOMAIN_ID_VALUE="30"
RMW_VALUE="rmw_fastrtps_cpp"

cd "$REPO_DIR"

xhost +local:docker >/dev/null 2>&1 || true

echo "[1/6] Docker compose up"
docker compose up -d --build

echo "[2/6] Container restart (temiz başlangıç)"
docker restart "$CONTAINER" >/dev/null
sleep 3

echo "[3/6] Build"
docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION="$RMW_VALUE" "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash &&
  cd /home/ros/ws &&
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
'

echo "[4/6] Gazebo başlatılıyor"
docker exec -d -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION="$RMW_VALUE" "$CONTAINER" bash -lc '
  export ROS_DOMAIN_ID='$ROS_DOMAIN_ID_VALUE' ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION='$RMW_VALUE' &&
  source /opt/ros/humble/setup.bash &&
  source /home/ros/ws/install/setup.bash &&
  ros2 launch leo_gz_bringup leo_gz.launch.py \
    sim_world:=/home/ros/ws/install/leo_gz_worlds/share/leo_gz_worlds/worlds/leo_obstacles.sdf \
    world_name:=leo_obstacles \
  2>&1 | tee /tmp/gazebo_matlab_test.log
'
sleep 25

echo "[5/6] Navigation + MATLAB bridge başlatılıyor"
docker exec -d -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION="$RMW_VALUE" "$CONTAINER" bash -lc '
  export ROS_DOMAIN_ID='$ROS_DOMAIN_ID_VALUE' ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION='$RMW_VALUE' &&
  source /opt/ros/humble/setup.bash &&
  source /home/ros/ws/install/setup.bash &&
  ros2 launch leo_gz_bringup navigation_matlab_test.launch.py \
    use_sim_time:=true launch_rviz:=true use_spoofer:=true \
  2>&1 | tee /tmp/navigation_matlab_test.log
'

echo "[6/6] Action server bekleniyor"
for i in $(seq 1 12); do
  if docker exec -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION="$RMW_VALUE" "$CONTAINER" bash -lc 'source /opt/ros/humble/setup.bash && ros2 action list | grep -q navigate_to_pose'; then
    echo "Nav2 hazır"
    break
  fi
  echo "Bekleniyor... ($i/12)"
  sleep 5
done

echo
echo "Hazır. Kontrol komutları:"
echo "docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=$RMW_VALUE $CONTAINER bash -lc 'source /opt/ros/humble/setup.bash && source /home/ros/ws/install/setup.bash && ros2 topic list'"
echo "docker exec -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VALUE -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=$RMW_VALUE $CONTAINER bash -lc 'source /opt/ros/humble/setup.bash && source /home/ros/ws/install/setup.bash && ros2 topic echo /external_goal_status'"
