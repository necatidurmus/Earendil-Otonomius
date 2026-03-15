#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /home/ros/ws/install/setup.bash ]; then
    source /home/ros/ws/install/setup.bash
fi

exec "$@"
