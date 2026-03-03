#!/bin/bash
set -e

# Ensure XDG_RUNTIME_DIR exists
export XDG_RUNTIME_DIR=/tmp/runtime-ros
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Setup ROS 2 environment
source "/opt/ros/humble/setup.bash"

# Execute the command passed into this entrypoint
exec "$@"
