#!/usr/bin/env bash
set -e

# Base ROS
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Your workspace overlay (built in image)
if [ -f "/ws/install/setup.bash" ]; then
  source "/ws/install/setup.bash"
fi

exec "$@"