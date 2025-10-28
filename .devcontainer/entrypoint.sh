#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash || true
# Auto-source build overlay if it exists
if [ -f /workspaces/ros2-ws/install/setup.bash ]; then
  source /workspaces/ros2-ws/install/setup.bash
fi
exec "$@"
