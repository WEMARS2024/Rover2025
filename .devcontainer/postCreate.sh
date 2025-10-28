#!/usr/bin/env bash
set -e
# Initialize rosdep (first time only)
sudo rosdep init 2>/dev/null || true
rosdep update

# Install dependencies for packages in src/
rosdep install --from-paths src --ignore-src -r -y

# First build (optional)
colcon build --symlink-install
echo 'source /workspaces/ros2-ws/install/setup.bash' >> ~/.bashrc
