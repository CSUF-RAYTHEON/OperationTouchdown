#!/bin/bash
set -e

ROS_DISTRO=${ROS_DISTRO:-jazzy}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Installing system tools ==="
sudo apt update
sudo apt install -y python3-vcstool python3-rosdep

echo "=== Importing source dependencies ==="
vcs import "$SCRIPT_DIR" < "$SCRIPT_DIR/deps.repos"

echo "=== Installing ROS dependencies via rosdep ==="
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$SCRIPT_DIR/ugv_ws"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building workspace ==="
colcon build --symlink-install

echo ""
echo "=== Done! Source your workspace with: ==="
echo "    source $SCRIPT_DIR/ugv_ws/install/setup.bash"
