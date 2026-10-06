#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
colcon build --symlink-install
echo 'Build complete. Run: bash run.sh'
