#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source install/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds.xml"
exec ros2 launch mtr_lidar_starter development.launch.py bag:="$PWD/bags/obstacle_case" "$@"
