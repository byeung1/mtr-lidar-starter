#!/usr/bin/env bash
# Run, rather than source: graphics overrides stay local to this process tree.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/vrx_ws/install/setup.bash"
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${VRX_ADAPTER:-NVIDIA}"
mode=${1:-gui}
if (($#)); then shift; fi
case "$mode" in
  gui) exec ros2 launch vrx_gz competition.launch.py world:=sydney_regatta "$@" ;;
  headless) exec ros2 launch vrx_gz competition.launch.py world:=sydney_regatta headless:=true "$@" ;;
  bridge)
    topic=/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points
    exec ros2 run ros_gz_bridge parameter_bridge "${topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" --ros-args -r "${topic}:=/lidar/points" ;;
  rviz)
    script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
    exec rviz2 -d "$script_dir/../config/vrx-lidar.rviz" --ros-args -p use_sim_time:=true "$@" ;;
  *) echo 'Usage: bash scripts/vrx-wsl.sh {gui|headless|bridge|rviz} [launch arguments]' >&2; exit 2 ;;
esac
