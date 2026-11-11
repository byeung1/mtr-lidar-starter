#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "$0")/.."
source /opt/ros/jazzy/setup.bash
source "$HOME/vrx_ws/install/setup.bash"
cmake -S nav2_ws/gazebo_click_goal -B nav2_ws/gazebo_click_goal/build
cmake --build nav2_ws/gazebo_click_goal/build -j2
export GZ_GUI_PLUGIN_PATH="$PWD/nav2_ws/gazebo_click_goal/build:${GZ_GUI_PLUGIN_PATH:-}"
exec env -u LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER=d3d12 \
  MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel gz sim -g --render-engine ogre \
  --gui-config "$PWD/nav2_ws/gazebo_click_goal/gui.config"
