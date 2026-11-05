#!/usr/bin/env bash
# Source from a ROS terminal. All dependency overrides stay in that terminal.
VRX_NAV_WS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$HOME/vrx_ws/install/setup.bash"
VRX_NAV_DEPS="$VRX_NAV_WS/.deps/root"
if test -d "$VRX_NAV_DEPS/opt/ros/jazzy/share/nav2_controller"; then
  export AMENT_PREFIX_PATH="$VRX_NAV_DEPS/opt/ros/jazzy:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$VRX_NAV_DEPS/opt/ros/jazzy:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$VRX_NAV_DEPS/opt/ros/jazzy/lib:$VRX_NAV_DEPS/opt/ros/jazzy/lib/x86_64-linux-gnu:$VRX_NAV_DEPS/usr/lib/x86_64-linux-gnu:$VRX_NAV_DEPS/usr/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$VRX_NAV_DEPS/opt/ros/jazzy/lib/python3.12/site-packages:$VRX_NAV_DEPS/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export PATH="$VRX_NAV_DEPS/opt/ros/jazzy/bin:$PATH"
fi
if test -f "$VRX_NAV_WS/install/setup.bash"; then
  source "$VRX_NAV_WS/install/setup.bash"
fi
