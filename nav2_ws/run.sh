#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec python3 "$VRX_NAV_WS/session.py" ros2 launch vrx_nav2 navigation.launch.py "$@"
