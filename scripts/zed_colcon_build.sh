#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="humble"
ROS_MASTER_URI="http://localhost:11311"

SETUP_FILE="/opt/ros/${ROS_DISTRO}/setup.bash"

# https://github.com/ament/ament_package/issues/148
set +u
# shellcheck source=/dev/null
source "${SETUP_FILE}"
set -u

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
