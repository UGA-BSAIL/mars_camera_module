#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="noetic"
ROS_MASTER_URI="http://localhost:11311"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/devel/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
    echo "Error: ROS workspace setup file not found: ${SETUP_FILE}" >&2
    echo "Please build the workspace first so catkin generates it (including devel/setup.bash)." >&2
    echo "Try running: catkin_make" >&2
    exit 1
fi

cd "${WORKSPACE_ROOT}"
# shellcheck source=/dev/null
source "${SETUP_FILE}"
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
