#!/bin/bash

# A wrapper around CMake so that it works with colcon but behaves
# nicely with CLion.

set -e

# Load ROS environment.
source /opt/ros/humble/setup.bash

cmake "$@"
