#!/bin/bash

# A wrapper around CMake so that it works with catkin but behaves
# nicely with CLion.

set -e

# Load ROS environment.
source /opt/ros/noetic/setup.bash

cmake "$@"
