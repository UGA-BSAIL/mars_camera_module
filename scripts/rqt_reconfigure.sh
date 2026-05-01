#!/bin/bash

# Helper script for running Rviz inside Docker.

set -e

# Allow ROS to resolve the master hostname.
echo "${ROS_IP} ros" >> /etc/hosts

source /opt/ros/noetic/setup.bash
rosrun rqt_reconfigure rqt_reconfigure