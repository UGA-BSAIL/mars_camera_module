#!/bin/bash

# Helper script for running Rviz inside Docker.

set -e

# Allow Rviz to resolve the master hostname.
echo "${ROS_IP} ros" >> /etc/hosts

rviz