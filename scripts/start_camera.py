#!/usr/bin/env python3


"""
This script handles deployment and management of the camera code for ROS2.
"""


from functools import cache
import os
import signal
import sys
from typing import Any

import confuse
from loguru import logger
from netifaces import interfaces, AF_INET, ifaddresses
import launch
from launch import LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

ROS_INTERFACE_NAME = "eth0"
"""
The name of the network interface that we are connected to the ROS2 system on.
"""


# Set up Confuse.
config = confuse.Configuration("camera_module", "config")


@cache
def _get_local_ip() -> str:
    assert (
        ROS_INTERFACE_NAME in interfaces()
    ), f"{ROS_INTERFACE_NAME} is not a valid network interface!"

    inet_address = ifaddresses(ROS_INTERFACE_NAME)[AF_INET]
    if len(inet_address) > 1:
        logger.warning("Found multiple inet addresses. Using the first one.")
    node_ip = inet_address[0]["addr"]
    logger.info("Node IP is {}.", node_ip)

    return node_ip


def _configure_env() -> None:
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ["ROS_IP"] = _get_local_ip()


def main() -> None:
    _configure_env()

    launch_file = config["launch_file"].as_path().as_posix()  # Path to *.launch.py
    node_name = config["node_name"].as_str()
    frame_id = config["frame_id"].as_str()
    model_dir = config["hailo"]["model_dir"].as_path() if "hailo" in config else ""
    capabilities = config["capabilities"] if "capabilities" in config else None
    if capabilities and capabilities["hw_video_encode"].get(bool):
        encoder = "h264_v4l2m2m"
        pixel_format = "yuv420p"
    else:
        encoder = "mjpeg"
        pixel_format = "yuvj420p"
    logger.debug("Selected encoder: {}", encoder)

    launch_args = [
        ("node_name", node_name),
        ("frame_id", frame_id),
        ("encoder", encoder),
        ("pixel_format", pixel_format),
        ("model_dir", str(model_dir)),
    ]

    logger.info("Launching {} with args {}...", launch_file, launch_args)
    launch_service = LaunchService()
    launch_description = launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments=launch_args,
        )
    ])
    launch_service.include_launch_description(launch_description)

    def shutdown_handler(*_: Any):
        logger.info("Got exit signal. Stopping launch service...")
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    sys.exit(launch_service.run())


if __name__ == "__main__":
    main()
