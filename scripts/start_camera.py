#!/usr/bin/env python3


"""
This script handles deployment and management of the camera code for ROS2.
"""

import os
import signal
import sys
from functools import cache
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import confuse
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from loguru import logger
from netifaces import AF_INET, ifaddresses, interfaces

import launch
from launch import LaunchService

ROS_INTERFACE_NAME = "eth0"
"""
The name of the network interface that we are connected to the ROS2 system on.
"""


# Set up Confuse.
config = confuse.Configuration("camera_module", "config")


@cache
def _get_local_ip() -> str:
    assert ROS_INTERFACE_NAME in interfaces(), (
        f"{ROS_INTERFACE_NAME} is not a valid network interface!"
    )

    inet_address = ifaddresses(ROS_INTERFACE_NAME)[AF_INET]
    if len(inet_address) > 1:
        logger.warning("Found multiple inet addresses. Using the first one.")
    node_ip = inet_address[0]["addr"]
    logger.info("Node IP is {}.", node_ip)

    return node_ip


def _configure_env(profile_path: Path) -> None:
    """Configure child processes to use Ethernet-only Fast DDS transport.

    Args:
        profile_path: Location to write the generated Fast DDS XML profile.
    """
    node_ip = _get_local_ip()
    profile_path.write_text(
        f"""<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>ethernet_only</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>{node_ip}</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="ethernet_only_participant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>ethernet_only</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
""",
        encoding="utf-8",
    )
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ["ROS_IP"] = node_ip
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile_path)
    logger.info(f"Using Ethernet-only Fast DDS profile at {profile_path}.")


def _launch_camera() -> None:
    """Launch and manage the configured camera processes."""
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
    ]

    logger.info("Launching {} with args {}...", launch_file, launch_args)
    launch_service = LaunchService()
    launch_description = launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments=launch_args,
            )
        ]
    )
    launch_service.include_launch_description(launch_description)

    def shutdown_handler(*_: Any):
        logger.info("Got exit signal. Stopping launch service...")
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    sys.exit(launch_service.run())


def main() -> None:
    """Keep the generated DDS profile available until the camera launch exits."""
    with TemporaryDirectory(prefix="camera-fastdds-") as profile_directory:
        _configure_env(Path(profile_directory) / "ethernet-only.xml")
        _launch_camera()


if __name__ == "__main__":
    main()
