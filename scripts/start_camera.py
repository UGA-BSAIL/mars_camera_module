#!/usr/bin/env python


"""
This script handles deployment and management of the camera code.
"""


from functools import cache, partial
import os
import signal
import sys
from typing import Any

import confuse
from loguru import logger
from netifaces import interfaces, AF_INET, ifaddresses
import roslaunch


ROS_INTERFACE_NAME = "eth0"
"""
The name of the network interface that we are connected to the ROS master on.
"""


# Set up Confuse.
config = confuse.Configuration("camera_module", "config")


@cache
def _get_local_ip() -> str:
    """
    Gets the ethernet IP address of this camera module.

    Returns:
        The IP address of this module.

    """
    assert ROS_INTERFACE_NAME in interfaces(), f"{ROS_INTERFACE_NAME} is not a valid network interface!"

    # Get the associated addresses.
    inet_address = ifaddresses(ROS_INTERFACE_NAME)[AF_INET]
    if len(inet_address) > 1:
        logger.warning("Found multiple inet addresses. Using the first one.")
    node_ip = inet_address[0]["addr"]
    logger.info("Node IP is {}.", node_ip)

    return node_ip


def _configure_env() -> None:
    """
    Sets the proper environment variables for ROS.

    """
    os.environ["ROS_MASTER_URI"] = config["ros_master_uri"].as_str()
    os.environ["ROS_IP"] = _get_local_ip()


def _run_camera() -> roslaunch.parent.ROSLaunchParent:
    """
    Runs the camera code.

    Returns:
        Returns the launcher.

    """
    # Start the node.
    launch_file = config["launch_file"].as_path()
    node_name = config["node_name"].as_str()
    frame_id = config["frame_id"].as_str()
    logger.info("Launching {}...", launch_file)

    launch_file = roslaunch.rlutil.resolve_launch_arguments((launch_file.as_posix(), "--wait"))[0]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launcher = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, [f"node_name:={node_name}", f"frame_id:={frame_id}"])])
    launcher.start()

    return launcher


def _on_program_exit(launcher: roslaunch.parent.ROSLaunchParent, *_: Any) -> None:
    """
    Handler that should run when the program exits, and cleans everything up.

    Args:
        launcher: The ROS launcher that is running.

    """
    logger.info("Got exit signal, stopping ROS...")
    launcher.shutdown()

    sys.exit()


def main() -> None:
    _configure_env()
    launcher = _run_camera()

    # Run a handler when we exit to stop ROS.
    handler = partial(_on_program_exit, launcher)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    launcher.spin()


if __name__ == "__main__":
    main()

