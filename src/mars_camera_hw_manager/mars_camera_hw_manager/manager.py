"""
Implements actual hardware management tasks.
"""

import subprocess


class Manager:
    """
    Implements actual hardware management tasks.
    """

    def __init__(self, logger):
        """
        Args:
            logger: The ROS logger to use.

        """
        self.__logger = logger

    def shutdown(self) -> None:
        """
        Shuts down the camera module.

        """
        self.__logger.info("Got shutdown command.")
        subprocess.run(["/sbin/shutdown", "-h", "now"])
