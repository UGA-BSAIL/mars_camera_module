"""
Implements actual hardware management tasks.
"""


import rospy
import subprocess


class Manager:
    """
    Implements actual hardware management tasks.
    """

    @staticmethod
    def shutdown() -> None:
        """
        Shuts down the camera module.

        """
        rospy.loginfo("Got shutdown command.")
        subprocess.run(["/sbin/shutdown", "-h", "now"])
