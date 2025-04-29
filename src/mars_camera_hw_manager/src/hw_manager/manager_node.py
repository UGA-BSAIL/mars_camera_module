"""
Node for camera hardware management tasks.
"""


import rospy

from mars_camera_hw_manager.msg import CameraControl

from .manager import Manager


class Node:
    """
    Main class for the node.
    """

    def __init__(self, *, manager: Manager):
        """
        Args:
            manager: The hardware manager implementation to use.
        """
        self.__manager = manager

        # Set up the subscription.
        self.__control_subscriber = rospy.Subscriber("~control", CameraControl, self.__handle_control)
        rospy.loginfo("Waiting for control commands...")

    def __handle_control(self, message: CameraControl) -> None:
        """
        Handles a new control message.

        Args:
            message: The message to handle.

        """
        rospy.loginfo("Received control message.")
        if message.shutdown:
            # Shutdown the node.
            self.__manager.shutdown()


def main() -> None:
    rospy.init_node("camera_hw_manager", anonymous=True)

    manager = Manager()
    Node(manager=manager)

    rospy.spin()


if __name__ == "__main__":
    main()
