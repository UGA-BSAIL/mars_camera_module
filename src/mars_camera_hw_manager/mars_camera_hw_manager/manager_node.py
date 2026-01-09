"""
Node for camera hardware management tasks.
"""

import rclpy
from rclpy.node import Node
from mars_camera_hw_manager_msgs.msg import CameraControl
from .manager import Manager


class ManagerNode(Node):
    """
    Main class for the node.
    """

    def __init__(self):
        super().__init__("camera_hw_manager")
        self.__manager = Manager(self.get_logger())

        # Set up the subscription.
        self.__control_subscriber = self.create_subscription(
            CameraControl, "control", self.__handle_control, 10
        )
        self.get_logger().info("Waiting for control commands...")

    def __handle_control(self, message: CameraControl) -> None:
        """
        Handles a new control message.

        Args:
            message: The message to handle.

        """
        self.get_logger().info("Received control message.")
        if message.shutdown:
            # Shutdown the node.
            self.__manager.shutdown()


def main() -> None:
    rclpy.init()
    node = ManagerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
