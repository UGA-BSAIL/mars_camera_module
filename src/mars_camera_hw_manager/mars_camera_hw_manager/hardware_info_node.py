#!/usr/bin/python3

"""
Node that broadcasts information about the camera module hardware.
"""

import confuse
import rclpy
from rclpy.node import Node
from mars_camera_hw_manager_msgs.msg import CameraInfo


# Set up Confuse.
config = confuse.Configuration("camera_module", "config")


def _make_info_message() -> CameraInfo:
    """
    Publishes information about the camera hardware.

    Returns:
        The camera info message.

    """
    frame_id = config["frame_id"].as_str()

    cap_config = config["capabilities"]
    supports_ir = cap_config["ir"].get(bool)

    return CameraInfo(frame_id=frame_id, supports_ir=supports_ir)


class CameraHWInfoNode(Node):
    def __init__(self):
        super().__init__('camera_hw_info')

        # Get the camera info message and publish it.
        self.publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        message = _make_info_message()
        self.publisher.publish(message)
        self.get_logger().info("Published camera hardware info.")


def main() -> None:
    rclpy.init()
    node = CameraHWInfoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
