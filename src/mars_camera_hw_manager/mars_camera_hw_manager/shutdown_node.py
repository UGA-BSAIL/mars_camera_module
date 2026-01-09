"""
A node that does nothing except send a shutdown command to the cameras.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from mars_camera_hw_manager_msgs.msg import CameraControl


# How long to wait for cameras to shut down, in seconds.
WAIT_TIME = 5


class CameraShutdownNode(Node):
    """
    Sends a shutdown signal to a camera.
    """
    def __init__(self):
        super().__init__("camera_shutdown")

        # Get the camera name to use.
        self.declare_parameter("camera_id", "camera")
        camera_id = self.get_parameter("camera_id").value

        # Publish the control message.
        topic = f"/{camera_id}/control"
        self.get_logger().info(f"Sending shutdown message on {topic}")
        self.publisher = self.create_publisher(CameraControl, topic, 10)
        msg = CameraControl()
        msg.shutdown = True
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = CameraShutdownNode()

    # Spin until our wait time has elapsed.
    start_time = node.get_clock().now()
    while node.get_clock().now() - start_time < Duration(seconds=WAIT_TIME):
        rclpy.spin_once(node, timeout_sec=WAIT_TIME)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
