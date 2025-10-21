#!/usr/bin/python3

"""
Node that broadcasts information about the camera module hardware.
"""


import confuse
import rospy

from mars_camera_hw_manager.msg import CameraInfo


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
    supports_stereo = cap_config["stereo"].get(bool)

    camera_info = CameraInfo(
        supports_ir=supports_ir, supports_stereo=supports_stereo
    )
    camera_info.header.frame_id = frame_id
    return camera_info


def main() -> None:
    rospy.init_node("camera_hw_info", anonymous=True)

    # Get the camera info message and publish it.
    publisher = rospy.Publisher("~camera_info", CameraInfo, queue_size=10, latch=True)
    message = _make_info_message()
    publisher.publish(message)

    rospy.loginfo("Published camera hardware info.")
    rospy.spin()


if __name__ == "__main__":
    main()
