#!/usr/bin/env python

"""
A node that does nothing except send a shutdown command to the cameras.
"""

import rospy

from mars_camera_hw_manager.msg import CameraControl


# How long to wait for cameras to shut down, in seconds.
WAIT_TIME = 5


def main():
    rospy.init_node("camera_shutdown", anonymous=True)

    # Get the camera name to use.
    camera_id = rospy.get_param("~camera_id", "camera")

    # Publish the control message.
    topic = "/" + camera_id + "/control"
    rospy.loginfo("Sending shutdown message on " + topic)
    publisher = rospy.Publisher(topic, CameraControl, queue_size=1, latch=True)
    publisher.publish(shutdown=True)

    rospy.sleep(WAIT_TIME)


if __name__ == "__main__":
    main()
