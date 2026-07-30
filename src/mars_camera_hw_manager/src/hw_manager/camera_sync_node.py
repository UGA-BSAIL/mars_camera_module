"""
Handles synchronizing frames between multiple cameras, such as in a stereo configuration.
"""

from collections import deque

import dynamic_reconfigure.client
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class CameraSyncNode:
    """
    Handles synchronizing frames between two cameras, such as in a stereo configuration.
    """

    _MAX_WINDOW = 10
    """
    The maximum window size to use when calculating errors.
    """

    def __init__(self, kp: float = 0.05):
        """
        Args:
            kp: The proportional gain to use for the control loop.

        """
        self.kp = kp

        # We need to manually specify the name of the camera nodes so that we
        # can configure them.
        primary_node_name = rospy.get_param("~primary_node_name")
        secondary_node_name = rospy.get_param("~secondary_node_name")
        self.primary_camera_topic = "~primary/frame_headers"
        self.secondary_camera_topic = f"/{secondary_node_name}/frame_headers"
        self.last_primary_frame_time = None
        self.last_secondary_frame_time = None

        base_frame_rate = float(rospy.get_param("~frame_rate"))
        self.base_frame_time = 1 / base_frame_rate

        # Keeps track of frame sequencing.
        self.num_primary_frames = 0
        self.num_secondary_frames = 0
        self.synchronization_errors = deque(maxlen=self._MAX_WINDOW)
        self.primary_offset = 0.0
        self.secondary_offset = 0.0

        self.primary_reconfigure_client = dynamic_reconfigure.client.Client(
            primary_node_name, timeout=30
        )
        self.secondary_reconfigure_client = dynamic_reconfigure.client.Client(
            secondary_node_name, timeout=30
        )

        rospy.Subscriber(self.primary_camera_topic, Header, self.primary_callback)
        rospy.Subscriber(self.secondary_camera_topic, Header, self.secondary_callback)

    def primary_callback(self, msg: Header) -> None:
        """
        Callback for the primary camera frame header.

        Args:
            msg (Header): The message containing the frame header data.
        """
        self.last_primary_frame_time = msg.stamp.to_sec()

        self.num_primary_frames += 1
        if self.num_primary_frames == self.num_secondary_frames:
            self.calculate_synchronization_error()

        if abs(self.num_primary_frames - self.num_secondary_frames) > 1:
            self.num_primary_frames = 0
            self.num_secondary_frames = 0

    def secondary_callback(self, msg: Header) -> None:
        """
        Callback for the secondary camera frame header.

        Args:
            msg (Header): The message containing the frame header data.
        """
        self.last_secondary_frame_time = msg.stamp.to_sec()

        self.num_secondary_frames += 1
        if self.num_primary_frames == self.num_secondary_frames:
            self.calculate_synchronization_error()

        if abs(self.num_primary_frames - self.num_secondary_frames) > 1:
            self.num_primary_frames = 0
            self.num_secondary_frames = 0

    def calculate_synchronization_error(self) -> None:
        """
        Calculates the synchronization error between the primary and secondary camera frames.
        Appends the error to the list of synchronization errors.
        """
        if (
            self.last_primary_frame_time is not None
            and self.last_secondary_frame_time is not None
        ):
            error_mag = (
                np.abs(self.last_primary_frame_time - self.last_secondary_frame_time)
                % self.base_frame_time
            )
            error_dir = (
                1
                if self.last_primary_frame_time >= self.last_secondary_frame_time
                else -1
            )
            if error_mag > self.base_frame_time / 2:
                # We can make a smaller adjustment by shifting in the opposite
                # direction.
                error_mag = self.base_frame_time - error_mag
                error_dir *= -1

            self.synchronization_errors.append(error_mag * error_dir)

    def control_loop(self) -> None:
        """
        Starts the control loop to adjust synchronization of the secondary camera to the primary camera.
        Runs at a rate of 5 Hz.
        """
        rate = rospy.Rate(5)  # 5 Hz control loop
        while not rospy.is_shutdown():
            if len(self.synchronization_errors) > 0:
                average_error = np.mean(list(self.synchronization_errors))
                if np.abs(average_error) < 0.5:
                    # A large average error probably means only one camera is running.
                    self.adjust_camera_settings(float(average_error))
                    self.synchronization_errors = (
                        deque()
                    )  # Reset errors after adjustment
            rate.sleep()

    def adjust_camera_settings(self, average_error: float) -> None:
        """
        Adjusts the camera settings based on the average synchronization error.
        Args:
            average_error (float): The average synchronization error to adjust the settings.
        """
        # Convert to us
        average_error *= 1e6
        secondary_offset = self.kp * average_error
        rospy.loginfo(
            f"Average camera sync error: {average_error / 1000} ms, offset: {secondary_offset}"
        )

        # Choose which camera to adjust the framerate of.
        if secondary_offset <= 0:
            # If we need to speed up the secondary camera, bring the secondary
            # camera to the base frame rate, and slow down the primary one.
            self.secondary_offset = 0
            self.primary_offset = -secondary_offset
        else:
            # If we need to slow down the secondary camera, bring the primary
            # camera to the base frame rate, and slow down the secondary one.
            self.primary_offset = 0
            self.secondary_offset = secondary_offset

        try:
            self.primary_reconfigure_client.update_configuration(
                # Lower offset means higher framerate.
                {"frame_duration_offset": self.primary_offset}
            )
            self.secondary_reconfigure_client.update_configuration(
                {"frame_duration_offset": self.secondary_offset}
            )
        except Exception as e:
            rospy.logerr(f"Failed to update configuration: {e}")


def main():
    rospy.init_node("camera_sync", anonymous=True)
    rospy.loginfo("Starting camera sync node...")

    camera_sync = CameraSyncNode(kp=float(rospy.get_param("~kp", 0.05)))
    camera_sync.control_loop()
