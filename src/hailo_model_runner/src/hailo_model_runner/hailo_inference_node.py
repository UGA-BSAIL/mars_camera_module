"""
Handles performing model inference on the HAILO device. It reads the raw images from a topic, performs the inference,
and outputs the bounding boxes to a topic.
"""

import threading
import time
from pathlib import Path
from queue import Empty, Full, Queue
from typing import Tuple

import cv2
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from libcamera_device.msg import Detection, FrameDetections

from .utils import HailoAsyncInference


class HailoInferenceManager:
    """
    Manages inference on the HAILO device.

    """

    def __init__(self, hef_path: Path):
        """
        Args:
            hef_path: The path to the model HEF file.

        """
        # We deliberately limit the size of the input queue to minimize latency.
        self.__input_queue = Queue(maxsize=1)
        self.__output_queue = Queue()

        rospy.loginfo(f"Loading model from file: {hef_path}")
        self.__async_inference = HailoAsyncInference(
            hef_path.as_posix(),
            self.__input_queue,
            self.__output_queue,
            send_original_frame=False,
        )
        self.__inference_thread = threading.Thread(target=self.__async_inference.run)
        self.__inference_started = False

        self.__box_output_name, self.__feature_output_name = (
            self.__get_model_output_names()
        )
        # Force the appearance feature output to float32.
        self.__async_inference.set_output_type(
            {self.__box_output_name: "FLOAT32", self.__feature_output_name: "FLOAT32"}
        )

        # Time at which we last logged inference stats.
        self.__last_inference_stat_time = time.time()
        # How many frames we've processed since the last stats log.
        self.__frames_since_last_stat = 0

        # Rate limitter to control CPU usage.
        self.__rate = rospy.Rate(100)

    def __get_model_output_names(self) -> Tuple[str, str]:
        """
        Returns:
            The names of the model box output, and appearance feature output, in that order.

        """
        output_names = self.__async_inference.get_output_names()
        output_shapes = self.__async_inference.get_output_shapes()
        assert len(output_names) == 2, "Model does not have 2 outputs."

        box_name = None
        feature_name = None
        for name, shape in zip(output_names, output_shapes):
            if shape[1] == 5:
                # This is the bounding box output.
                box_name = name
            else:
                # This must be the appearance feature output.
                feature_name = name

        return box_name, feature_name

    def __log_stats_periodically(self, last_header: Header) -> None:
        """
        Calculates inference statistics and logs them periodically.

        Args:
            last_header: The header from the most recent frame we received.

        """
        # We run this every time frame detections are received.
        self.__frames_since_last_stat += 1

        sampling_time = time.time() - self.__last_inference_stat_time
        if sampling_time < 10:
            # Don't need to log yet.
            return

        # Compute inference speed statistics.
        fps = self.__frames_since_last_stat / sampling_time
        latency = rospy.get_rostime().to_sec() - last_header.stamp.to_sec()
        rospy.loginfo(f"HAILO inference stats: FPS: {fps:.2f}, latency: {latency:.3f}s")

        self.__last_inference_stat_time = time.time()
        self.__frames_since_last_stat = 0

    @staticmethod
    def __img_to_numpy(image: Image) -> np.array:
        """
        Converts a ROS image message to a Numpy array.

        Args:
            image: The input image.

        Returns:
            The output array, of shape `[H, W, C]`.

        """
        # We only support BGR8 images.
        assert image.encoding == "bgr8", f"Unsupported image encoding: {image.encoding}"

        height = image.height
        width = image.width
        channels = len(image.data) // (height * width)

        # Convert the image data to a numpy array
        img_np = np.frombuffer(image.data, dtype=np.uint8).reshape(
            height, width, channels
        )
        # Convert from BGR to RGB.
        img_np = img_np[:, :, ::-1]

        # If the image is grayscale, convert it to RGB
        if channels == 1:
            img_np = np.stack((img_np,) * 3, axis=-1)

        return img_np

    def on_image_received(self, image: Image) -> None:
        """
        Callback to run when a new image message is received from ROS.

        Args:
            image: The image message.

        """
        self.__rate.sleep()

        header = image.header
        image = self.__img_to_numpy(image)
        if image.shape[0] > image.shape[1]:
            # Un-rotate rotated stereo image.
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        image = cv2.resize(image, (960, 540))

        # Write it to the input queue.
        try:
            self.__input_queue.put((image[None, :, :, :], [header]), block=False)
        except Full:
            rospy.logdebug("Dropping HAILO input frame due to full queue.")

    def start(self) -> None:
        """
        Starts the inference thread.

        """
        self.__inference_thread.start()
        self.__inference_started = True

    def shutdown(self) -> None:
        """
        Shuts down the inference pipeline.

        """
        if not self.__inference_started:
            return

        # This special value will tell it to stop the runner thread.
        rospy.loginfo("Waiting for inference thread to exit...")
        self.__input_queue.put(None)
        self.__inference_thread.join()

    def handle_inference_result(self, publisher: rospy.Publisher) -> None:
        """
        Handles new inference results from the NPU.

        Args:
            publisher: Publisher to use for publishing results.

        """
        # Read from the queue.
        try:
            _, header, inference_results = self.__output_queue.get(timeout=1)
        except Empty:
            return

        # Convert to a ROS message.
        self.__log_stats_periodically(header)
        boxes = np.concatenate(inference_results[self.__box_output_name], axis=0)
        appearance_features = inference_results[self.__feature_output_name]

        confidences = boxes[:, -1]
        boxes = boxes[:, :-1]
        centers = 0.5 * (boxes[:, :2] + boxes[:, 2:])
        sizes = boxes[:, 2:] - boxes[:, :2]
        print(centers, sizes)
        detections = []
        for center, size, confidence in zip(centers, sizes, confidences):
            center_y, center_x = center
            height, width = size
            detections.append(
                Detection(
                    center_x=center_x,
                    center_y=center_y,
                    width=width,
                    height=height,
                    confidence=confidence,
                )
            )

        print(appearance_features.shape)
        frame_detections = FrameDetections(
            header=header,
            detections=detections,
            appearance_features=appearance_features.tobytes(),
            appearance_feature_shape=[1] + list(appearance_features.shape),
        )
        publisher.publish(frame_detections)


def main() -> None:
    # Initialize the ROS node.
    rospy.init_node("hailo_inference")

    # Path to the model HEF file to load.
    hef_file_path = Path(rospy.get_param("~hef_file_path"))
    # Set up the inference pipeline.
    inference_manager = HailoInferenceManager(hef_file_path)
    # Publisher for detections
    detection_publisher = rospy.Publisher(
        "~detections",
        FrameDetections,
        queue_size=10,
    )
    rospy.Subscriber(
        "~image",
        Image,
        inference_manager.on_image_received,
        queue_size=1,
    )
    inference_manager.start()

    # Run until shutdown.
    while not rospy.is_shutdown():
        inference_manager.handle_inference_result(detection_publisher)

    inference_manager.shutdown()
