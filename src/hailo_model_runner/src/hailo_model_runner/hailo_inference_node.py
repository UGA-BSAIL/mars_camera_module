"""
Handles performing model inference on the HAILO device. It reads the raw images from a topic, performs the inference,
and outputs the bounding boxes to a topic.
"""

from pathlib import Path
from queue import Queue, Full, Empty
import threading
from typing import Tuple
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rospy.numpy_msg import numpy_msg

import numpy as np

from .utils import HailoAsyncInference
from hailo_model_runner.msg import Detection, FrameDetections


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
            # Force the appearance feature output to float32.
            # output_type={"yolov8m_flower_renamed/conv20": "FLOAT32"},
            send_original_frame=False,
        )
        self.__inference_thread = threading.Thread(target=self.__async_inference.run)
        self.__inference_started = False

        self.__box_output_name, self.__feature_output_name = (
            self.__get_model_output_names()
        )

        # Time at which we last logged inference stats.
        self.__last_inference_stat_time = time.time()
        # How many frames we've processed since the last stats log.
        self.__frames_since_last_stat = 0

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
        header = image.header
        image = self.__img_to_numpy(image)
        # Downsample by half. We do it in this simplistic way to save CPU time.
        image = image[::2, ::2, :]

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
        boxes = inference_results[self.__box_output_name][0][0]
        appearance_features = inference_results[self.__feature_output_name][0]

        confidences = boxes[:, -1]
        boxes = boxes[:, :-1]
        centers = 0.5 * (boxes[:, :2] + boxes[:, 2:])
        sizes = boxes[:, 2:] - boxes[:, :2]
        detections = []
        for center, size, confidence in zip(centers, sizes, confidences):
            center_x, center_y = center
            width, height = size
            detections.append(
                Detection(
                    center_x=center_x,
                    center_y=center_y,
                    width=width,
                    height=height,
                    confidence=confidence,
                )
            )

        frame_detections = FrameDetections(
            header=header,
            detections=detections,
            appearance_features=appearance_features.tobytes(),
            appearance_feature_shape=[1] + list(appearance_features.shape),
        )
        publisher.publish(frame_detections)


class DetectionsSubscribeListener(rospy.SubscribeListener):
    """
    Listens for subscribe/unsubscribe events. It will start and stop the detection pipeline
    based on whether any one is subscribed to the detections topic.
    """

    def __init__(self, inference_manager: HailoInferenceManager):
        """
        Args:
            inference_manager: The inference manager to use for handling images.

        """
        super().__init__()

        self.__inference_manager = inference_manager
        self.__subscriber = None
        # Tracks number of subscribers.
        self.__num_subscribers = 0

    def peer_subscribe(self, *_) -> None:
        if not self.__num_subscribers:
            # Subscribe to the image topic.
            rospy.loginfo(
                "Detections subscriber connected. Starting detection pipeline."
            )
            self.__subscriber = rospy.Subscriber(
                "~image",
                Image,
                self.__inference_manager.on_image_received,
                queue_size=1,
            )

        self.__num_subscribers += 1

    def peer_unsubscribe(self, _1, _2) -> None:
        self.__num_subscribers -= 1

        if not self.__num_subscribers and self.__subscriber is not None:
            # Unsubscribe from the image topic.
            rospy.loginfo(
                "Detections subscriber disconnected. Stopping detection pipeline."
            )
            self.__subscriber.unregister()
            self.__subscriber = None


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
        subscriber_listener=DetectionsSubscribeListener(inference_manager),
    )
    inference_manager.start()

    # Run until shutdown.
    while not rospy.is_shutdown():
        inference_manager.handle_inference_result(detection_publisher)

    inference_manager.shutdown()
