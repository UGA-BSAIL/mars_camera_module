"""
Monitors the hardware on the device that it is running on and reports status.
"""


import rospy

from ros_hw_monitor.msg import System
from .jetson_monitor import IS_JETSON, JetsonMonitor
from .monitor import Monitor


def _update_stats(monitor: Monitor, publisher: rospy.Publisher) -> None:
    """
    Updates the hardware stats and publishes them.
    """
    message = System()

    message.processes = monitor.get_processes()
    message.gpu_usage = 0.0
    if isinstance(monitor, JetsonMonitor):
        message.gpu_usage = monitor.gpu_usage()

    publisher.publish(message)


def main() -> None:
    rospy.init_node("hardware_monitor")
    rospy.loginfo("Starting hardware monitoring...")

    monitor = Monitor()
    if IS_JETSON:
        rospy.loginfo("Detected Jetson hardware.")
        monitor = JetsonMonitor()

    # Create the publisher.
    publisher = rospy.Publisher("system_info", System, queue_size=1)

    # Get update rate in Hz.
    update_rate = int(rospy.get_param("~update_rate", 1))
    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        _update_stats(monitor, publisher)
        rate.sleep()

