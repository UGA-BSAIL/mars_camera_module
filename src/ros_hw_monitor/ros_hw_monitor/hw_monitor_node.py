"""
Monitors the hardware on the device that it is running on and reports status.
"""

import rclpy
from rclpy.node import Node
from ros_hw_monitor_msgs.msg import System
from .jetson_monitor import IS_JETSON, JetsonMonitor
from .monitor import Monitor


class HardwareMonitorNode(Node):
    def __init__(self):
        super().__init__("hardware_monitor")
        self.get_logger().info("Starting hardware monitoring...")

        self.monitor = Monitor(self.get_logger())
        if IS_JETSON:
            self.get_logger().info("Detected Jetson hardware.")
            self.monitor = JetsonMonitor(self.get_logger())

        self.publisher = self.create_publisher(System, "system_info", 10)
        self.update_rate = self.declare_parameter("update_rate", 1).value
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)

    def timer_callback(self):
        message = System()
        message.processes = self.monitor.get_processes()
        message.gpu_usage = 0.0
        if isinstance(self.monitor, JetsonMonitor):
            message.gpu_usage = self.monitor.gpu_usage()

        message.cpu_temp, message.gpu_temp, message.npu_temp = self.monitor.get_temps()

        self.publisher.publish(message)

def main():
    rclpy.init()
    hw_monitor_node = HardwareMonitorNode()
    rclpy.spin(hw_monitor_node)
    hw_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()