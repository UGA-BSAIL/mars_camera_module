from typing import List, Tuple
from pathlib import Path

import psutil

import rospy
from ros_hw_monitor.msg import Process

try:
    from hailo_platform import Device
except ImportError:
    # This system does not have the HAILO RT library installed.
    Device = None


class Monitor:
    """
    Monitors basic hardware info for the device it is running on.
    """

    def __init__(self):
        self.__hailo_devices = []
        if Device is not None:
            # Enumerate HAILO targets.
            devices_info = Device.scan()
            self.__hailo_devices = [Device(d) for d in devices_info]
            rospy.loginfo(f"Found {len(devices_info)} HAILO devices.")

    @staticmethod
    def __message_from_process(process: psutil.Process) -> Process:
        """
        Creates a process message from psutil.

        Args:
            process: The process to create a message from.

        Returns:
            The created message.

        """
        with process.oneshot():
            return Process(
                pid=process.pid,
                name=process.name(),
                command_line=process.cmdline(),
                cpu_usage=process.cpu_percent(),
                memory_usage=process.memory_percent(),
            )

    def get_processes(self) -> List[Process]:
        """
        Gets the list of processes currently running on the system.

        Returns:
            Information for each process.

        """
        processes = []
        for proc in psutil.process_iter():
            try:
                processes.append(self.__message_from_process(proc))
            except psutil.NoSuchProcess:
                pass
        return processes

    def get_temps(self) -> Tuple[float, float, float]:
        """
        Returns:
            The CPU, GPU, and NPU temperatures of this device. Temperatures <0 mean
            we couldn't read that sensor.

        """
        # Reading from /sys is a fairly portable way to get temperatures.
        raw_cpu_temp = float(Path("/sys/class/thermal/thermal_zone0/temp").read_text())
        cpu_temp = raw_cpu_temp / 1000

        # Measure NPU temperatures.
        max_hailo_temp = -1.0
        for device in self.__hailo_devices:
            hailo_temp = device.control.get_chip_temperature().ts0_temperature
            max_hailo_temp = max(hailo_temp, max_hailo_temp)

        return cpu_temp, -1, max_hailo_temp
