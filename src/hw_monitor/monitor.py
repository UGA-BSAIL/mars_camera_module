from typing import List, Tuple
from pathlib import Path

import psutil

from ros_hw_monitor.msg import Process


class Monitor:
    """
    Monitors basic hardware info for the device it is running on.
    """

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

    def get_temps(self) -> Tuple[float, float]:
        """
        Returns:
            The CPU and GPU temperatures of this device. Temperatures <0 mean
            we couldn't read that sensor.

        """
        # Reading from /sys is a fairly portable way to get temperatures.
        raw_temp = float(Path("/sys/class/thermal/thermal_zone0/temp").read_text())
        return raw_temp / 1000, -1

