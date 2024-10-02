import time
from bisect import bisect_left

from gpiozero import PWMLED

import rospy
from ros_hw_monitor.msg import System


class FanController:
    """
    Handles controlling the fan.
    """

    _FAN_CURVE = {30: 0.0, 40: 0.1, 50: 0.25, 60: 0.5, 65: 0.75, 70: 1.0}
    """
    Maps temperature readings (in C) to fan PWM values.
    """
    _FAN_CURVE_TEMPS = list(_FAN_CURVE.keys())

    _FAN_TEST_TIME = 3
    """
    How long to test the fan for, in seconds.
    """
    _HYSTERESIS = 5
    """
    Amount of hysteresis, in C.
    """

    def __init__(self, fan_pin: int):
        """
        Args:
            fan_pin: The GPIO pin that the fan is connected to.

        """
        # We can use the PWMLED class to control the fan.
        self.__fan = PWMLED(fan_pin)
        # Default to 100% before we read the temperature.
        self.__fan.value = 1.0
        # Time at which we started the fan test.
        self.__fan_test_start_time = time.time()

    def __find_speed(self, temp: float) -> float:
        """
        Calculates a fan speed for the given temperature.

        Args:
            temp: The temperature, in C.

        Returns:
            The fan speed.

        """
        index = bisect_left(self._FAN_CURVE_TEMPS, int(temp))
        if index >= len(self._FAN_CURVE_TEMPS):
            # If it's above the highest temperature, use the max speed.
            index = -1

        temp_threshold = self._FAN_CURVE_TEMPS[index]
        speed = self._FAN_CURVE[temp_threshold]

        if speed < self.__fan.value:
            # Apply the hysteresis before spinning down the fan.
            speed = self.__find_speed(temp + self._HYSTERESIS)

        return speed

    def on_system_message(self, message: System) -> None:
        """
        Callback to run whenever a new message from the hardware monitor
        node is received.

        Args:
            message: The received message.

        """
        if time.time() - self.__fan_test_start_time < self._FAN_TEST_TIME:
            # Keep the fan at 100% while testing.
            self.__fan.value = 1.0
            return

        temp = max(message.cpu_temp, message.gpu_temp)
        speed = self.__find_speed(temp)

        # Set the speed.
        if speed != self.__fan.value:
            rospy.loginfo(f"Setting fan speed to {speed}.")
            self.__fan.value = speed


def main() -> None:
    rospy.init_node("fan_controller", anonymous=True)

    controller = FanController(int(rospy.get_param("~fan_pin")))
    rospy.Subscriber(
        "~system_info",
        System,
        controller.on_system_message,
    )

    rospy.spin()