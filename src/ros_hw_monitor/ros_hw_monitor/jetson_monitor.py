from typing import Tuple, Any

from .monitor import Monitor

IS_JETSON = True
try:
    from jtop import jtop
except ImportError:
    # We can't use this module.
    IS_JETSON = False


class JetsonMonitor(Monitor):
    """
    Specialized monitor for the Jetson.
    """

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)

        self.__jetson = jtop()
        self.__jetson.start()

    def __del__(self):
        self.__jetson.close()

    def gpu_usage(self) -> float:
        """
        Returns:
            The total percentage of GPU usage.
        """
        # First GPU should be the internal one.
        gpu_key = next(iter(self.__jetson.gpu.keys()))
        return self.__jetson.gpu[gpu_key]["status"]["load"]

    def get_temps(self) -> Tuple[float, float, float]:
        temps = self.__jetson.temperature
        return temps["CPU"]["temp"], temps["GPU"]["temp"], -1
