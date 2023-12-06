"""
Handles running launch files.
"""


from concurrent.futures import ProcessPoolExecutor, wait
from typing import Tuple, List
import time

from loguru import logger
import roslaunch


class ProcessListener(roslaunch.pmon.ProcessListener):
    """
    Listens to the status of ROS processes to determine when we should exit.
    """

    def __init__(self):
        # Indicates whether all processes have exited.
        self.__all_finished = False

    def process_died(self, name: str, _) -> None:
        logger.debug("Process {} exited.", name)
        self.__all_finished = True

    @property
    def all_finished(self) -> bool:
        """
        Returns:
            True if all processes have finished.

        """
        return self.__all_finished


def _run_launch(launch_file: Tuple[str, str], ros_args: List[str] = []) -> None:
    """
    Runs a launch file.

    Args:
        launch_file: The launch file to run.
        ros_args: The list of arguments to pass to ROS.

    """
    launch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)[0]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    listener = ProcessListener()
    launcher = roslaunch.parent.ROSLaunchParent(
        uuid,
        [(launch_file, ros_args)],
        process_listeners=[listener],
    )
    launcher.start()

    # Wait for it to finish.
    while not listener.all_finished:
        time.sleep(0.1)
        launcher.spin_once()
    launcher.shutdown()


class LaunchManager:
    """
    Handles running launch files.
    """

    def __init__(self, launch_file: Tuple[str, str], ros_args: List[str] = []):
        """
        Args:
            launch_file: The launch file to run, as a package and file name.
            ros_args: The arguments to pass to the launch file.

        """
        self.__launch_file = launch_file
        self.__ros_args = ros_args

        # Internal pool used for running launch files in a separate process.
        # This gets around some issues with roslaunch and threading.
        self.__pool = ProcessPoolExecutor(max_workers=1)
        # Future representing the launch file.
        self.__future = None

    def start(self) -> None:
        """
        Starts the launch file.

        """
        logger.info("Running launch file {}...", self.__launch_file)
        self.__future = self.__pool.submit(
            _run_launch, self.__launch_file, self.__ros_args
        )

    def wait(self) -> None:
        """
        Waits until the launch file has finished executing.

        """
        if self.__future is None:
            # No launch file to wait for.
            return

        wait([self.__future])
        logger.debug("Done running launch file {}.", self.__launch_file)
