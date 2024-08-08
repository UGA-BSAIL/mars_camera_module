"""
Handles running launch files.
"""


from multiprocessing import Event, Process
from typing import Tuple, List, Optional
import time

from loguru import logger
import roslaunch


_UUID = roslaunch.rlutil.get_or_generate_uuid(
    options_runid=None, options_wait_for_master=True
)
"""
UUID to use for launch files.
"""
roslaunch.configure_logging(_UUID)


class ProcessListener(roslaunch.pmon.ProcessListener):
    """
    Listens to the status of ROS processes to determine when we should exit.
    """

    def __init__(self):
        # Indicates whether all processes have exited.
        self.__all_finished = False

        # Total number of running processes.
        self.__num_processes = None
        # Names of the processes that have exited.
        self.__exited_processes = set()

    def process_died(self, name: str, _) -> None:
        logger.debug("Process {} exited.", name)
        self.__exited_processes.add(name)

    def set_num_processes(self, num_processes: int) -> None:
        """
        Args:
            num_processes: The number of processes to wait for.

        """
        self.__num_processes = num_processes

    @property
    def all_finished(self) -> bool:
        """
        Returns:
            True if all processes have finished.

        """
        return len(self.__exited_processes) == self.__num_processes


def _run_launch(
    launch_file: Tuple[str, str],
    *,
    ros_args: List[str] = [],
    started_event: Event,
    stop_event: Optional[Event] = None
) -> None:
    """
    Runs a launch file.

    Args:
        launch_file: The launch file to run.
        ros_args: The list of arguments to pass to ROS.
        started_event: Event that will be set when the launch file has started.
        stop_event: If present, will check whether this event is set, and if so,
            will stop roslaunch prematurely.

    """
    logger.info("Entered _run_launch.")
    launch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)[0]
    listener = ProcessListener()
    launcher = roslaunch.parent.ROSLaunchParent(
        _UUID,
        [(launch_file, ros_args)],
        process_listeners=[listener],
    )
    logger.info("Created launch parent.")
    try:
        launcher.start()
    except roslaunch.core.RLException:
        logger.warning(
            "Cannot start launch file {} because manager has "
            "already exited.",
            launch_file,
        )
    started_event.set()

    # Set the number of processes to wait for.
    active_processes, _ = launcher.pm.get_process_names_with_spawn_count()
    num_processes = sum(c for _, c in active_processes)
    logger.debug("Waiting for {} processes.", num_processes)
    listener.set_num_processes(num_processes)

    # Wait for it to finish.
    logger.info("Waiting for launch file exit...")
    while not listener.all_finished:
        time.sleep(0.1)
        if stop_event is not None and stop_event.is_set():
            break
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

        # Runs launch files in a separate process.
        # This gets around some issues with roslaunch and threading.
        self.__process = None

        # Internal event that can be set to indicate that the launch file
        # should be stopped.
        self.__stop_event = None

    def start(self) -> None:
        """
        Starts the launch file.

        """
        # Event that's triggered once roslaunch has fully initialized.
        started_event = Event()
        self.__stop_event = Event()

        successfully_started = False
        while not successfully_started:
            logger.info("Running launch file {}...", self.__launch_file)
            self.__process = Process(
                target=_run_launch,
                args=(self.__launch_file,),
                kwargs=dict(
                    ros_args=self.__ros_args,
                    started_event=started_event,
                    stop_event=self.__stop_event,
                ),
            )
            logger.info("Starting process.")
            self.__process.start()

            # Wait for it to start before returning.
            logger.info("Waiting for process start...")
            successfully_started = started_event.wait(timeout=10)
            if not successfully_started:
                logger.warning(
                    "Launch file did not start properly. Retrying..."
                )
                self.__process.kill()
            else:
                logger.debug("Launch file is now running.")

    def wait(self) -> None:
        """
        Waits until the launch file has finished executing.

        """
        if self.__process is None:
            # No launch file to wait for.
            return
        if not self.__process.is_alive():
            # Already finished.
            logger.info("Process for {} already exited.", self.__launch_file)
            self.__process = None
            return

        self.__process.join()
        self.__process = None
        logger.debug("Done running launch file {}.", self.__launch_file)

    def stop(self) -> None:
        """
        Stops the launch file.

        """
        if self.__process is None:
            # No launch file to stop.
            return

        logger.info("Stopping launch file {}...", self.__launch_file)
        self.__stop_event.set()
