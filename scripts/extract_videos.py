#!/usr/bin/env python


"""
This script handles extracting saved videos from rosbag files.
"""

from functools import partial
from pathlib import Path
import signal
import sys
from tempfile import TemporaryDirectory
from typing import Any
import argparse
import roslaunch
from ffmpeg import FFmpeg, Progress
from loguru import logger
import time

SPLIT_BAG_LAUNCH = ("ffmpeg_image_transport_tools", "split_bag_mars.launch")
"""
Launch file to use for extracting video data from the bag files.
"""


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


def _split_bag(*, bag_file: Path, output_dir: Path) -> None:
    """
    Splits the video data out of the bag file.

    Args:
        bag_file: The bag file to split data from.
        output_dir: The directory to write output to. Output will be
            written to files named "camera_x" for each camera.

    """
    out_file_base = output_dir / "camera_"

    # Start the node.
    launch_file = roslaunch.rlutil.resolve_launch_arguments(SPLIT_BAG_LAUNCH)[0]
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    listener = ProcessListener()
    launcher = roslaunch.parent.ROSLaunchParent(
        uuid,
        [
            (
                launch_file,
                [
                    f"bag:={bag_file.absolute().as_posix()}",
                    f"out_file_base:={out_file_base.absolute().as_posix()}",
                    "write_time_stamps:=true",
                ],
            )
        ],
        process_listeners=[listener],
    )
    launcher.start()

    # Run a handler when we exit to stop ROS.
    handler = partial(_on_program_exit, launcher)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    # Wait for it to finish.
    while not listener.all_finished:
        time.sleep(0.01)
        launcher.spin_once()
    launcher.shutdown()


def _transcode_video(
    *,
    input_file: Path,
    output_file: Path,
    encoder: str = "h264",
    decoder: str = "h264",
    bitrate: str = "24M",
) -> None:
    """
    Transcodes an extracted video.

    Args:
        input_file: The raw extracted video.
        output_file: The desired output file.
        encoder: The FFmpeg encoder to use.
        decoder: The FFmpeg decoder to use.
        bitrate: The bitrate to output transcoded videos at.

    """
    logger.info("Transcoding {} to {}...", input_file, output_file)
    ffmpeg = (
        FFmpeg()
        .input(input_file.as_posix(), {"c:v": decoder, "framerate": 24})
        .output(output_file.as_posix(), {"c:v": encoder, "b:v": bitrate})
    )

    @ffmpeg.on("stderr")
    def _on_stderr(line: str) -> None:
        # Log this for debugging.
        logger.debug("ffmpeg: {}", line)

    ffmpeg.execute()


def _on_program_exit(launcher: roslaunch.parent.ROSLaunchParent, *_: Any) -> None:
    """
    Handler that should run when the program exits, and cleans everything up.

    Args:
        launcher: The ROS launcher that is running.

    """
    logger.info("Got exit signal, stopping ROS...")
    launcher.shutdown()

    sys.exit()


def _process_bag(*, bag_file: Path, output_base: Path, **ffmpeg_kwargs: Any) -> None:
    """
    Processes a bagfile, extracting and transcoding each video.

    Args:
        bag_file: The input bagfile.
        output_base: The base name for the outputs. "_cam0", "_cam1", etc. will be
            tacked onto the end for each individual camera video.
        **ffmpeg_kwargs: Will be forwarded to `_transcode_video`.

    """
    logger.info("Extracting videos from bagfile {}...", bag_file)

    # Extract raw videos to temporary files.
    with TemporaryDirectory() as video_dir:
        video_dir = Path(video_dir)
        logger.debug("Using temporary video directory {}.", video_dir)

        _split_bag(bag_file=bag_file, output_dir=video_dir)

        # Transcode those videos.
        for i, video_file in enumerate(sorted(video_dir.glob("*.h265"))):
            output_file = output_base.parent / f"{output_base.name}_cam{i}.mp4"
            _transcode_video(
                input_file=video_file, output_file=output_file, **ffmpeg_kwargs
            )

        # Copy timestamps as well.
        for i, ts_file in enumerate(sorted(video_dir.glob("*.txt"))):
            output_file = output_base.parent / f"{output_base.name}_cam{i}_ts.txt"
            ts_file.rename(output_file)


def _make_parser() -> argparse.ArgumentParser:
    """
    Returns:
        A parser for command line arguments.

    """
    parser = argparse.ArgumentParser(description="Extracts videos from rosbags.")

    parser.add_argument(
        "bag_file", help="The bagfile to extract videos from.", type=Path
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="The base path to write output files to.",
        default="./video",
    )

    parser.add_argument(
        "-e", "--encoder", help="The encoder to use for FFmpeg.", default="h264"
    )
    parser.add_argument(
        "-d", "--decoder", help="The decoder to use for FFmpeg.", default="h264"
    )
    parser.add_argument(
        "-b",
        "--bitrate",
        help="The target bitrate to use for transcoded videos.",
        default="24M",
    )

    return parser


def main() -> None:
    parser = _make_parser()
    cli_args = parser.parse_args()

    _process_bag(
        bag_file=cli_args.bag_file,
        output_base=cli_args.output,
        encoder=cli_args.encoder,
        decoder=cli_args.decoder,
        bitrate=cli_args.bitrate,
    )


if __name__ == "__main__":
    main()
