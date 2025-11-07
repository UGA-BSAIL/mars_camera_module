#!/usr/bin/env python


"""
This script handles extracting saved videos from rosbag files.
"""

from functools import partial
from pathlib import Path
from typing import Optional, Dict, Callable, Any, List
import signal
import sys
from tempfile import TemporaryDirectory
import argparse
import shutil
import roslaunch
from ffmpeg import FFmpeg, FFmpegError, Progress
from loguru import logger
import bagpy
import pandas as pd
from pandas.errors import EmptyDataError

from .launch_manager import LaunchManager


SPLIT_BAG_LAUNCH = ("ffmpeg_image_transport_tools", "split_bag_mars.launch")
"""
Launch file to use for extracting video data from the bag files.
"""


def _split_bag(
    *, bag_file: Path, output_dir: Path, handle_signals: bool = True
) -> None:
    """
    Splits the video data out of the bag file.

    Args:
        bag_file: The bag file to split data from.
        output_dir: The directory to write output to. Output will be
            written to files named "camera_x" for each camera.
        handle_signals: If true, add handlers for SIGTERM and SIGINT so that
            it remains responsive to the user.

    """
    out_file_base = output_dir / "camera_"

    # Start the node.
    launcher = LaunchManager(
        SPLIT_BAG_LAUNCH,
        ros_args=[
            f"bag:={bag_file.absolute().as_posix()}",
            f"out_file_base:={out_file_base.absolute().as_posix()}",
            "write_time_stamps:=true",
        ],
    )
    launcher.start()

    # Run a handler when we exit to stop ROS.
    if handle_signals:
        handler = partial(_on_program_exit, launcher)
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)

    # Wait for it to finish.
    launcher.wait()


def _find_video_length(video_path: Path) -> int:
    """
    Finds the number of frames in a video file.

    Args:
        video_path: The video to analyze.

    Returns:
        The number of frames.

    """
    # Find the timestamps file.
    timestamp_path = video_path.parent / f"{video_path.stem}_ts.txt"
    logger.debug("Reading data from timestamp file: {}", timestamp_path)
    try:
        timestamp_data = pd.read_csv(timestamp_path, sep=" ", header=None)
    except EmptyDataError:
        logger.warning("No timestamp data found in {}", timestamp_path)
        # Return some really big number here. Since this is in the
        # denominator for progress calculations, it will effectively always
        # make it look like we have 0 progress.
        return sys.maxsize

    return timestamp_data.index[-1]


def _transcode_video(
    *,
    input_file: Path,
    output_file: Path,
    ffmpeg_exe: Optional[Path] = None,
    encoder: str = "h264",
    decoder: str = "h264",
    bitrate: str = "24M",
    on_progress: Optional[Callable[[float], None]],
) -> None:
    """
    Transcodes an extracted video.

    Args:
        input_file: The raw extracted video.
        output_file: The desired output file.
        ffmpeg_exe: Specify an executable to use for FFMpeg. Otherwise, it
            will use the default one.
        encoder: The FFmpeg encoder to use.
        decoder: The FFmpeg decoder to use.
        bitrate: The bitrate to output transcoded videos at.
        on_progress: Callback to run whenever we make progress in the
            transcode. Will be called with the current fractional completion.

    """
    # Figure out how many frames there are in the video.
    num_frames = _find_video_length(input_file)

    logger.info("Transcoding {} to {}...", input_file, output_file)
    ffmpeg = (
        FFmpeg(ffmpeg_exe.as_posix() if ffmpeg_exe else "ffmpeg")
        .input(input_file.as_posix(), {"c:v": decoder, "r": 24})
        .output(
            output_file.as_posix(),
            {
                "c:v": encoder,
                "b:v": bitrate,
                "movflags": "+faststart",
                # This magic filter detects videos that are recorded in
                # portrait (such as from the stereo cameras),
                # and automatically rotates them. Landscape videos are not
                # affected.
                "vf": "rotate='if(lt(iw,ih),-PI/2,0)':ow='max(iw,ih)':oh='min(iw,ih)'",
            },
        )
    )

    @ffmpeg.on("stderr")
    def _on_stderr(line: str) -> None:
        # Log this for debugging.
        logger.debug("ffmpeg: {}", line)

    @ffmpeg.on("progress")
    def _on_progress(progress: Progress) -> None:
        if not on_progress:
            return

        # Run the callback.
        fraction_done = progress.frame / num_frames
        on_progress(fraction_done)

    ffmpeg.execute()


def _extract_topics(
    bag_file: Path, *, topics: List[str], output_base: Path
) -> None:
    """
    Extracts additional topics from the bag file as CSV files.

    Args:
        bag_file: The bag file to read from.
        topics: The topics to extract.
        output_base: The output directory to write CSV files to.

    """
    reader = bagpy.bagreader(bag_file.as_posix(), tmp=True)

    for topic in topics:
        logger.info("Extracting data for {}...", topic)
        topic_path = reader.message_by_topic(topic)

        output_dir = output_base.parent
        output_path = (
            output_dir / f"{output_base.name}{topic.replace('/', '_')}.csv"
        )
        shutil.move(topic_path, output_path)


def _on_program_exit(
    launcher: roslaunch.parent.ROSLaunchParent, *_: Any
) -> None:
    """
    Handler that should run when the program exits, and cleans everything up.

    Args:
        launcher: The ROS launcher that is running.

    """
    logger.info("Got exit signal, stopping ROS...")
    launcher.shutdown()

    sys.exit()


def process_bag(
    *,
    bag_file: Path,
    output_base: Path,
    on_progress: Optional[Callable[[float], None]] = None,
    handle_signals: bool = True,
    decoders: Optional[Dict[str, str]] = None,
    **ffmpeg_kwargs: Any,
) -> List[Path]:
    """
    Processes a bagfile, extracting and transcoding each video.

    Args:
        bag_file: The input bagfile.
        output_base: The base name for the outputs. "_cam0", "_cam1", etc. will be
            tacked onto the end for each individual camera video.
        on_progress: Callback to run whenever we make progress in the
            transcode. Will be called with the current fractional completion.
        handle_signals: If true, add handlers for SIGTERM and SIGINT so that
            it remains responsive to the user.
        decoders: Maps video file extensions to the FFMPEG decoders to use
            for them.
        **ffmpeg_kwargs: Will be forwarded to `_transcode_video`.

    Returns:
        The paths to the video files it extracted.

    """
    if decoders is None:
        decoders = {
            ".h264": "h264_nvv4l2dec",
            ".h265": "hevc_nvv4l2dec",
            ".mjpeg": "mjpeg",
        }

    logger.info("Extracting videos from bagfile {}...", bag_file)

    # Extract raw videos to temporary files.
    video_output_files = []
    with TemporaryDirectory(
        dir=output_base.parent, prefix=".videos_"
    ) as video_dir:
        video_dir = Path(video_dir)
        logger.debug("Using temporary video directory {}.", video_dir)

        _split_bag(
            bag_file=bag_file,
            output_dir=video_dir,
            handle_signals=handle_signals,
        )

        # Find extracted video files.
        video_files = set(video_dir.glob("camera_*"))
        # Ignore timestamps.
        video_files -= set(video_dir.glob("*.txt"))
        video_files = sorted(video_files)

        def _on_progress(fraction_done: float, video_index: int) -> None:
            if on_progress is None:
                return
            # We have to translate per-video progress into global progress.
            previous_video_fraction = 1.0 / len(video_files) * video_index
            fraction_done = previous_video_fraction + fraction_done / len(
                video_files
            )
            on_progress(fraction_done)

        # Transcode those videos.
        for i, video_file in enumerate(video_files):
            output_file = output_base.parent / f"{output_base.name}_cam{i}.mp4"
            video_output_files.append(output_file)
            decoder = decoders[video_file.suffix]
            try:
                _transcode_video(
                    input_file=video_file,
                    output_file=output_file,
                    on_progress=partial(_on_progress, i),
                    decoder=decoder,
                    **ffmpeg_kwargs,
                )
            except FFmpegError as err:
                logger.error("FFMPeg failed, skipping: {}", err)
                continue

        # Copy timestamps as well.
        for i, ts_file in enumerate(sorted(video_dir.glob("*.txt"))):
            output_file = (
                output_base.parent / f"{output_base.name}_cam{i}_ts.txt"
            )
            shutil.copyfile(ts_file, output_file)

        return video_output_files


def _make_parser() -> argparse.ArgumentParser:
    """
    Returns:
        A parser for command line arguments.

    """
    parser = argparse.ArgumentParser(
        description="Extracts videos from rosbags."
    )

    parser.add_argument(
        "bag_file", help="The bagfile to extract videos from.", type=Path
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="The base path to write output files to.",
        default="/host/video",
    )

    parser.add_argument(
        "-e",
        "--encoder",
        help="The encoder to use for FFmpeg.",
        default="h264",
    )
    parser.add_argument(
        "-b",
        "--bitrate",
        help="The target bitrate to use for transcoded videos.",
        default="24M",
    )

    parser.add_argument(
        "-t",
        "--extra-topics",
        nargs="+",
        default=["/gps1/fix", "/gps2/fix"],
        help="Additional topics to extract CSV data for.",
    )

    return parser


def main() -> None:
    parser = _make_parser()
    cli_args = parser.parse_args()

    process_bag(
        bag_file=cli_args.bag_file,
        output_base=cli_args.output,
        encoder=cli_args.encoder,
        bitrate=cli_args.bitrate,
    )

    if cli_args.extra_topics:
        # Extract extra topics.
        _extract_topics(
            cli_args.bag_file,
            topics=cli_args.extra_topics,
            output_base=cli_args.output,
        )


if __name__ == "__main__":
    main()
