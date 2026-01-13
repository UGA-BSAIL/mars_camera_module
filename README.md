# ROS2 package with messages for the ffmpeg_image_transport

This package has definitions for ROS2 messages of the
[ffmpeg image transport](https://github.com/ros-misc-utilities/ffmpeg_image_transport).
See the [ffmpeg\_encoder\_decoder](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder) package
for more details about the encoding and decoding process, and the meaning
of the ``encoding`` message field.

The ``encoding`` message field is a string with semicolon-separated tokens that
indicate the codec (first token) and the image formats used at various stages of the
encoding process: ``codec;av_source_pixel_format;cv_bridge_target_format;ros_encoding``.

- ``ros_encoding`` is the pixel format used by the original ROS message that was passed to
   the ``ffmpeg_encoder_decoder`` module, e.g. ``bgr8``. This token allows the decoder later to restore
   the original message format if desired.
- ``cv_bridge_target_format`` is the pixel format that the cv\_bridge converts
  the ``ros_encoding`` into. This is typically a format that is more suited for ffmpeg compression.
- ``av_source_pixel_format`` is the pixel format that libswscale converts the ``cv_bridge_target_format``
  into. This token uses libav convention (e.g. ``yuv420p``), and denotes the pixel format
  that is fed into the libav encoder.
- ``codec`` is the codec used for ultimately encoding the packet, e.g. ``h264``, ``hevc`` etc.
  Follows libav convention.

The encoding process is best understood by looking at the [ffmpeg\_encoder\_decoder documentation](https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder).

## License
This package is released under the [Apache-2 license](LICENSE).
