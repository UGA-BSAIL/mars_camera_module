#!/bin/bash

./configure --enable-shared --prefix="$1/external" --enable-nvv4l2dec --enable-libv4l2 --extra-libs="-L/usr/lib/aarch64-linux-gnu/tegra -lnvbuf_utils -lv4l2" --extra-cflags="-I /usr/src/jetson_multimedia_api/include/"
