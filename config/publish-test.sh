#!/bin/bash
exec gst-launch-1.0 -q \
  videotestsrc is-live=true pattern=smpte \
  ! video/x-raw,width=1280,height=720,framerate=30/1,format=I420 \
  ! timeoverlay halignment=center valignment=bottom font-desc="Sans 28" \
  ! queue max-size-buffers=2 leaky=downstream \
  ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=3000 \
            key-int-max=30 sliced-threads=true byte-stream=true \
  ! h264parse config-interval=-1 \
  ! rtspclientsink location=rtsp://127.0.0.1:8554/${MTX_PATH:-test} protocols=tcp latency=0
