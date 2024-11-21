#!/bin/bash
set -e


if [ "$MODE" = "compress" ]; then
    # Start ROS2 nodes concurrently
    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish raw in:=/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" -r __ns:=/camera &

    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish raw in:=/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" -r __ns:=/camera_01 &

    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish raw in:=/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" -r __ns:=/camera_02 &

elif [ "$MODE" = "decompress" ]; then
    # Start ROS2 nodes concurrently
    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -r __ns:=/camera -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc" &

    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -r __ns:=/camera_01 -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc" &

    source /opt/ros/humble/setup.bash && \
    ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -r __ns:=/camera_02 -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc" &

else
    echo "Invalid MODE: $MODE. Use 'compress' or 'decompress'." &
    #exit 1
fi

# Keep the container running
wait
