More notes on this project at:

Quickstart using docker:
```
xhost +local:docker && docker compose up
```

and join from another term:
```
docker exec -it isaac_cam_conveyor /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"
```

Dealing with image compression:

- Insall image transport packages on all machines:

```
sudo apt install ros-<ros-distro>-image-transport
sudo apt install ros-<ros-distro>-compressed-image-transport
sudo apt-get install ros-${ROS_DISTRO}-ffmpeg-image-transport
```

- Run this node on the sender machine:

```
ros2 run image_transport republish raw in:=/camera/rgb
```

- Run this node on the reciever machine:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
```

Or play the bag file:
```
ros2 bag play /rosbags/box_move --loop
```