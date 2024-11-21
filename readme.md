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

- Run this node on the sender machine:

```
ros2 run image_transport republish raw in:=/camera/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" --namespace cam_compressed/
```

- Run this node on the reciever machine:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
```

Or play the bag file:
```
ros2 bag play /rosbags/box_move --loop
```

or start the yolo_ros container independently:
```
docker run -it --rm --gpus all yolo_ros

ros2 launch yolo_bringup yolo.launch.py model:=src/models/best640.pt input_image_topic:=/camera/rgb threshold:=0.7 imgsz_height:=480
```