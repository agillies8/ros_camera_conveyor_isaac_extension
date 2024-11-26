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

- Run this script on the sender machine, replacing raw in with the topic you want to compress. You can run this in different terminals to compress multiple streams in parallel:

```
ros2 run image_transport republish raw in:=/camera/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" --namespace cam_compressed/
```

- OR to run compression on all the streams from a single terminal without docker, go to the /compression_container dir and run:
```
MODE=compress ./entrypoint.sh
```

- On the reciever machine, outside docker, you can run:
```
ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
```
- You can also just run the docker compose file in the root dir, and uncomment the section that spins up the decoder container.



Starting the yolo_ros container independently:
- If you want to run the yolo_ros container on its own, go to the yolo_ros dir and run:
```
docker run -it --rm --gpus all yolo_ros

ros2 launch yolo_bringup yolo.launch.py model:=src/models/best640.pt input_image_topic:=/camera/rgb threshold:=0.7 imgsz_height:=480
```

