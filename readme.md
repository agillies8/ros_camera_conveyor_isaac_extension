More notes on this project at:

https://loud-podium-a0b.notion.site/ROS2-Isaac-Sim-Camera-Conveyor-Sorter-121c986e685f80549003d6d6afc4863f?pvs=74

Quickstart using docker:
```
xhost +local:docker && docker compose up
```

RUNNING STACK ON SINGLE MACHINE (No Compression):

0. Before starting a few key settings to make sure of:
    - The compression container in the main docker compose file is commented out
    - The yolo containers are subscribed directly to the isaac sim cam topics, eg:
    ```
    ros2 launch yolo_bringup yolo.launch.py model:=src/models/best640.pt input_image_topic:=/camera_01/rgb threshold:=0.7 imgsz_height:=480 namespace:=kicker_01
    ```
    - network mode: host is commented out on all containers

1. Start Isaac Sim:
    - Isaac sim will start automatically from winth a docker container
    - Add the "Isaac Cam Conveyor" extension with the isaac extension manager if isnt done so already
    - Also make sure the conveyor extensions are loaded. You need the 0.4.0 and 1.1.0 old versions before isaac 4.5
    - Click on the extension and click "load"
    - Once the secne is loaded, click "run" (Dont hit play in the sidebar)

2. Start the Ros and YOLO stack with docker compose:
    - From the repo root, enter:
    ```
    xhost +local:docker && docker compose up
    ```
    - You should see rviz spin up and eventually load the kicker URDFs

RUNNING STACK ON TWO MACHINES (with Compression):

0. Before starting a few key settings to make sure of:
    - The compression container in the main docker compose file is uncommented
    - The yolo containers are subscribed to the uncompressed image topics, eg:
    ```
    ros2 launch yolo_bringup yolo.launch.py model:=src/models/best640.pt input_image_topic:=/camera/image_raw/uncompressed threshold:=0.7 imgsz_height:=480 namespace:=kicker
    ```
    - network mode: host is commented out on all containers

1. Start Isaac Sim:
    - Add the "Isaac Cam Conveyor" extension with the isaac extension manager
    - Click on the extension and click "load"
    - Once the secne is loaded, click "run" (Dont hit play in the sidebar)

2. Start compression on the Isaac Sim machine:
    - To run compression on all the streams from a single terminal without docker, go to the /compression_container dir and run:
    ```
    MODE=compress ./entrypoint.sh
    ```
    - OR, you can compress each stream individually. replace '/camera/rgb' with the topic you want to compress. You can run this in different terminals to compress multiple streams in parallel (needs ffmpeg and ros installed locally, I wasnt ever able to get this to work from inside a docker container):

    ```
    ros2 run image_transport republish raw in:=/camera/rgb --ros-args -p "ffmpeg_image_transport.encoding:=hevc_nvenc" --namespace cam_compressed/
    ```

3. Start decompression
    - This should happen automatically if you uncommented the decompression container service
    - Alternatively, you can do it manually. On the reciever machine, outside docker (assuming ros and ffmpeg is installed), you can run:
    ```
    ros2 run image_transport republish ffmpeg in/ffmpeg:=out/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
    ```
4. Start the Ros and YOLO stack with docker compose:
    - From the repo root, enter:
    ```
    xhost +local:docker && docker compose up
    ```
    - You should see rviz spin up and eventually load the kicker URDFs

Other useful commands or tidbits:

To join main ros service from another terminal:

    ```
    docker exec -it isaac_cam_conveyor /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && exec bash"
    ```
Starting the yolo_ros container independently:

- If you want to run the yolo_ros container on its own, go to the yolo_ros dir and run:
    ```
    docker run -it --rm --gpus all yolo_ros
    ```
    Then once inside, start yolo:
    ```
    ros2 launch yolo_bringup yolo.launch.py model:=src/models/best640.pt input_image_topic:=/camera/rgb threshold:=0.7 imgsz_height:=480
    ```

First time setup:

YOLO ROS:
- You need to get it here: https://github.com/mgonzs13/yolo_ros
    ```
    git clone https://github.com/mgonzs13/yolo_ros.git
    cd yolo_ros
    docker build -t yolo_ros .
    ```
