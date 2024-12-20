ARG ROS_DISTRO=humble
 
########################################
# Base Image for ISAAC CAMERA EXTENSION #
########################################
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
 
# Create Colcon workspace with external dependencies
RUN mkdir -p /ros2_ws/src
 
# Build the base Colcon workspace, installing dependencies first.
WORKDIR /ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

RUN sudo apt-get upgrade -y
RUN sudo apt install ros-humble-joint-state-publisher && \
sudo apt install ros-humble-joint-state-publisher-gui
RUN sudo apt-get install ros-humble-image-transport && \
sudo apt-get install ros-humble-compressed-image-transport
RUN sudo apt-get install -y ros-humble-ffmpeg-image-transport
RUN source /opt/ros/humble/setup.bash