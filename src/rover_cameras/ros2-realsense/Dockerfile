FROM ros:humble-ros-base

RUN apt update && apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-web-video-server \
    ros-humble-rosbridge-server \
    ros-humble-image-proc \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# SHELL ["/bin/bash", "-c"]
#
# CMD source /opt/ros/humble/setup.bash && \
#      ros2 launch realsense2_camera rs_launch.py enable_colorizer:=true & \
# #    ros2 launch realsense2_camera rs_launch.py & \
#      ros2 run web_video_server web_video_server & \
#      ros2 run image_proc image_proc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# CMD bash
