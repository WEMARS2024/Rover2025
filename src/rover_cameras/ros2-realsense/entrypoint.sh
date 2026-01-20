#!/usr/bin/env bash

# launch tmux, then load different ros2 processess into diff panes

set -e

source /opt/ros/humble/setup.bash

SESSION=ros

tmux new-session -d -s $SESSION

# realsense cam package
tmux send-keys -t $SESSION:0 \
  "ros2 launch realsense2_camera rs_launch.py" C-m
  # "ros2 launch realsense2_camera rs_launch.py enable_colorizer:=true" C-m
tmux rename-window realsense_camera

# image_proc package
tmux new-window -t $SESSION -n image_proc
tmux send-keys -t $SESSION:image_proc \
  "ros2 run image_proc image_proc" C-m

# web_video_server package
tmux new-window -t $SESSION -n web_video
tmux send-keys -t $SESSION:web_video \
  "ros2 run web_video_server web_video_server" C-m

# list topics, free bash terminal
tmux new-window -t $SESSION -n topics
tmux send-keys -t $SESSION:topics \
  "ros2 topic list" C-m

exec tmux attach -t $SESSION
