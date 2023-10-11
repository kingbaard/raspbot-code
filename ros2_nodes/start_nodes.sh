#!/bin/bash
cd /root/ros2_ws
colcon build --symlink-install
tmux \
    new-session 'ros2 run in_class motor' \; \
    new-window 'ros2 run ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"' \