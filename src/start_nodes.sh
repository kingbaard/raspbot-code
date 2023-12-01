#!/bin/bash
cd /root/ros2_ws
colcon build --symlink-install
tmux \
    new-session 'ros2 run in_class motor' \; \
    new-window 'ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[640,480]"' \
    new-window 'ros2 run raspbot apriltag' \
    new-window 'ros2 run raspbot motors' \
    new-window 'ros2 run raspbot sonar' \
    new-window 'ros2 run raspbot keyboard' \
    new-window 'ros2 topic pub /warehouse_control std_msgs/msg/Bool "{ data: True }"'