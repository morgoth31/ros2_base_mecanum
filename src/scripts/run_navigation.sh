#!/bin/bash
set -e
# Install dependencies and build the workspace
if [ -d "/ros_ws/src" ]; then
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install
    source /ros_ws/install/setup.bash
fi

ros2 launch yahboom_rosmaster_bringup rosmaster_x3_navigation.launch.py