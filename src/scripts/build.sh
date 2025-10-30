#!/bin/bash
set -e
# Install dependencies and build the workspace
if [ -d "/ros_ws/src" ]; then
    rosdep init || true
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install
    source /ros_ws/install/setup.bash
fi
