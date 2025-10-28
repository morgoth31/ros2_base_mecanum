#!/bin/bash
set -e

# Source ROS setup script
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "ROS setup script not found!"
    exit 1
fi

# Install dependencies and build the workspace
if [ -d "/ros_ws/src" ]; then
    rosdep init || true
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install
    source /ros_ws/install/setup.bash
fi

# Execute the command passed to the container
exec "$@"
