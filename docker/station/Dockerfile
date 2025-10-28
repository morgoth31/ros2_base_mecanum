# Use ros:jazzy-desktop as the base image
FROM ros:jazzy-desktop

# Set the environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Install additional tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-rqt* \
    terminator \
    python3-scipy \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# GPU Support (commented out for testing in environments without a GPU)
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Create a ROS 2 workspace
WORKDIR /ros_ws

# Default command to keep the container running
CMD ["bash"]
