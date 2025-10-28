# Use ros:jazzy-ros-base as the base image
FROM ros:jazzy-ros-base

# Set the environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Install common system dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Create a ROS 2 workspace
WORKDIR /ros_ws

# Default command to keep the container running
CMD ["bash"]
