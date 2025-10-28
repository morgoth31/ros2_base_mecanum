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
    && rm -rf /var/lib/apt/lists/*

# Copy and compile the src workspace (for custom messages)
WORKDIR /ros_ws
COPY src /ros_ws/src
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# GPU Support (commented out for testing in environments without a GPU)
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Configure the CMD or ENTRYPOINT to automatically source /ros_ws/install/setup.bash
CMD ["/bin/bash", "-c", "source /ros_ws/install/setup.bash && /bin/bash"]
