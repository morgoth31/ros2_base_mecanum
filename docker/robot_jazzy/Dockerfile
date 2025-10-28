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
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Create a ROS 2 workspace
WORKDIR /ros_ws

# Copy the entire src directory from the repository into /ros_ws/src
COPY src /ros_ws/src

# Install colcon-common-extensions and rosdep
RUN pip3 install -U colcon-common-extensions rosdep

# Initialize rosdep, then run it to install all ROS dependencies for the workspace
RUN rosdep init && rosdep update && rosdep install --from-paths src -y --ignore-src

# Build the workspace using colcon build --symlink-install
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Configure the CMD or ENTRYPOINT to automatically source /ros_ws/install/setup.bash
CMD ["/bin/bash", "-c", "source /ros_ws/install/setup.bash && /bin/bash"]
