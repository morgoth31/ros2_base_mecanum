# Yahboom Rosmaster Bringup

This folder contains the launch files for bringing up the Yahboom Rosmaster robot.

## Purpose

The `yahboom_rosmaster_bringup` package is responsible for starting the robot's drivers and other essential nodes. It contains the top-level launch files for bringing up the real robot and the simulation.

## Key Files

- `launch/load_ros2_controllers.launch.py`: The launch file for loading the ROS 2 controllers.
- `launch/rosmaster_x3_navigation.launch.py`: The launch file for starting the navigation stack.

## Adaptation

To adapt this folder to your needs, you would typically modify the launch files to change the robot's startup behavior. For example, you might add or remove nodes, or change the parameters of the existing nodes.
