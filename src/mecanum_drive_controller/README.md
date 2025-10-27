# Mecanum Drive Controller

This folder contains the source code for the `mecanum_drive_controller`, a ROS 2 package that provides a controller for a mecanum wheel robot.

## Purpose

The `mecanum_drive_controller` is responsible for taking `geometry_msgs/Twist` messages as input and converting them into wheel velocity commands for a mecanum drive robot. It also publishes odometry information.

## Key Files

- `src/mecanum_drive_controller.cpp`: The main implementation of the controller.
- `include/mecanum_drive_controller/mecanum_drive_controller.hpp`: The header file for the controller.
- `mecanum_drive_controller.xml`: The controller plugin description file.

## Adaptation

To adapt this folder to your needs, you would typically not need to modify the source code. Instead, you would configure the controller through the `ros2_controllers.yaml` file in your robot's description package. This is where you would set the joint names, wheel parameters, and other settings.
