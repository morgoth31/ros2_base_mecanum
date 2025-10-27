# Yahboom Rosmaster

This folder contains the `yahboom_rosmaster` metapackage, which groups together all the packages related to the Yahboom Rosmaster robot.

## Purpose

The `yahboom_rosmaster` metapackage simplifies the process of building and running the Yahboom Rosmaster robot by providing a single entry point to all the necessary packages. It also contains the top-level launch files for bringing up the robot.

## Key Files

- `package.xml`: The package manifest for the metapackage.
- `CMakeLists.txt`: The CMake build file for the metapackage.

## Adaptation

To adapt this folder to your needs, you would typically add or remove dependencies in the `package.xml` file to match the packages you are using. You might also modify the launch files to change the robot's startup behavior.
