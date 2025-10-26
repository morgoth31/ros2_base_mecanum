# ros2-mecanum-bot
ROS2 Mecanum wheel robot

## Getting started

#### Prerequisites
This project is build and tested on Ubuntu 22.04 LTS with ROS 2 Humble LTS. But is mean to run in a docker 

#### Setup workspace

```
./run_station.sh
```
This function launch a docker container 

#### Install dependencies
```
apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

#### Build and run
```
colcon build --symlink-install
source install/setup.bash
ros2 launch mecanumbot_bringup mecanumbot_hardware.py
```

#### Visualize the robot

```
cd ~/workspaces/ros2-mecanum-bot
source install/setup.bash
ros2 launch mecanumbot_bringup rviz2.py
```
