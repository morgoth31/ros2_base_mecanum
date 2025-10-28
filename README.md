# ros2-mecanum-bot
ROS2 Mecanum wheel robot

## Getting started

#### Prerequisites
This project is build and tested on Ubuntu 22.04 LTS with ROS 2 Humble LTS. But is mean to run in a docker 

#### Setup workspace

```
# first time need to build
docker compose up --build -d

```
This function launch a docker container 

docker exec -it ros_humble_desktop bash


#### Install dependencies
```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

#### Build and run
```
source /opt/ros/humble/setup.sh
colcon build --symlink-install
source install/setup.bash
ros2 launch mecanumbot_bringup mecanumbot_hardware.py
```

#### Visualize the robot

```
docker exec -it ros_humble_desktop bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mecanumbot_bringup rviz2.py
```


## relaunch when already build


```
source /opt/ros/humble/setup.bash
source /home/ros/ros2_ws/install/setup.bash
ros2 launch mecanumbot_bringup mecanumbot_hardware.py
```