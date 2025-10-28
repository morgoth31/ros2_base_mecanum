# ROS 2 Mecanum Bot

This repository contains the ROS 2 packages for a mecanum wheel robot, along with a complete Docker-based development and deployment environment.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Building the Docker Images](#building-the-docker-images)
- [Usage](#usage)
  - [Simulation Environment](#simulation-environment)
  - [Robot Deployment](#robot-deployment)
  - [Station (Monitoring)](#station-monitoring)
- [Testing](#testing)

## Prerequisites

- Docker Engine
- Docker Compose
- (Optional) NVIDIA Container Toolkit for GPU acceleration in RViz and Gazebo.

## Project Structure

```
.
├── compose/                # Docker Compose files
│   ├── simulation.yml
│   ├── robot.yml
│   └── station.yml
├── docker/                 # Dockerfiles
│   ├── robot_humble/
│   │   └── Dockerfile
│   ├── robot_jazzy/
│   │   └── Dockerfile
│   └── station/
│       └── Dockerfile
├── src/                    # ROS 2 source code
├── worlds/                 # Gazebo worlds
└── README.md
```

## Getting Started

### Building the Docker Images

You can build all the images at once or individually.

**Build all images:**

```bash
sudo docker compose -f compose/simulation.yml build
```

**Build a specific image:**

```bash
# Build the Humble robot image
sudo docker build -t robot_humble -f docker/robot_humble/Dockerfile .

# Build the Jazzy robot image
sudo docker build -t robot_jazzy -f docker/robot_jazzy/Dockerfile .

# Build the station image
sudo docker build -t station -f docker/station/Dockerfile .
```

## Usage

### Simulation Environment

The simulation environment launches Gazebo, the robot nodes (both Humble and Jazzy), and the station tools (RViz, RQT).

**Launch the simulation:**

```bash
sudo docker compose -f compose/simulation.yml up --build
```

**Verification:**

- All services (gazebo, robot_nodes_humble, robot_nodes_jazzy, station_tools) should start without errors.
- To verify communication, open a new terminal and list the ROS 2 topics:
  ```bash
  ros2 topic list
  ```
  You should see topics from Gazebo and the robot nodes.

### Robot Deployment

This launches the ROS 2 nodes for the physical robot.

**Launch the robot nodes:**

```bash
sudo docker compose -f compose/robot.yml up --build
```

**Verification:**

On the robot, you can check the running Docker containers and view the logs to ensure the nodes are active.

```bash
sudo docker ps
sudo docker logs <container_name>
```

### Station (Monitoring)

This launches the station tools (RViz, RQT) to monitor the robot.

**Launch the station tools:**

```bash
sudo docker compose -f compose/station.yml up --build
```

**Verification:**

- After launching `robot.yml` on the robot and `station.yml` on the station (on the same network and with the same `ROS_DOMAIN_ID`), RViz should open.
- If you add a display for a topic like `/odom`, you should see the data being published by the robot.

## Testing

Each Dockerfile builds the ROS 2 workspace and runs tests. You can also run the tests manually inside a running container.

```bash
# Get the ID of the container
sudo docker ps

# Exec into the container
sudo docker exec -it <container_id> bash

# Run tests
source /ros_ws/install/setup.bash
colcon test
```
