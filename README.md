# ROS 2 Mecanum Bot

This repository contains the ROS 2 packages for a mecanum wheel robot, along with a complete Docker-based development and deployment environment.

## Table of Contents

- [ROS 2 Mecanum Bot](#ros-2-mecanum-bot)
  - [Table of Contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Project Structure](#project-structure)
  - [Getting Started](#getting-started)
  - [Usage](#usage)
    - [Simulation Environment](#simulation-environment)
    - [Robot Deployment](#robot-deployment)
    - [Station (Monitoring)](#station-monitoring)
  - [Development Workflow](#development-workflow)

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
├── docker/                 # Dockerfiles and scripts
│   ├── robot_humble/
│   │   └── Dockerfile
│   ├── robot_jazzy/
│   │   └── Dockerfile
│   ├── station/
│   │   └── Dockerfile
│   └── scripts/
│       └── entrypoint.sh
├── src/                    # ROS 2 source code
├── worlds/                 # Gazebo worlds
└── README.md
```

## Getting Started

When you first launch one of the environments, Docker Compose will build the necessary images. The initial build may take some time as it downloads the base ROS images and installs dependencies.

Subsequent launches will be much faster. The ROS 2 workspace is built automatically every time a container starts.

## Usage

### Simulation Environment

The simulation environment launches Gazebo, the robot nodes (both Humble and Jazzy), and the station tools (RViz, RQT).

**Launch the simulation:**

```bash
#docker compose -f compose/simulation.yml up --build
docker build -t robot_jazzy -f docker/robot_jazzy/Dockerfile .
docker build -t robot_humble -f docker/robot_humble/Dockerfile .
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

## Development Workflow

The `src` directory is mounted into the containers, so you can edit the code on your host machine and the changes will be reflected inside the containers. To rebuild the workspace after making changes, simply restart the containers.

```bash
sudo docker compose -f compose/simulation.yml restart
```
