# !/bin/bash
# Build all docker images for the ROS2 Mecanum project
# Usage: ./build_docker_compose.sh
# Make sure you have Docker and Docker Compose installed
# Run this script from the root directory of the project

docker build -t robot_jazzy -f docker/robot_jazzy/Dockerfile .
docker build -t robot_humble -f docker/robot_humble/Dockerfile .
docker build -t robot_humble -f docker/robot_humble/Dockerfile .

#docker compose -f compose/robot.yml up -d
#docker compose -f compose/station.yml up
#docker compose -f compose/simulation.yml
