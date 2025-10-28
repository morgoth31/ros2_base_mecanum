# !/bin/bash
# Build all docker images for the ROS2 Mecanum project
# Usage: ./build_docker_compose.sh
# Make sure you have Docker and Docker Compose installed
# Run this script from the root directory of the project

docker compose -f compose/robot.yml build
docker compose -f compose/station.yml build
docker compose -f compose/simulation.yml build
