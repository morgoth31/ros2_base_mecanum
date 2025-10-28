#!/bin/bash
# Permet aux conteneurs Docker d'accéder au serveur X de l'hôte
xhost +local:docker

docker compose -f compose/station.yml up -d

docker exec -it ros_station bash
