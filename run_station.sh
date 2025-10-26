#!/bin/bash
# Permet aux conteneurs Docker d'accéder au serveur X de l'hôte
xhost +local:docker

# Fichier: .env
# Récupère automatiquement l'UID et le GID de l'utilisateur qui lance la commande

docker container kill ros_humble_desktop

docker compose up -d

docker exec -it ros_humble_desktop bash

#source /opt/ros/humble/setup.bash

#rviz2