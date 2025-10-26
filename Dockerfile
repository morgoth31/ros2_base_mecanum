# 1. Base sur l'image ROS existante
FROM osrf/ros:humble-desktop-full

# 2. Définition des variables d'environnement par défaut
# Ces valeurs par défaut seront écrasées par l'exécution du compose, mais sont utiles pour le build.
ARG USER_UID=1000
ARG USER_GID=1000

# 3. Définition du nom d'utilisateur standard dans le conteneur
ENV USER_NAME=ros

# 4. Exécution des commandes en tant que root
USER root

# 5. Création du groupe et de l'utilisateur avec l'UID/GID spécifié
# - g${USER_GID} : Crée le groupe s'il n'existe pas.
# - ${USER_NAME} : Nom de l'utilisateur
# - -u ${USER_UID} : Définit l'UID
# - -g ${USER_GID} : Définit le GID principal
# - -m : Crée le répertoire home
# - -s /bin/bash : Définit le shell par défaut
# Note : Nous utilisons le nom 'ros' qui correspond au HOME défini dans docker-compose.yml
RUN groupadd -g ${USER_GID} ${USER_NAME} || true \
    && useradd -u ${USER_UID} -g ${USER_GID} -m -s /bin/bash ${USER_NAME} \
    # 6. Ajout de l'utilisateur aux groupes nécessaires (ex: sudo, dialout, video si besoin)
    && usermod -aG sudo ${USER_NAME} \
    # 7. Donner la permission à l'utilisateur de s'exécuter dans le répertoire de travail
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

# 8. Bascule vers l'utilisateur créé pour les commandes suivantes (non nécessaire ici, mais bonne pratique)
# USER ${USER_NAME}