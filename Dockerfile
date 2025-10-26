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


RUN groupadd -g ${USER_GID} ${USER_NAME} 2>/dev/null || groupadd ${USER_NAME} \
    \
    # Créer l'utilisateur avec l'UID de l'hôte et le groupe associé
    && useradd -u ${USER_UID} -g ${USER_NAME} -m -s /bin/bash ${USER_NAME} \
    \
    # Ajouter l'utilisateur aux groupes nécessaires (sudo, video pour le GPU, etc.)
    # Note : L'image de base doit avoir le paquet `sudo` installé.
    && usermod -aG sudo,video ${USER_NAME} \
    \
    # **Règle Critique : Permettre l'exécution de sudo sans mot de passe (NOPASSWD)**
    # Crée un fichier de configuration pour sudoers.
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/99-${USER_NAME}-nopasswd \
    \
    #  S'assurer que le répertoire home est correctement possédé (pour les fichiers de config)
    && chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

#  Bascule vers l'utilisateur créé pour les commandes suivantes
# USER ${USER_NAME}