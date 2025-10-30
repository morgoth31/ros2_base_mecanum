# yahboomcar_msgs

## Description

Ce paquet ROS2 définit un ensemble de types de messages (`.msg`) personnalisés utilisés par les différents nœuds du projet Yahboom car. Les paquets de messages sont fondamentaux dans ROS car ils permettent de créer des structures de données standardisées pour la communication entre les nœuds.

Le fait de définir ces messages dans un paquet séparé permet à de multiples autres paquets (comme `yahboomcar_mediapipe`, `yahboomcar_astra`, etc.) de les utiliser comme une dépendance commune.

## Fonctionnement

Ce paquet ne contient aucun code exécutable. Son seul rôle est de définir la structure de plusieurs messages. Lors de la compilation de l'espace de travail ROS2 (avec `colcon build`), les fichiers `.msg` sont utilisés pour générer automatiquement le code source correspondant en C++ et en Python.

Une fois le paquet compilé et l'environnement sourcé, les autres nœuds peuvent importer et utiliser ces types de messages comme n'importe quel autre type de message ROS2 standard (par exemple, `std_msgs/String` ou `geometry_msgs/Twist`).

## Messages Définis

Voici une description des messages contenus dans le répertoire `msg/` :

### `PointArray.msg`

-   **Structure** : `geometry_msgs/Point[] points`
-   **Description** : Représente une liste ou un tableau de points. Chaque point est de type `geometry_msgs/Point`, qui contient des coordonnées `x`, `y`, et `z`.
-   **Utilisation typique** : Utilisé par le paquet `yahboomcar_mediapipe` pour publier la liste de tous les points de repère (landmarks) détectés pour les mains, la pose, etc.

### `Position.msg`

-   **Structure** :
    -   `float32 anglex`
    -   `float32 angley`
    -   `float32 distance`
-   **Description** : Contient les coordonnées d'un point d'intérêt. `anglex` et `angley` représentent souvent les coordonnées en pixels (x, y) d'un objet détecté dans une image, tandis que `distance` peut représenter sa taille (rayon) ou sa distance réelle.
-   **Utilisation typique** : Utilisé par les nœuds de suivi par couleur (`yahboomcar_astra`) pour publier la position de l'objet coloré détecté.

### `Target.msg`

-   **Structure** :
    -   `string frame_id`
    -   `builtin_interfaces/Time stamp`
    -   `float32 scores`
    -   `float32 ptx`, `float32 pty`
    -   `float32 distw`, `float32 disth`
    -   `float32 centerx`, `float32 centery`
-   **Description** : Conçu pour contenir des informations détaillées sur une seule cible détectée par un modèle de détection d'objets. Il inclut un score de confiance, les coordonnées du coin supérieur gauche (`ptx`, `pty`), la largeur et la hauteur de la boîte englobante (`distw`, `disth`), et les coordonnées de son centre.
-   **Utilisation typique** : Probablement utilisé pour des tâches de détection d'objets basées sur l'IA (non vu dans les paquets précédents).

### `TargetArray.msg`

-   **Structure** : `yahboomcar_msgs/Target[] data`
-   **Description** : Représente une liste de cibles détectées, où chaque élément est un message de type `Target`.
-   **Utilisation typique** : Pour publier les résultats d'un détecteur d'objets qui peut identifier plusieurs objets dans une seule image.

### `ImageMsg.msg`

-   **Structure** :
    -   `int32 height`
    -   `int32 width`
    -   `int32 channels`
    -   `uint8[] data`
-   **Description** : Un format de message d'image personnalisé, plus simple que le `sensor_msgs/Image` standard. Il contient les dimensions de l'image et les données brutes des pixels.
-   **Utilisation typique** : Pour des applications où un format d'image simplifié est suffisant et plus facile à manipuler.
