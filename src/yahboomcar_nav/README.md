# yahboomcar_nav

## Description

Ce paquet ROS2 est dédié à la **navigation autonome** du robot Yahboom. Il contient toutes les configurations, cartes et fichiers de lancement nécessaires pour utiliser la pile de navigation de ROS2, **Nav2**.

Ce paquet permet au robot d'effectuer des tâches telles que :
-   **La cartographie (SLAM)** : Construire une carte 2D d'un environnement inconnu.
-   **La localisation** : Se situer avec précision à l'intérieur d'une carte préexistante.
-   **La planification de trajectoire** : Calculer un chemin optimal d'un point A à un point B tout en évitant les obstacles.
-   **Le suivi de chemin** : Exécuter la trajectoire planifiée.

## Fonctionnement

Ce paquet s'appuie fortement sur l'écosystème Nav2 et fournit les "briques" de configuration pour l'adapter au robot Yahboom.

### 1. Cartographie (SLAM - Simultaneous Localization and Mapping)

-   **Objectif** : Créer une carte.
-   **Fichiers de lancement** : Le paquet fournit des fichiers de lancement pour différentes méthodes de SLAM, comme `map_gmapping_a1_launch.py`, `cartographer_launch.py`, ou `map_rtabmap_launch.py`.
-   **Processus** : Lorsque l'un de ces fichiers est lancé, il démarre un nœud SLAM qui écoute les données du Lidar (`/scan`) et de l'odométrie (`/odom`) pour construire progressivement une carte 2D de l'environnement, publiée sur le topic `/map`.

### 2. Navigation

-   **Objectif** : Naviguer de manière autonome dans une carte existante.
-   **Fichiers de lancement** : Des fichiers comme `navigation_dwa_launch.py` ou `navigation_teb_launch.py` lancent la pile Nav2 complète.
-   **Processus** : Ces fichiers de lancement chargent la pile Nav2, qui est un ensemble de serveurs ROS2 orchestrés :
    -   **Serveur de Carte (`map_server`)** : Charge une carte enregistrée.
    -   **Localisation (`amcl`)** : Utilise un filtre à particules pour estimer la position du robot dans la carte en se basant sur les données du Lidar et de l'odométrie.
    -   **Planificateur Global (`planner_server`)** : Calcule un chemin à long terme du point de départ à la destination.
    -   **Planificateur Local (`controller_server`)** : Calcule les commandes de vitesse à court terme (`/cmd_vel`) pour suivre le chemin global tout en évitant les obstacles locaux (détectés en temps réel par le Lidar). Différents algorithmes comme DWA (Dynamic Window Approach) ou TEB (Timed Elastic Band) peuvent être utilisés ici.
    -   **Serveur de Comportement (`behavior_server`)** : Gère des comportements complexes comme la récupération en cas d'échec.

### Diagramme de flux (Navigation)

```mermaid
graph TD
    subgraph "Entrées"
        A[Topic: /scan]
        B[Topic: /odom]
        C[Topic: /tf]
        D[Interface Utilisateur <br> (RViz2 ou API) <br> Objectif de navigation]
    end

    subgraph "Pile Nav2"
        E[Localisation (AMCL)]
        F[Serveur de Carte]
        G[Planificateur Global]
        H[Planificateur Local (DWA/TEB)]
        I[Arbre de Comportement]
    end

    subgraph "Sortie"
        J[Topic: /cmd_vel]
    end

    A & B & C --> E;
    E -- Pose estimée --> G & H;
    F -- Carte --> E & G;
    D -- Envoie l'objectif --> I;
    I -- Gère le processus --> G & H;
    G -- Chemin global --> H;
    H -- Commandes de vitesse --> J;
```

## Contenu

-   **`launch/`** : Un grand nombre de fichiers de lancement pour démarrer différents modes (SLAM, localisation, navigation) avec différents algorithmes.
-   **`maps/`** : Contient des cartes pré-enregistrées de l'environnement au format `.yaml` et `.pgm`.
-   **`params/`** : Fichiers de configuration YAML (`.yaml`) très importants qui contiennent tous les paramètres pour les différents composants de Nav2 (par exemple, `dwa_nav_params.yaml`). C'est ici que l'on ajuste le comportement du robot (vitesse, accélération, tolérances, etc.).
-   **`rviz/`** : Fichiers de configuration RViz2 pour visualiser la carte, la position du robot, le chemin planifié et les données des capteurs.

## Utilisation

-   **Pour cartographier** : `ros2 launch yahboomcar_nav map_gmapping_a1_launch.py`
-   **Pour naviguer** : `ros2 launch yahboomcar_nav navigation_dwa_launch.py map:=/chemin/vers/ma/carte.yaml`
