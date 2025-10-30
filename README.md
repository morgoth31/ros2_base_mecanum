# Yahboom ROS2 Robot

Ce projet contient la pile logicielle ROS2 pour une gamme de robots éducatifs Yahboom, incluant des modèles avec différentes cinématiques (Ackermann, différentiel, Mécanum) et une large variété de capteurs.

## Architecture Logicielle

Le projet est organisé en plusieurs paquets ROS2, chacun avec une responsabilité spécifique. Un aperçu de l'architecture et du flux de données principal pour la navigation est présenté ci-dessous.

### Diagramme des Topics de Navigation

```mermaid
graph TD
    subgraph "Contrôle Manuel"
        A[Joystick] --> B(joy_node);
        B -- /joy --> C(yahboomcar_ctrl);
    end

    subgraph "Drivers Matériels"
        D(Driver du Robot <br> yahboomcar_bringup) -- Lit les données bas niveau --> E[Matériel <br> (Moteurs, IMU)];
    end

    subgraph "Estimation de la Pose"
        D -- /vel_raw --> F(yahboomcar_base_node);
        D -- /imu/data_raw --> G(imu_filter_madgwick);
        F -- /odom_raw --> H(robot_localization EKF);
        G -- /imu/data --> H;
    end

    subgraph "Pile de Navigation (Nav2)"
        I(Nav2 <br> yahboomcar_nav)
    end

    subgraph "Capteurs"
        J[Lidar] -- /scan --> I;
        K[Caméra] -- /image_raw, /depth --> L{Applications de Vision};
    end

    C -- /cmd_vel --> D;
    I -- /cmd_vel --> D;
    H -- /odometry/filtered --> I;

    style D fill:#f9f,stroke:#333,stroke-width:2px
    style F fill:#f9f,stroke:#333,stroke-width:2px
    style C fill:#f9f,stroke:#333,stroke-width:2px
    style I fill:#ccf,stroke:#333,stroke-width:2px
end
```

## Démarrage Rapide (Bringup)

La manière la plus simple de démarrer le robot est d'utiliser les fichiers de lancement principaux du paquet `yahboomcar_bringup`. Ces fichiers chargent les pilotes, les modèles de robot, les nœuds d'odométrie et la fusion de capteurs.

1.  **Ouvrez un terminal** et connectez-vous au robot (ou à l'environnement de développement).

2.  **Sourcez l'environnement ROS2** et l'espace de travail :
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/yahboomcar_ros2_ws/install/setup.bash
    ```

3.  **Lancez le bringup** correspondant à votre modèle de robot. Par exemple, pour le robot R2 :
    ```bash
    ros2 launch yahboomcar_bringup yahboomcar_bringup_R2_launch.py
    ```
    Pour le robot X3 :
    ```bash
    ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
    ```
    À ce stade, le robot est actif. Vous devriez pouvoir le contrôler avec un joystick si celui-ci est configuré.

## Visualisation avec RViz2

RViz2 est l'outil de visualisation 3D principal de ROS2. Il vous permet de "voir" ce que le robot voit et pense.

1.  **Ouvrez un second terminal.**

2.  **Sourcez l'environnement** comme précédemment.

3.  **Lancez RViz2** avec une configuration prédéfinie. Le paquet `yahboomcar_bringup` contient souvent un argument pour cela, ou vous pouvez le lancer séparément. Pour lancer RViz2 avec une configuration de navigation :
    ```bash
    ros2 launch yahboomcar_nav display_nav_launch.py
    ```
    Dans RViz2, vous pourrez visualiser :
    -   Le **modèle 3D du robot**.
    -   Les **données du Lidar** (points rouges).
    -   La **carte** (si un nœud de navigation ou de cartographie est actif).
    -   Le **chemin de navigation** global et local.
    -   Les **transformations TF** (arborescence des repères de coordonnées).

## Débogage en Ligne de Commande

Voici quelques commandes `ros2` utiles pour inspecter l'état du système :

-   **Lister tous les topics actifs** :
    ```bash
    ros2 topic list
    ```

-   **Afficher les messages d'un topic** (par exemple, les commandes de vitesse) :
    ```bash
    ros2 topic echo /cmd_vel
    ```

-   **Afficher le débit d'un topic** (utile pour vérifier si un capteur publie) :
    ```bash
    ros2 topic hz /scan
    ```

-   **Lister tous les nœuds actifs** :
    ```bash
    ros2 node list
    ```

-   **Inspecter un nœud** (voir ses abonnements, publications, services, etc.) :
    ```bash
    ros2 node info /base_node
    ```

-   **Visualiser l'arbre des transformations TF** :
    ```bash
    # Dans un nouveau terminal
    ros2 run tf2_tools view_frames
    # Cela génère un fichier PDF appelé frames.pdf
    ```

-   **Afficher la transformation entre deux repères** (par exemple, `odom` et `base_link`) :
    ```bash
    ros2 run tf2_ros tf2_echo odom base_link
    ```
