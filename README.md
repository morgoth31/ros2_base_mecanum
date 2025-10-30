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

## Identification des Nœuds

Ce projet est composé de nombreux nœuds ROS2. Voici une classification de leur rôle.

### 1. Nœuds du Robot (Nœuds Principaux)

Ces nœuds sont essentiels au fonctionnement du robot, que ce soit pour le contrôle de bas niveau, la localisation, ou les comportements autonomes.

-   **Pilotes Matériels (`yahboomcar_bringup`)**: `Ackman_driver_R2`, `Mcnamu_driver_X3` - Interface directe avec le matériel pour lire les capteurs et commander les moteurs.
-   **Odométrie (`yahboomcar_base_node`)**: `base_node_R2`, `base_node_X3` - Calcule la position du robot à partir des données des roues.
-   **Contrôle Manuel (`yahboomcar_ctrl`)**: `yahboom_joy_*`, `yahboom_keyboard` - Permet de piloter le robot avec un joystick ou un clavier.
-   **Navigation Autonome (`yahboomcar_nav`)**: Nœuds de la pile Nav2 (`amcl`, `planner_server`, `controller_server`) pour la localisation et la planification de trajectoire.
-   **SLAM (`yahboomcar_nav`, `yahboomcar_slam`)**: `slam_toolbox`, `gmapping`, `cartographer`, `orbslam_rgbd_pose` - Pour la cartographie de l'environnement.
-   **Comportements Autonomes**:
    -   `yahboomcar_laser`: `laser_Avoidance_*`, `laser_Tracker_*` - Évitement d'obstacles et suivi par Lidar.
    -   `yahboomcar_linefollow`: `follow_line_*` - Suivi de ligne par caméra.
    -   `yahboomcar_astra`: `colorTracker` - Suivi d'objet par couleur.
-   **Contrôle Vocal (`yahboomcar_voice_ctrl`)**: `Voice_Ctrl_*` - Ajoute une couche de commande vocale aux fonctionnalités existantes.
-   **Traitement de Capteurs**:
    -   `imu_filter_madgwick`: `imu_filter_madgwick_node` - Filtre et améliore les données de l'IMU.
    -   `robot_localization`: `ekf_node` - Fusionne les données d'odométrie et d'IMU pour une estimation de pose plus précise.

### 2. Nœuds de Supervision (Monitoring)

Ces nœuds sont utilisés pour la visualisation, le débogage et la conversion de données. Ils ne sont généralement pas critiques pour le fonctionnement du robot mais sont essentiels pour le développement.

-   **Visualisation du Modèle**: `robot_state_publisher`, `joint_state_publisher` - Publie la structure et l'état du robot pour la visualisation dans RViz2.
-   **Interface de Visualisation 3D**: `rviz2` - L'outil principal pour visualiser l'état du robot et les données des capteurs.
-   **Conversion de Données pour la Visualisation**:
    -   `laserscan_to_point_pulisher`: Convertit les données Lidar (`LaserScan`) en un nuage de points (`Path`) plus facile à visualiser.
    -   `robot_pose_publisher_ros2`: Convertit la transformation TF du robot en un message de `Pose` qui peut être facilement tracé.
    -   `yahboomcar_point`: Convertit les points détectés par MediaPipe en un nuage de points (`PointCloud2`) pour RViz2.
    -   `yahboomcar_visual`: `laser_to_image` - Crée une image en vue de dessus à partir des données Lidar.

### 3. Nœuds de Simulation

Ce projet n'utilise pas de nœuds spécifiques à la simulation (comme des plugins Gazebo dédiés). La stratégie adoptée est d'utiliser les mêmes nœuds que pour le robot réel en activant le mode de simulation de ROS2.

-   **Utilisation du temps de simulation**: En lançant les nœuds du robot avec le paramètre `use_sim_time:=true`, les nœuds ignorent l'horloge système et utilisent l'horloge publiée par le simulateur (par exemple, Gazebo). Cela permet à la grande majorité des nœuds (localisation, navigation, contrôle) de fonctionner de manière identique en simulation et dans le monde réel.
