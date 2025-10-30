

##debuguer ROS2 en ligne de commande

```bash
ros2 topic echo <nom_du_topic>
```
    Rôle : C'est l'outil de surveillance le plus direct. Il s'abonne au topic spécifié et affiche le contenu de chaque message publié en temps réel dans le terminal.

```bash
ros2 topic hz <nom_du_topic>
```
    Rôle : Calcule et affiche la fréquence de publication moyenne (en Hertz) d'un topic. C'est un outil essentiel pour surveiller la "santé" d'un nœud et vérifier qu'il publie des données à la cadence attendue.

```bash
ros2 topic bw <nom_du_topic>
```
    Rôle : Mesure et affiche la bande passante (en octets par seconde) utilisée par un topic. C'est un outil de diagnostic de performance pour identifier les topics qui consomment une quantité excessive de ressources réseau.

```bash
ros2 topic info <nom_du_topic>
```
    Rôle : Fournit des métadonnées sur un topic, incluant son type de message, le nombre de "publishers" (émetteurs) et le nombre de "subscribers" (récepteurs).

```bash
ros2 topic list
```
    Rôle : Liste tous les topics actuellement enregistrés dans le système ROS 2, qu'ils aient ou non des publishers/subscribers.


Debuguer en graphique

rqt_plot

        Rôle : Un plugin rqt extrêmement utile pour la surveillance de données numériques. Il permet de tracer en temps réel l'évolution d'un ou plusieurs champs numériques à l'intérieur d'un message.

        Exemple : Vous pouvez tracer la composante linear.x d'un topic geometry_msgs/Twist pour voir la consigne de vitesse de votre robot.

    rqt_graph

        Rôle : Ne surveille pas le contenu des topics, mais surveille l'architecture du système. Il affiche un graphe en temps réel des nœuds actifs et des topics qui les relient. C'est fondamental pour comprendre qui parle à qui.

    rviz2

        Rôle : L'outil de visualisation 3D principal. C'est une forme avancée de "moniteur" spécifiquement conçue pour les données complexes et géométriques. Vous l'utilisez pour surveiller visuellement :

            Les données de capteurs (sensor_msgs/LaserScan, sensor_msgs/PointCloud2, sensor_msgs/Image).

            Les transformations de repères (tf2).

            Les états du robot (via robot_state_publisher et l'URDF).

            Les données de navigation (nav_msgs/Path, nav_msgs/Odometry).

    rqt_topic

        Rôle : Un plugin rqt qui fournit une interface graphique pour des fonctionnalités similaires à ros2 topic list, info, et echo. Il permet de sélectionner un topic dans une liste et d'inspecter les messages publiés.



