# yahboomcar_laser

## Description

Ce paquet ROS2 regroupe plusieurs fonctionnalités basées sur l'utilisation d'un capteur Lidar (Laser scanner). Il fournit des nœuds pour implémenter des comportements autonomes simples comme l'évitement d'obstacles et le suivi de personnes ou d'objets.

Le paquet contient différentes versions des nœuds, adaptées à divers modèles de robots (R2, X3) et de Lidars (A1, 4ROS).

## Fonctionnement

Chaque nœud de ce paquet est un script Python indépendant qui exécute une tâche spécifique.

### 1. Nœuds d'évitement d'obstacles (ex: `laser_Avoidance_a1_X3`)

-   **Objectif** : Permettre au robot de se déplacer de manière autonome tout en évitant les collisions.
-   **Méthode** :
    1.  Le nœud s'abonne au topic `/scan` pour recevoir les données du Lidar.
    2.  Il analyse le balayage laser en le divisant en trois zones : avant, gauche et droite.
    3.  Pour chaque zone, il vérifie si des obstacles sont détectés en dessous d'une distance de sécurité (`ResponseDist`).
    4.  En fonction des zones où des obstacles sont présents, il publie une commande de vitesse (`geometry_msgs/msg/Twist`) sur le topic `/cmd_vel` pour faire tourner le robot dans la direction la plus dégagée ou pour avancer si la voie est libre.
    5.  Ce comportement est désactivé si un contrôle manuel (joystick) est actif.

### 2. Nœuds de suivi de cible (ex: `laser_Tracker_a1_X3`)

-   **Objectif** : Permettre au robot de suivre un objet ou une personne en mouvement.
-   **Méthode** :
    1.  Le nœud s'abonne également au topic `/scan`.
    2.  Il recherche le point de mesure le plus proche dans le champ de vision du Lidar, supposant que ce point correspond à la cible à suivre.
    3.  Il utilise deux contrôleurs PID pour ajuster la vitesse du robot :
        -   Un **PID linéaire** pour maintenir une distance constante (`ResponseDist`) par rapport à la cible.
        -   Un **PID angulaire** pour faire tourner le robot afin que la cible reste toujours en face de lui (à l'angle 0°).
    4.  Les commandes de vitesse résultantes sont publiées sur `/cmd_vel`.

### Diagramme de flux (Général)

```mermaid
graph TD
    A[Topic: /scan <br> sensor_msgs/msg/LaserScan] --> B{Nœud Laser <br> (Avoidance ou Tracker)};
    C[Topic: /JoyState <br> std_msgs/msg/Bool] --> B;
    B -- Analyse des données & Calcul PID --> B;
    B --> D[Topic: /cmd_vel <br> geometry_msgs/msg/Twist];
```

## Entrées attendues

-   **Topic :** `/scan`
-   **Type de message :** `sensor_msgs/msg/LaserScan`
-   **Description :** Données du Lidar.
-   **Topic :** `/JoyState`
-   **Type de message :** `std_msgs/msg/Bool`
-   **Description :** Indique si un joystick est actif, afin de désactiver le comportement autonome.

## Sorties générées

-   **Topic :** `/cmd_vel`
-   **Type de message :** `geometry_msgs/msg/Twist`
-   **Description :** Commandes de vitesse pour le robot.

## Utilisation

Ces nœuds sont généralement lancés via des fichiers de lancement dédiés. L'utilisateur peut choisir de lancer un nœud d'évitement ou un nœud de suivi en fonction de l'application souhaitée.

## Fichiers de configuration

Les principaux comportements de ces nœuds sont contrôlés par des paramètres ROS2, qui peuvent être ajustés dans les fichiers de lancement :
-   **`Switch`** (bool) : Un interrupteur logiciel pour activer ou désactiver le nœud.
-   **`linear` / `angular`** (double) : Vitesses maximales pour le robot.
-   **`LaserAngle`** (double) : L'angle de détection (en degrés) à gauche et à droite du robot.
-   **`ResponseDist`** (double) : La distance de sécurité (pour l'évitement) ou la distance de suivi (pour le tracker).
