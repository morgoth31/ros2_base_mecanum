# yahboomcar_base_node

## Description

Ce paquet ROS2 est responsable du calcul et de la publication de l'odométrie du robot. L'odométrie est l'estimation de la position et de l'orientation du robot dans le temps, basée sur les données de mouvement de ses roues (généralement issues des encodeurs moteur).

Ce paquet contient plusieurs nœuds, chacun adapté à un modèle de robot Yahboom spécifique (X1, X3, R2), car leurs cinématiques (la manière dont le mouvement des roues se traduit en mouvement du robot) sont différentes.

## Fonctionnement

Chaque nœud de ce paquet fonctionne de manière similaire :
1.  **Abonnement à la vitesse brute** : Le nœud s'abonne au topic `/vel_raw`, qui transporte un message `geometry_msgs/msg/Twist`. Ce message contient les vitesses linéaires et angulaires brutes, supposées provenir d'une source de bas niveau comme un microcontrôleur lisant les encodeurs des moteurs.
2.  **Calcul de l'odométrie** : À chaque réception d'un message de vitesse, le nœud intègre ces vitesses dans le temps pour mettre à jour la pose (position `x`, `y` et l'orientation `theta`) du robot. Le calcul exact dépend de la cinématique du robot :
    *   **Ackermann (ex: R2)** : Le calcul prend en compte l'angle de braquage des roues.
    *   **Différentiel ou Mécanum (ex: X3)** : Le calcul utilise directement la vitesse angulaire.
3.  **Publication de l'odométrie** : Le nœud publie la pose et la vitesse calculées sous la forme d'un message `nav_msgs/msg/Odometry` sur le topic `/odom_raw`. Ce message est un format standard utilisé par de nombreux paquets de navigation ROS2.
4.  **Publication de la transformation TF** : Si le paramètre `pub_odom_tf` est activé, le nœud publie également la transformation (TF) entre le repère `odom` (un repère fixe qui représente le point de départ du robot) et le repère `base_footprint` (un repère attaché à la base du robot). Cette transformation est essentielle pour la pile de navigation ROS2.

### Diagramme de flux (Mermaid)

```mermaid
graph TD
    A[Topic: /vel_raw <br> geometry_msgs/msg/Twist] --> B{Nœud: base_node};
    B -- Intégration temporelle --> C(Calcul de la pose [x, y, θ]);
    C --> D[Topic: /odom_raw <br> nav_msgs/msg/Odometry];
    C --> E[Topic: /tf <br> tf2_msgs/msg/TFMessage];
```

## Entrées attendues

-   **Topic :** `/vel_raw`
-   **Type de message :** `geometry_msgs/msg/Twist`
-   **Description :** Vitesses brutes du robot. `linear.x` représente la vitesse linéaire, et `linear.y` (pour Ackermann) ou `angular.z` (pour différentiel/mécanum) est utilisé pour le changement de direction.

## Sorties générées

-   **Topic :** `/odom_raw`
-   **Type de message :** `nav_msgs/msg/Odometry`
-   **Description :** Estimation de la pose et de la vitesse du robot.
-   **Topic :** `/tf`
-   **Type de message :** `tf2_msgs/msg/TFMessage`
-   **Description :** Transformation entre les repères `odom` et `base_footprint`.

## Fichiers de configuration

Ce nœud est configurable via des paramètres ROS2, qui peuvent être définis dans un fichier de lancement :
-   **`wheelbase`** (double) : L'empattement du robot (distance entre les essieux avant et arrière), crucial pour les modèles Ackermann.
-   **`odom_frame`** (string, default: "odom") : Le nom du repère d'odométrie.
-   **`base_footprint_frame`** (string, default: "base_footprint") : Le nom du repère de la base du robot.
-   **`linear_scale_x` / `linear_scale_y` / `angular_scale`** (double, default: 1.0) : Facteurs de calibration pour ajuster les vitesses si les mesures des encodeurs ne sont pas parfaites.
-   **`pub_odom_tf`** (bool, default: true) : Active ou désactive la publication de la transformation TF.
