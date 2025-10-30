# yahboomcar_ctrl

## Description

Ce paquet ROS2 est dédié au contrôle manuel du robot Yahboom. Il fournit des nœuds pour piloter le robot à l'aide de différents dispositifs d'entrée, comme un joystick (manette de jeu) ou le clavier de l'ordinateur.

Il permet de traduire les commandes de l'utilisateur en messages `geometry_msgs/msg/Twist` qui sont envoyés au pilote du robot.

## Fonctionnement

Ce paquet contient plusieurs nœuds, chacun adapté à un type de contrôle ou à un modèle de robot spécifique.

### Nœud de contrôle par Joystick (ex: `yahboom_joy_R2`)

1.  **Abonnement aux données du joystick** : Le nœud s'abonne au topic `/joy`, qui est publié par le paquet `joy` de ROS2. Ce topic envoie des messages contenant l'état des axes (joysticks analogiques) et des boutons de la manette.
2.  **Mappage des commandes** : Le nœud interprète les données du joystick. Par exemple :
    *   Le joystick gauche peut contrôler la vitesse linéaire (`linear.x`) et l'angle de braquage (`linear.y` pour un robot Ackermann).
    *   Des boutons spécifiques peuvent être assignés à des actions comme activer un buzzer, changer les motifs d'une lumière RGB, ou ajuster des "vitesses" (gears) pour rendre le robot plus ou moins sensible aux commandes.
3.  **Publication des commandes de vitesse** : Les commandes interprétées sont converties en un message `Twist` et publiées sur le topic `/cmd_vel`.
4.  **Gestion de l'état** : Le nœud publie également un message booléen sur le topic `/JoyState`. Cela permet à d'autres nœuds (comme les nœuds de suivi d'objet) de savoir si le contrôle manuel est actif, afin d'éviter des commandes conflictuelles.
5.  **Annulation de la navigation** : Un bouton est généralement assigné pour annuler les objectifs de navigation autonomes en cours, redonnant ainsi le contrôle manuel à l'utilisateur.

### Nœud de contrôle par Clavier (`yahboom_keyboard`)

De manière similaire au nœud de joystick, ce nœud lit les frappes du clavier de l'ordinateur et les convertit en commandes de vitesse `/cmd_vel`. Par exemple, les touches fléchées peuvent être utilisées pour diriger le robot.

### Diagramme de flux (Joystick)

```mermaid
graph TD
    A[Périphérique : Joystick] --> B(Nœud: joy_node);
    B -- /joy --> C(Nœud: yahboom_joy_R2);
    C -- Mappage des axes/boutons --> C;
    C --> D[Topic: /cmd_vel <br> geometry_msgs/msg/Twist];
    C --> E[Topic: /JoyState <br> std_msgs/msg/Bool];
    C --> F[Autres Topics <br> (Buzzer, RGBLight, ...)];
```

## Entrées attendues

-   **Topic :** `/joy`
-   **Type de message :** `sensor_msgs/msg/Joy`
-   **Description :** Données brutes provenant du pilote du joystick.

## Sorties générées

-   **Topic :** `/cmd_vel`
-   **Type de message :** `geometry_msgs/msg/Twist`
-   **Description :** Commande de vitesse pour le robot.
-   **Topic :** `/JoyState`
-   **Type de message :** `std_msgs/msg/Bool`
-   **Description :** Indique si le contrôle par joystick est actuellement actif. `True` signifie que l'utilisateur est en train de contrôler le robot manuellement.
-   **Autres topics** : `/Buzzer`, `/RGBLight`, `/move_base/cancel` pour contrôler des fonctionnalités supplémentaires.

## Utilisation

Le nœud de contrôle est généralement lancé en même temps que le reste du robot via les fichiers de lancement du paquet `yahboomcar_bringup`. Il nécessite que le nœud `joy_node` (du paquet `joy`) soit également lancé pour lire les données du périphérique physique.

## Configuration

La sensibilité et la vitesse maximale du robot peuvent être ajustées via des paramètres ROS2 dans les fichiers de lancement, tels que :
-   `xspeed_limit`
-   `yspeed_limit`
-   `angular_speed_limit`
