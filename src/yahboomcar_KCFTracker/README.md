# yahboomcar_KCFTracker

## Description

Ce paquet ROS2 implémente un nœud de suivi d'objet visuel basé sur l'algorithme KCF (Kernelized Correlation Filters). Il permet à un utilisateur de sélectionner un objet dans le flux vidéo d'une caméra, puis de commander le robot pour qu'il suive cet objet en maintenant une distance constante.

## Fonctionnement

Le nœud principal, `ImageConverter`, effectue les tâches suivantes :
1.  **Abonnement aux flux vidéo** : Il s'abonne aux topics d'images RGB et de profondeur d'une caméra.
2.  **Sélection de l'objet** : Une fenêtre OpenCV est affichée, permettant à l'utilisateur de dessiner un rectangle autour de l'objet à suivre avec la souris.
3.  **Initialisation du tracker** : Une fois l'objet sélectionné, le tracker KCF est initialisé avec cette région d'intérêt.
4.  **Suivi** : Pour chaque nouvelle image, le tracker met à jour la position de l'objet et dessine un cadre autour de lui. L'image résultante est publiée sur un topic pour la visualisation.
5.  **Asservissement** : Lorsque le suivi est activé (via un topic ou une touche du clavier), le nœud utilise les données de l'image de profondeur pour calculer la distance à l'objet.
6.  **Contrôle du robot** : Deux contrôleurs PID sont utilisés :
    *   Un PID linéaire pour ajuster la vitesse du robot afin de maintenir une distance de consigne (`minDist`).
    *   Un PID angulaire pour ajuster la vitesse de rotation afin de garder l'objet centré dans le champ de vision de la caméra.
7.  **Publication des commandes** : Les vitesses calculées sont publiées sur un topic `cmd_vel` sous forme de messages `geometry_msgs/msg/Twist`.

### Diagramme de flux (Mermaid)

```mermaid
graph TD
    subgraph "Entrées"
        A[Topic: Image RGB <br> sensor_msgs/msg/Image]
        B[Topic: Image de profondeur <br> sensor_msgs/msg/Image]
        C[Topic: Booléen d'activation <br> std_msgs/msg/Bool]
        D[Interface Utilisateur <br> (Fenêtre OpenCV)]
    end
    subgraph "Nœud: KCF_Tracker"
        E
    end
    subgraph "Sorties"
        F[Topic: /cmd_vel <br> geometry_msgs/msg/Twist]
        G[Topic: Image avec suivi <br> sensor_msgs/msg/Image]
    end
    A --> E;
    B --> E;
    C --> E;
    D -- Sélection ROI --> E;
    E -- Calcul de la pose et distance --> E;
    E -- Contrôle PID --> E;
    E --> F;
    E --> G;
```

## Entrées attendues

-   **Topic d'image RGB** : Le nom du topic est généralement remappé dans le fichier de lancement.
-   **Topic d'image de profondeur** : Le nom du topic est également remappé.
-   **Topic d'activation** : Un topic de type `std_msgs/msg/Bool` pour activer/désactiver le mode de suivi à distance.
-   **Interaction utilisateur** : Sélection manuelle de l'objet dans la fenêtre OpenCV.

## Sorties générées

-   **Topic de commande de vitesse (`/cmd_vel`)** : Messages `geometry_msgs/msg/Twist` pour contrôler les moteurs du robot.
-   **Topic d'image de suivi** : Publie l'image de la caméra avec le rectangle de suivi dessiné dessus.

## Fichiers de configuration

La configuration se fait principalement via des paramètres ROS2 définis dans le fichier de lancement (`.launch.py`), tels que :
-   `minDist_` : La distance de consigne à maintenir par rapport à l'objet suivi (en mètres).
-   Les gains des contrôleurs PID peuvent être ajustés dans le code source (`KCF_Tracker.cpp`).

## Configuration requise

1.  **Caméra** : Une caméra RGB-D (couleur + profondeur) est nécessaire.
2.  **Dépendances** : Ce paquet dépend de `OpenCV` et `cv_bridge`.
3.  **Lancement** : Le nœud doit être lancé via un fichier de lancement qui remappe correctement les topics d'entrée de la caméra.
