# yahboomcar_linefollow

## Description

Ce paquet ROS2 implémente une fonctionnalité de suivi de ligne pour le robot Yahboom. Il utilise les données d'une caméra pour détecter une ligne d'une couleur spécifique sur le sol et commande les moteurs du robot pour la suivre.

Le paquet intègre également une fonction d'évitement d'obstacles simple basée sur un Lidar pour arrêter le robot si un obstacle se trouve sur son chemin.

## Fonctionnement

Le nœud principal de suivi de ligne (par exemple, `follow_line_a1_X3`) combine plusieurs tâches :

1.  **Acquisition d'image** : Le nœud capture un flux vidéo directement depuis une caméra connectée (via OpenCV).
2.  **Calibration de la couleur** :
    *   Le nœud dispose d'un mode de calibration (`init`). Une fenêtre OpenCV s'affiche, permettant à l'utilisateur de sélectionner la ligne à suivre avec la souris.
    *   Le système calcule la plage de couleurs HSV (Teinte, Saturation, Valeur) de la zone sélectionnée et l'enregistre dans un fichier de configuration (`LineFollowHSV.text`).
3.  **Détection de la ligne** :
    *   Pour chaque image, le nœud applique un filtre de couleur en utilisant la plage HSV calibrée pour isoler la ligne du reste de l'image.
    *   Il traite l'image binaire résultante pour trouver le centre de la ligne dans le champ de vision de la caméra.
4.  **Contrôle par PID** :
    *   Un contrôleur PID est utilisé pour l'asservissement.
    *   L'**erreur** d'entrée du PID est la différence horizontale entre le centre de la ligne détectée et le centre de l'image.
    *   La **sortie** du PID est une commande de vitesse angulaire (`angular.z`) qui vise à minimiser cette erreur, gardant ainsi le robot centré sur la ligne.
    *   Le robot avance avec une vitesse linéaire constante (`linear.x`).
5.  **Évitement d'obstacles** :
    *   Le nœud s'abonne en parallèle au topic `/scan` d'un Lidar.
    *   Si un obstacle est détecté droit devant et à une distance inférieure à `ResponseDist`, le robot s'arrête et un buzzer est activé pour signaler le danger.
6.  **Désactivation par joystick** : Le suivi de ligne est automatiquement mis en pause si un contrôle manuel via un joystick est détecté (via le topic `/JoyState`).

### Diagramme de flux

```mermaid
graph TD
    subgraph "Entrées"
        A[Capteur: Caméra] --> C{Nœud: follow_line};
        B[Capteur: Lidar <br> (/scan)] --> C;
        D[Interface Utilisateur <br> (Calibration)] --> C;
        E[Topic: /JoyState] --> C;
    end

    subgraph "Processus"
        C -- Détection de ligne --> F(Calcul du centre de la ligne);
        F -- Erreur de position --> G(Contrôle PID);
        B -- Détection d'obstacle --> H(Logique d'arrêt d'urgence);
    end

    subgraph "Sorties"
        G & H -- Commandes --> I[Topic: /cmd_vel <br> geometry_msgs/msg/Twist];
        C --> J[Topic: /Buzzer];
        C --> K[Topic: /linefollow/rgb <br> (Image de débogage)];
    end
```

## Entrées attendues

-   **Caméra** : Un flux vidéo (le nœud utilise `cv.VideoCapture(0)`).
-   **Lidar (`/scan`)** : Données de balayage laser pour l'évitement d'obstacles.
-   **Joystick (`/JoyState`)** : Pour la désactivation du mode autonome.

## Sorties générées

-   **`/cmd_vel`** : Commandes de vitesse pour suivre la ligne ou s'arrêter.
-   **`/Buzzer`** : Active un buzzer en cas d'obstacle.
-   **`/linefollow/rgb`** : Publie l'image traitée pour le débogage.

## Utilisation

1.  **Calibration** : Lancez le nœud. En mode `init` (touche 'i'), sélectionnez la ligne dans la fenêtre OpenCV pour sauvegarder sa couleur.
2.  **Suivi** : Passez en mode `tracking` (barre d'espace). Le robot commencera à suivre la ligne de la couleur calibrée.

## Fichiers de configuration

-   **`LineFollowHSV.text`** : Fichier texte contenant la plage de couleurs HSV de la ligne à suivre.
-   **Paramètres ROS2** : Les gains du PID, la vitesse linéaire, et les seuils de détection d'obstacles peuvent être configurés via des paramètres dans un fichier de lancement.
