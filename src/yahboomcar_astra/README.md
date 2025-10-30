# yahboomcar_astra

## Description

Ce paquet ROS2 fournit des fonctionnalités de suivi d'objet basées sur la couleur, conçues pour être utilisées avec une caméra RGB-D comme l'Orbbec Astra. Il est divisé en deux nœuds principaux :

1.  **`colorHSV`** : Un outil de calibration pour déterminer la plage de couleurs (HSV) de l'objet que l'on souhaite suivre.
2.  **`colorTracker`** : Le nœud principal qui effectue le suivi de l'objet et commande le robot pour le suivre.

## Fonctionnement

### 1. Nœud de Calibration (`colorHSV`)

Ce nœud est la première étape du processus.
-   Il s'abonne à un flux d'images RGB.
-   Il affiche une fenêtre OpenCV où l'utilisateur peut dessiner un rectangle autour de l'objet d'intérêt.
-   Il calcule la plage de valeurs HSV (Teinte, Saturation, Valeur) correspondant à la région sélectionnée.
-   Il sauvegarde cette plage HSV dans un fichier texte (`colorHSV.text`) pour une utilisation ultérieure par le nœud de suivi.
-   Il publie en continu la position (en pixels) et la taille du plus grand contour correspondant à la couleur détectée sur le topic `/Current_point`.

### 2. Nœud de Suivi (`colorTracker`)

Ce nœud utilise les informations de calibration pour effectuer le suivi.
-   Il s'abonne au topic `/Current_point` pour obtenir la position de l'objet dans l'image.
-   Il s'abonne à un flux d'images de profondeur pour déterminer la distance entre le robot et l'objet.
-   Il utilise deux contrôleurs PID pour calculer les commandes de vitesse :
    -   Un **PID linéaire** pour maintenir une distance de consigne par rapport à l'objet.
    -   Un **PID angulaire** pour garder l'objet centré dans l'image.
-   Il publie les commandes de vitesse sur le topic `/cmd_vel`.
-   Il s'abonne également à un topic `/JoyState` pour désactiver le suivi si un joystick est utilisé, évitant ainsi les conflits de commande.

### Diagramme de flux (Mermaid)

```mermaid
graph TD
    subgraph "Étape 1: Calibration"
        A[Topic: Image RGB] --> B(Nœud: colorHSV);
        C[Interface Utilisateur <br> (Fenêtre OpenCV)] -- Sélection ROI --> B;
        B -- Sauvegarde --> D[Fichier: colorHSV.text];
        B -- Position de l'objet --> E[Topic: /Current_point];
    end

    subgraph "Étape 2: Suivi"
        F[Topic: Image de profondeur] --> G(Nœud: colorTracker);
        E --> G;
        H[Topic: /JoyState] --> G;
        G -- Calcul PID --> G;
        G --> I[Topic: /cmd_vel];
    end
```

## Entrées attendues

-   **Pour `colorHSV`** :
    -   Topic d'image RGB (ex: `/camera/color/image_raw`).
-   **Pour `colorTracker`** :
    -   Topic de position de l'objet (`/Current_point`).
    -   Topic d'image de profondeur (ex: `/camera/depth/image_raw`).
    -   Topic d'état du joystick (`/JoyState`).

## Sorties générées

-   **Par `colorHSV`** :
    -   Topic `/Current_point` (`yahboomcar_msgs/msg/Position`) : Contient les coordonnées en pixels et la taille de l'objet détecté.
-   **Par `colorTracker`** :
    -   Topic `/cmd_vel` (`geometry_msgs/msg/Twist`) : Commandes de vitesse pour le robot.

## Fichiers de configuration

-   **`yahboomcar_astra/colorHSV.text`** : Fichier contenant les valeurs HSV min/max de la couleur à suivre. Ce fichier est généré par le nœud `colorHSV`.
-   **Paramètres ROS2** : Les gains des PID et la distance de suivi pour le `colorTracker` peuvent être configurés via des paramètres ROS2 dans un fichier de lancement.

## Utilisation

1.  Lancer le nœud `colorHSV` et une source d'images (nœud de caméra ou rosbag).
2.  Dans la fenêtre OpenCV, sélectionner l'objet à suivre pour générer le fichier `colorHSV.text`.
3.  Arrêter le nœud `colorHSV`.
4.  Lancer le nœud `colorTracker` avec le nœud `colorHSV` (qui lira maintenant le fichier de configuration) et le nœud de la caméra. Le robot commencera à suivre l'objet de la couleur spécifiée.
