# yahboomcar_mediapipe

## Description

Ce paquet ROS2 intègre la bibliothèque **MediaPipe** de Google pour fournir des capacités avancées de perception par vision par ordinateur. MediaPipe est un framework open-source qui offre des modèles pré-entraînés pour diverses tâches, telles que la détection des mains, de la pose du corps, du visage, et bien plus encore.

Ce paquet encapsule plusieurs de ces modèles dans des nœuds ROS2, permettant ainsi au robot de "voir" et de comprendre des éléments complexes dans son environnement.

## Fonctionnement

Le paquet est composé de plusieurs nœuds, chacun spécialisé dans une tâche de détection spécifique.

### Nœud de détection des mains (`01_HandDetector`)

-   **Objectif** : Détecter une ou plusieurs mains dans le champ de vision de la caméra et extraire les positions de leurs points de repère (landmarks).
-   **Méthode** :
    1.  Le nœud capture le flux vidéo d'une caméra via OpenCV.
    2.  Chaque image est traitée par le modèle **MediaPipe Hands**.
    3.  Si des mains sont détectées, le modèle renvoie les coordonnées 3D (x, y, z normalisées) de 21 points de repère pour chaque main.
    4.  Le nœud publie ces points sous la forme d'un message personnalisé `yahboomcar_msgs/msg/PointArray` sur le topic `/mediapipe/points`.
    5.  Il affiche également une fenêtre de visualisation montrant l'image de la caméra avec les points de repère et les connexions dessinés sur les mains détectées.

### Autres Nœuds

Les autres nœuds suivent un principe similaire pour différentes tâches :
-   **`02_PoseDetector`** : Détecte les points de repère du corps humain (épaules, coudes, poignets, etc.).
-   **`03_Holistic`** : Un modèle complet qui combine la détection de la pose, du visage et des mains.
-   **`04_FaceMesh`** : Crée un maillage 3D détaillé du visage.
-   **`05_FaceEyeDetection`** : Spécialisé dans la détection du visage et des yeux.

Ces nœuds publient également les points détectés sur des topics ROS2, les rendant disponibles pour d'autres parties du système robotique qui pourraient les utiliser pour l'interaction homme-robot, le contrôle gestuel, etc.

### Diagramme de flux (Exemple avec HandDetector)

```mermaid
graph TD
    A[Capteur: Caméra] --> B{Nœud: HandDetector};
    B -- Traitement par MediaPipe --> B;
    B --> C[Topic: /mediapipe/points <br> yahboomcar_msgs/msg/PointArray];
    B --> D[Visualisation OpenCV];
    C --> E[Autres Nœuds <br> (ex: Contrôle gestuel)];
```

## Entrées attendues

-   **Caméra** : Un flux vidéo, généralement capturé via `cv.VideoCapture(0)`.

## Sorties générées

-   **Topic `/mediapipe/points`** (ou similaire) :
    -   **Type de message** : `yahboomcar_msgs/msg/PointArray`
    -   **Description** : Une liste de points `geometry_msgs/msg/Point`, où chaque point représente un repère détecté par le modèle MediaPipe. Les coordonnées sont généralement normalisées (entre 0 et 1) par rapport à la taille de l'image.

## Utilisation

Chaque nœud peut être lancé individuellement via `ros2 run`. Par exemple :
```bash
ros2 run yahboomcar_mediapipe 01_HandDetector
```
Cela démarrera la détection des mains et ouvrira une fenêtre de visualisation. Les données des points de repère seront publiées sur ROS2 et pourront être utilisées par d'autres nœuds.

## Dépendances

-   **OpenCV** : Pour l'acquisition et le traitement d'images.
-   **MediaPipe** : La bibliothèque Python principale pour les modèles de détection.
-   **`yahboomcar_msgs`** : Pour les types de messages personnalisés comme `PointArray`.
