# yahboomcar_point

## Description

Ce paquet ROS2 sert d'utilitaire pour convertir les données de points de repère (landmarks) générées par MediaPipe en un format de nuage de points standardisé.

L'objectif principal est de rendre les données de perception de MediaPipe facilement visualisables dans des outils ROS comme RViz2 et compatibles avec d'autres nœuds qui travaillent avec des nuages de points (par exemple, ceux de la bibliothèque PCL).

## Fonctionnement

Le paquet contient un nœud C++ principal (`pub_point`) qui réalise une conversion de type de message.

1.  **Abonnement aux points MediaPipe** :
    *   Le nœud s'abonne au topic `/mediapipe/points`.
    *   Ce topic transporte des messages de type `yahboomcar_msgs/msg/PointArray`, qui est une liste personnalisée des points de repère détectés par un nœud du paquet `yahboomcar_mediapipe`.

2.  **Conversion en Nuage de Points** :
    *   À la réception d'un message `PointArray`, le nœud parcourt chaque point de la liste.
    *   Il crée une structure de données de nuage de points en utilisant la **Point Cloud Library (PCL)**.
    *   Chaque `geometry_msgs/Point` du message d'entrée est converti en un `pcl::PointXYZRGB` dans le nuage de points.
    *   Le nœud assigne une couleur statique (verte, dans ce cas) à chaque point pour une meilleure visualisation.

3.  **Publication du Nuage de Points** :
    *   Le nuage de points PCL est ensuite reconverti en un message ROS2 standard de type `sensor_msgs/msg/PointCloud2`.
    *   Ce message est publié sur le topic `/mediapipe_cloud`.

### Diagramme de flux

```mermaid
graph TD
    subgraph "Paquet : yahboomcar_mediapipe"
        A[Nœud: HandDetector (ou autre)] -- Publie --> B[Topic: /mediapipe/points <br> yahboomcar_msgs/msg/PointArray];
    end

    subgraph "Paquet : yahboomcar_point"
        C(Nœud: pub_point) -- S'abonne à --> B;
        C -- Convertit en PCL & Colore en vert --> C;
        C -- Publie --> D[Topic: /mediapipe_cloud <br> sensor_msgs/msg/PointCloud2];
    end

    subgraph "Outils ROS"
        E(RViz2) -- S'abonne à --> D;
        E -- Affiche --> F[Visualisation 3D des points];
    end
```

## Entrées attendues

-   **Topic :** `/mediapipe/points`
-   **Type de message :** `yahboomcar_msgs/msg/PointArray`
-   **Description :** Une liste des points de repère (main, pose, etc.) détectés par un nœud MediaPipe.

## Sorties générées

-   **Topic :** `/mediapipe_cloud`
-   **Type de message :** `sensor_msgs/msg/PointCloud2`
-   **Description :** Les mêmes points de repère, mais formatés en tant que nuage de points 3D coloré, prêt à être visualisé dans RViz2.

## Utilisation

Ce nœud est généralement lancé en parallèle avec l'un des nœuds de détection du paquet `yahboomcar_mediapipe`. Une fois les deux nœuds en cours d'exécution, vous pouvez ajouter un afficheur de type `PointCloud2` dans RViz2 et le configurer pour écouter le topic `/mediapipe_cloud` afin de voir les points de repère détectés en 3D.
