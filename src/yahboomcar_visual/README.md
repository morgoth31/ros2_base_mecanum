# yahboomcar_visual

## Description

Ce paquet ROS2 regroupe une collection de nœuds utilitaires dédiés à des tâches de **vision par ordinateur** et de **visualisation**. Il sert de boîte à outils pour le traitement d'images, la réalité augmentée simple, et la conversion de données de capteurs en formats visuels.

## Fonctionnement

Le paquet contient plusieurs nœuds Python indépendants, chacun ayant un rôle spécifique.

### Nœud de Réalité Augmentée (`simple_AR`)

-   **Objectif** : Superposer des objets 3D virtuels sur un flux vidéo en direct.
-   **Méthode** :
    1.  Le nœud utilise une caméra et recherche un motif connu dans l'image (un échiquier - "chessboard").
    2.  Lorsqu'il détecte l'échiquier, il utilise les paramètres de calibration de la caméra pour calculer sa pose (position et orientation) par rapport à l'échiquier.
    3.  Il utilise ensuite cette pose pour projeter des points 3D (définis en code) sur l'image 2D, donnant l'illusion que des objets virtuels (comme des cubes, des pyramides, etc.) sont dessinés sur le monde réel.
    4.  L'image résultante est publiée sur le topic `/simpleAR/camera` et affichée dans une fenêtre OpenCV.

### Nœud de conversion Lidar vers Image (`laser_to_image`)

-   **Objectif** : Visualiser les données d'un Lidar 2D sous la forme d'une image en vue de dessus.
-   **Méthode** :
    1.  Le nœud s'abonne au topic `/scan`.
    2.  Il convertit le message `LaserScan` en un nuage de points (`PointCloud2`).
    3.  Il projette ensuite ces points sur une image 2D, créant une "vue d'oiseau" de l'environnement. L'intensité des pixels dans l'image peut représenter la hauteur (coordonnée Z) des points.
    4.  Cette image est publiée sur le topic `/laserImage` et affichée.

### Autres Nœuds Utilitaires

-   **`pub_image`** : Un nœud simple qui lit un fichier image depuis le disque et le publie de manière répétée sur un topic ROS2.
-   **`astra_rgb_image` / `astra_depth_image`** : Nœuds qui s'abonnent aux topics d'images de la caméra Astra et les republient, potentiellement après un traitement simple.
-   **`astra_image_flip`** : Un nœud qui s'abonne à un topic d'image, retourne l'image horizontalement, et la republie.
-   **`astra_color_point`** : Combine les données de couleur et de profondeur pour créer un nuage de points coloré.

### Diagramme de flux (Exemple avec `simple_AR`)

```mermaid
graph TD
    subgraph "Entrées"
        A[Capteur: Caméra]
        B[Fichier: astra.yaml <br> (Paramètres de calibration)]
        C[Topic: /Graphics_topic <br> (Choix de la forme à dessiner)]
    end

    subgraph "Processus"
        D(Nœud: simple_AR) -- Détecte l'échiquier --> E(Calcule la pose);
        E -- Projette les points 3D --> F(Dessine la forme sur l'image);
    end

    subgraph "Sorties"
        G[Fenêtre OpenCV]
        H[Topic: /simpleAR/camera <br> (Image avec AR)]
    end

    A & B & C --> D;
    D -- Image traitée --> G & H;
```

## Utilisation

Ces nœuds sont généralement utilisés pour des tâches de débogage, de visualisation ou comme composants dans des applications plus larges. Par exemple, `laser_to_image` est excellent pour vérifier rapidement le fonctionnement d'un Lidar, tandis que `simple_AR` peut être utilisé pour des démonstrations de réalité augmentée.
