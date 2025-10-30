# yahboomcar_slam

## Description

Ce paquet ROS2 est dédié à la **cartographie 3D** et au **SLAM Visuel (VSLAM)**, en utilisant principalement l'algorithme **ORB-SLAM**.

Contrairement au paquet `yahboomcar_nav` qui se concentre sur le SLAM 2D basé sur Lidar, ce paquet utilise les données d'une caméra RGB-D (couleur + profondeur) pour construire des représentations tridimensionnelles de l'environnement et pour estimer la trajectoire du robot.

Il s'intègre également avec **OctoMap**, un outil qui permet de créer des cartes d'occupation 3D volumétriques.

## Fonctionnement

Ce paquet sert principalement de configuration et de point de lancement pour un wrapper ROS2 de l'algorithme ORB-SLAM (probablement le paquet `ros2_orbslam`).

### 1. SLAM Visuel (ORB-SLAM)

-   **Objectif** : Estimer la trajectoire 6-DoF (degrés de liberté : x, y, z, roulis, tangage, lacet) du robot et construire une carte de points 3D de l'environnement en utilisant uniquement les données d'une caméra.
-   **Fichiers de lancement** : Des fichiers comme `orbslam_base_launch.py` orchestrent le démarrage. Ils lancent le nœud principal d'ORB-SLAM (`rgbd_pose` ou similaire) en lui fournissant les fichiers de configuration nécessaires.
-   **Processus** :
    1.  Le nœud ORB-SLAM s'abonne aux topics d'images synchronisées RGB et de profondeur de la caméra.
    2.  Il extrait des caractéristiques visuelles (les "ORB features") de chaque image.
    3.  En faisant correspondre ces caractéristiques entre les images successives, il résout simultanément deux problèmes :
        -   **Localisation** : Où se trouve la caméra dans le monde ?
        -   **Cartographie** : Où se trouvent les points 3D correspondant à ces caractéristiques dans le monde ?
    4.  Le résultat est une estimation de la pose de la caméra, qui est publiée sous forme de transformation TF (par exemple, de `map` à `camera`).

### 2. Cartographie 3D (OctoMap)

-   **Objectif** : Convertir le nuage de points sparse (peu dense) généré par ORB-SLAM en une carte d'occupation 3D dense.
-   **Fichiers de lancement** : `camera_octomap_launch.py` et d'autres fichiers associés.
-   **Processus** :
    1.  Un nœud `octomap_server` s'abonne au nuage de points 3D (généré à partir des images de profondeur et de la pose estimée par ORB-SLAM).
    2.  Il intègre ces points dans une structure de données en arbre (un "Octree"), qui représente l'espace 3D comme une grille de voxels (pixels 3D).
    3.  Chaque voxel a un état : occupé, libre ou inconnu. Cela permet une représentation efficace de l'espace 3D pour la planification de trajectoire.

### Diagramme de flux

```mermaid
graph TD
    subgraph "Entrées"
        A[Capteur: Caméra RGB-D <br> (Images couleur & profondeur)]
    end

    subgraph "Processus de SLAM"
        B(Nœud: ORB-SLAM)
    end

    subgraph "Processus de Cartographie 3D"
        C(Nœud: OctoMap Server)
    end

    subgraph "Sorties"
        D[Pose de la Caméra <br> (Topic /tf)]
        E[Nuage de Points de la Carte]
        F[Carte d'Occupation 3D <br> (OctoMap)]
    end

    A --> B;
    B --> D;
    B -- Nuage de points sparse --> E;
    E -- (et/ou données caméra + pose) --> C;
    C --> F;
```

## Contenu

-   **`launch/`** : Fichiers de lancement pour démarrer les différentes configurations de VSLAM et de cartographie OctoMap.
-   **`params/`** : Fichiers de configuration critiques pour ORB-SLAM, notamment :
    -   `ORBvoc.txt` : Le dictionnaire de "mots visuels" nécessaire pour la reconnaissance de lieux.
    -   `rgbd.yaml` : Les paramètres intrinsèques de la caméra et d'autres réglages d'ORB-SLAM.
-   **`rviz/`** : Fichiers de configuration RViz2 pour visualiser la pose de la caméra, le nuage de points de la carte et la carte OctoMap.
-   **`src/`** et **`include/`** : Peuvent contenir du code source pour des nœuds de support (par exemple, pour la conversion de données).

## Utilisation

Pour lancer une session de VSLAM, vous utiliseriez un des fichiers de lancement principaux. Par exemple :
```bash
ros2 launch yahboomcar_slam orbslam_base_launch.py
```
Cela lancerait le robot, le nœud ORB-SLAM, et tout ce qui est nécessaire pour commencer à estimer la pose du robot à partir des données de la caméra.
