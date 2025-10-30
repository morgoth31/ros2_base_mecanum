# yahboomcar_description_x1

## Description

Ce paquet ROS2 contient la description physique du robot Yahboom X1. Similaire au paquet `yahboomcar_description`, il fournit une représentation standardisée de la structure mécanique du robot, de ses capteurs et de leurs relations spatiales via le format URDF.

Il est probable que ce paquet soit une version spécifique ou alternative pour le modèle X1, potentiellement avec des capteurs ou des configurations de liens différents.

## Fonctionnement

Ce paquet fonctionne sur le même principe que `yahboomcar_description`. Il ne contient pas de nœuds exécutables mais fournit les fichiers de données nécessaires pour que d'autres nœuds puissent comprendre la structure physique du robot.

1.  **Fichiers URDF/Xacro** :
    *   Les fichiers `urdf/yahboomcar_X1.urdf.xacro` et `urdf/yahboomcar_X1.urdf` définissent la structure de liens et d'articulations (`links` et `joints`) spécifique au modèle X1.

2.  **Maillages (Meshes)** :
    *   Le répertoire `meshes/` contient les modèles 3D (`.STL`, `.dae`) pour chaque partie du robot X1.

3.  **Configuration RViz** :
    *   Le répertoire `rviz/` contient des fichiers de configuration pour visualiser le modèle du robot X1 dans RViz2.

### Diagramme de flux

```mermaid
graph TD
    subgraph "Paquet : yahboomcar_description_x1"
        A[Fichier Xacro <br> (yahboomcar_X1.urdf.xacro)]
        B[Fichiers de maillage]
    end

    subgraph "Processus de Lancement"
        C(Utilitaire xacro) -- Convertit --> D[Paramètre robot_description];
        A -- Inclut --> B;
    end

    subgraph "Nœuds ROS"
        E(Nœud: robot_state_publisher) -- Lit --> D;
        E -- Publie les transformations --> F[Topic: /tf];
        G(RViz2) -- Lit --> F;
        G -- Affiche le modèle 3D --> H[Visualisation du Robot X1];
    end
```

## Utilisation

Ce paquet est utilisé comme une dépendance par les fichiers de lancement spécifiques au robot X1 (par exemple, dans `yahboomcar_bringup`). Le nœud `robot_state_publisher` charge la description du robot X1 pour publier son arbre de transformations TF, qui est essentiel pour la navigation, la manipulation et la visualisation.

## Contenu

-   **`urdf/`** : Fichiers Xacro et URDF décrivant la structure du robot X1 (et aussi X3 dans ce cas).
-   **`meshes/`** : Modèles 3D des composants du robot.
-   **`rviz/`** : Fichiers de configuration pour RViz2.
-   **`launch/`** : Fichiers de lancement pour visualiser le modèle du robot dans RViz2.
-   **`config/`** : Peut contenir des fichiers de configuration supplémentaires, par exemple pour les contrôleurs (`ros2_control`).
