# yahboomcar_description

## Description

Ce paquet ROS2 contient la description physique du robot Yahboom. La "description" d'un robot dans ROS est une représentation standardisée de sa structure mécanique, de ses capteurs et de leurs relations spatiales.

L'élément central de ce paquet est le fichier URDF (Unified Robot Description Format), qui est un format XML permettant de modéliser le robot.

## Fonctionnement

Ce paquet ne contient pas de nœuds exécutables, mais plutôt des fichiers de données qui sont utilisés par d'autres nœuds, principalement `robot_state_publisher`.

1.  **Fichiers URDF/Xacro** :
    *   Les fichiers se trouvent dans le répertoire `urdf/`.
    *   **Xacro (`.urdf.xacro`)** : Il s'agit d'un langage de macro qui simplifie la création de fichiers URDF complexes. Les fichiers Xacro sont utilisés pour définir la structure du robot de manière modulaire et paramétrable. Ils sont ensuite convertis en fichiers URDF.
    *   **URDF (`.urdf`)** : Le fichier URDF final est un arbre de `links` (liens) et de `joints` (articulations).
        *   **Links** : Représentent les parties rigides du robot (le châssis, une roue, un support de capteur, etc.). Chaque lien a des propriétés visuelles (comment il apparaît dans une simulation ou RViz), de collision (sa forme pour la détection de collisions) et inertielles (sa masse et son moment d'inertie).
        *   **Joints** : Connectent les liens entre eux et définissent le type de mouvement autorisé (par exemple, `continuous` pour une roue qui tourne sans fin, `revolute` pour une articulation avec des limites, ou `fixed` pour une connexion rigide).

2.  **Maillages (Meshes)** :
    *   Le répertoire `meshes/` contient les modèles 3D des différentes parties du robot, généralement au format `.STL` ou `.dae`.
    *   Les fichiers URDF font référence à ces maillages pour définir la géométrie visuelle et de collision de chaque lien.

3.  **Configuration RViz** :
    *   Le répertoire `rviz/` contient des fichiers de configuration (`.rviz`) pour l'outil de visualisation RViz2. Ces fichiers définissent quelles informations afficher (le modèle du robot, les données des capteurs, etc.) et comment les afficher.

### Diagramme de flux

```mermaid
graph TD
    subgraph "Paquet : yahboomcar_description"
        A[Fichier Xacro <br> (ex: yahboomcar_R2.urdf.xacro)]
        B[Fichiers de maillage <br> (ex: base_link.STL)]
    end

    subgraph "Processus de Lancement"
        C(Utilitaire xacro) -- Convertit --> D[Paramètre robot_description];
        A -- Inclut --> B;
    end

    subgraph "Nœuds ROS"
        E(Nœud: robot_state_publisher) -- Lit --> D;
        E -- Publie les transformations --> F[Topic: /tf];
        G(RViz2) -- Lit --> F;
        G -- Affiche le modèle 3D --> H[Visualisation du Robot];
    end
```

## Utilisation

Ce paquet est une dépendance fondamentale pour presque toutes les opérations du robot. Il est chargé par les fichiers de lancement du paquet `yahboomcar_bringup`. Le nœud `robot_state_publisher` lit le paramètre `robot_description` (qui contient l'URDF) et publie les transformations TF entre les différents liens du robot. Cela permet à n'importe quel autre nœud ROS2 de connaître la configuration spatiale du robot à tout moment.

## Contenu

-   **`urdf/`** : Fichiers Xacro et URDF décrivant la structure du robot.
-   **`meshes/`** : Modèles 3D des composants du robot.
-   **`rviz/`** : Fichiers de configuration pour RViz2.
-   **`launch/`** : Fichiers de lancement pour visualiser le modèle du robot dans RViz2 indépendamment du reste du système.
