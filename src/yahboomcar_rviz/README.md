# yahboomcar_rviz

## Description

Ce paquet ROS2 semble être dédié à la **visualisation** du robot Yahboom, principalement avec l'outil **RViz2**.

Il partage une structure très similaire à celle d'un paquet de description (comme `yahboomcar_description`), contenant des répertoires pour les fichiers URDF, les maillages (meshes) et les configurations RViz.

Il est possible que ce paquet soit une version plus ancienne, simplifiée, ou une alternative destinée spécifiquement à des tâches de visualisation de base, sans inclure toutes les complexités des modèles de robots spécifiques.

## Fonctionnement

Comme un paquet de description, `yahboomcar_rviz` ne contient probablement pas de nœuds exécutables. Son rôle est de fournir les fichiers nécessaires pour afficher le robot dans RViz2.

-   **`urdf/`** : Contient un fichier de description du robot (`yahboomcar.urdf`). Ce modèle peut être une version générique ou simplifiée du robot.
-   **`meshes/`** : Contient les modèles 3D (`.STL`, `.dae`) des différentes parties du robot, référencés par le fichier URDF.
-   **`rviz/`** : Contient des fichiers de configuration (`.rviz`) qui définissent une scène de visualisation par défaut pour le robot, en affichant le modèle du robot et potentiellement d'autres visualisations de données de capteurs.
-   **`launch/`** : Fournit des fichiers de lancement pour démarrer facilement RViz2 avec le modèle du robot et la configuration de visualisation prédéfinie.

### Diagramme de flux

```mermaid
graph TD
    A[Fichier de Lancement <br> (dans yahboomcar_rviz)] -- Charge --> B[Fichier URDF <br> (dans yahboomcar_rviz)];
    A -- Lance --> C(Nœud: robot_state_publisher);
    A -- Lance --> D(Application: RViz2);

    C -- Lit le paramètre 'robot_description' (contenant l'URDF) --> C;
    C -- Publie --> E[Topic: /tf];

    D -- Utilise la configuration .rviz --> D;
    D -- S'abonne à /tf --> D;
    D -- Affiche --> F[Visualisation 3D du Robot];
```

## Utilisation

L'utilisation principale de ce paquet est de lancer une session RViz2 pour visualiser le robot. Cela peut être fait en utilisant l'un des fichiers de lancement du paquet. Par exemple :
```bash
ros2 launch yahboomcar_rviz display_car_launch.py
```
Cette commande lancerait probablement RViz2 et chargerait le modèle URDF du robot, permettant à l'utilisateur de voir à quoi ressemble le robot et de vérifier que son arbre de transformations TF est correct.

Ce paquet est particulièrement utile pour le débogage et le développement, car il offre un moyen rapide de visualiser le robot sans avoir à lancer l'ensemble de la pile logicielle.
