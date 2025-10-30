# yahboomcar_voice_ctrl

## Description

Ce paquet ROS2 ajoute une couche de **contrôle vocal** aux fonctionnalités existantes du robot Yahboom. Il fournit une série de nœuds qui permettent de commander diverses actions du robot, comme le déplacement, l'activation des lumières, ou le lancement de modes autonomes, en utilisant des commandes vocales.

Ce paquet s'appuie sur un module de reconnaissance vocale matériel pour interpréter les commandes de l'utilisateur.

## Fonctionnement

Le principe de ce paquet est de "wrapped" (envelopper) des fonctionnalités existantes avec une logique de reconnaissance vocale. La plupart des nœuds de ce paquet sont des versions modifiées de nœuds d'autres paquets.

### Nœud de contrôle principal (ex: `Voice_Ctrl_Mcnamu_driver_X3`)

-   **Base de pilote** : Le nœud intègre la fonctionnalité de base d'un pilote de robot, comme la communication avec la carte microcontrôleur pour lire les données des capteurs (IMU, batterie) et envoyer des commandes aux moteurs. Il publie ces données sur les topics ROS habituels (`/imu/data_raw`, `/vel_raw`, etc.) et s'abonne à `/cmd_vel`.
-   **Intégration de la reconnaissance vocale** :
    1.  Le nœud utilise une bibliothèque (`Speech_Lib`) pour communiquer avec le module de reconnaissance vocale matériel.
    2.  Dans une boucle périodique, il interroge le module pour savoir si une nouvelle commande vocale a été reconnue (`spe.speech_read()`).
    3.  Le module renvoie un identifiant numérique correspondant à la commande détectée (par exemple, 4 pour "Avancer", 11 pour "Lumière rouge").
-   **Exécution des commandes** :
    1.  Un grand bloc de conditions (`if/elif`) associe chaque identifiant de commande à une action spécifique.
    2.  Les actions peuvent être :
        -   **Déplacement** : Publier un message `Twist` sur `/cmd_vel` ou appeler directement la fonction de contrôle moteur pour une durée déterminée (par exemple, avancer pendant 5 secondes).
        -   **Contrôle des lumières** : Appeler les fonctions pour changer la couleur ou l'effet des LEDs RGB.
        -   **Autres fonctions** : Activer le buzzer, afficher le niveau de la batterie, etc.

### Autres Nœuds

Les autres nœuds du paquet, comme `Voice_Ctrl_follow_line_a1_X3` ou `Voice_Ctrl_colorTracker`, appliquent cette même logique pour contrôler des fonctionnalités plus complexes. Par exemple, une commande vocale comme "Suivi de ligne" pourrait activer ou désactiver le nœud de suivi de ligne.

### Diagramme de flux

```mermaid
graph TD
    subgraph "Entrées"
        A[Utilisateur] -- Parle --> B[Module de Reconnaissance Vocale];
        C[Topics de Commande <br> (ex: /cmd_vel)];
    end

    subgraph "Processus"
        B -- Identifiant de commande --> D{Nœud: Voice_Ctrl_*};
        C --> D;
        D -- Logique de commande --> D;
    end

    subgraph "Sorties"
        D -- Commandes bas niveau --> E[Matériel du Robot <br> (Moteurs, Lumières, Buzzer)];
        D -- Publications de données --> F[Topics ROS <br> (IMU, Odom, etc.)];
    end
```

## Utilisation

Pour utiliser le contrôle vocal, il faut lancer le nœud `Voice_Ctrl_*` correspondant à la fonctionnalité souhaitée. Par exemple, pour contrôler le robot X3 par la voix, on lancerait :
```bash
ros2 run yahboomcar_voice_ctrl Voice_Ctrl_Mcnamu_driver_X3
```
Ensuite, l'utilisateur peut énoncer les commandes vocales prédéfinies pour contrôler le robot.

## Dépendances

-   **Bibliothèques matérielles** : `Rosmaster_Lib` pour la communication avec le robot et `Speech_Lib` pour l'interface avec le module vocal.
-   Les nœuds de ce paquet remplacent souvent les nœuds de base (par exemple, `Voice_Ctrl_Mcnamu_driver_X3` remplace le pilote standard de `yahboomcar_bringup`), il ne faut donc pas les lancer en même temps.
