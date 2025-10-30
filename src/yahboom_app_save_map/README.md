# yahboom_app_save_map

## Description

Ce paquet ROS2 fournit une fonctionnalité de sauvegarde de cartes via une interface de service ROS2. Il est conçu pour permettre à des applications externes, comme une application mobile ou web, de demander la sauvegarde de la carte de navigation actuellement générée par un algorithme SLAM.

Le paquet inclut deux composants principaux :
1.  **Un serveur de service (`server`)** : Un nœud ROS2 qui attend les requêtes de sauvegarde de carte.
2.  **Un client de service (`client`)** : Un exemple de nœud pour envoyer une requête de sauvegarde au serveur.

## Fonctionnement

### Serveur (`server`)

Le nœud serveur crée un service ROS2 nommé `yahboomAppSaveMap`. Lorsqu'il reçoit une requête, il effectue les actions suivantes :
1.  Il extrait le nom de la carte (`map_name`) de la requête.
2.  Il génère un chemin complet pour stocker la carte, généralement dans le répertoire `src/yahboomcar_nav/maps/`.
3.  Il exécute l'outil en ligne de commande `map_saver_cli` de `nav2_map_server` pour sauvegarder la carte actuelle sur le disque.
4.  Il enregistre les métadonnées de la carte (nom, ID unique, chemin) dans une base de données SQLite (`xgo.db`) pour une gestion ultérieure.

### Client (`client`)

Le nœud client est un exemple simple qui montre comment appeler le service `yahboomAppSaveMap`. Il envoie une requête avec un nom de carte prédéfini pour déclencher le processus de sauvegarde.

### Diagramme de flux (Mermaid)

```mermaid
graph TD
    A[Application Externe <br> (ex: Client ROS2, App mobile)] -- Requête de sauvegarde --> B(Service: /yahboomAppSaveMap <br> Nœud: server);
    B -- Exécute --> C{map_saver_cli};
    C -- Sauvegarde --> D[Fichiers de carte <br> (map.yaml, map.pgm)];
    B -- Enregistre --> E[Base de données SQLite <br> (xgo.db)];
```

## Entrées attendues

-   **Service :** `yahboomAppSaveMap`
-   **Type de service :** `yahboom_web_savmap_interfaces/srv/WebSaveMap`
-   **Contenu de la requête :** `string mapname` - Le nom souhaité pour la carte.

## Sorties générées

-   **Fichiers de carte :** Un fichier de métadonnées `.yaml` et un fichier d'image `.pgm` sont créés dans `src/yahboomcar_nav/maps/`.
-   **Base de données :** Une nouvelle entrée est ajoutée à la table `xgo_map` dans la base de données `/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/yahboom_app_save_map/yahboom_app_save_map/xgo.db`.
-   **Réponse du service :** Le service renvoie le nom de la carte en confirmation.

## Fichiers de configuration

-   **Base de données SQLite :** `src/yahboom_app_save_map/yahboom_app_save_map/xgo.db`. Le chemin vers cette base de données est codé en dur dans le script du serveur.

## Configuration requise

1.  **Dépendances :** Le paquet `nav2_map_server` doit être installé, car le serveur dépend de son exécutable `map_saver_cli`.
2.  **Chemins de fichiers :** Les chemins pour la sauvegarde de la carte et la base de données sont codés en dur dans le script du serveur. Si vous utilisez ce paquet dans un autre environnement, vous devrez modifier ces chemins.
3.  **Permissions :** Le nœud doit avoir les permissions nécessaires pour écrire dans le répertoire de sauvegarde des cartes et dans la base de données SQLite.
