# my_ws

Ce workspace contient divers outils ROS 2 en cours de développement, qui ne rentrent pas dans `hive_ws`.  
Il héberge un package (`hive_data_collector`) proposant :

1. **Un node C++ de “crop” du nuage de points**  
   - Souscrit à `/lidar_points`, filtre la zone d’intérêt (ex. ±10 m) et republie sur `/lidar_points_crop`.  
   - Objectif : réduire la densité de données en ne conservant que l’environnement proche.

2. **Un script Python pour enregistrer des rosbags** (`rosbag_recorder_manager.py`)  
   - Interface Tkinter permettant de :
     - Sélectionner **quels topics** enregistrer.  
     - Définir un **nom** de capture.  
     - Suivre la **fréquence** de chaque topic, la **taille** du bag, etc.  
   - Lance `ros2 bag record` en sous-processus et propose un bouton pour **arrêter** l’enregistrement.  
   - Les rosbags sont rangés, par défaut, dans `my_ws/data/data_hive_data_collector`.

3. **Un script Python pour rejouer les rosbags** (`rosbag_player_manager.py`)  
   - Liste les rosbags présents dans `my_ws/data/data_hive_data_collector`.  
   - Permet de **lire** un bag à une vitesse donnée (`--rate`) et de définir un **offset** de départ (`--start-offset`).  
   - Affiche un temps écoulé, propose de **supprimer** un bag, ou de voir `ros2 bag info`.  

### Fonctionnalités futures / idées
- [ ] **Retour en arrière** : pouvoir reculer la lecture du bag (impliquerait `--start-offset` + recalcul).  
- [ ] **Relance automatique** en fin de bag (mode “loop”).  

---

## Utilisation

1. **Compilation**  
   - Placez tout votre code dans `~/my_ws/src/hive_data_collector/`.  
   - Depuis `~/my_ws`, lancez :
     ```bash
     colcon build
     source install/setup.bash
     ```

2. **Node C++ (crop)**  
   - Lancez :
     ```bash
     ros2 run hive_data_collector lidar_crop_node
     ```
   - Souscrit par défaut à `/lidar_points`, publie sur `/lidar_points_crop`.

3. **Enregistreur rosbag**  
   - Script Python :
     ```bash
     ros2 run hive_data_collector rosbag_recorder_manager.py
     ```
   - Interface graphique pour configurer l’enregistrement, surveiller les fréquences, stopper le bag, etc.

4. **Lecteur rosbag**  
   - Script Python :
     ```bash
     ros2 run hive_data_collector rosbag_player_manager.py
     ```
   - Choix du `rate`, d’un `offset`, et possibilité de voir/supprimer les bags.  

---

## Arborescence indicative

