# Projet 3 -

Ce projet est composé de trois parties principales :

- **Client (Frontend)** : Développé en Angular.
- **Backend** : Développé en Nest.js.
- **Partie embarquée** : ROS (Python).

## Installation et Lancement

### 1. Client (Angular)

1. **Installation des dépendances :**

   - Accédez au répertoire du client (`/client`) et exécutez la commande suivante :
     ```bash
     npm install
     ```

2. **Démarrage du client :**

   - Une fois les dépendances installées, démarrez l'application Angular avec :
     ```bash
     npm start
     ```

3. **Accès au client :**
   - Ouvrez un navigateur et accédez à l'URL suivante : `http://localhost:4200`.

### 2. Backend (Nest.js)

1. **Installation des dépendances :**

   - Accédez au répertoire du backend (`/server`) et exécutez la commande suivante :
     ```bash
     npm install
     ```

2. **Démarrage du backend :**

   - Pour démarrer le serveur Nest.js, exécutez les commandes suivantes:

     ```bash
     source /opt/ros/humble/setup.bash
     npm start
     ```

3. **Vérification du backend :**
   - Le serveur backend sera accessible sur `http://localhost:3000`.

### 3. Partie Embarquée

## Lancement de la simulation

   - Lancez le script de démarrage avec la commande suivante :
   ``` bash
   source ./launch_script.sh simulation
   ```
   - Installer et lancer les dépendances du serveur
   - Installer et lancer les dépendances du client
   - Ouvrez un navigateur et accédez à l'URL suivante : `http://localhost:4200`.

## Lancement des robots

### 1. Préparation des robots

#### Sur le robot 1 :

1. **Allumer le robot** :
   - Assurez-vous que le robot est allumé correctement. Si nécessaire, utilisez l'interface physique du robot pour démarrer.
2. **Déverrouiller l'interface graphique** :

   - Accédez à l'interface graphique directement sur l'écran du robot. Si une session est verrouillée, déverrouillez-la avec le mot de passe **!mistlab**.

3. **Vérification du speaker** :

   - Ouvrez les paramètres du robot.
   - Sous les réglages audio, vérifiez que le speaker sélectionné est bien `"USB..."` ou `"Digital output audio"`.
   - Assurez-vous que le volume est supérieur à 0.
   - Testez le speaker en lançant un son pour confirmer qu'il fonctionne correctement.

4. **Connexion SSH** :

   - Depuis votre ordinateur, connectez-vous au robot via SSH en utilisant l'IP du robot:
     ```bash
     ssh nvidia@<ip_robot1>
     ```

5. **Accéder au répertoire du projet** :

   - Une fois connecté en SSH, naviguez dans le répertoire du projet avec la commande suivante :
     ```bash
     cd ~/inf3995/project_ws
     ```

6. **Mettre à jour le projet** :

   - Assurez-vous que le projet est à jour en exécutant :
     ```bash
     git pull
     ```

7. **Lancer le robot 1** :
   - Lancez les commandes ci-dessous dans trois terminaux:
   ATTENTION : Dans chaque nouvel terminal, avant de taper les commandes assurez vous de taper : export ROBOT_ID=X avec X = 1 ou 2
     ```bash
     ros2 launch limo_bringup limo_start.launch.py
     ros2 launch limo_bringup navigation2.launch.py 
     ros2 launch random_walker random_walker.launch.py 
     ```
     N'oubliez pas de sourcer avant chaque commande en faisant:
     ```bash
     source /opt/ros/humble/setup.bash
     source install/setup.bash 
     ```
     
     Faites pareil pour le robot 2 après avoir répété les étapes 3.1 à 3.4 en faisant:
    


2. **Configurer le domaine ROS** :

   - Définissez le domaine ROS :
     ```bash
     export ROS_DOMAIN_ID=49
     ```


