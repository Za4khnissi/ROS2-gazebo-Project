# Projet 3 -

Ce projet est composé de trois parties principales :

- **Client (Frontend)** : Développé en Angular.
- **Backend** : Développé en Nest.js.
- **Partie embarquée** : ROS (Python).

## Accès scripts de lancement

Executer les commandes suivantes à la racine du projet pour autoriser l'execution des scripts de lancement

   ```bash
     sudo chmod 777 launch_station.sh
     sudo chmod 777 launch_robot.sh
   ```

## Lancement de la simulation

## Station au sol ( Client + Serveuer)

1. Mettez la variable SIMULATION à 1 dans le fichier .env

2. **Utilisation du script de lancement**
   - Executer le script via : 
     ```bash
     source launch_station.sh
     ```

### 1. Client (Angular)

1. **Accès au client :**
   - Ouvrez un navigateur et accédez à l'URL suivante : `http://localhost:4200`.

### 2. Backend (Nest.js)

1. **Vérification du backend :**
   - Le serveur backend sera accessible sur `http://localhost:3000`.

### 3. Partie Embarquée

## Lancement de la simulation

  Une fois la station au sol lancée avec les étapes plus haut, sélectionnez le bouton “Simulation”.
  Puis dans l’interface qui s’affiche cliquer sur “Start ROS” pour lancer la simulation Gazebo.
  Patientez 1 minute le temps que le système complet soit lancé.

## Lancement des robots

### 1. Préparation des robots

#### Sur les robots :

1.  **Allumer le robot** :
    - Assurez-vous que le robot est allumé correctement. Si nécessaire, utilisez l'interface physique du robot pour démarrer.
2.  **Déverrouiller l'interface graphique** :

    - Accédez à l'interface graphique directement sur l'écran du robot. Si une session est verrouillée, déverrouillez-la avec le mot de passe **!mistlab**.

3.  **Vérification du speaker** :

    - Ouvrez les paramètres du robot.
    - Sous les réglages audio, vérifiez que le speaker sélectionné est bien `"USB..."` ou `"Digital output audio"`.
    - Assurez-vous que le volume est supérieur à 0.
    - Testez le speaker en lançant un son pour confirmer qu'il fonctionne correctement.

4.  **Connexion SSH** :

    - Depuis votre ordinateur, connectez-vous au robot via SSH en utilisant l'IP du robot:
      ```bash
      ssh nvidia@<ip_robot>
      ```

5.  **Accéder au répertoire du projet** :

    - Une fois connecté en SSH, naviguez dans le répertoire du projet avec la commande suivante :
      ```bash
      cd ~/inf3995/
      ```

6.  **Mettre à jour le projet** :

    - Assurez-vous que le projet est à jour en exécutant :
      ```bash
      git pull
      ```

7.  **Lancer le robot 1 ou 2** :

    - Assurez vous que la station au sol et les robots soient sur le même réseau WIFI puis suivre les étapes suivantes:
      Sur le premier robot, placez vous à la racine du projet et executez la commande suivante:
      ```bash
      source launch_robot.sh 1
      ```
      Sur le deuxième robot, placez vous à la racine du projet et executez la commande suivante:
      ```bash
      source launch_robot.sh 1
      ```
      Sur la station au sol, mettre la variable SIMULATION à 0 dans le fichier .env
      À la racine du repo, executer la commande
      ```bash
      source launch_station.sh
      ```