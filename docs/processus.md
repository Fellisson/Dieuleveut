# Processus

## Objectif

Ce document decrit la partie "processus" du projet, c'est-a-dire les scripts,
fonctions et methodes utilises pour produire les simulations.

## Organisation

### Ballistique

Les fichiers du dossier `src/ballistique/` couvrent :

- le calcul de trajectoire avec gravite et trainee ;
- la comparaison de plusieurs angles de tir ;
- la comparaison avec et sans trainee ;
- la generation d'animations et de videos.

Fichiers principaux :

- `compare_projectile_angles.m`
- `projectile_trajectory.m`
- `projectile_trajectory_initial.m`
- `projectile_trajectory_drag_comparison.m`
- `projectile_trajectory_drag_comparison_video.m`

### Controle PID

Les fichiers du dossier `src/controle_pid/` couvrent :

- la simulation verticale avec regulation altitude/angle ;
- la simulation 2D avec perturbations ;
- l'animation temporelle du systeme ;
- l'enregistrement video pour certains scenarios.

Fichiers principaux :

- `controle_altitude_angle_PID_animation.m`
- `controle_2D_PID_animation_video_01.m`

## Logique generale

Le processus suit en general ces etapes :

1. definir les parametres physiques ;
2. definir les conditions initiales et consignes ;
3. executer la simulation numerique ;
4. calculer les indicateurs utiles ;
5. afficher ou animer les courbes ;
6. sauvegarder les resultats dans `out/`.

## Point d'entree

Le fichier `setup_paths.m` prepare l'environnement MATLAB en ajoutant
les sous-dossiers de `src/` au path.
