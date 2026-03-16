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
- un noyau commun de simulation 2D reutilisable ;
- une visualisation 3D branchee sur les donnees du modele 2D ;
- une simulation 3D autonome avec commande PID ;
- une simulation 3D de type balistique commandee ;
- l'animation temporelle du systeme ;
- l'enregistrement video pour certains scenarios.

Fichiers principaux :

- `controle_altitude_angle_PID_animation.m`
- `controle_2D_PID_animation_video_01.m`
- `simulate_controle_2D_PID.m`
- `animation_3D_visualisation.m`
- `controle_3D_PID_animation.m`
- `controle_3D_PID_ballistique.m`

## Relation entre les fichiers PID

Le fichier `simulate_controle_2D_PID.m` joue le role de noyau commun pour la
simulation 2D. Il calcule les etats, commandes et perturbations du modele
plan.

Les fichiers suivants reutilisent ce noyau :

- `controle_2D_PID_animation_video_01.m` pour la vue 2D animee ;
- `animation_3D_visualisation.m` pour une vue 3D basee sur les donnees 2D.

Les fichiers suivants sont des modeles 3D distincts :

- `controle_3D_PID_animation.m` pour un suivi 3D commande ;
- `controle_3D_PID_ballistique.m` pour une phase propulsee suivie d'une
  phase balistique avec impact.

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
