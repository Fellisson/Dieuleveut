# Simulation Balistique Et Controle PID Sous MATLAB

Projet MATLAB de simulation balistique 2D/3D, comparaison d'angles de tir, etude de l'effet du frottement aerodynamique, visualisations animees et scenarios avec et sans controle PID.

## Vue d'ensemble

Ce projet regroupe plusieurs simulations de dynamique de projectile et de vehicule guide sous MATLAB. Il permet d'etudier :

- la trajectoire balistique en 2D
- l'influence de l'angle de lancement
- l'effet du frottement de l'air
- la comparaison entre cas avec et sans trainee
- des animations et videos automatiques
- des scenarios 3D avec et sans commande PID

Le depot produit des sorties dans `out/` sous forme d'images, de logs et de videos, afin de faciliter l'analyse et la presentation des resultats.

## Structure

```text
Felix Nimy/
|- src/
|  |- ballistique/
|  |  |- compare_projectile_angles.m
|  |  |- compare_projectile_angles_drag_modes.m
|  |  |- projectile_trajectory.m
|  |  |- projectile_trajectory_initial.m
|  |  |- projectile_trajectory_drag_comparison.m
|  |  `- projectile_trajectory_drag_comparison_video.m
|  `- controle_pid/
|     |- animation_3D_visualisation_avec_PID.m
|     |- animation_3D_visualisation_sans_PID.m
|     |- controle_2D_PID_animation_video_01.m
|     |- controle_3D_ballistique_avec_PID.m
|     |- controle_3D_ballistique_sans_PID.m
|     `- controle_altitude_angle_PID_animation.m
|- docs/
|- out/
|  |- images/
|  |- logs/
|  `- videos/
|- setup_paths.m
`- README.md
```

## Fonctionnalites

- simulation de trajectoire balistique 2D
- comparaison de plusieurs angles de tir
- comparaison avec frottement et sans frottement
- export automatique des courbes, logs et videos
- animation MP4 des trajectoires
- simulation 3D avec controle PID
- simulation 3D sans PID pour comparaison

## Fichiers principaux

### Ballistique

- `src/ballistique/projectile_trajectory.m`
- `src/ballistique/projectile_trajectory_initial.m`
- `src/ballistique/compare_projectile_angles.m`
- `src/ballistique/compare_projectile_angles_drag_modes.m`
- `src/ballistique/projectile_trajectory_drag_comparison.m`
- `src/ballistique/projectile_trajectory_drag_comparison_video.m`

### Controle PID

- `src/controle_pid/controle_3D_ballistique_avec_PID.m`
- `src/controle_pid/controle_3D_ballistique_sans_PID.m`
- `src/controle_pid/animation_3D_visualisation_avec_PID.m`
- `src/controle_pid/animation_3D_visualisation_sans_PID.m`

## Resultats obtenus

Le projet genere automatiquement des fichiers dans `out/` :

- `out/images/` pour les figures PNG
- `out/logs/` pour les fichiers texte
- `out/videos/` pour les animations MP4

Exemples de sorties :

- comparaison de trajectoires pour plusieurs angles
- comparaison avec et sans frottement
- animation d'une trajectoire balistique
- visualisations 3D avec et sans PID

## Lancement

Depuis MATLAB :

```matlab
run('setup_paths.m')
```

### Exemples ballistiques

```matlab
projectile_trajectory
projectile_trajectory_initial(300, 30)
projectile_trajectory_initial(300, 45)
compare_projectile_angles
compare_projectile_angles_drag_modes
projectile_trajectory_drag_comparison
projectile_trajectory_drag_comparison_video
```

### Exemples 3D avec ou sans PID

```matlab
controle_3D_ballistique_avec_PID
controle_3D_ballistique_sans_PID
animation_3D_visualisation_avec_PID
animation_3D_visualisation_sans_PID
```

Depuis Git Bash :

```bash
cd "/c/Users/Felix NIMY/OneDrive/Documents/MATLAB/Felix Nimy"
"/d/LOGICIELS/bin/matlab.exe" -batch "run('setup_paths.m'); compare_projectile_angles"
```

## Interpretation physique

Les comparaisons montrent notamment que :

- l'angle de portee maximale n'est plus necessairement `45 deg` en presence de frottement
- dans ce modele, `30 deg` donne une meilleure portee que `45 deg`
- les tirs plus eleves augmentent le temps de vol et l'altitude maximale
- le frottement penalise fortement les trajectoires hautes en dissipant plus d'energie
- les versions avec PID et sans PID permettent d'observer l'influence du controle sur la stabilite du vol

## Objectif pedagogique

Ce projet est concu pour illustrer :

- les lois du mouvement balistique
- l'effet du frottement de l'air
- l'importance de l'angle initial
- la difference entre trajectoire libre et trajectoire controlee
- l'interet du PID dans un modele dynamique

## Auteur

Felix Nimy

