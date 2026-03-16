# Projet MATLAB - Structure reorganisee

## Vue d'ensemble

Le projet est maintenant organise pour separer clairement :

- les processus de calcul dans `src/`
- la documentation dans `docs/`
- les resultats generes dans `out/`

Cette repartition permet de distinguer facilement ce que le code fait
de ce qu'il produit.

## Structure

```text
Felix Nimy/
|- src/
|  |- ballistique/
|  |  |- compare_projectile_angles.m
|  |  |- projectile_trajectory.m
|  |  |- projectile_trajectory_initial.m
|  |  |- projectile_trajectory_drag_comparison.m
|  |  `- projectile_trajectory_drag_comparison_video.m
|  `- controle_pid/
|     |- controle_altitude_angle_PID_animation.m
|     |- controle_2D_PID_animation_video_01.m
|     |- simulate_controle_2D_PID.m
|     |- animation_3D_visualisation.m
|     |- controle_3D_PID_animation.m
|     `- controle_3D_PID_ballistique.m
|- docs/
|  |- processus.md
|  |- resultats.md
|  `- analyse_resultats.md
|- out/
|  |- logs/
|  |- images/
|  `- videos/
|- setup_paths.m
`- README.md
```

## Repartition demandee

### Cote processus

Le dossier `src/` contient les scripts et fonctions MATLAB classes par theme :

- `src/ballistique` pour les simulations de projectile
- `src/controle_pid` pour les simulations de controle

### Cote resultats

Le dossier `out/` centralise toutes les sorties d'execution :

- `out/logs` pour les fichiers texte
- `out/images` pour les figures exportees
- `out/videos` pour les animations MP4

## Utilisation

Depuis la racine du projet dans MATLAB :

```matlab
run('setup_paths.m')
compare_projectile_angles
projectile_trajectory
projectile_trajectory_drag_comparison
projectile_trajectory_drag_comparison_video
controle_altitude_angle_PID_animation
controle_2D_PID_animation_video_01
animation_3D_visualisation
controle_3D_PID_animation
controle_3D_PID_ballistique
```

Le script `setup_paths.m` ajoute automatiquement `src/` et ses sous-dossiers
au path MATLAB.

## Documentation

- [Processus](docs/processus.md)
- [Resultats](docs/resultats.md)
- [Analyse](docs/analyse_resultats.md)

## Remarque

Les scripts ont ete adaptes pour enregistrer leurs sorties dans `out/`
independamment du dossier courant d'execution.

Les vues 2D et 3D du controle PID s'appuient maintenant sur un noyau commun
de simulation : `simulate_controle_2D_PID.m`.
