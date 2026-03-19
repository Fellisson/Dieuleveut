# Dieuleveut

Projet MATLAB de simulation balistique et de controle dynamique, avec visualisations 2D/3D, comparaisons physiques et animations exportees en images et videos.

## Apercu

Ce depot regroupe plusieurs travaux autour :

- de la trajectoire balistique en 2D
- de l'effet du frottement aerodynamique
- de la comparaison de plusieurs angles de tir
- de l'animation des resultats
- de modeles 3D avec et sans controle PID

Le projet a une vocation a la fois pedagogique et experimentale : il permet d'observer l'influence des parametres physiques sur la portee, l'altitude, le temps de vol et la stabilite.

## Points forts

- simulations balistiques 2D parametrees
- comparaison multi-angles : `20°`, `30°`, `45°`, `60°`, `75°`
- comparaison avec frottement et sans frottement
- animations MP4 generees automatiquement
- scenarios 3D avec PID et sans PID
- export automatique des figures, logs et videos

## Structure Du Projet

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

## Scripts Principaux

### Ballistique

- `projectile_trajectory.m` : simulation balistique principale
- `projectile_trajectory_initial.m` : version initiale parametree avec animation
- `compare_projectile_angles.m` : comparaison de plusieurs angles de tir
- `compare_projectile_angles_drag_modes.m` : comparaison multi-angles avec et sans frottement
- `projectile_trajectory_drag_comparison.m` : comparaison detaillee de deux cas
- `projectile_trajectory_drag_comparison_video.m` : video dediee a la comparaison avec/sans trainee

### Controle PID

- `controle_3D_ballistique_avec_PID.m`
- `controle_3D_ballistique_sans_PID.m`
- `animation_3D_visualisation_avec_PID.m`
- `animation_3D_visualisation_sans_PID.m`

## Resultats Produits

Les scripts exportent automatiquement leurs sorties dans `out/` :

- `out/images/` : figures PNG
- `out/logs/` : fichiers texte de resultats
- `out/videos/` : animations MP4

Exemples de sorties generees :

- `comparaison_trajectoires_angles.png`
- `comparaison_trajectoires_angles.mp4`
- `comparaison_angles_avec_sans_frottement.png`
- `comparaison_angles_avec_sans_frottement.mp4`
- `projectile_trajectory_initial_animation_30deg.mp4`
- `controle_3D_ballistique_avec_PID.mp4`

## Demarrage Rapide

Depuis MATLAB :

```matlab
run('setup_paths.m')
```

Exemples de lancement :

```matlab
projectile_trajectory
projectile_trajectory_initial(300, 30)
projectile_trajectory_initial(300, 45)
compare_projectile_angles
compare_projectile_angles_drag_modes
controle_3D_ballistique_avec_PID
controle_3D_ballistique_sans_PID
```

Depuis Git Bash :

```bash
cd "/c/Users/Felix NIMY/OneDrive/Documents/MATLAB/Felix Nimy"
"/d/LOGICIELS/bin/matlab.exe" -batch "run('setup_paths.m'); compare_projectile_angles"
```

## Lecture Physique Des Resultats

Les simulations montrent notamment que :

- en presence de frottement, `45°` n'est plus necessairement l'angle de portee maximale
- dans ce modele, `30°` donne une meilleure portee que `45°`
- les grands angles augmentent l'altitude maximale et le temps de vol
- le frottement penalise davantage les trajectoires hautes, car le projectile reste plus longtemps dans l'air
- les versions avec PID et sans PID permettent d'observer l'effet du controle sur la trajectoire et la stabilisation

Exemple de tendance observee avec frottement :

| Angle | Portee | Altitude max | Interpretation |
|---|---:|---:|---|
| `20°` | ~0.617 km | ~0.109 km | tir tendu, rapide, faible altitude |
| `30°` | ~0.633 km | ~0.184 km | meilleur compromis portee / elevation |
| `45°` | ~0.578 km | ~0.295 km | plus haut, plus long, mais moins loin |
| `60°` | ~0.449 km | ~0.391 km | trajectoire tres cabree |
| `75°` | ~0.253 km | ~0.457 km | quasi verticale, altitude forte, faible portee |

## Objectif Pedagogique

Ce projet permet d'illustrer :

- les lois du mouvement balistique
- le role de la gravite et du frottement de l'air
- l'influence de l'angle initial sur la trajectoire
- la difference entre vol libre et vol controle
- l'interet du controle PID dans un modele dynamique

## Auteur

Felix Nimy

