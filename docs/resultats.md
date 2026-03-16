# Resultats

## Objectif

Ce document decrit la partie "resultats" du projet, c'est-a-dire toutes
les sorties generees par les simulations.

## Dossier de sortie

Toutes les sorties sont centralisees dans `out/`.

### Logs

Le dossier `out/logs/` contient les resumes texte, par exemple :

- `log.txt`
- `log_compare_angles.txt`
- `log_drag_comparison.txt`

### Images

Le dossier `out/images/` contient les figures exportees, par exemple :

- courbes vitesse/position ;
- trajectoires comparees ;
- captures d'animations ;
- vue PID altitude/angle ;
- vue 3D controlee ;
- vue 3D balistique avec decor.

### Videos

Le dossier `out/videos/` contient les fichiers MP4, par exemple :

- `drag_comparison_animation.mp4`
- `controle_2D_PID_animation.mp4`
- `animation_3D_visualisation.mp4`
- `controle_3D_PID_animation.mp4`
- `controle_3D_PID_ballistique.mp4`

## Lecture des resultats

Les resultats permettent de comparer :

- le temps de vol ;
- la portee ;
- l'altitude maximale ;
- l'effet de la trainee ;
- la stabilite du controle PID ;
- la reponse du systeme face aux perturbations ;
- la difference entre un vol stabilise et un vol balistique guide ;
- le comportement de la trajectoire dans un environnement 3D.
