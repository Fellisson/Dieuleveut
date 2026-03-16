# Projet MATLAB - Balistique et Controle PID

## Vue d'ensemble

Ce dossier contient plusieurs scripts et fonctions MATLAB pour simuler :

- le mouvement d'un projectile avec ou sans resistance de l'air ;
- la comparaison de trajectoires pour plusieurs angles de tir ;
- le controle PID d'un vehicule en altitude et en 2D, avec animations.

Le projet semble avoir une vocation pedagogique : il melange modelisation physique,
visualisation, animation et generation de sorties graphiques ou video.

## Fichiers principaux

### Balistique

- `projectile_trajectory.m`
  Simulation d'une trajectoire de projectile avec gravite et trainee aerodynamique.
  Le fichier calcule la trajectoire, trace plusieurs courbes, anime le mouvement
  et enregistre des resultats dans un fichier log.

- `projectile_trajectory_initial.m`
  Copie actuelle de `projectile_trajectory.m`. Ce fichier semble conserver
  une version initiale du meme travail.

- `compare_projectile_angles.m`
  Compare plusieurs trajectoires pour une meme vitesse initiale et des angles
  de tir differents. Le script produit une figure de synthese et un fichier log.

- `projectile_trajectory_drag_comparison.m`
  Compare deux cas de propulsion balistique : avec trainee et sans trainee.
  Le fichier affiche les vitesses, positions, trajectoires, masse et poussee.

- `projectile_trajectory_drag_comparison_video.m`
  Variante orientee animation/video du comparatif avec et sans trainee.
  Le resultat attendu est une video MP4.

### Controle PID

- `controle_altitude_angle_PID_animation.m`
  Simule un systeme vertical controle par deux boucles PID :
  une pour l'altitude et une pour l'angle. Le script inclut des perturbations
  et une animation de la reponse.

- `controle_2D_PID_animation_video_01.m`
  Etend le controle a un mouvement 2D avec perturbations horizontales,
  verticales et angulaires. Le script genere egalement une video MP4.

## Organisation logique du code

On retrouve deux styles de fichiers dans ce dossier :

- des fonctions MATLAB, surtout pour la partie balistique ;
- des scripts executables, surtout pour la partie controle PID.

La partie balistique est generalement structuree en sous-fonctions :

- calcul de trajectoire ;
- calcul de grandeurs physiques ;
- traces et animations.

La partie PID suit plutot cette logique :

- definition des parametres physiques ;
- definition des consignes ;
- reglage des gains PID ;
- simulation par integration d'Euler ;
- animation et visualisation des resultats.

## Sorties produites

Selon le fichier execute, le projet peut produire :

- des figures MATLAB ;
- des images dans un dossier `images` ;
- des journaux texte dans un dossier `out` ;
- des videos MP4 dans un dossier `videos`.

## Remarques actuelles

- Plusieurs chemins de sortie sont relatifs et parfois incoherents selon le
  dossier d'execution.
- Certains fichiers utilisent `cd(...)` pour changer de repertoire, ce qui rend
  l'execution moins robuste.
- Quelques valeurs sont codees en dur, par exemple la vitesse initiale ou
  l'angle de tir.
- `projectile_trajectory_initial.m` duplique actuellement `projectile_trajectory.m`.

## Utilisation rapide

Dans MATLAB, depuis ce dossier, on peut typiquement lancer :

```matlab
compare_projectile_angles
projectile_trajectory
projectile_trajectory_drag_comparison
projectile_trajectory_drag_comparison_video
```

Pour les scripts PID :

```matlab
controle_altitude_angle_PID_animation
controle_2D_PID_animation_video_01
```

## Etape suivante recommandee

Une fois la documentation en place, la suite la plus utile serait :

1. uniformiser les chemins de sortie ;
2. factoriser les parametres communs ;
3. separer clairement calcul, affichage et sauvegarde.
