# Rapport Succinct d'Analyse des Resultats

## Objet

Ce document presente une lecture rapide des principaux resultats obtenus
avec les simulations balistiques et les commandes PID du projet.

## 1. Trajectoire simple du projectile

D'apres [out/logs/log.txt](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/out/logs/log.txt), le projectile lance a `300 m/s` et `20 deg` atteint environ :

- temps de vol : `8.94 s`
- portee : `0.617 km`
- altitude maximale : `0.109 km`

### Interpretation

La trajectoire reste relativement tendue : la portee est importante par rapport
a l'altitude maximale. Cela correspond bien a un tir a angle faible.
La chaleur produite indique que la trainee dissipe une partie notable
de l'energie mecanique.

## 2. Comparaison de plusieurs angles de tir

D'apres [out/logs/log_compare_angles.txt](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/out/logs/log_compare_angles.txt), on observe :

- `20 deg` : `0.617 km` de portee, `0.109 km` d'altitude max
- `30 deg` : `0.633 km` de portee, `0.184 km` d'altitude max
- `45 deg` : `0.578 km` de portee, `0.295 km` d'altitude max

### Interpretation

Sans trainee, on attend souvent une portee maximale proche de `45 deg`.
Ici, la presence de la trainee modifie ce resultat :

- l'angle `30 deg` donne la meilleure portee parmi les cas testes ;
- l'angle `45 deg` augmente fortement l'altitude et le temps de vol ;
- mais cette augmentation de temps expose plus longtemps le projectile aux pertes dues a l'air.

Conclusion : dans ce modele avec resistance de l'air, un angle intermediaire
est plus favorable a la portee qu'un angle de `45 deg`.

## 3. Comparaison avec et sans trainee

D'apres [out/logs/log_drag_comparison.txt](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/out/logs/log_drag_comparison.txt), les ecarts sont tres marquants :

- avec trainee : `0.795 km` de portee, `0.196 km` d'altitude max, `11.67 s` de vol
- sans trainee : `94.533 km` de portee, `9.398 km` d'altitude max, `80 s` de vol

### Interpretation

La trainee aerodynamique a ici un effet majeur sur la dynamique :

- elle reduit tres fortement la portee ;
- elle limite fortement l'altitude atteinte ;
- elle reduit aussi la duree totale du mouvement utile.

L'ecart tres important entre les deux cas montre que, pour ce type de
simulation, ignorer la resistance de l'air conduit a des resultats tres
optimistes et physiquement peu realistes.

## 4. Controle PID en altitude et en attitude

Le script [src/controle_pid/controle_altitude_angle_PID_animation.m](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/src/controle_pid/controle_altitude_angle_PID_animation.m) produit maintenant l'image :

- [out/images/controle_altitude_angle_PID_animation.png](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/out/images/controle_altitude_angle_PID_animation.png)

### Interpretation

Les courbes montrent qualitativement :

- un suivi de la consigne d'altitude ;
- une correction de l'angle malgre les perturbations ;
- un effet visible des saturations et de l'amortissement ;
- une stabilisation apres les rafales verticales et angulaires.

Cela indique que les gains PID choisis assurent un comportement globalement
stable et une bonne rejection des perturbations dans le cadre du modele.

## 5. Controle PID 2D

Le script 2D produit la video :

- [out/videos/controle_2D_PID_animation.mp4](c:/Users/Felix%20NIMY/OneDrive/Documents/MATLAB/Felix%20Nimy/out/videos/controle_2D_PID_animation.mp4)

### Interpretation

Cette simulation montre que le systeme controle doit gerer en meme temps :

- le deplacement horizontal ;
- l'altitude ;
- l'orientation ;
- les perturbations de vent.

Le comportement reste exploitable visuellement, ce qui suggere que la commande
PID maintient l'ensemble du systeme dans un regime controle, meme en presence
de perturbations successives.

## Conclusion generale

Les resultats confirment trois idees principales :

1. la resistance de l'air influence fortement la trajectoire d'un projectile ;
2. l'angle optimal de portee depend du modele physique retenu ;
3. les commandes PID implementees stabilisent correctement le systeme dans les
   scenarios testes.

Dans l'ensemble, les simulations sont coherentes avec les attentes physiques
et pedagogiquement utiles pour illustrer l'effet des parametres de tir et
des lois de commande.
