# Format des données du niveau

Les cartes sont des fichiers texte UTF-8 dans data/levels/. Les coordonnées sont exprimées en mètres sur le plan XZ. Les nombres utilisent un point décimal. Un commentaire commence par #.

| Instruction | Signification |
|---|---|
| title texte libre | Nom affiché |
| bounds xmin zmin xmax zmax | Limites physiques |
| radius valeur | Rayon utilisé par les collisions physiques |
| player x z | Position initiale du joueur |
| enemy light x z | Position et type d'un marqueur ennemi |
| enemy heavy x z | Position et type d'un marqueur ennemi |
| wall xmin zmin xmax zmax hauteur | Obstacle rectangulaire solide |
| surface N x1 z1 ... xN zN | Données géométriques sources affichables |

Un seul bounds, radius et player est autorisé. Les coordonnées des murs, les limites, le rayon et les positions physiques initiales sont vérifiés par le chargeur.

Les lignes surface sont lues et stockées pour leur affichage. Le chargeur vérifie leur syntaxe et les valeurs finies. Il ne fournit pas de validation de convexité, de graphe, de portails, de point dans polygone ni de requête de chemin : cela appartient au composant étudiant.

## Exemple minimal

~~~text
title Salle de travail
bounds -5 -5 5 5
radius 0.25
player -2 0
enemy light 2 0
surface 4 -4.7 -4.7 4.7 -4.7 4.7 4.7 -4.7 4.7
~~~

Ces sommets sont seulement des données d'entrée possibles. À vous de décider comment votre composant les valide et les transforme en surface navigable.

## Modifier une scène

Modifiez arena.level ou pillars.level, puis reconstruisez l'application pour recopier les données près de l'exécutable. Pour utiliser directement le dossier source :

~~~bash
./build/campus_path_engine --data ./data
~~~

Sous Windows avec Visual Studio :

~~~powershell
.\build\Release\campus_path_engine.exe --data .\data
~~~

F5 alterne entre les deux fichiers livrés. Pour ajouter une carte, ajoutez-la aux données et modifiez la sélection de scène dans src/engine/main.cpp.

La présence de surfaces d'exemple ne contraint pas votre format final. Vous pouvez étendre les données, les types et le chargeur pour votre projet.
