# Répartition entre moteur et projet

## Fourni dans src/engine

- Fenêtre, caméra et rendu via raylib.
- Commandes et boucle de simulation à pas fixe.
- Chargement des données.
- Collisions entre disque et murs rectangulaires.
- Déplacement manuel et glissement du joueur.
- Création de marqueurs aux positions initiales.
- Stockage des données de surface pour le debug.
- Affichage des statuts et des résultats produits par les modules.

Le mouvement du joueur ne consulte pas le NavMesh. Le socle reste utilisable lorsque celui-ci est vide.

Les outils de collision disponibles sont des primitives physiques. Ils ne calculent ni trajet, ni suite de régions, ni décision d'agent.

## À développer dans src/student

- Recherches Dijkstra et A*.
- Composant NavMesh.
- Planification et suivi.
- IA ennemie.
- Gameplay du Hack'n Slash.

Les en-têtes dans include/cpe exposent les types de données et les signatures. Vous pouvez les faire évoluer.

## Appels déjà branchés

- Le chargement fournit les données à SetSourcePolygons, puis appelle Build.
- Le clic gauche appelle FindPath.
- Le rendu lit IsBuilt, Polygons, Portals et les points de Path.
- La simulation appelle UpdateGameplay, puis UpdateEnemy pour chaque entité.
- Votre IA devra appeler Navigator::Plan et Navigator::Follow selon ses décisions.

Il n'y a pas de planification ou de suivi implicite ailleurs dans le moteur. Un ennemi ne bouge pas tant que votre code ne modifie pas sa position.

## Statuts

NotImplemented indique une fonctionnalité encore absente ; il est distinct de NoPath et InvalidInput.

Retournez Found uniquement lorsqu'un résultat est réellement calculé. Une fonction vide ne doit pas être présentée comme un échec normal du monde de jeu.

Les métriques sont des champs à renseigner par les modules étudiants ; le rendu se contente de les afficher.

## Bibliothèque de calcul

cpe_core ne dépend pas de raylib. Les tests peuvent être exécutés avec CPE_BUILD_APP=OFF sans accès à une session graphique.

La bibliothèque raylib embarquée fournit le rendu, les entrées et des primitives mathématiques. Elle n'apporte pas les fonctionnalités de navigation évaluées dans ce kit.
