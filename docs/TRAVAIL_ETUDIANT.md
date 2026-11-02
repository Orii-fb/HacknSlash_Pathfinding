# Travail à réaliser en C++

Ce document décrit les points d'extension et les résultats attendus. Les implémentations des fonctionnalités du projet sont absentes.

## 1. Dijkstra

Fichier : **src/student/pathfinding.cpp**.

La fonction reçoit un graphe, un départ et une arrivée. Votre résultat doit distinguer succès, absence de chemin et entrée invalide. Un succès contient les identifiants de sommets dans l'ordre du déplacement ainsi qu'un coût cohérent.

Traitez notamment départ égal à arrivée, destination inaccessible, poids nul, poids négatif et identifiant invalide. Renseignez les métriques demandées dans le module.

La structure Graph est un type de données fourni. Elle ne construit pas le graphe de votre niveau.

## 2. A*

Même fichier. La fonction reçoit en plus un vecteur d'heuristiques indexé par sommet.

Définissez et vérifiez son contrat, puis justifiez le choix de l'heuristique dans votre rapport. Comparez les résultats avec Dijkstra sur les mêmes graphes et les mêmes coûts. Le test fourni vérifie un graphe pondéré ; ajoutez vos propres cas.

Aucune fonction de recherche de la version formateur n'est présente dans le kit.

## 3. NavMesh

Fichier : **src/student/navmesh.cpp**.

Les données sources sont accessibles par source_. Leur lecture est déjà assurée par le moteur ; elles ne sont pas une structure de navigation exploitable.

À vous de développer :

- Validation et représentation des polygones navigables.
- Localisation d'un point et contrôle des segments.
- Construction des adjacences, des portails et du graphe pondéré.
- Recherche d'une suite de régions.
- Construction d'un trajet géométrique valide.
- Gestion du rayon et des cas d'échec.

Les champs polygons_, portals_, graph_ et built_ servent au stockage et au debug. Ils sont vides au départ. Sur succès de Build, définissez built_=true et retournez true. Définissez aussi une politique de rejet pour les données invalides.

FindPath est appelée par le clic de sonde du moteur. Les chemins et portails que vous y produisez sont affichés par le rendu fourni.

## 4. Suivi de chemin

Fichier : **src/student/navigation_agent.cpp**.

Plan et Follow sont des points d'entrée vides. Vous devez définir les conditions de requête, la temporisation, le stockage et la consommation du trajet, ainsi que la gestion d'un échec ou d'un blocage.

Le composant retourne la position initiale tant que vous ne l'avez pas développé. Les collisions physiques sont disponibles via Level::CanMove et Level::MoveSliding ; elles ne constituent pas un système de navigation.

## 5. IA ennemie

Fichier : **src/student/enemy_ai.cpp**.

UpdateEnemy est appelée à chaque pas de simulation. Ajoutez les données de votre agent dans simulation.hpp, puis implémentez perception, décisions, transitions et interaction avec votre composant de navigation.

L'énumération State fournit des noms lisibles par le debug. Elle ne contient aucune transition ni règle de décision. Le type light/heavy des données est seulement un marqueur et une différence visuelle ; les comportements distincts sont à programmer.

## 6. Gameplay

Fichier : **src/student/gameplay.cpp**.

UpdateGameplay reçoit la pression de la touche d'action et le temps écoulé. À vous de développer les attaques, dégâts, points de vie, morts, vagues, victoire et défaite. Aucun de ces systèmes n'est préprogrammé.

La fonction Reset du socle remet les positions et le temps à zéro. Étendez-la pour réinitialiser votre propre état de partie, vos compteurs et vos composants.

## 7. Validation du projet

Les tests de contrats vérifient quelques résultats, pas la totalité des exigences. Ajoutez des tests de calcul, des scénarios reproductibles et des mesures.

Conservez une démonstration jouable à chaque étape. Le rendu, les collisions et le chargement doivent rester utilisables même lorsqu'un module étudiant est encore incomplet.

Le CDC reste la référence pour le périmètre, les livrables, le calendrier en huit séances et le barème.
