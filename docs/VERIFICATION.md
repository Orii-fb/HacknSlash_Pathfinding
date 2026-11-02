# Vérification du socle étudiant

Vérifications effectuées le 22 octobre 2026 sous Linux x86-64, avec GCC/G++ 13.3 et CMake 3.31.6.

## État livré

- Dijkstra et A* : fonctions vides, statut NotImplemented.
- NavMesh : construction, localisation, contrôle de segment et requête à implémenter.
- SourcePolygons : stockage brut des données pour le rendu uniquement.
- Navigator : requête et suivi à implémenter.
- IA : point d'entrée vide, entités immobiles.
- Gameplay : point d'entrée vide, aucune attaque, vie, mort, vague ou fin de partie.

Aucun code de la référence formateur n'est inclus dans les modules étudiants.

## Vérifications techniques

| Vérification | Résultat |
|---|---|
| Configuration CMake avec CPE_BUILD_APP=OFF | Réussie |
| Compilation de la bibliothèque et des tests | Réussie |
| Tests du socle fourni | 23 vérifications réussies |
| Tests Dijkstra, A* et NavMesh | 3 tests signalés Skipped, car non implémentés |
| Compilation du code de l'application avec -Wall -Wextra -Wpedantic -Werror | Réussie |
| Édition de liens avec la vraie bibliothèque raylib 5.5 | Réussie |
| Exécution de l'option --help | Réussie |
| Audit des sources propres au kit | Absence des implémentations complètes précédentes |

Les tests du socle vérifient le chargement, les limites, les collisions, l'absence de traversée de mur, le déplacement avant implémentation du NavMesh et la remise à zéro.

Les tests étudiants sont des contrats de résultats. Ils ne contiennent ni solution de Dijkstra/A*, ni oracle de recherche de chemin, ni construction automatique d'un NavMesh. Complétez-les pour couvrir votre projet.

## Limites

Le lancement visuel de la fenêtre et les commandes physiques n'ont pas été validés dans l'environnement de préparation, qui ne dispose pas d'une session graphique exploitable.

Windows et macOS restent à compiler et à valider sur les postes du campus. Le kit fournit les sources et les scripts, pas un exécutable Windows précompilé.

Le socle n'exige pas un NavMesh fonctionnel pour charger un niveau ou déplacer le joueur. Une fois les fonctionnalités développées, les scénarios de navigation, d'IA et de gameplay doivent être ajoutés à la recette.
