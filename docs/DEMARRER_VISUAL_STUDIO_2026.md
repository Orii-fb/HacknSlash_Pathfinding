# Compiler et développer le moteur avec Visual Studio 2026

**Auteur: William Rao Fernandes**

## 1. Outils nécessaires

Utilisez Visual Studio 2026 avec la charge **Développement Desktop en C++**, les outils MSVC pour x64/x86 et un SDK Windows 10 ou 11. VS Code seul ne suffit pas. Le compilateur est MSVC, pas GCC.

Le moteur fournit une solution native `.sln` : aucune étape de génération CMake n’est nécessaire pour ce parcours. Les sources de raylib 5.5 sont incluses. Un pilote prenant en charge OpenGL 3.3 est nécessaire pour la fenêtre.

Le fichier `.vsconfig` à la racine décrit la charge de travail à installer. Si un outil est absent, passer par Visual Studio Installer ou l’administration de la salle ; une solution seule ne fournit pas le compilateur.

## 2. Ouvrir et compiler

1. Extraire tout le dossier, par exemple dans `C:\Projets\CampusPathEngine_Etudiant`. Ne pas ouvrir la solution depuis le ZIP.
2. Ouvrir `CampusPathEngine.sln` avec Visual Studio 2026, via **Fichier → Ouvrir → Projet/Solution**. Ne pas créer un nouveau projet console et ne pas recopier les sources dans un projet vide.
3. Dans la barre supérieure, sélectionner **Debug** et **x64**.
4. Dans l’Explorateur de solutions, clic droit sur **campus_path_engine → Définir comme projet de démarrage**.
5. Faire **Générer → Générer la solution** (`Ctrl+Maj+B`). La première compilation construit aussi raylib.
6. Lire **Sortie → Génération** en cas d’échec et commencer par la première erreur réelle.
7. Une fois la génération réussie, lancer avec **Ctrl+F5**. Utiliser **F5** pour les points d’arrêt.

Fermer l’application avant de reconstruire si Windows refuse d’écrire l’exécutable.

## 3. Comprendre les projets

| Projet | Rôle |
|---|---|
| `campus_path_engine` | Application : fenêtre, entrées et rendu |
| `cpe_core` | Socle C++ et fichiers à compléter par le binôme |
| `raylib` | Bibliothèque d’affichage et d’entrées compilée depuis ses sources C |
| `cpe_tests` | Tests du socle technique |
| `cpe_student_tests` | Quelques contrats de Dijkstra, A* et NavMesh |

Les références de projets assurent les dépendances : l’application utilise `cpe_core` et `raylib`. Ne pas ajouter leurs `.cpp` à l’application pour « aider » le lien : les mêmes définitions seraient compilées deux fois.

Les résultats de compilation sont dans `bin\x64\Debug\` ou `bin\x64\Release\`. Les objets intermédiaires sont séparés dans `obj\`, par projet et configuration. Le répertoire de travail pour F5 est la racine du moteur ; les données sont également copiées dans `bin/.../data` après génération de l’application.

## 4. Ce que montre la première exécution

Le joueur se déplace et respecte les collisions physiques. Les ennemis sont des marqueurs statiques. Les surfaces affichées sont des données sources, pas un NavMesh calculé. Une sonde de recherche peut afficher `NotImplemented`.

C’est l’état initial attendu : ni poursuite, ni attaque, ni vagues ne sont déjà réalisées. La touche d’action appelle un point d’entrée encore vide.

| Commande | Fonction du socle |
|---|---|
| Flèches, ZQSD ou WASD | Déplacement manuel |
| F1 | Afficher/masquer les surfaces |
| F2 | Afficher/masquer les chemins disponibles |
| F3 | Afficher/masquer le HUD |
| Tab | Choisir Dijkstra/A* pour les requêtes |
| Clic gauche | Sonde de recherche vers une position |
| N ou clic droit | Sélection d’un marqueur ennemi |
| F5 dans la fenêtre du moteur | Changer de scène |
| P | Pause |
| R | Réinitialiser la simulation ; ne retire pas la pause |
| Espace | Point d’entrée gameplay à développer |
| Échap | Quitter |

F5 dans Visual Studio lance le débogage ; F5 lorsque la fenêtre du moteur a le focus change la scène. Le clic gauche n’est pas une commande de déplacement du joueur.

## 5. Exécuter les tests

Pour le socle : définir `cpe_tests` comme projet de démarrage, puis Ctrl+F5. Le kit initial attend `PASS - 23 engine checks (graphics excluded)`.

Pour un contrat étudiant : définir `cpe_student_tests` comme projet de démarrage. Dans **Propriétés du projet → Propriétés de configuration → Débogage → Arguments de commande**, saisir `dijkstra`, puis lancer. Refaire avec `astar` et `navmesh`.

Au départ, `SKIP` et un code de sortie **77** indiquent que la fonctionnalité reste à implémenter. Un projet qui compile et un test ignoré ne démontrent pas que l’algorithme fonctionne.

Depuis un terminal de développement, à la racine du moteur :

```bat
msbuild CampusPathEngine.sln /m:2 /p:Configuration=Debug /p:Platform=x64
bin\x64\Debug\cpe_tests.exe
bin\x64\Debug\cpe_student_tests.exe dijkstra
bin\x64\Debug\cpe_student_tests.exe astar
bin\x64\Debug\cpe_student_tests.exe navmesh
```

Ces programmes sont des exécutables console, pas des tests découverts automatiquement par l’Explorateur de tests de Visual Studio. Les scénarios jouables et vos tests supplémentaires restent à écrire.

## 6. Où implémenter le projet ?

Dans `cpe_core`, le filtre **Travail etudiant** regroupe :

| Source | Travail |
|---|---|
| `src/student/pathfinding.cpp` | Dijkstra et A* |
| `src/student/navmesh.cpp` | Validation, régions, portails, graphe et requêtes |
| `src/student/navigation_agent.cpp` | Planification et suivi |
| `src/student/enemy_ai.cpp` | Perception, décisions et comportements |
| `src/student/gameplay.cpp` | Combat, vie, vagues et fin de partie |

Les contrats sont dans `include/cpe/navigation.hpp` et `include/cpe/simulation.hpp`. `docs/TRAVAIL_ETUDIANT.md` précise les responsabilités.

Le laboratoire utilisait `Point`, `NavPath` et des fonctions globales. Le moteur utilise `cpe::Vec2`, `cpe::Path` et d’autres signatures. Son A* reçoit un tableau d’estimations, pas une lambda. Adapter le code compris en cours au contrat du moteur ; ne pas recopier les en-têtes du laboratoire dans le projet.

### Ajouter une nouvelle source

Clic droit sur **cpe_core → Ajouter → Nouvel élément**. Enregistrer le nouveau fichier dans `src/student/`, l’ajouter à `cpe_core` une seule fois, et placer sa déclaration dans un en-tête approprié. Si la portabilité CMake est conservée, ajouter aussi la source à `CMakeLists.txt`.

Inclure les `.hpp`, jamais les `.cpp`. Ne pas recréer les types existants et ne pas ajouter un second `main` à l’application. Les définitions non `inline` doivent se trouver dans un seul fichier source.

## 7. Déboguer

Poser un point d’arrêt dans le point d’entrée étudiant choisi, lancer avec F5, puis déclencher son appel dans le jeu. Au départ, `NavMesh::Build` est appelé au chargement et la sonde peut appeler `FindPath`. Les hooks de simulation sont invoqués par la boucle de jeu.

Pour les tests, placer un point d’arrêt dans `Dijkstra` et lancer le contrat avec l’argument `dijkstra`. Inspecter les paramètres, le graphe et la valeur retournée.

## 8. Dépannage

| Message ou symptôme | Action |
|---|---|
| MSB8020, outils v145 absents | Installer les outils C++ de VS 2026 ; ne pas basculer arbitrairement de compilateur |
| MSB8036, SDK absent | Installer un SDK Windows compatible ou recibler sur un SDK déjà installé |
| `raylib.h` introuvable | Vérifier l’extraction complète et ouvrir la solution fournie |
| LNK2005, déjà défini | Rechercher une définition dupliquée ou un `.cpp` inclus |
| LNK2019, symbole externe non résolu | Vérifier signature et présence du `.cpp` dans le bon projet |
| `pch.h` exigé après ajout d’un projet personnel | Utiliser les projets fournis, qui n’exigent pas d’en-tête précompilé |
| L’exécutable ne s’écrit plus | Fermer l’ancienne instance du jeu |
| Les ennemis ne font rien | Normal avant les implémentations ; inspecter les points d’entrée |
| Aucune fenêtre malgré une génération réussie | Lire la sortie console, vérifier projet de démarrage et pilote OpenGL |

## 9. Versionnement et livraison

Versionner les sources, en-têtes, données, `.sln`, `.vcxproj`, `.filters`, documentation et tests. Ne pas versionner `.vs/`, `bin/`, `obj/` ou les réglages `.user` personnels. Garder les licences des dépendances.

Pour livrer un exécutable, compiler en Release x64 et fournir l’exécutable avec `data/`, puis tester sur un poste cible. Les projets utilisent le runtime MSVC dynamique : le redistribuable Visual C++ x64 adapté doit être présent sur un poste de diffusion. Le poste de développement avec Visual Studio dispose normalement des composants requis.

## 10. Vérification de ce kit

La structure des projets, leurs chemins et les références ont été contrôlés. Les tests du socle C++ sont vérifiés séparément sous Linux. Cette préparation ne constitue pas une compilation MSVC ni une validation graphique Windows : effectuer la première génération et le lancement sur un poste de la salle avant la présentation.

Référence du toolset : [CMake — Visual Studio 18 2026](https://cmake.org/cmake/help/v4.2/generator/Visual%20Studio%2018%202026.html). Les projets natifs fournis ciblent v145. CMake reste une voie de portabilité et ne fait pas partie du parcours de compilation décrit ici.
