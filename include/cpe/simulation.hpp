#pragma once
#include "level.hpp"
#include <cstddef>

namespace cpe {
struct Player { Vec2 position; Vec2 facing{0,-1}; };

// Ce composant expose les données au debug ; le suivi est à implémenter.
struct Navigator {
    Path path;
    std::size_t next = 0;
    std::size_t requests = 0;
    void Invalidate();
    void Plan(const Level& level, Vec2 position, Vec2 target, Algorithm algorithm);
    Vec2 Follow(const Level& level, Vec2 position, double speed, double dt);
};
enum class State { Waiting, Idle, Chase, Windup, Recover, Dead };
const char* Name(State value);

// Entités de démonstration immobiles ; aucun comportement n'est fourni.
// Ajoutez ici les données nécessaires à votre propre IA et votre combat.
struct Enemy {
    Vec2 position;
    EnemyKind kind = EnemyKind::Light;
    State state = State::Waiting;
    Navigator navigator;
};
class Simulation {
public:
    explicit Simulation(Level level);
    Level level;
    Player player;
    std::vector<Enemy> enemies;
    Algorithm algorithm = Algorithm::AStar;
    double elapsed = 0;

    // Fourni : déplacement, collisions, boucle et appels des points d'extension.
    void Update(Vec2 movement, bool actionPressed, double dt);
    void Reset();
    void SetAlgorithm(Algorithm value);
    std::size_t RequestCount() const;

private:
    // À réaliser dans src/student/enemy_ai.cpp et gameplay.cpp.
    void UpdateEnemy(Enemy& enemy, double dt);
    void UpdateGameplay(bool actionPressed, double dt);
};
}
