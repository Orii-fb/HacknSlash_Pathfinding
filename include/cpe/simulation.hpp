#pragma once
#include "level.hpp"
#include <cstddef>

namespace cpe {struct Player {
    Vec2 position; 
    Vec2 facing{0,-1};
    int hp = 10;
    int maxHp = 10;
};
    
    namespace combat
    {
        constexpr double PlayerSpeed = 3.5;
        constexpr double PlayerAttackRange = 1.35;
        constexpr double PlayerAttackDamage = 1;
        constexpr double PlayerAttackCooldown = 0.45;
    
        constexpr double LightSpeed = 2.5;
        constexpr double LightAttackRange = 1.15;
        constexpr double LightAttackDamage = 1;
        constexpr double LightAttackCooldown = 1.2;
        constexpr double LightWindup = 0.45;
        constexpr double LightRecovery = 0.65;
        constexpr double LightHp = 2;
    
        constexpr double HeavySpeed = 1.8;
        constexpr double HeavyAttackRange = 1.55;
        constexpr double HeavyAttackDamage = 2;
        constexpr double HeavyAttackCooldown = 1.8;
        constexpr double HeavyWindup = 0.9;
        constexpr double HeavyRecovery = 1;
        constexpr double HeavyHp = 4;
    }

// Ce composant expose les données au debug ; le suivi est à implémenter.
struct Navigator {
    Path path;
    std::size_t next = 0;
    std::size_t requests = 0;
    bool hasTarget;
    Vec2 lastTarget;

    void Invalidate();
    void Plan(const Level& level, Vec2 position, Vec2 target, Algorithm algorithm);
    Vec2 Follow(const Level& level, Vec2 position, double speed, double dt);
};
enum class State { Waiting, Idle, Chase, Windup, Recover, Dead };
const char* Name(State value);

// Entités de démonstration immobiles ; aucun comportement n'est fourni.
// Ajoutez ici les données nécessaires à votre propre IA et votre combat.
struct Enemy {
    double speed = 2;
    Vec2 position;
    EnemyKind kind = EnemyKind::Light;
    State state = State::Waiting;
    Navigator navigator;
    int hp = combat::LightHp;
    int maxHp = combat::LightHp;
    double attackRange = combat::LightAttackRange;
    double stateTime = 0;
    double cooldown = 0;
    bool TargetIsInRange(Vec2 to);
    
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
