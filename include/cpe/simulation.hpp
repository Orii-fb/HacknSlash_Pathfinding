#pragma once
#include "level.hpp"
#include <cstddef>

namespace cpe {
struct Player {
    Vec2 position;
    Vec2 facing{0,-1};
    int hitPoints = 10;
    int maxHitPoints = 10;
    double attackCooldown = 0;
    double attackDisplayTime = 0;
    double hitFlashTime = 0;
    Vec2 knockbackVelocity;
    double knockbackTime = 0;
};

namespace combat {
constexpr double kPlayerSpeed = 3.5;
constexpr double kPlayerAttackRange = 1.35;
constexpr double kPlayerAttackCosine = 0.5;
constexpr double kPlayerAttackCooldown = 0.45;
constexpr double kPlayerAttackDisplayTime = 0.18;
constexpr int kPlayerAttackDamage = 1;
constexpr double kPlayerKnockbackSpeed = 4.5;

constexpr double kLightSpeed = 2.5;
constexpr double kLightAttackRange = 1.15;
constexpr double kLightWindup = 0.45;
constexpr double kLightRecovery = 0.65;
constexpr double kLightAttackCooldown = 1.2;
constexpr double kLightOrbitSpeed = 0.7;
constexpr int kLightHitPoints = 2;
constexpr int kLightDamage = 1;
constexpr double kLightKnockbackSpeed = 4.0;

constexpr double kHeavySpeed = 1.8;
constexpr double kHeavyAttackRange = 1.55;
constexpr double kHeavyWindup = 0.9;
constexpr double kHeavyRecovery = 1.0;
constexpr double kHeavyAttackCooldown = 1.8;
constexpr int kHeavyHitPoints = 4;
constexpr int kHeavyDamage = 2;
constexpr double kHeavyKnockbackSpeed = 2.8;

constexpr double kKnockbackDuration = 0.18;
constexpr double kKnockbackDamping = 12.0;
}

struct Navigator {
    Path path;
    std::size_t next = 0;
    std::size_t requests = 0;
    Vec2 target;
    bool hasTarget = false;
    double retryAfter = 0;
    double stuckFor = 0;
    void Invalidate();
    void Plan(const Level& level, Vec2 position, Vec2 target, Algorithm algorithm);
    Vec2 Follow(const Level& level, Vec2 position, double speed, double dt);
};
enum class State { Waiting, Idle, Chase, Windup, Recover, Dead };
const char* Name(State value);

// Enemy state and combat timers are consumed by the simulation and debug view.
struct Enemy {
    Vec2 position;
    Vec2 facing{0,-1};
    EnemyKind kind = EnemyKind::Light;
    State state = State::Waiting;
    Navigator navigator;
    int hitPoints = combat::kLightHitPoints;
    int maxHitPoints = combat::kLightHitPoints;
    double stateTime = 0;
    double attackCooldown = 0;
    double hitFlashTime = 0;
    Vec2 knockbackVelocity;
    double knockbackTime = 0;
};
class Simulation {
public:
    explicit Simulation(Level level);
    Level level;
    Player player;
    std::vector<Enemy> enemies;
    Algorithm algorithm = Algorithm::AStar;
    double elapsed = 0;

    // Fixed-step movement, collision handling and gameplay/AI updates.
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
