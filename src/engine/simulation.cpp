#include "cpe/simulation.hpp"
#include <utility>

namespace cpe {
Simulation::Simulation(Level source) : level(std::move(source)) { Reset(); }

void Simulation::Reset() {
    player = Player{};
    player.position = level.playerSpawn;
    enemies.clear();
    elapsed = 0;
    for (const auto& spawn : level.spawns) {
        Enemy enemy;
        enemy.position = spawn.position;
        enemy.kind = spawn.kind;
        switch (enemy.kind)
        {
        case EnemyKind::Light:
            enemy.maxHp = combat::LightHp;
            enemy.speed = combat::LightSpeed;
            enemy.cooldown = combat::LightAttackCooldown;
            enemy.attackRange = combat::LightAttackRange;
            enemy.recovery = combat::LightRecovery;
            enemy.rangeDetection = combat::LightRangeDetection;
            break;
        case EnemyKind::Heavy:
            enemy.maxHp = combat::HeavyHp;
            enemy.speed = combat::HeavySpeed;
            enemy.cooldown = combat::HeavyAttackCooldown;
            enemy.attackRange = combat::HeavyAttackRange;
            enemy.recovery = combat::HeavyRecovery;
            enemy.rangeDetection = combat::HeavyRangeDetection;
            break;
        default:
            break;
        }
        enemies.push_back(enemy);
    }
}

void Simulation::SetAlgorithm(Algorithm value) {
    algorithm = value;
    for (auto& enemy : enemies) enemy.navigator.Invalidate();
}

std::size_t Simulation::RequestCount() const {
    std::size_t count = 0;
    for (const auto& enemy : enemies) count += enemy.navigator.requests;
    return count;
}

void Simulation::Update(Vec2 movement, bool actionPressed, double dt) {
    if (!std::isfinite(dt) || dt <= 0) return;
    dt = std::min(dt, 0.05);
    elapsed += dt;

    if (Finite(movement) && Length(movement) > 1e-8) {
        Vec2 direction = Normalized(movement);
        player.facing = direction;
        player.position = level.MoveSliding(
            player.position, direction * (3.5 * dt), level.agentRadius);
    }

    UpdateGameplay(actionPressed, dt);
    for (auto& enemy : enemies) UpdateEnemy(enemy, dt);
}
bool Enemy::TargetIsInRange(Vec2 to)
{
    if (Distance(position, to) <= attackRange) {
        return true;
    }

    return false;
}
}
