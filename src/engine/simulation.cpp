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
        enemy.maxHitPoints = spawn.kind == EnemyKind::Heavy
            ? combat::kHeavyHitPoints : combat::kLightHitPoints;
        enemy.hitPoints = enemy.maxHitPoints;
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
    if (player.hitPoints <= 0) return;
    elapsed += dt;

    player.attackCooldown = std::max(0.0, player.attackCooldown - dt);
    player.attackDisplayTime = std::max(0.0, player.attackDisplayTime - dt);
    player.hitFlashTime = std::max(0.0, player.hitFlashTime - dt);

    if (Finite(movement) && Length(movement) > 1e-8) {
        Vec2 direction = Normalized(movement);
        player.facing = direction;
        player.position = level.MoveSliding(
            player.position, direction * (combat::kPlayerSpeed * dt), level.agentRadius);
    }

    if (player.knockbackTime > 0) {
        const Vec2 moved = level.MoveSliding(
            player.position, player.knockbackVelocity * dt, level.agentRadius);
        if (Finite(moved)) player.position = moved;
        player.knockbackTime = std::max(0.0, player.knockbackTime - dt);
        player.knockbackVelocity = player.knockbackVelocity *
            std::exp(-combat::kKnockbackDamping * dt);
    }

    UpdateGameplay(actionPressed, dt);
    for (auto& enemy : enemies) {
        UpdateEnemy(enemy, dt);
        if (player.hitPoints <= 0) break;
    }
}
}
