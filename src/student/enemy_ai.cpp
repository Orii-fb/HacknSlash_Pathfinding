#include "cpe/simulation.hpp"
#include <algorithm>
#include <cmath>

namespace cpe {
namespace {
constexpr double kDetectionRange = 8.0;
constexpr double kDisengageRange = 12.0;
constexpr double kStoppingDistance = 0.9;
}

void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    if (enemy.state == State::Dead) return;

    enemy.attackCooldown = std::max(0.0, enemy.attackCooldown - dt);
    enemy.hitFlashTime = std::max(0.0, enemy.hitFlashTime - dt);

    if (enemy.knockbackTime > 0) {
        const Vec2 moved = level.MoveSliding(
            enemy.position, enemy.knockbackVelocity * dt, level.agentRadius);
        if (Finite(moved)) enemy.position = moved;
        enemy.knockbackTime = std::max(0.0, enemy.knockbackTime - dt);
        enemy.knockbackVelocity = enemy.knockbackVelocity *
            std::exp(-combat::kKnockbackDamping * dt);
        return;
    }

    if (!Finite(enemy.position) || !Finite(player.position)) {
        enemy.navigator.Invalidate();
        enemy.state = State::Idle;
        return;
    }

    const double distance = Distance(enemy.position, player.position);
    if (!std::isfinite(distance)) return;

    const Vec2 toPlayer = player.position - enemy.position;
    if (distance > 1e-8) enemy.facing = Normalized(toPlayer);

    if (enemy.state == State::Windup) {
        enemy.stateTime = std::max(0.0, enemy.stateTime - dt);
        if (enemy.kind == EnemyKind::Light && distance > 1e-8) {
            const Vec2 tangent{-enemy.facing.z, enemy.facing.x};
            const Vec2 moved = level.MoveSliding(
                enemy.position, tangent * (combat::kLightOrbitSpeed * dt), level.agentRadius);
            if (Finite(moved)) enemy.position = moved;
        }
        if (enemy.stateTime <= 0) {
            const double impactDistance = Distance(enemy.position, player.position);
            const double range = enemy.kind == EnemyKind::Heavy
                ? combat::kHeavyAttackRange : combat::kLightAttackRange;
            if (impactDistance <= range && level.Visible(enemy.position, player.position)) {
                const int damage = enemy.kind == EnemyKind::Heavy
                    ? combat::kHeavyDamage : combat::kLightDamage;
                player.hitPoints = std::max(0, player.hitPoints - damage);
                player.hitFlashTime = 0.25;
                const Vec2 awayFromEnemy = impactDistance > 1e-8
                    ? Normalized(player.position - enemy.position)
                    : enemy.facing * -1.0;
                player.knockbackVelocity = awayFromEnemy * combat::kPlayerKnockbackSpeed;
                player.knockbackTime = combat::kKnockbackDuration;
                enemy.attackCooldown = enemy.kind == EnemyKind::Heavy
                    ? combat::kHeavyAttackCooldown : combat::kLightAttackCooldown;
            }
            enemy.state = State::Recover;
            enemy.stateTime = enemy.kind == EnemyKind::Heavy
                ? combat::kHeavyRecovery : combat::kLightRecovery;
        }
        return;
    }

    if (enemy.state == State::Recover) {
        enemy.stateTime = std::max(0.0, enemy.stateTime - dt);
        if (enemy.stateTime <= 0)
            enemy.state = State::Chase;
        return;
    }

    if (enemy.state != State::Chase) {
        enemy.state = State::Idle;
        if (distance > kDetectionRange) return;
        enemy.state = State::Chase;
    } else if (distance > kDisengageRange) {
        enemy.navigator.Invalidate();
        enemy.state = State::Idle;
        return;
    }

    const double attackRange = enemy.kind == EnemyKind::Heavy
        ? combat::kHeavyAttackRange : combat::kLightAttackRange;
    if (distance <= attackRange && enemy.attackCooldown <= 0 &&
        level.Visible(enemy.position, player.position)) {
        enemy.state = State::Windup;
        enemy.stateTime = enemy.kind == EnemyKind::Heavy
            ? combat::kHeavyWindup : combat::kLightWindup;
        enemy.navigator.Invalidate();
        return;
    }

    if (distance <= kStoppingDistance) {
        if (enemy.navigator.path.search.status == Status::Found)
            enemy.navigator.Invalidate();
        return;
    }

    const double speed = enemy.kind == EnemyKind::Heavy
        ? combat::kHeavySpeed : combat::kLightSpeed;
    enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
    enemy.position = enemy.navigator.Follow(level, enemy.position, speed, dt);
}
}
