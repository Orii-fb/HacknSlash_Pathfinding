#include "cpe/simulation.hpp"
#include <cmath>

namespace cpe {
void Simulation::UpdateGameplay(bool actionPressed, double dt) {
    (void)dt;
    if (!actionPressed || player.attackCooldown > 0 || player.hitPoints <= 0) return;

    const Vec2 facing = Normalized(player.facing);
    if (!Finite(facing) || Length(facing) <= 1e-8) return;

    player.facing = facing;
    player.attackCooldown = combat::kPlayerAttackCooldown;
    player.attackDisplayTime = combat::kPlayerAttackDisplayTime;

    for (auto& enemy : enemies) {
        if (enemy.state == State::Dead || !Finite(enemy.position)) continue;
        const Vec2 offset = enemy.position - player.position;
        const double distance = Length(offset);
        if (!std::isfinite(distance) || distance > combat::kPlayerAttackRange) continue;
        if (distance > 1e-8 && Dot(facing, Normalized(offset)) < combat::kPlayerAttackCosine)
            continue;
        if (!level.Visible(player.position, enemy.position)) continue;

        enemy.hitPoints = std::max(0, enemy.hitPoints - combat::kPlayerAttackDamage);
        enemy.hitFlashTime = 0.2;
        const Vec2 knockbackDirection = distance > 1e-8 ?
            Normalized(offset) : facing;
        const double knockbackSpeed = enemy.kind == EnemyKind::Heavy
            ? combat::kHeavyKnockbackSpeed : combat::kLightKnockbackSpeed;
        enemy.knockbackVelocity = knockbackDirection * knockbackSpeed;
        enemy.knockbackTime = combat::kKnockbackDuration;
        enemy.navigator.Invalidate();
        if (enemy.hitPoints == 0) {
            enemy.state = State::Dead;
            enemy.stateTime = 0;
        } else {
            enemy.state = State::Chase;
            enemy.stateTime = 0;
        }
    }
}
}
