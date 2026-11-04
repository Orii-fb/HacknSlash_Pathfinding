#include "cpe/simulation.hpp"
#include <cmath>

namespace cpe {
namespace {
constexpr double kDetectionRange = 8.0;
constexpr double kDisengageRange = 12.0;
constexpr double kStoppingDistance = 0.9;
constexpr double kLightSpeed = 2.5;
constexpr double kHeavySpeed = 1.8;
}

void Simulation::UpdateEnemy(Enemy& enemy, double dt) {
    if (enemy.state == State::Dead ||
        enemy.state == State::Windup ||
        enemy.state == State::Recover)
        return;

    if (!Finite(enemy.position) || !Finite(player.position)) {
        enemy.navigator.Invalidate();
        enemy.state = State::Idle;
        return;
    }

    const double distance = Distance(enemy.position, player.position);
    if (!std::isfinite(distance)) return;

    if (enemy.state != State::Chase) {
        enemy.state = State::Idle;
        if (distance > kDetectionRange) return;
        enemy.state = State::Chase;
    } else if (distance > kDisengageRange) {
        enemy.navigator.Invalidate();
        enemy.state = State::Idle;
        return;
    }

    if (distance <= kStoppingDistance) {
        if (enemy.navigator.path.search.status == Status::Found)
            enemy.navigator.Invalidate();
        return;
    }

    const double speed = enemy.kind == EnemyKind::Heavy ? kHeavySpeed : kLightSpeed;
    enemy.navigator.Plan(level, enemy.position, player.position, algorithm);
    enemy.position = enemy.navigator.Follow(level, enemy.position, speed, dt);
}
}
