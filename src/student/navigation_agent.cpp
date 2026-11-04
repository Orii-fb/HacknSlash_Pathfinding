#include "cpe/simulation.hpp"
#include <algorithm>
#include <cmath>

namespace cpe {
namespace {
constexpr double kTargetChangeDistance = 0.12;
constexpr double kWaypointRadius = 0.05;
constexpr double kRetryDelay = 0.25;
constexpr double kReplanInterval = 0.08;
constexpr double kStuckDelay = 0.5;
}

void Navigator::Plan(const Level& level, Vec2 position,
                    Vec2 destination, Algorithm algorithm) {
    if (!Finite(position) || !Finite(destination) ||
        !level.CanStand(position, level.agentRadius)) {
        Invalidate();
        path.search.status = Status::InvalidInput;
        if (Finite(destination)) {
            target = destination;
            hasTarget = true;
            retryAfter = kRetryDelay;
        }
        return;
    }

    const bool targetChanged = !hasTarget ||
        Distance(target, destination) > kTargetChangeDistance;
    if (!targetChanged) {
        if (path.search.status == Status::Found && next < path.points.size())
            return;
        if (path.search.status == Status::Found &&
            Distance(position, target) <= kWaypointRadius)
            return;
        if (retryAfter > 0) return;
    } else if (retryAfter > 0) {
        return;
    }

    target = destination;
    hasTarget = true;
    path = level.navmesh.FindPath(position, destination, algorithm);
    ++requests;
    next = 0;
    stuckFor = 0;
    retryAfter = path.search.status == Status::Found ? kReplanInterval : kRetryDelay;

    if (path.search.status != Status::Found || path.points.empty()) return;
    if (!Finite(path.points.front())) {
        path = Path{};
        path.search.status = Status::InvalidInput;
        return;
    }
    if (Distance(position, path.points.front()) <= kWaypointRadius) next = 1;
}

Vec2 Navigator::Follow(const Level& level, Vec2 position, double speed, double dt) {
    if (!Finite(position)) return position;
    if (std::isfinite(dt) && dt > 0)
        retryAfter = std::max(0.0, retryAfter - dt);

    if (!std::isfinite(speed) || speed <= 0 ||
        !std::isfinite(dt) || dt <= 0 ||
        path.search.status != Status::Found || !Finite(target))
        return position;

    while (next < path.points.size() && !Finite(path.points[next])) {
        path = Path{};
        path.search.status = Status::InvalidInput;
        next = 0;
        retryAfter = kRetryDelay;
        return position;
    }
    while (next < path.points.size() &&
           Distance(position, path.points[next]) <= kWaypointRadius)
        ++next;
    if (next >= path.points.size()) return position;

    const Vec2 waypoint = path.points[next];
    const Vec2 offset = waypoint - position;
    const double distance = Length(offset);
    if (!std::isfinite(distance)) return position;
    const double step = speed * dt;
    if (!std::isfinite(step)) return position;

    const Vec2 moved = level.MoveSliding(
        position, Normalized(offset) * std::min(distance, step), level.agentRadius);
    if (!Finite(moved)) return position;

    const double remaining = Distance(moved, waypoint);
    if (remaining < distance - 1e-5) {
        stuckFor = 0;
    } else {
        stuckFor += dt;
        if (stuckFor >= kStuckDelay) {
            path = Path{};
            path.search.status = Status::NoPath;
            next = 0;
            retryAfter = kRetryDelay;
            stuckFor = 0;
        }
    }

    if (next < path.points.size() &&
        Distance(moved, path.points[next]) <= kWaypointRadius)
        ++next;
    return moved;
}
}
