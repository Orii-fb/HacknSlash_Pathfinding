#include "cpe/simulation.hpp"
namespace cpe {

void Navigator::Plan(const Level& level, Vec2 position,
                     Vec2 target, Algorithm algorithm) {
    // TODO J4 : gérer la requête, sa temporisation et les échecs.
    if (!Finite(position) || !Finite(target)) {
        path.search.status = Status::InvalidInput;
        Invalidate();
        return;

        if (Finite(target)) {
            hasTarget = true;
        }
    }

    int locatePos = level.navmesh.Locate(position);
    int locateTarget = level.navmesh.Locate(target);
    bool bothCanStand = !level.CanStand(position, level.agentRadius && !level.CanStand(position, level.agentRadius));

    if (!locatePos || !locateTarget || bothCanStand) {
        Invalidate();
        return;
    }
    bool hasPath = path.search.status == Status::Found;
    bool targetMove = !hasTarget || Distance(target, lastTarget) > 0.75;
    bool hasFinished = next >= path.points.size() && Distance(position, target);
    
    if (!hasPath && !targetMove && !hasFinished) {
        return;
    }

    path = level.navmesh.FindPath(position, target, algorithm);

    next = path.points.size() > 1 ? 1 : path.points.size();

    lastTarget = target;

    hasTarget = true;

    requests++;
}


Vec2 Navigator::Follow(const Level& level, Vec2 position, double speed, double dt) {
    // TODO J4 : consommer les waypoints et appliquer un déplacement valide.
    double budget = speed * dt;
    Vec2 waypoint = path.points[next];
    double distance = Distance(path.points[next], position);


    if (level.CanMove(position, lastTarget, level.agentRadius)) {
        level.MoveSliding(position, lastTarget, level.agentRadius);
    }

    return position;
}
}
