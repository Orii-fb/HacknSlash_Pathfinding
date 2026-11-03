#include "cpe/simulation.hpp"
namespace cpe {
void Navigator::Plan(const Level& level, Vec2 position,
                     Vec2 target, Algorithm algorithm) {
    // TODO J4 : gérer la requête, sa temporisation et les échecs.
    if (!Finite(position) && !Finite(target)) {
        Invalidate();
    }

    int locatePos = level.navmesh.Locate(position);
    int locateTarget = level.navmesh.Locate(target);

    if (!locatePos && !locateTarget) {
        Invalidate();
    }

    

    level.CanMove(position, target, level.agentRadius);

    level.MoveSliding(position, target, level.agentRadius);

    (void)level;
    (void)position;
    (void)target;
    (void)algorithm;
}
Vec2 Navigator::Follow(const Level& level, Vec2 position, double speed, double dt) {
    // TODO J4 : consommer les waypoints et appliquer un déplacement valide.
    (void)level;
    (void)speed;
    (void)dt;
    return position;
}
}
