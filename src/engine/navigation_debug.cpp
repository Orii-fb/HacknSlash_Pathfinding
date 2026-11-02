#include "cpe/simulation.hpp"
namespace cpe {
const char* Name(Algorithm a) { return a == Algorithm::AStar ? "A*" : "Dijkstra"; }
const char* Name(Status s) {
    switch (s) {
        case Status::NotImplemented: return "A IMPLEMENTER";
        case Status::Found: return "Found";
        case Status::NoPath: return "NoPath";
        case Status::OutsideNavMesh: return "OutsideNavMesh";
        case Status::InvalidInput: return "InvalidInput";
    }
    return "Unknown";
}
const char* Name(State s) {
    switch (s) {
        case State::Waiting: return "IA A IMPLEMENTER";
        case State::Idle: return "Idle";
        case State::Chase: return "Chase";
        case State::Windup: return "Windup";
        case State::Recover: return "Recover";
        case State::Dead: return "Dead";
    }
    return "Unknown";
}
void NavMesh::SetSourcePolygons(const std::vector<std::vector<Vec2>>& source) {
    source_ = source;
    polygons_.clear();
    portals_.clear();
    graph_.clear();
    built_ = false;
}
void Navigator::Invalidate() {
    path = Path{};
    next = 0;
}
}
