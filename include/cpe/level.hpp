#pragma once
#include "navigation.hpp"
#include <filesystem>
#include <string>

namespace cpe {
enum class EnemyKind { Light,Heavy };
struct Spawn { Vec2 position;EnemyKind kind; };
struct Wall { Box box;double height=1.5; };
struct Level {
    std::string title;
    double agentRadius=0.25;
    Box bounds;
    Vec2 playerSpawn;
    std::vector<Spawn> spawns;
    std::vector<Wall> walls;
    NavMesh navmesh;
    static Level Load(const std::filesystem::path& file);
    bool CanStand(Vec2 position,double radius)const;
    bool CanMove(Vec2 from,Vec2 to,double radius)const;
    Vec2 MoveSliding(Vec2 from,Vec2 displacement,double radius)const;
    bool Visible(Vec2 from,Vec2 to)const;
};
}
