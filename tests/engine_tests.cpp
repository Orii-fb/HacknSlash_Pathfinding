#include "cpe/simulation.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>
static int checks=0;
static void Check(bool condition,const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
int main() {
    try {
        const std::filesystem::path data=CPE_TEST_DATA_DIR;
        for (const char* filename : {"arena.level","pillars.level"}) {
            cpe::Level level=cpe::Level::Load(data/"levels"/filename);
            Check(level.CanStand(level.playerSpawn,level.agentRadius),"valid player spawn");
            Check(!level.CanStand({100,0},level.agentRadius),"world bounds");
            Check(!level.CanStand({0,0},level.agentRadius),"wall collision");
            Check(!level.CanMove({-5,0},{5,0},level.agentRadius),"no tunnelling through wall");
            Check(!level.Visible({-5,0},{5,0}),"physical wall blocks segment");
            Check(level.CanMove(level.playerSpawn,level.playerSpawn+cpe::Vec2{0,.1},level.agentRadius),
                "movement independent of unfinished navigation");
            Check(!level.navmesh.SourcePolygons().empty(),"source data are loaded for drawing");
            cpe::Simulation simulation(std::move(level));
            auto old=simulation.player.position;
            simulation.Update({0,1},false,1.0/60);
            Check(cpe::Distance(old,simulation.player.position)>0,"player can move before pathfinding");
            Check(simulation.level.CanStand(simulation.player.position,simulation.level.agentRadius),
                "player respects physical collisions");
            simulation.Reset();
            Check(cpe::Near(simulation.player.position,simulation.level.playerSpawn),"reset player");
            Check(simulation.elapsed==0,"reset elapsed time");
        }
        bool rejected=false;
        try { cpe::Level::Load(std::filesystem::path(CPE_TEST_FIXTURE_DIR)/"invalid.level"); }
        catch (const std::exception&) { rejected=true; }
        Check(rejected,"invalid file rejected");
        std::cout << "PASS - " << checks << " engine checks (graphics excluded)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
