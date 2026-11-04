#include "cpe/simulation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// Contrats de résultat uniquement : aucune implémentation d'algorithme.
static void Check(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static int SearchContract(bool astar) {
    cpe::Graph graph(6);
    auto add=[&](int a,int b,double cost) {
        graph[a].push_back({b,cost});
        graph[b].push_back({a,cost});
    };
    add(0,1,2); add(0,2,5); add(1,2,1); add(1,3,4);
    add(2,3,1); add(2,4,7); add(3,4,3);
    auto query=[&](const cpe::Graph& g,int from,int to) {
        if (astar) {
            std::vector<double> h(g.size(),0.0);
            if (to==4 && g.size()==6) h={7,5,4,3,0,0};
            return cpe::AStar(g,from,to,h);
        }
        return cpe::Dijkstra(g,from,to);
    };
    auto result=query(graph,0,4);
    if (result.status==cpe::Status::NotImplemented) {
        std::cout << "SKIP - " << (astar?"A*":"Dijkstra") << " a implementer\n";
        return 77;
    }
    Check(result.status==cpe::Status::Found,"weighted graph should be reachable");
    Check(std::isfinite(result.cost)&&std::abs(result.cost-7)<1e-4,"expected cost is 7");
    Check(result.nodes.size()>=2 && result.nodes.front()==0 && result.nodes.back()==4,
        "path endpoints");
    double sum=0;
    for (std::size_t i=1;i<result.nodes.size();++i) {
        int a=result.nodes[i-1],b=result.nodes[i];
        Check(a>=0 && a<static_cast<int>(graph.size()),"valid node in path");
        bool exists=false;
        for (const auto& edge:graph[a]) if (edge.to==b) {sum+=edge.cost; exists=true; break;}
        Check(exists,"path must use actual graph edges");
    }
    Check(std::abs(sum-result.cost)<1e-4,"path cost matches returned cost");
    Check(query(graph,0,5).status==cpe::Status::NoPath,"isolated node");
    Check(query(graph,-1,4).status==cpe::Status::InvalidInput,"invalid index");
    auto same=query(graph,0,0);
    Check(same.status==cpe::Status::Found && same.cost==0,"same start goal");
    Check(result.expanded>0 && std::isfinite(result.microseconds),
        "search metrics should be populated");
    graph[0][0].cost=-1;
    Check(query(graph,0,4).status==cpe::Status::InvalidInput,"negative edge");
    graph[0][0].cost=2;
    graph[5].push_back({6,1});
    Check(query(graph,0,4).status==cpe::Status::InvalidInput,"invalid edge endpoint");
    graph[5].clear();
    if (astar) {
        Check(cpe::AStar(graph,0,4,{0,0}).status==cpe::Status::InvalidInput,
            "heuristic size");
        auto invalidGoalHeuristic=std::vector<double>(graph.size(),0.0);
        invalidGoalHeuristic[4]=1.0;
        Check(cpe::AStar(graph,0,4,invalidGoalHeuristic).status==
            cpe::Status::InvalidInput,"heuristic must be zero at goal");

        cpe::Graph reopening(4);
        reopening[0]={{1,2.0},{2,1.0}};
        reopening[1]={{3,2.0}};
        reopening[2]={{1,0.5},{3,100.0}};
        auto reopened=cpe::AStar(reopening,0,3,{3.5,0.0,2.5,0.0});
        Check(reopened.status==cpe::Status::Found &&
            std::abs(reopened.cost-3.5)<1e-4,"A* should reopen improved nodes");
    }
    std::cout << "PASS - search result contract\n";
    return 0;
}
static int NavMeshContract() {
    cpe::NavMesh mesh;
    mesh.SetSourcePolygons({
        {{0,0},{4,0},{4,2},{0,2}},
        {{4,0},{6,0},{6,2},{4,2}},
        {{4,2},{6,2},{6,6},{4,6}},
        {{8,0},{10,0},{10,2},{8,2}}
    });
    if (!mesh.Build()) {
        std::cout << "SKIP - NavMesh::Build a implementer\n";
        return 77;
    }
    Check(mesh.IsBuilt(),"Build must set built_ on success");
    Check(mesh.Portals().size()==2,"two portals in the conforming L fixture");
    Check(mesh.Locate({1,1})>=0,"inside point");
    Check(mesh.Locate({3,3})==-1,"outside point");
    Check(mesh.SegmentInside({1,1},{5,1}),"horizontal segment inside the surface");
    Check(!mesh.SegmentInside({1,1},{5,5}),"do not cut the L corner");
    for (auto algorithm : {cpe::Algorithm::Dijkstra,cpe::Algorithm::AStar}) {
        auto path=mesh.FindPath({1,1},{5,5},algorithm);
        Check(path.search.status==cpe::Status::Found,"route through L");
        Check(!path.points.empty(),"non-empty geometric route");
        Check(cpe::Near(path.points.front(),{1,1}) && cpe::Near(path.points.back(),{5,5}),
            "geometric endpoints");
        for (std::size_t i=1;i<path.points.size();++i)
            Check(mesh.SegmentInside(path.points[i-1],path.points[i]),"valid route segments");
        Check(mesh.FindPath({1,1},{9,1},algorithm).search.status==cpe::Status::NoPath,
            "disconnected island");
        Check(mesh.FindPath({1,1},{3,3},algorithm).search.status==cpe::Status::OutsideNavMesh,
            "outside destination");
    }
    const auto directPath=mesh.FindPath({1,1},{5,1},cpe::Algorithm::Dijkstra);
    Check(directPath.search.status==cpe::Status::Found &&
        directPath.points.size()==2 && std::abs(directPath.length-4.0)<1e-4,
        "path smoothing avoids unnecessary polygon-center detours");

    cpe::Level level;
    level.bounds={{0,0},{8,4}};
    level.agentRadius=0.25;
    level.navmesh.SetSourcePolygons({{{0,0},{8,0},{8,4},{0,4}}});
    Check(level.navmesh.Build(),"navigator test navmesh");
    cpe::Navigator navigator;
    navigator.Plan(level,{1,1},{7,1},cpe::Algorithm::Dijkstra);
    Check(navigator.path.search.status==cpe::Status::Found,"navigator plans a route");
    Check(navigator.requests==1,"first plan makes one request");
    navigator.Plan(level,{1,1},{7,1},cpe::Algorithm::Dijkstra);
    Check(navigator.requests==1,"unchanged target does not replan");

    cpe::Vec2 position{1,1};
    for (int step=0;step<100 && cpe::Distance(position,{7,1})>0.05;++step) {
        position=navigator.Follow(level,position,2.0,0.05);
        Check(level.CanStand(position,level.agentRadius),"navigator respects collisions");
    }
    Check(cpe::Distance(position,{7,1})<=0.05,"navigator reaches its destination");
    navigator.Plan(level,position,{7,1},cpe::Algorithm::Dijkstra);
    Check(navigator.requests==1,"completed route does not replan");

    navigator.Invalidate();
    navigator.Plan(level,position,{9,1},cpe::Algorithm::Dijkstra);
    Check(navigator.path.search.status==cpe::Status::OutsideNavMesh,
        "navigator preserves planning failure");
    navigator.Plan(level,position,{9,1},cpe::Algorithm::Dijkstra);
    Check(navigator.requests==2,"failed route is throttled");

    cpe::Level blockedLevel=level;
    blockedLevel.walls.push_back({{{3,0},{4,2}},1.5});
    cpe::Navigator blockedNavigator;
    blockedNavigator.Plan(blockedLevel,{1,1},{7,1},cpe::Algorithm::Dijkstra);
    position={1,1};
    for (int step=0;step<30 &&
         blockedNavigator.path.search.status==cpe::Status::Found;++step)
        position=blockedNavigator.Follow(blockedLevel,position,2.0,0.05);
    Check(blockedNavigator.path.search.status==cpe::Status::NoPath,
        "blocked navigator abandons its route");
    const std::size_t blockedRequests=blockedNavigator.requests;
    for (int step=0;step<6;++step)
        position=blockedNavigator.Follow(blockedLevel,position,2.0,0.05);
    blockedNavigator.Plan(blockedLevel,position,{7,1},cpe::Algorithm::Dijkstra);
    Check(blockedNavigator.requests==blockedRequests+1,
        "blocked navigator retries after its cooldown");

    cpe::Level chaseLevel=level;
    chaseLevel.bounds={{0,0},{24,4}};
    chaseLevel.navmesh.SetSourcePolygons({{{0,0},{24,0},{24,4},{0,4}}});
    Check(chaseLevel.navmesh.Build(),"wide pursuit navmesh");
    chaseLevel.playerSpawn={6,2};
    chaseLevel.spawns={{{1,2},cpe::EnemyKind::Light}};
    cpe::Simulation simulation(chaseLevel);
    const cpe::Vec2 initialEnemyPosition=simulation.enemies[0].position;
    simulation.Update({},false,0.05);
    Check(simulation.enemies[0].state==cpe::State::Chase,
        "enemy detects and begins pursuing the player");
    Check(cpe::Distance(simulation.enemies[0].position,initialEnemyPosition)>0,
        "pursuing enemy follows its path");
    Check(simulation.RequestCount()==1,"pursuit plans a path");
    simulation.Update({},false,0.05);
    Check(simulation.RequestCount()==1,"pursuit reuses its route between replans");

    double maximumEnemyStep=0;
    for (int step=0;step<100;++step) {
        const cpe::Vec2 before=simulation.enemies[0].position;
        simulation.Update({1,0},false,0.05);
        maximumEnemyStep=std::max(maximumEnemyStep,
            cpe::Distance(before,simulation.enemies[0].position));
    }
    Check(simulation.RequestCount()>1,
        "moving player causes pursuit routes to be updated");
    Check(maximumEnemyStep<=2.5*0.05+1e-6,
        "light enemy stays slower than the player");
    Check(cpe::Distance(simulation.enemies[0].position,simulation.player.position)>1.5,
        "moving player can maintain distance from a pursuing enemy");

    chaseLevel.spawns={{{1,2},cpe::EnemyKind::Heavy}};
    cpe::Simulation heavySimulation(chaseLevel);
    heavySimulation.Update({},false,0.05);
    const cpe::Vec2 heavyStart=heavySimulation.enemies[0].position;
    heavySimulation.Update({},false,0.05);
    Check(cpe::Distance(heavySimulation.enemies[0].position,heavyStart)<=1.8*0.05+1e-6,
        "heavy enemy moves at a slower pace");

    simulation.player.position={40,2};
    simulation.Update({},false,0.05);
    Check(simulation.enemies[0].state==cpe::State::Idle,
        "enemy disengages after the player leaves pursuit range");

    simulation.enemies[0].state=cpe::State::Dead;
    simulation.player.position={2,2};
    const cpe::Vec2 deadEnemyPosition=simulation.enemies[0].position;
    simulation.Update({},false,0.05);
    Check(simulation.enemies[0].state==cpe::State::Dead &&
        cpe::Near(simulation.enemies[0].position,deadEnemyPosition),
        "dead enemy does not resume pursuit");

    cpe::Level combatLevel;
    combatLevel.bounds={{-5,-5},{5,5}};
    combatLevel.agentRadius=0.25;
    combatLevel.playerSpawn={-1,0};
    combatLevel.spawns={{{0,0},cpe::EnemyKind::Light}};
    combatLevel.navmesh.SetSourcePolygons({{{-5,-5},{5,-5},{5,5},{-5,5}}});
    Check(combatLevel.navmesh.Build(),"combat test navmesh");
    cpe::Simulation combatSimulation(combatLevel);
    combatSimulation.player.facing={1,0};
    const cpe::Vec2 enemyBeforeKnockback=combatSimulation.enemies[0].position;
    combatSimulation.Update({},true,0.05);
    Check(combatSimulation.enemies[0].hitPoints==cpe::combat::kLightHitPoints-1,
        "player attack damages an enemy in its forward range");
    Check(combatSimulation.enemies[0].position.x>enemyBeforeKnockback.x &&
        combatSimulation.enemies[0].knockbackTime>0,
        "player attack knocks a living enemy away");
    combatSimulation.Update({},true,0.05);
    Check(combatSimulation.enemies[0].hitPoints==cpe::combat::kLightHitPoints-1,
        "player attack cooldown prevents duplicate hits");
    for (int step=0;step<30 &&
         combatSimulation.player.hitPoints==combatSimulation.player.maxHitPoints;++step)
        combatSimulation.Update({},false,0.05);
    Check(combatSimulation.player.hitPoints<combatSimulation.player.maxHitPoints,
        "enemy impact damages the player after its windup");
    const cpe::Vec2 playerAfterImpact=combatSimulation.player.position;
    for (int step=0;step<4;++step) combatSimulation.Update({},false,0.05);
    Check(cpe::Distance(combatSimulation.player.position,playerAfterImpact)>0.1,
        "enemy impact knocks the player away");
    for (int step=0;step<12;++step) combatSimulation.Update({},false,0.05);
    combatSimulation.player.position={-1,0};
    combatSimulation.enemies[0].position={0,0};
    combatSimulation.enemies[0].knockbackVelocity={};
    combatSimulation.enemies[0].knockbackTime=0;
    combatSimulation.enemies[0].state=cpe::State::Chase;
    combatSimulation.Update({},true,0.05);
    Check(combatSimulation.enemies[0].state==cpe::State::Dead &&
        combatSimulation.enemies[0].hitPoints==0,
        "player can kill an enemy and killed enemies stop acting");
    const cpe::Vec2 slainPosition=combatSimulation.enemies[0].position;
    for (int step=0;step<5;++step) combatSimulation.Update({},false,0.05);
    Check(cpe::Near(combatSimulation.enemies[0].position,slainPosition),
        "dead enemy remains stationary");
    combatSimulation.Reset();
    Check(combatSimulation.player.hitPoints==combatSimulation.player.maxHitPoints &&
        combatSimulation.enemies[0].hitPoints==combatSimulation.enemies[0].maxHitPoints &&
        combatSimulation.enemies[0].state==cpe::State::Waiting,
        "reset restores combat health and enemy state");

    cpe::Simulation behindTarget(combatLevel);
    behindTarget.enemies[0].position={-1,0.8};
    behindTarget.player.facing={1,0};
    behindTarget.Update({},true,0.05);
    Check(behindTarget.enemies[0].hitPoints==behindTarget.enemies[0].maxHitPoints,
        "melee attack does not hit a target behind the player");

    cpe::Level blockedCombatLevel=combatLevel;
    blockedCombatLevel.walls.push_back({{{-0.1,-1},{0.1,1}},1.5});
    cpe::Simulation blockedAttack(blockedCombatLevel);
    blockedAttack.player.facing={1,0};
    blockedAttack.Update({},true,0.05);
    Check(blockedAttack.enemies[0].hitPoints==blockedAttack.enemies[0].maxHitPoints,
        "wall blocks the player's melee attack");

    cpe::Level knockbackWallLevel=combatLevel;
    knockbackWallLevel.playerSpawn={-2,0};
    knockbackWallLevel.spawns={{{-1,0},cpe::EnemyKind::Light}};
    knockbackWallLevel.walls.push_back({{{-2.6,-1},{-2.4,1}},1.5});
    cpe::Simulation wallKnockback(knockbackWallLevel);
    wallKnockback.player.facing={1,0};
    wallKnockback.Update({},true,0.05);
    for (int step=0;step<10;++step) wallKnockback.Update({},false,0.05);
    Check(knockbackWallLevel.CanStand(wallKnockback.player.position,
        knockbackWallLevel.agentRadius) && wallKnockback.player.position.x>=-2.15-1e-6,
        "wall collision prevents knockback from pushing the player through geometry");

    cpe::Simulation blockedEnemyAttack(blockedCombatLevel);
    for (int step=0;step<30;++step) blockedEnemyAttack.Update({},false,0.05);
    Check(blockedEnemyAttack.player.hitPoints==blockedEnemyAttack.player.maxHitPoints,
        "wall blocks enemy damage at attack impact");

    cpe::Simulation escapedWindup(combatLevel);
    escapedWindup.Update({},false,0.05);
    Check(escapedWindup.enemies[0].state==cpe::State::Windup,
        "enemy starts a close-range attack windup");
    escapedWindup.player.position={-4,0};
    for (int step=0;step<12;++step) escapedWindup.Update({},false,0.05);
    Check(escapedWindup.player.hitPoints==escapedWindup.player.maxHitPoints,
        "enemy rechecks range at attack impact");

    cpe::Level heavyCombatLevel=combatLevel;
    heavyCombatLevel.spawns={{{0,0},cpe::EnemyKind::Heavy}};
    cpe::Simulation heavyCombat(heavyCombatLevel);
    heavyCombat.Update({},false,0.05);
    Check(heavyCombat.enemies[0].state==cpe::State::Windup,
        "heavy enemy telegraphs its attack");
    const cpe::Vec2 heavyWindupPosition=heavyCombat.enemies[0].position;
    for (int step=0;step<5;++step) heavyCombat.Update({},false,0.05);
    Check(cpe::Near(heavyCombat.enemies[0].position,heavyWindupPosition),
        "heavy enemy stays still during its attack windup");
    for (int step=0;step<30 &&
         heavyCombat.player.hitPoints==heavyCombat.player.maxHitPoints;++step)
        heavyCombat.Update({},false,0.05);
    Check(heavyCombat.player.hitPoints==heavyCombat.player.maxHitPoints-
        cpe::combat::kHeavyDamage,"heavy enemy deals its distinct impact damage");
    const cpe::Vec2 heavyPlayerAfterImpact=heavyCombat.player.position;
    for (int step=0;step<4;++step) heavyCombat.Update({},false,0.05);
    Check(cpe::Distance(heavyCombat.player.position,heavyPlayerAfterImpact)>0.1,
        "heavy attack applies a clear knockback to the player");

    cpe::Simulation playerDeath(heavyCombatLevel);
    playerDeath.player.hitPoints=cpe::combat::kHeavyDamage;
    for (int step=0;step<24;++step) playerDeath.Update({},false,0.05);
    Check(playerDeath.player.hitPoints==0,"player can be defeated by an enemy attack");
    const cpe::Vec2 deathPlayerPosition=playerDeath.player.position;
    const cpe::Vec2 deathEnemyPosition=playerDeath.enemies[0].position;
    playerDeath.Update({1,0},true,0.05);
    Check(cpe::Near(playerDeath.player.position,deathPlayerPosition) &&
        cpe::Near(playerDeath.enemies[0].position,deathEnemyPosition),
        "simulation stops after player death until reset");
    playerDeath.Reset();
    Check(playerDeath.player.hitPoints==playerDeath.player.maxHitPoints,
        "reset recovers the player from defeat");

    cpe::Level arenaLevel;
    arenaLevel.bounds={{-8,-6},{8,6}};
    arenaLevel.agentRadius=0.25;
    arenaLevel.walls.push_back({{{-1.5,-2},{1.5,2}},1.5});
    arenaLevel.navmesh.SetSourcePolygons({
        {{-7.7,-5.7},{-1.8,-5.7},{-1.8,-2.3},{-7.7,-2.3}},
        {{-1.8,-5.7},{1.8,-5.7},{1.8,-2.3},{-1.8,-2.3}},
        {{1.8,-5.7},{7.7,-5.7},{7.7,-2.3},{1.8,-2.3}},
        {{-7.7,-2.3},{-1.8,-2.3},{-1.8,2.3},{-7.7,2.3}},
        {{1.8,-2.3},{7.7,-2.3},{7.7,2.3},{1.8,2.3}},
        {{-7.7,2.3},{-1.8,2.3},{-1.8,5.7},{-7.7,5.7}},
        {{-1.8,2.3},{1.8,2.3},{1.8,5.7},{-1.8,5.7}},
        {{1.8,2.3},{7.7,2.3},{7.7,5.7},{1.8,5.7}}
    });
    Check(arenaLevel.navmesh.Build(),"arena pursuit navmesh");
    arenaLevel.playerSpawn={-5,0};
    arenaLevel.spawns={{{0,-2.5},cpe::EnemyKind::Light}};
    cpe::Simulation arenaSimulation(arenaLevel);
    Check(!arenaLevel.Visible(arenaSimulation.enemies[0].position,
        arenaSimulation.player.position),"arena wall blocks direct sight");
    const cpe::Vec2 arenaEnemyStart=arenaSimulation.enemies[0].position;
    arenaSimulation.Update({},false,0.05);
    Check(arenaSimulation.enemies[0].state==cpe::State::Chase &&
        cpe::Distance(arenaSimulation.enemies[0].position,arenaEnemyStart)>0,
        "enemy starts navigating toward a nearby player behind a wall");
    const double initialArenaDistance=
        cpe::Distance(arenaSimulation.enemies[0].position,arenaSimulation.player.position);
    for (int step=0;step<100;++step)
        arenaSimulation.Update({},false,0.05);
    Check(cpe::Distance(arenaSimulation.enemies[0].position,arenaSimulation.player.position)<
        initialArenaDistance-1.0,"enemy follows a smoothed route around the arena wall");

    std::cout << "PASS - NavMesh result contract\n";
    return 0;
}
int main(int argc,char** argv) {
    try {
        if (argc!=2) throw std::runtime_error("Use dijkstra, astar or navmesh");
        std::string which=argv[1];
        if (which=="dijkstra") return SearchContract(false);
        if (which=="astar") return SearchContract(true);
        if (which=="navmesh") return NavMeshContract();
        throw std::runtime_error("Unknown test");
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
