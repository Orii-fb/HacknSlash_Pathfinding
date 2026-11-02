#include "cpe/level.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cpe {
Level Level::Load(const std::filesystem::path& file){
    std::ifstream input(file);
    if(!input)throw std::runtime_error("Cannot open level: "+file.string());
    Level result;std::vector<std::vector<Vec2>> polygons;
    std::string line;int number=0;bool gotBounds=false,gotPlayer=false,gotRadius=false;
    while(std::getline(input,line)){
        ++number;line=line.substr(0,line.find('#'));std::istringstream in(line);std::string key;
        if(!(in>>key))continue;
        auto error=[&](const std::string& message){throw std::runtime_error(file.filename().string()+":"+std::to_string(number)+": "+message);};
        if(key=="title"){std::getline(in>>std::ws,result.title);if(result.title.empty())error("empty title");continue;}
        if(key=="bounds"){
            if(gotBounds)error("duplicate bounds");
            if(!(in>>result.bounds.min.x>>result.bounds.min.z>>result.bounds.max.x>>result.bounds.max.z))error("bounds xmin zmin xmax zmax");
            gotBounds=true;
        }else if(key=="radius"){
            if(gotRadius)error("duplicate radius");
            if(!(in>>result.agentRadius))error("invalid radius");
            gotRadius=true;
        }else if(key=="player"){
            if(gotPlayer)error("duplicate player");
            if(!(in>>result.playerSpawn.x>>result.playerSpawn.z))error("player x z");
            gotPlayer=true;
        }else if(key=="enemy"){
            std::string kind;Spawn spawn;
            if(!(in>>kind>>spawn.position.x>>spawn.position.z))error("enemy light|heavy x z");
            if(kind!="light"&&kind!="heavy")error("unknown enemy kind");
            spawn.kind=kind=="light"?EnemyKind::Light:EnemyKind::Heavy;
            result.spawns.push_back(spawn);
        }else if(key=="wall"){
            Wall wall;
            if(!(in>>wall.box.min.x>>wall.box.min.z>>wall.box.max.x>>wall.box.max.z>>wall.height))error("wall xmin zmin xmax zmax height");
            if(!Finite(wall.box.min)||!Finite(wall.box.max)||wall.box.min.x>=wall.box.max.x||wall.box.min.z>=wall.box.max.z||!std::isfinite(wall.height)||wall.height<=0)error("invalid wall dimensions");
            result.walls.push_back(wall);
        }else if(key=="surface"){
            int count=0;if(!(in>>count)||count<3||count>64)error("surface count x1 z1 ... (3 to 64 vertices)");
            std::vector<Vec2> v(static_cast<std::size_t>(count));
            for(auto& p:v)if(!(in>>p.x>>p.z))error("missing polygon coordinate");
            polygons.push_back(v);
        }else error("unknown key "+key);
        std::string extra;if(in>>extra)error("unexpected trailing token "+extra);
    }
    if(!gotBounds||!gotPlayer||!gotRadius)throw std::runtime_error("Level needs bounds, player and radius");
    if(result.title.empty())result.title=file.stem().string();
    if(!Finite(result.bounds.min)||!Finite(result.bounds.max)||result.bounds.min.x>=result.bounds.max.x||result.bounds.min.z>=result.bounds.max.z)
        throw std::runtime_error("Invalid level bounds");
    if(!std::isfinite(result.agentRadius)||result.agentRadius<=0||result.agentRadius>2)throw std::runtime_error("Radius must be in (0,2]");
    // Données de surface chargées uniquement pour leur affichage.
    // La construction du NavMesh appartient au code étudiant.
    for(const auto& vertices:polygons)
        for(Vec2 point:vertices)
            if(!Finite(point))throw std::runtime_error("Non-finite source surface");
    result.navmesh.SetSourcePolygons(polygons);
    result.navmesh.Build(); // Retourne false tant que le module n'est pas implémenté.
    if(!result.CanStand(result.playerSpawn,result.agentRadius))
        throw std::runtime_error("Invalid player spawn (physical collision)");
    for(const auto& spawn:result.spawns)
        if(!result.CanStand(spawn.position,result.agentRadius))
            throw std::runtime_error("Invalid enemy spawn (physical collision)");
    return result;
}
bool Level::CanStand(Vec2 p,double radius)const{
    if(!Finite(p)||!std::isfinite(radius)||radius<0)return false;
    if(p.x<bounds.min.x+radius||p.x>bounds.max.x-radius||p.z<bounds.min.z+radius||p.z>bounds.max.z-radius)return false;
    for(const auto& wall:walls)if(DiskOverlaps(wall.box,p,radius))return false;
    return true;
}
bool Level::CanMove(Vec2 from,Vec2 to,double radius)const{
    if(!CanStand(from,radius)||!CanStand(to,radius))return false;
    // Expanded AABB is conservative around corners and prevents tunnelling.
    for(const auto& wall:walls)if(SegmentHits(Expanded(wall.box,std::max(0.0,radius-1e-7)),from,to))return false;
    return true;
}
Vec2 Level::MoveSliding(Vec2 from,Vec2 delta,double radius)const{
    if(!Finite(delta))return from;
    Vec2 direct=from+delta;if(CanMove(from,direct,radius))return direct;
    Vec2 x=from+Vec2{delta.x,0};if(CanMove(from,x,radius))from=x;
    Vec2 z=from+Vec2{0,delta.z};if(CanMove(from,z,radius))from=z;
    return from;
}
bool Level::Visible(Vec2 from,Vec2 to)const{
    if(!Finite(from)||!Finite(to))return false;
    for(const auto& wall:walls)if(SegmentHits(wall.box,from,to))return false;
    return true;
}
}
