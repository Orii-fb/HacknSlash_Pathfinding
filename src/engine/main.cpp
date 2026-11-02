#include "cpe/simulation.hpp"
#include "raylib.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using cpe::Vec2;

static Vector3 World(Vec2 p, float y = .04f) {
    return {static_cast<float>(p.x), y, static_cast<float>(p.z)};
}
static void DrawPolygon(const std::vector<Vec2>& vertices, Color color) {
    for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
        DrawTriangle3D(World(vertices[0]), World(vertices[i]), World(vertices[i+1]), color);
        DrawTriangle3D(World(vertices[0]), World(vertices[i+1]), World(vertices[i]), color);
    }
    for (std::size_t i = 0; i < vertices.size(); ++i)
        DrawLine3D(World(vertices[i], .06f), World(vertices[(i+1)%vertices.size()], .06f), BLUE);
}
static void DrawNavigation(const cpe::NavMesh& mesh) {
    if (!mesh.IsBuilt()) {
        // Visualisation des surfaces sources, sans connexion ni calcul de navigation.
        for (const auto& source : mesh.SourcePolygons())
            DrawPolygon(source, Color{90,140,180,45});
        return;
    }
    for (const auto& polygon : mesh.Polygons())
        DrawPolygon(polygon.vertices, Color{60,180,170,80});
    for (const auto& portal : mesh.Portals())
        DrawLine3D(World(portal.p,.08f), World(portal.q,.08f), ORANGE);
}
static void DrawPath(const cpe::Path& path, Color color) {
    for (std::size_t i=1; i<path.points.size(); ++i)
        DrawLine3D(World(path.points[i-1],.15f), World(path.points[i],.15f), color);
}
static fs::path FindData(const char* executable, const fs::path& overrideDir) {
    if (!overrideDir.empty()) {
        if (!fs::exists(overrideDir/"levels/arena.level"))
            throw std::runtime_error("--data needs the directory containing levels/");
        return overrideDir;
    }
    fs::path exe = fs::absolute(executable).parent_path();
    for (const auto& path : {exe/"data",fs::current_path()/"data",fs::path(CPE_SOURCE_DATA_DIR)})
        if (fs::exists(path/"levels/arena.level")) return path;
    throw std::runtime_error("Cannot find data/. Use --data PATH.");
}
static bool MouseGround(Camera3D camera, Vec2& result) {
    Ray ray = GetScreenToWorldRay(GetMousePosition(),camera);
    if (std::abs(ray.direction.y)<1e-6f) return false;
    float t = -ray.position.y/ray.direction.y;
    if (t<0) return false;
    result={ray.position.x+ray.direction.x*t,ray.position.z+ray.direction.z*t};
    return true;
}
int main(int argc, char** argv) {
    try {
        fs::path dataOverride;
        bool smoke = false;
        for (int i=1; i<argc; ++i) {
            std::string arg=argv[i];
            if (arg=="--data" && i+1<argc) dataOverride=argv[++i];
            else if (arg=="--smoke-test") smoke=true;
            else if (arg=="--help") {
                std::cout << "Campus Path Engine - Student starter\n--data PATH --smoke-test\n";
                return 0;
            } else throw std::runtime_error("Unknown or incomplete option: "+arg);
        }
        const fs::path data=FindData(argv[0],dataOverride);
        auto sim=std::make_unique<cpe::Simulation>(cpe::Level::Load(data/"levels/arena.level"));
        bool showSurface=true,showPaths=true,showHud=true,paused=false,actionPending=false;
        bool hasProbe=false;
        cpe::Path probe;
        Vec2 probeGoal;
        int scene=0,selected=0,frames=0;
        double accumulator=0;

        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
        InitWindow(1280,800,"Campus Path Engine - Socle etudiant");
        SetWindowMinSize(960,600);
        SetTargetFPS(smoke?0:60);
        Camera3D camera{};
        camera.position={13,19,17}; camera.target={0,0,0}; camera.up={0,1,0};
        camera.fovy=20; camera.projection=CAMERA_ORTHOGRAPHIC;

        while (!WindowShouldClose()) {
            double frame=std::min(static_cast<double>(GetFrameTime()),.1);
            if (smoke) frame=1.0/60;
            if (IsKeyPressed(KEY_F1)) showSurface=!showSurface;
            if (IsKeyPressed(KEY_F2)) showPaths=!showPaths;
            if (IsKeyPressed(KEY_F3)) showHud=!showHud;
            if (IsKeyPressed(KEY_P)) { paused=!paused; accumulator=0; actionPending=false; }
            if (IsKeyPressed(KEY_N) && !sim->enemies.empty())
                selected=(selected+1)%static_cast<int>(sim->enemies.size());
            if (IsKeyPressed(KEY_R)) { sim->Reset(); selected=0; hasProbe=false; }
            if (IsKeyPressed(KEY_F5)) {
                int nextScene=1-scene;
                auto next=std::make_unique<cpe::Simulation>(
                    cpe::Level::Load(data/"levels"/(nextScene?"pillars.level":"arena.level")));
                next->SetAlgorithm(sim->algorithm);
                sim=std::move(next); scene=nextScene; selected=0; hasProbe=false;
            }
            if (IsKeyPressed(KEY_TAB)) {
                sim->SetAlgorithm(sim->algorithm==cpe::Algorithm::AStar
                    ?cpe::Algorithm::Dijkstra:cpe::Algorithm::AStar);
                if (hasProbe) probe=sim->level.navmesh.FindPath(
                    sim->player.position,probeGoal,sim->algorithm);
            }
            Vec2 mouse;
            if (GetMouseY()>155 && MouseGround(camera,mouse)) {
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    hasProbe=true; probeGoal=mouse;
                    probe=sim->level.navmesh.FindPath(sim->player.position,mouse,sim->algorithm);
                }
                if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                    double best=1;
                    for (int i=0; i<static_cast<int>(sim->enemies.size()); ++i) {
                        double d=cpe::Distance(mouse,sim->enemies[i].position);
                        if (d<best) { best=d; selected=i; }
                    }
                }
            }
            Vec2 movement{
                double((IsKeyDown(KEY_D)||IsKeyDown(KEY_RIGHT)) -
                       (IsKeyDown(KEY_A)||IsKeyDown(KEY_Q)||IsKeyDown(KEY_LEFT))),
                double((IsKeyDown(KEY_S)||IsKeyDown(KEY_DOWN)) -
                       (IsKeyDown(KEY_W)||IsKeyDown(KEY_Z)||IsKeyDown(KEY_UP)))
            };
            if (!paused && IsKeyPressed(KEY_SPACE)) actionPending=true;
            if (!paused) {
                accumulator+=frame;
                while (accumulator>=1.0/60) {
                    sim->Update(movement,actionPending,1.0/60);
                    actionPending=false; accumulator-=1.0/60;
                }
            }

            BeginDrawing();
            ClearBackground(Color{21,26,36,255});
            BeginMode3D(camera);
            auto bounds=sim->level.bounds;
            DrawCube(World((bounds.min+bounds.max)*.5,-.13f),
                float(bounds.max.x-bounds.min.x),.2f,
                float(bounds.max.z-bounds.min.z),Color{98,107,115,255});
            if (showSurface) DrawNavigation(sim->level.navmesh);
            for (const auto& wall : sim->level.walls) {
                Vec2 at=(wall.box.min+wall.box.max)*.5;
                float sx=float(wall.box.max.x-wall.box.min.x);
                float sz=float(wall.box.max.z-wall.box.min.z);
                DrawCube(World(at,float(wall.height*.5)),sx,float(wall.height),sz,Color{47,58,76,255});
                DrawCubeWires(World(at,float(wall.height*.5)),sx,float(wall.height),sz,Color{118,140,170,255});
            }
            float radius=float(sim->level.agentRadius);
            DrawCylinder(World(sim->player.position,0),radius,radius,.7f,12,GREEN);
            DrawLine3D(World(sim->player.position,.8f),
                World(sim->player.position+sim->player.facing*.7,.8f),WHITE);
            for (int i=0; i<static_cast<int>(sim->enemies.size()); ++i) {
                const auto& e=sim->enemies[i];
                bool heavy=e.kind==cpe::EnemyKind::Heavy;
                DrawCylinder(World(e.position,0),radius,radius,heavy?.85f:.58f,heavy?6:10,heavy?MAROON:RED);
                if (i==selected) DrawCircle3D(World(e.position,.08f),radius+.15f,{1,0,0},90,YELLOW);
                if (showPaths && i==selected) DrawPath(e.navigator.path,MAGENTA);
            }
            if (hasProbe) {
                DrawSphere(World(probeGoal,.2f),.12f,probe.search.status==cpe::Status::Found?SKYBLUE:ORANGE);
                if (showPaths) DrawPath(probe,SKYBLUE);
            }
            EndMode3D();
            if (showHud) {
                DrawRectangle(12,12,GetScreenWidth()-24,140,Fade(BLACK,.84f));
                DrawText("CAMPUS PATH ENGINE - SOCLE ETUDIANT",25,24,22,WHITE);
                DrawText(TextFormat("%s | Selection: %s | NavMesh: %s",
                    sim->level.title.c_str(),cpe::Name(sim->algorithm),
                    sim->level.navmesh.IsBuilt()?"Construit":"A IMPLEMENTER"),25,55,18,LIGHTGRAY);
                DrawText("Navigation, IA et gameplay : fonctions a completer dans src/student/",25,80,18,YELLOW);
                if (hasProbe) {
                    DrawText(TextFormat("Sonde: %s | %d developpes | %.2f us",
                        cpe::Name(probe.search.status),probe.search.expanded,probe.queryMicroseconds),
                        25,105,18,SKYBLUE);
                }
                DrawText("F1 surfaces | F2 chemins | F3 HUD | Tab algo | N selection | F5 scene | P pause | R reset",
                    25,130,16,GRAY);
                DrawRectangle(12,GetScreenHeight()-48,GetScreenWidth()-24,36,Fade(BLACK,.8f));
                DrawText("ZQSD / WASD / fleches : bouger | Clic gauche : requete | Espace : point d'extension gameplay",
                    25,GetScreenHeight()-39,16,WHITE);
            }
            if (paused) DrawText("PAUSE",GetScreenWidth()/2-55,170,32,YELLOW);
            EndDrawing();
            if (smoke && ++frames>=90) break;
        }
        CloseWindow();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Campus Path Engine: " << error.what() << '\n';
        if (IsWindowReady()) CloseWindow();
        return 1;
    }
}
