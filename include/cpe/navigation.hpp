#pragma once
#include "math.hpp"
#include <limits>
#include <vector>

namespace cpe {
// Contrats de données fournis ; aucun algorithme n'est implémenté.
enum class Algorithm { Dijkstra, AStar };
enum class Status { NotImplemented, Found, NoPath, OutsideNavMesh, InvalidInput };
const char* Name(Algorithm value);
const char* Name(Status value);

struct Edge { int to; double cost; };
using Graph = std::vector<std::vector<Edge>>;

struct SearchResult {
    Status status = Status::NotImplemented;
    std::vector<int> nodes;
    double cost = std::numeric_limits<double>::infinity();
    int expanded = 0;
    double microseconds = 0;
};

// À réaliser dans src/student/pathfinding.cpp.
// Found : chemin départ -> arrivée et coût.
// NoPath : graphe valide mais destination inaccessible.
// InvalidInput : indices, poids ou heuristique invalides.
SearchResult Dijkstra(const Graph& graph, int start, int goal);
SearchResult AStar(const Graph& graph, int start, int goal,
                   const std::vector<double>& heuristic);

struct Polygon { std::vector<Vec2> vertices; Vec2 center; };
struct Portal { int a, b; Vec2 p, q; };
struct Path {
    SearchResult search;
    std::vector<Vec2> points;
    double length = std::numeric_limits<double>::infinity();
    double queryMicroseconds = 0;
};

class NavMesh {
public:
    // Fourni : stockage brut pour afficher les données du niveau.
    // Ne construit ni polygones de navigation, ni graphe, ni portails.
    void SetSourcePolygons(const std::vector<std::vector<Vec2>>& source);
    const std::vector<std::vector<Vec2>>& SourcePolygons() const { return source_; }

    // À réaliser dans src/student/navmesh.cpp.
    // Build doit remplir polygons_, portals_, graph_ et définir built_=true.
    bool Build();
    bool IsBuilt() const { return built_; }
    int Locate(Vec2 point) const;
    bool Contains(int polygon, Vec2 point) const;
    bool SegmentInside(Vec2 from, Vec2 to) const;
    Path FindPath(Vec2 from, Vec2 to, Algorithm algorithm) const;

    // Le debug peut lire le résultat une fois construit par l'étudiant.
    const std::vector<Polygon>& Polygons() const { return polygons_; }
    const std::vector<Portal>& Portals() const { return portals_; }
    const Graph& GetGraph() const { return graph_; }
private:
    std::vector<std::vector<Vec2>> source_;
    std::vector<Polygon> polygons_;
    std::vector<Portal> portals_;
    Graph graph_;
    bool built_ = false;
};
}
