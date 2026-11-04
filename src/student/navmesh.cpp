#include "cpe/navigation.hpp"
namespace cpe {
bool NavMesh::Build() {
    // TODO J3 : valider et structurer source_, calculer les adjacences/portails.
    // Remplir polygons_, portals_, graph_, puis définir built_=true.
    return false;
}
int NavMesh::Locate(Vec2 point) const {
    // TODO J3 : localiser le point dans un polygone navigable.
    if (!Finite(point)) {
        return -1;
    }

    for (int i = 0; i < polygons_.size(); i++) {
        if (Contains(i, point)) {
            return i;
        }
    }

    return -1;
}
bool NavMesh::Contains(int polygon, Vec2 point) const {
    // TODO J3 : test point dans polygone.
    const std::vector<Vec2> verticesPoly = polygons_[polygon].vertices;

    bool positiveSide = false;
    bool negativeSide = false;

    double epsilon = 1e-8;

    for (int i = 0; i < verticesPoly.size(); i++) {
        Vec2 a = verticesPoly[i];
        Vec2 b = verticesPoly[(i+1) % verticesPoly.size()];

        double side = Cross(b - a, point - a);

        if (side > epsilon) positiveSide = true;
        if (side <= -epsilon) negativeSide = true;
    }

    if (positiveSide && negativeSide) {
        return false;
    }
    return true;
}
bool NavMesh::SegmentInside(Vec2 from, Vec2 to) const {
    // TODO J3 : vérifier qu'un segment reste dans la surface navigable.
    (void)from;
    (void)to;
    return false;
}

const Portal* NavMesh::Between(int a, int b) const{
    for (Portal p : portals_) {
        bool sensA = p.a == a && p.b == b;
        bool sensB = p.b == a && p.a == b;

        if (sensA || sensB) {
            return &p;
        }
    }

    return nullptr;
}

const Vec2 Middle(Vec2 a, Vec2 b) {
    return { (a.x + b.x) / 2, (a.z + b.z) / 2 };
}

Path NavMesh::FindPath(Vec2 from, Vec2 to, Algorithm algorithm) const {
    // TODO J3 : relier localisation, recherche sur graphe et trajet géométrique.
    int locateFrom = Locate(from);
    int locateTarget = Locate(to);
    Path path;

    std::vector<double> heuristicList(polygons_.size());
    for (int i = 0; i < polygons_.size(); i++) {
        Vec2 a = polygons_[i].center;
        heuristicList[i] = Distance(a, to);
    }
    
    switch (algorithm) {
    case Algorithm::Dijkstra:
        path = { Dijkstra(graph_, locateFrom, locateTarget) };
        break;
    case Algorithm::AStar:
        path = { AStar(graph_, locateFrom, locateTarget, heuristicList) };
        break;
    }

    if (path.search.status != Status::Found) {
        path.search.status = Status::NoPath;
        return path;
    }
    
    path.points.push_back(from);

    if (locateFrom == locateTarget) {
        path.points.push_back(to);
    }

    for (int i = 1; i < path.search.nodes.size(); i++) {
        int a = path.search.nodes[i - 1];
        int b = path.search.nodes[i];

        const Portal* p = Between(a, b);
        if (p != nullptr) {
            path.points.push_back(Middle(p->p, p->q));
        }
    }

    return path;
}
}
