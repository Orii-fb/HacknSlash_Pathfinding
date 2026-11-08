#include "cpe/navigation.hpp"
#include <stdexcept>
namespace cpe {
void NavMesh::AddUndirected(Graph& g, int indexA, int indexB, double cost) {
    g.at(indexA).push_back({ indexB, cost });
    g.at(indexB).push_back({ indexA, cost });
}
bool NavMesh::Build() {
    // TODO J3 : valider et structurer source_, calculer les adjacences/portails.
    // Remplir polygons_, portals_, graph_, puis définir built_=true.
    polygons_.clear();
    portals_.clear();
    graph_.clear();

    if (source_.empty()) {
        return false;
    }

    //liste des potentiels portal
    std::vector<int> indexPolyAretes;
    std::vector<std::pair<Vec2, Vec2>> aretes;
    std::vector<Vec2> polySource;
    for (int i = 0; i < source_.size(); i++) {
        polySource = source_[i];
        if (polySource.size() < 2) {
            return false;
        }
        polygons_.push_back({ polySource });

        double xMiddle = 0;
        double zMiddle = 0;

        for (int j = 0; j < polySource.size(); j++) {
            // calcul du centre
            xMiddle += polySource[j].x;
            zMiddle += polySource[j].z;

            //recherche des portals
            Vec2 pointA = polySource[j];
            Vec2 pointB = polySource[(j + 1) % polySource.size()];
            std::pair<Vec2, Vec2> arete = { pointA, pointB };
            //si l'arete est dans la liste

            bool found = false;
            for (int k = 0; k < aretes.size(); k++) {
                if (Near(aretes[k].first, arete.first) && Near(aretes[k].second, arete.second) ||
                    Near(aretes[k].first, arete.second) && Near(aretes[k].second, arete.first)) {
                    //recuperer le portail avec points d'extremité et l'index des polygons
                    portals_.push_back({ indexPolyAretes[k], i, aretes[k].first, aretes[k].second });
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                aretes.push_back({ pointA, pointB });
                indexPolyAretes.push_back(i);

            }

        }

        polygons_[i].center = { xMiddle / static_cast<double>(polySource.size()) , zMiddle / static_cast<double>(polySource.size()) };
        
    }
    
    graph_.assign(polygons_.size(), {});

    for (const Portal& portal : portals_) {
        if (!Finite(portal.p) ||
            !Finite(portal.q) ||
            Distance(portal.p, portal.q) < 1e-8) {
            throw std::invalid_argument(
                "Portail invalide ou de longueur nulle."
            );
        }

        const int regionA = portal.a;
        const int regionB = portal.b;

        // Les deux extrémités doivent appartenir aux deux régions.
        if (!Contains(regionA, portal.p) ||
            !Contains(regionA, portal.q) ||
            !Contains(regionB, portal.p) ||
            !Contains(regionB, portal.q)) {
            throw std::invalid_argument(
                "Le portail n'appartient pas aux deux regions."
            );
        }

        const Vec2 middle = portal.Middle();

        const double cost =
            Distance(polygons_[regionA].center, middle)
            + Distance(middle, polygons_[regionB].center);

        AddUndirected(graph_, portal.a, portal.b, cost);
    }

    built_ = true;

    return true;
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
