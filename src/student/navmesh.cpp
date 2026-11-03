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
Path NavMesh::FindPath(Vec2 from, Vec2 to, Algorithm algorithm) const {
    // TODO J3 : relier localisation, recherche sur graphe et trajet géométrique.
    (void)from;
    (void)to;
    (void)algorithm;
    return {};
}
}
