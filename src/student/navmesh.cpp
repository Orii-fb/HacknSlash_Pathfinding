#include "cpe/navigation.hpp"
#include <utility>
namespace cpe {
bool NavMesh::Build() {
    polygons_.clear();
    portals_.clear();
    graph_.clear();
    built_ = false;

    if (source_.empty()) return false;

    std::vector<Polygon> polygons;
    polygons.reserve(source_.size());

    for (const auto& vertices : source_) {
        if (vertices.size() < 3) return false;

        for (std::size_t i = 0; i < vertices.size(); ++i) {
            if (!Finite(vertices[i])) return false;
            for (std::size_t j = i + 1; j < vertices.size(); ++j) {
                if (Near(vertices[i], vertices[j])) return false;
            }
        }

        double twiceArea = 0.0;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            twiceArea += Cross(vertices[i], vertices[(i + 1) % vertices.size()]);
        }
        if (!std::isfinite(twiceArea) || std::abs(twiceArea) <= 1e-10) return false;

        const int winding = twiceArea > 0.0 ? 1 : -1;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            const Vec2 edge = vertices[(i + 1) % vertices.size()] - vertices[i];
            for (const Vec2 vertex : vertices) {
                const double side = Cross(edge, vertex - vertices[i]);
                if (!std::isfinite(side)) return false;
                if ((winding > 0 && side < -1e-10) ||
                    (winding < 0 && side > 1e-10)) return false;
            }
        }

        Vec2 center{};
        for (const Vec2 vertex : vertices) center = center + vertex;
        center = center * (1.0 / static_cast<double>(vertices.size()));
        if (!Finite(center)) return false;
        polygons.push_back({vertices, center});
    }

    std::vector<Portal> portals;
    for (std::size_t a = 0; a < polygons.size(); ++a) {
        for (std::size_t b = a + 1; b < polygons.size(); ++b) {
            for (std::size_t edgeA = 0; edgeA < polygons[a].vertices.size(); ++edgeA) {
                const Vec2 a0 = polygons[a].vertices[edgeA];
                const Vec2 a1 = polygons[a].vertices[
                    (edgeA + 1) % polygons[a].vertices.size()];
                for (std::size_t edgeB = 0; edgeB < polygons[b].vertices.size(); ++edgeB) {
                    const Vec2 b0 = polygons[b].vertices[edgeB];
                    const Vec2 b1 = polygons[b].vertices[
                        (edgeB + 1) % polygons[b].vertices.size()];
                    if ((Near(a0, b0) && Near(a1, b1)) ||
                        (Near(a0, b1) && Near(a1, b0))) {
                        portals.push_back({static_cast<int>(a), static_cast<int>(b), a0, a1});
                    }
                }
            }
        }
    }

    Graph graph(polygons.size());
    for (const Portal& portal : portals) {
        const Vec2 midpoint = (portal.p + portal.q) * 0.5;
        const double cost = Distance(polygons[portal.a].center, midpoint) +
                            Distance(midpoint, polygons[portal.b].center);
        if (!std::isfinite(cost)) return false;
        graph[portal.a].push_back({portal.b, cost});
        graph[portal.b].push_back({portal.a, cost});
    }

    polygons_ = std::move(polygons);
    portals_ = std::move(portals);
    graph_ = std::move(graph);
    built_ = true;
    return true;
}
int NavMesh::Locate(Vec2 point) const {
    if (!Finite(point)) return -1;

    for (std::size_t polygonIndex = 0; polygonIndex < polygons_.size(); ++polygonIndex) {
        if (Contains(static_cast<int>(polygonIndex), point))
            return static_cast<int>(polygonIndex);
    }

    return -1;
}
bool NavMesh::Contains(int polygon, Vec2 point) const {
    if (polygon < 0 || polygon >= static_cast<int>(polygons_.size()) || !Finite(point))
        return false;

    const auto& vertices = polygons_[polygon].vertices;
    if (vertices.size() < 3) return false;

    constexpr double epsilon = 1e-10;
    bool hasPositiveSide = false;
    bool hasNegativeSide = false;
    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const Vec2 edge = vertices[(i + 1) % vertices.size()] - vertices[i];
        const double side = Cross(edge, point - vertices[i]);
        if (!std::isfinite(side)) return false;
        if (side > epsilon) hasPositiveSide = true;
        if (side < -epsilon) hasNegativeSide = true;
        if (hasPositiveSide && hasNegativeSide) return false;
    }

    return true;
}
bool NavMesh::SegmentInside(Vec2 from, Vec2 to) const {
    if (!Finite(from) || !Finite(to) || polygons_.empty()) return false;

    constexpr double epsilon = 1e-10;
    constexpr double parameterEpsilon = 1e-10;
    const Vec2 direction = to - from;
    if (!Finite(direction)) return false;

    std::vector<std::pair<double, double>> intervals;
    intervals.reserve(polygons_.size());

    for (const Polygon& polygon : polygons_) {
        const auto& vertices = polygon.vertices;
        if (vertices.size() < 3) continue;

        double twiceArea = 0.0;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            twiceArea += Cross(vertices[i], vertices[(i + 1) % vertices.size()]);
        }
        if (!std::isfinite(twiceArea) || std::abs(twiceArea) <= epsilon) continue;
        const double winding = twiceArea > 0.0 ? 1.0 : -1.0;

        double enter = 0.0;
        double leave = 1.0;
        bool intersects = true;
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            const Vec2 edgeStart = vertices[i];
            const Vec2 edge = vertices[(i + 1) % vertices.size()] - edgeStart;
            const double startSide = winding * Cross(edge, from - edgeStart);
            const double sideChange = winding * Cross(edge, direction);
            if (!std::isfinite(startSide) || !std::isfinite(sideChange)) {
                intersects = false;
                break;
            }

            if (std::abs(sideChange) <= epsilon) {
                if (startSide < -epsilon) {
                    intersects = false;
                    break;
                }
                continue;
            }

            const double boundary = (-epsilon - startSide) / sideChange;
            if (sideChange > 0.0)
                enter = std::max(enter, boundary);
            else
                leave = std::min(leave, boundary);

            if (enter > leave + parameterEpsilon) {
                intersects = false;
                break;
            }
        }

        if (intersects && leave >= -parameterEpsilon &&
            enter <= 1.0 + parameterEpsilon) {
            intervals.push_back({
                std::max(0.0, enter),
                std::min(1.0, leave)
            });
        }
    }

    double coveredUntil = 0.0;
    while (coveredUntil < 1.0 - parameterEpsilon) {
        double furthest = coveredUntil;
        for (const auto& interval : intervals) {
            if (interval.first <= coveredUntil + parameterEpsilon)
                furthest = std::max(furthest, interval.second);
        }
        if (furthest <= coveredUntil + parameterEpsilon) return false;
        coveredUntil = furthest;
    }

    return true;
}
Path NavMesh::FindPath(Vec2 from, Vec2 to, Algorithm algorithm) const {
    // TODO J3 : relier localisation, recherche sur graphe et trajet géométrique.
    (void)from;
    (void)to;
    (void)algorithm;
    return {};
}
}
