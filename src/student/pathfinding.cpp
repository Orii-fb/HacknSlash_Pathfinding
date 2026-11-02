#include "cpe/navigation.hpp"
namespace cpe {
SearchResult Dijkstra(const Graph& graph, int start, int goal) {
    // TODO J1 : implémenter et instrumenter Dijkstra.
    (void)graph;
    (void)start;
    (void)goal;
    return {};
}
SearchResult AStar(const Graph& graph, int start, int goal,
                   const std::vector<double>& heuristic) {
    // TODO J2 : implémenter A* et justifier le contrat de l'heuristique.
    (void)graph;
    (void)start;
    (void)goal;
    (void)heuristic;
    return {};
}
}
