#include "cpe/navigation.hpp"
#include <queue>
namespace cpe {
SearchResult Dijkstra(const Graph& graph, int start, int goal) {
    // TODO J1 : implémenter et instrumenter Dijkstra.
    SearchResult result;
    int current = start;
    double candidate = 0;
    int next = start;
    if (start == goal) {

    }

    //cost minimal enregistré
    std::vector<double> g(graph.size(), std::numeric_limits<double>::infinity());

    //la position de la precedante position
    std::vector<int> previousPos(graph.size(), -1);

    //cost actuel, cost savegardé, index
    using Entry = std::tuple<double, double, int>;
    //liste trier en fonction du 3eme type
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;

    queue.push({0, 0, start});

    while (!queue.empty()) {
        auto [priority, savedG, u] = queue.top();
        queue.pop();


        if (u == goal) {
            result.status = Status::Found;
            result.cost = g[goal];

            for (int i = goal; i != -1; i = previousPos[i]) {
                result.nodes.push_back(i);
            }

        }
    }
    // start 
    // vector minCost(taille du nombre de vertex) pour chaque pos t'as le cost minimal pour y aler
    // tant que start != goal  
    // ActualCost 
    // for each edge dans la position
    //   if ActualCost + cost edge < minCostPos de son index
    // mincostpos = ActualCost + cost edge


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
