#include "cpe/navigation.hpp"
#include <queue>
namespace cpe {
SearchResult Dijkstra(const Graph& graph, int start, int goal) {
    // TODO J1 : implémenter et instrumenter Dijkstra.
    const std::vector<double> heuristic(graph.size(), 0);
    return AStar(graph, start, goal, heuristic);
}
SearchResult AStar(const Graph& graph, int start, int goal,
                   const std::vector<double>& heuristic) {
    // TODO J2 : implémenter A* et justifier le contrat de l'heuristique.
    SearchResult result;

    //verification des entrées
    if (start < 0 && start >= graph.size()) {
        result.status = Status::InvalidInput;
        return result;
    }

    if (goal < 0 && goal >= graph.size()) {
        result.status = Status::InvalidInput;
        return result;
    }

    //cost minimal enregistré (l'index etant la position de la valeur)
    std::vector<double> g(graph.size(), std::numeric_limits<double>::infinity());
    g[start] = 0;

    //la position de la précédente position
    std::vector<int> previousPos(graph.size(), -1);

    //cost avec heuristique, cost savegardé, index
    using Entry = std::tuple<double, double, int>;

    //liste trier en fonction du 1er type de Entry
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;

    queue.push({ heuristic[start], 0, start});

    while (!queue.empty()) {
        auto [priority, savedG, u] = queue.top();
        queue.pop();

        if (savedG > g[u]) {
            continue;
        }

        if (u == goal) {
            result.status = Status::Found;
            result.cost = g[goal];

            for (int i = goal; i != -1; i = previousPos[i]) {
                result.nodes.push_back(i);
            }
            //reverse car est inversé avec la boucle for vu que tu commences par goal (et ne pas inclure trop de value inutile à bouclé)
            std::reverse(result.nodes.begin(), result.nodes.end());
            return result;
        }
        result.expanded++;

        for (const Edge& edge : graph[u]) {
            double candidate = g[u] + edge.cost;

            if (candidate < g[edge.to]) {
                g[edge.to] = candidate;
                previousPos[edge.to] = u;
                queue.push({candidate + heuristic[edge.to], candidate, edge.to});
            }
        }

    }

    result.status = Status::NoPath;

    return result;
    // start 
    // vector minCost(taille du nombre de vertex) pour chaque pos t'as le cost minimal pour y aler
    // tant que start != goal  
    // ActualCost 
    // for each edge dans la position
    //   if ActualCost + cost edge < minCostPos de son index
    // mincostpos = ActualCost + cost edge
}
}
