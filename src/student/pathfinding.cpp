#include "cpe/navigation.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <tuple>

namespace cpe {
namespace {
struct QueueEntry {
    double estimate;
    double cost;
    int node;
};

struct QueueEntryGreater {
    bool operator()(const QueueEntry& lhs, const QueueEntry& rhs) const {
        return std::tie(lhs.estimate, lhs.cost, lhs.node) >
               std::tie(rhs.estimate, rhs.cost, rhs.node);
    }
};

SearchResult Search(const Graph& graph, int start, int goal,
                    const std::vector<double>* heuristic) {
    const auto started = std::chrono::steady_clock::now();
    SearchResult result;
    const auto finish = [&]() {
        result.microseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started).count();
        return result;
    };
    const auto invalid = [&]() {
        result.status = Status::InvalidInput;
        result.nodes.clear();
        result.cost = std::numeric_limits<double>::infinity();
        return finish();
    };

    if (graph.empty() || start < 0 || goal < 0 ||
        static_cast<std::size_t>(start) >= graph.size() ||
        static_cast<std::size_t>(goal) >= graph.size()) {
        return invalid();
    }
    if (heuristic != nullptr) {
        if (heuristic->size() != graph.size() || (*heuristic)[goal] != 0.0)
            return invalid();
        for (const double estimate : *heuristic) {
            if (!std::isfinite(estimate) || estimate < 0.0) return invalid();
        }
    }
    for (const auto& edges : graph) {
        for (const Edge& edge : edges) {
            if (edge.to < 0 || static_cast<std::size_t>(edge.to) >= graph.size() ||
                !std::isfinite(edge.cost) || edge.cost < 0.0) {
                return invalid();
            }
        }
    }

    const auto estimateFor = [&](int node) {
        return heuristic == nullptr ? 0.0 : (*heuristic)[node];
    };
    std::vector<double> costs(graph.size(), std::numeric_limits<double>::infinity());
    std::vector<int> previous(graph.size(), -1);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open;
    costs[start] = 0.0;
    open.push({estimateFor(start), 0.0, start});

    while (!open.empty()) {
        const QueueEntry current = open.top();
        open.pop();
        if (current.cost > costs[current.node]) continue;

        ++result.expanded;
        if (current.node == goal) {
            result.status = Status::Found;
            result.cost = current.cost;
            for (int node = goal; node != -1; node = previous[node])
                result.nodes.push_back(node);
            std::reverse(result.nodes.begin(), result.nodes.end());
            return finish();
        }

        for (const Edge& edge : graph[current.node]) {
            const double candidate = current.cost + edge.cost;
            const double estimate = candidate + estimateFor(edge.to);
            if (!std::isfinite(candidate) || !std::isfinite(estimate))
                return invalid();
            if (candidate >= costs[edge.to]) continue;
            costs[edge.to] = candidate;
            previous[edge.to] = current.node;
            open.push({estimate, candidate, edge.to});
        }
    }

    result.status = Status::NoPath;
    return finish();
}
}

SearchResult Dijkstra(const Graph& graph, int start, int goal) {
    return Search(graph, start, goal, nullptr);
}

SearchResult AStar(const Graph& graph, int start, int goal,
                   const std::vector<double>& heuristic) {
    return Search(graph, start, goal, &heuristic);
}
}
