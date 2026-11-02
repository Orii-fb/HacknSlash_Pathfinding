#include "cpe/navigation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// Contrats de résultat uniquement : aucune implémentation d'algorithme.
static void Check(bool condition,const char* message) {
    if (!condition) throw std::runtime_error(message);
}
static int SearchContract(bool astar) {
    cpe::Graph graph(6);
    auto add=[&](int a,int b,double cost) {
        graph[a].push_back({b,cost});
        graph[b].push_back({a,cost});
    };
    add(0,1,2); add(0,2,5); add(1,2,1); add(1,3,4);
    add(2,3,1); add(2,4,7); add(3,4,3);
    auto query=[&](const cpe::Graph& g,int from,int to) {
        if (astar) {
            std::vector<double> h(g.size(),0.0);
            if (to==4 && g.size()==6) h={7,5,4,3,0,0};
            return cpe::AStar(g,from,to,h);
        }
        return cpe::Dijkstra(g,from,to);
    };
    auto result=query(graph,0,4);
    if (result.status==cpe::Status::NotImplemented) {
        std::cout << "SKIP - " << (astar?"A*":"Dijkstra") << " a implementer\n";
        return 77;
    }
    Check(result.status==cpe::Status::Found,"weighted graph should be reachable");
    Check(std::isfinite(result.cost)&&std::abs(result.cost-7)<1e-4,"expected cost is 7");
    Check(result.nodes.size()>=2 && result.nodes.front()==0 && result.nodes.back()==4,
        "path endpoints");
    double sum=0;
    for (std::size_t i=1;i<result.nodes.size();++i) {
        int a=result.nodes[i-1],b=result.nodes[i];
        Check(a>=0 && a<static_cast<int>(graph.size()),"valid node in path");
        bool exists=false;
        for (const auto& edge:graph[a]) if (edge.to==b) {sum+=edge.cost; exists=true; break;}
        Check(exists,"path must use actual graph edges");
    }
    Check(std::abs(sum-result.cost)<1e-4,"path cost matches returned cost");
    Check(query(graph,0,5).status==cpe::Status::NoPath,"isolated node");
    Check(query(graph,-1,4).status==cpe::Status::InvalidInput,"invalid index");
    auto same=query(graph,0,0);
    Check(same.status==cpe::Status::Found && same.cost==0,"same start goal");
    graph[0][0].cost=-1;
    Check(query(graph,0,4).status==cpe::Status::InvalidInput,"negative edge");
    std::cout << "PASS - search result contract\n";
    return 0;
}
static int NavMeshContract() {
    cpe::NavMesh mesh;
    mesh.SetSourcePolygons({
        {{0,0},{4,0},{4,2},{0,2}},
        {{4,0},{6,0},{6,2},{4,2}},
        {{4,2},{6,2},{6,6},{4,6}},
        {{8,0},{10,0},{10,2},{8,2}}
    });
    if (!mesh.Build()) {
        std::cout << "SKIP - NavMesh::Build a implementer\n";
        return 77;
    }
    Check(mesh.IsBuilt(),"Build must set built_ on success");
    Check(mesh.Portals().size()==2,"two portals in the conforming L fixture");
    Check(mesh.Locate({1,1})>=0,"inside point");
    Check(mesh.Locate({3,3})==-1,"outside point");
    Check(mesh.SegmentInside({1,1},{5,1}),"horizontal segment inside the surface");
    Check(!mesh.SegmentInside({1,1},{5,5}),"do not cut the L corner");
    for (auto algorithm : {cpe::Algorithm::Dijkstra,cpe::Algorithm::AStar}) {
        auto path=mesh.FindPath({1,1},{5,5},algorithm);
        Check(path.search.status==cpe::Status::Found,"route through L");
        Check(!path.points.empty(),"non-empty geometric route");
        Check(cpe::Near(path.points.front(),{1,1}) && cpe::Near(path.points.back(),{5,5}),
            "geometric endpoints");
        for (std::size_t i=1;i<path.points.size();++i)
            Check(mesh.SegmentInside(path.points[i-1],path.points[i]),"valid route segments");
        Check(mesh.FindPath({1,1},{9,1},algorithm).search.status==cpe::Status::NoPath,
            "disconnected island");
        Check(mesh.FindPath({1,1},{3,3},algorithm).search.status==cpe::Status::OutsideNavMesh,
            "outside destination");
    }
    std::cout << "PASS - NavMesh result contract\n";
    return 0;
}
int main(int argc,char** argv) {
    try {
        if (argc!=2) throw std::runtime_error("Use dijkstra, astar or navmesh");
        std::string which=argv[1];
        if (which=="dijkstra") return SearchContract(false);
        if (which=="astar") return SearchContract(true);
        if (which=="navmesh") return NavMeshContract();
        throw std::runtime_error("Unknown test");
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
