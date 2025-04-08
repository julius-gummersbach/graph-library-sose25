#include "AdjacentListGraph.h"
#include <iostream>
namespace graph {

  using namespace std;

  AdjacentListGraph::AdjacentListGraph(istream& input) : SuperGraph(input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    input >> numNodes;
    adjacencyList.resize(numNodes);

    std::vector<std::pair<int, int>> edges;
    std::vector<int> outDegree(numNodes, 0);
    int a, b;
    while (input >> a >> b) {
      edges.emplace_back(a, b);
      outDegree[a]++;
      if (!directed) outDegree[b]++;
    }

    for (int i = 0; i < numNodes; ++i) {
      adjacencyList[i].reserve(outDegree[i]);
    }

    for (const auto& [from, to] : edges) {
      adjacencyList[from].emplace_back(to);
      if (!directed) {
        adjacencyList[to].emplace_back(from);
      }
    }
  }

  AdjacentListGraph::~AdjacentListGraph() = default;

  const vector<int>& AdjacentListGraph::getAdjacentNodes(int node) {
    return adjacencyList[node];
  }
}