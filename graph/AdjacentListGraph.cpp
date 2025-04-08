#include "AdjacentListGraph.h"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <execution>

namespace graph {

  using namespace std;

  AdjacentListGraph::AdjacentListGraph(istream& input) : SuperGraph(input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    input >> numNodes;
    adjacencyList.resize(numNodes);

    std::vector<std::pair<int, int>> edges;
    std::vector outDegree(numNodes, 0);
    {
      int a, b;
      while (input >> a >> b) {
        edges.emplace_back(a, b);
        outDegree[a]++;
        if (!directed) outDegree[b]++;
      }
    }

    for (int i = 0; i < numNodes; ++i) {
      adjacencyList[i].reserve(outDegree[i]);
    }

    if (numNodes > 1000000) {
      std::vector<std::mutex> nodeLocks(numNodes);
      std::for_each(std::execution::par, edges.begin(), edges.end(), [&](const std::pair<int, int>& edge) {
          const int a = edge.first;
          const int b = edge.second;
          {
              std::lock_guard lockA(nodeLocks[a]);
              adjacencyList[a].push_back(b);
          }
          if (!directed) {
              std::lock_guard lockB(nodeLocks[b]);
              adjacencyList[b].push_back(a);
          }
      });
    } else {
      for (const auto&[first, second] : edges) {
        const int a = first;
        const int b = second;
        adjacencyList[a].push_back(b);
        if (!directed) adjacencyList[b].push_back(a);
      }
    }
  }

  AdjacentListGraph::~AdjacentListGraph() = default;

  const vector<int>& AdjacentListGraph::getAdjacentNodes(int node) {
    return adjacencyList[node];
  }
}
