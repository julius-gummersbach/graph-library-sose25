#include "AdjacentListGraph.h"

#include <iostream>
#include <sstream>

#include "../edge/SingleWeightedEdge.h"

namespace graph {
  using namespace std;

  AdjacentListGraph::AdjacentListGraph() = default;

  AdjacentListGraph::AdjacentListGraph(const vector<std::shared_ptr<edge::SuperEdge>>& mstEdgeList) {
    numNodes = mstEdgeList.size() + 1;
    adjacencyList.resize(numNodes);
    for (const auto& edge : mstEdgeList) {
      adjacencyList[edge->getFrom()][edge->getTo()] = edge;
      if (!directed) {
        adjacencyList[edge->getTo()][edge->getFrom()] = edge;
      }
    }
    initialized = true;
  }

  void AdjacentListGraph::initializeFromInput(istream &input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string line;
    std::getline(input, line);
    std::istringstream ls(line);
    ls >> numNodes;

    adjacencyList.resize(numNodes);

    std::getline(input, line);
    ls = std::istringstream(line);
    int u, v;
    double w;
    ls >> u >> v;
    if (ls >> w) {
      weighted = true;
    }

    do {
      if (line.empty()) continue;
      ls = std::istringstream(line);
      ls >> u >> v;
      if (weighted) {
        ls >> w;
        adjacencyList[u][v] = std::make_shared<const edge::SingleWeightedEdge>(u,v, w);
        if (!directed) {
          adjacencyList[v][u] = std::make_shared<const edge::SingleWeightedEdge>(v,u, w);
        }
      } else {
        adjacencyList[u][v] = std::make_shared<const edge::SuperEdge>(u,v);
        if (!directed) {
          adjacencyList[v][u] = std::make_shared<const edge::SuperEdge>(v,u);
        }
      }
    } while (std::getline(input, line));
    initialized = true;
  }


  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentListGraph::getAdjacent(const int node) {
    vector<std::shared_ptr<const edge::SuperEdge>> result;
    result.reserve(adjacencyList[node].size());
    for (const auto& edge : adjacencyList[node]) {
      result.push_back(edge.second);
    }
    return result;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentListGraph::getEdges() {
    throw runtime_error("Not implemented yet.");
    /* implemented, but not tested yet
    std::vector<std::array<int, 2>> edges;
    for (int i = 0; i < numNodes; i++) {
      for (int j = 0; j < adjacencyList[i].size(); j++) {
        if (adjacencyList[i][j] >= i) edges.push_back(std::array{i, adjacencyList[i][j]});
      }
    }
    return edges;*/
  }

  std::shared_ptr<const edge::SuperEdge> AdjacentListGraph::getEdge(const int u, const int v) {
    return adjacencyList[u][v];
  }
}
