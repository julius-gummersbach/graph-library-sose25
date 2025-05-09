#include "AdjacentListGraph.h"

#include <iostream>
#include <sstream>

namespace graph {
  using namespace std;

  AdjacentListGraph::AdjacentListGraph() = default;

  AdjacentListGraph::AdjacentListGraph(const vector<edge_t>& mstEdgeList) {
    numNodes = mstEdgeList.size() + 1;
    adjacencyList.resize(numNodes);
    weights.resize(numNodes);
    for (auto list : adjacencyList) {
      list.resize(15);
    }
    for (int i = 0; i < numNodes; ++i) {
      adjacencyList[i].reserve(15);
      weights[i].reserve(15);
    }
    for (auto edge : mstEdgeList) {
      adjacencyList[get<0>(edge)].push_back(get<1>(edge));
      adjacencyList[get<1>(edge)].push_back(get<0>(edge));
      weights[get<0>(edge)].push_back(get<2>(edge));
      weights[get<1>(edge)].push_back(get<2>(edge));
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
    weights.resize(numNodes);
    for (int i = 0; i < numNodes; ++i) {
      adjacencyList[i].reserve(15);
      weights[i].reserve(15);
    }

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
      adjacencyList[u].push_back(v);
      if (!directed && u != v) adjacencyList[v].push_back(u);
      if (weighted) {
        ls >> w;
        weights[u].push_back(w);
        if (!directed && u != v) weights[v].push_back(w);
      }
    } while (std::getline(input, line));
    initialized = true;
  }

  const vector<int> & AdjacentListGraph::getAdjacentNodes(int node) {
    return adjacencyList[node];
  }

  std::vector<SuperGraph::edge_t> AdjacentListGraph::getEdges() {
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

  double AdjacentListGraph::getWeight(const int u, const int v) {
    if (!weighted) {
      throw runtime_error("Graph is not weighted.");
    }
    for (int i = 0; i < adjacencyList[u].size(); i++) {
      if (adjacencyList[u][i] == v) {
        return weights[u][i];
      }
    }
    throw runtime_error("Edge not found.");
  }
}
