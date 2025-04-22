#include "AdjacentListGraph.h"

#include <iostream>
#include <sstream>

namespace graph {
  using namespace std;

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
    if (!initialized) {
      throw runtime_error("Graph not initialized.");
    }
    return adjacencyList[node];
  }

  std::vector<std::array<int, 2> > AdjacentListGraph::getEdges() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
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

  double AdjacentListGraph::getWeight(std::array<int, 2> edge) {
    if (!initialized) {
      throw runtime_error("Graph not initialized.");
    }
    if (!weighted) {
      throw runtime_error("Graph is not weighted.");
    }
    for (int i = 0; i < adjacencyList[edge[0]].size(); i++) {
      if (adjacencyList[edge[0]][i] == edge[1]) {
        return weights[edge[0]][i];
      }
    }
    throw runtime_error("Edge not found.");
  }
}
