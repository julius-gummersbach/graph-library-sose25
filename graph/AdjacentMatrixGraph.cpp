#include <string>
#include <sstream>
#include "AdjacentMatrixGraph.h"

#include <cfloat>
#include <iostream>

#include "../edge/SingleWeightedEdge.h"

namespace graph {

  void AdjacentMatrixGraph::initializeFromInput(std::istream &input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string line;
    std::getline(input, line);
    std::istringstream ls(line);
    ls >> numNodes;
    adjacencyMatrix = std::vector<std::shared_ptr<const edge::SuperEdge>>();
    adjacencyMatrix.resize(numNodes * numNodes);
    for (int i = 0; i < numNodes; i++) {
      for (int j = 0; j < numNodes; j++) {
        adjacencyMatrix[i * numNodes + j] = nullptr;
      }
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

      int index;
      if (!directed && v < u) {
        index = v * numNodes + u;
      } else {
        index = u * numNodes + v;
      }

      if (weighted) {
        ls >> w;
        adjacencyMatrix[index] = std::make_shared<const edge::SingleWeightedEdge>(u, v, w);
      } else {
        adjacencyMatrix[index] = std::make_shared<const edge::SuperEdge>(u,v);
      }
    } while (std::getline(input, line));
    initialized = true;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentMatrixGraph::getAdjacent(const int node) {
    if (adjacencyCache.contains(node)) {
      return adjacencyCache.at(node);
    }
    std::vector<std::shared_ptr<const edge::SuperEdge>> adjacent;
    for (int i = 0; i < numNodes; i++) {
      int index;
      if (!directed && i < node) {
        index = i * numNodes + node;
      } else {
        index = node * numNodes + i;
      }
      if (adjacencyMatrix[index] != nullptr) adjacent.push_back(adjacencyMatrix[index]);
    }
    adjacencyCache[node] = adjacent;
    return adjacent;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentMatrixGraph::getEdges() {
    throw std::runtime_error("Not implemented yet.");
  }

  std::shared_ptr<const edge::SuperEdge> AdjacentMatrixGraph::getEdge(const int u, const int v) {
    if (!directed && v < u) return adjacencyMatrix[v * numNodes + u];
    return adjacencyMatrix[u * numNodes + v];
  }
}
