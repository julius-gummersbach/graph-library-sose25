#include <string>
#include <sstream>
#include "AdjacentMatrixGraph.h"

#include <cfloat>
#include <iostream>

namespace graph {

  void AdjacentMatrixGraph::initializeFromInput(std::istream &input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string line;
    std::getline(input, line);
    std::istringstream ls(line);
    ls >> numNodes;
    adjacencyMatrix = new double[numNodes * numNodes];
    for (int i = 0; i < numNodes; i++) {
      for (int j = 0; j < numNodes; j++) {
        adjacencyMatrix[i * numNodes + j] = DBL_MAX;
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
      double weight = 1;
      if (weighted) {
        ls >> weight;
      }

      if (!directed && v < u) {
        adjacencyMatrix[v * numNodes + u] = weight;
      } else {
        adjacencyMatrix[u * numNodes + v] = weight;
      }
    } while (std::getline(input, line));
    initialized = true;
  }

  const std::vector<int>& AdjacentMatrixGraph::getAdjacentNodes(int node) {
    if (adjacencyCache.contains(node)) {
      return adjacencyCache.at(node);
    }
    std::vector<int> adjacentNodes;
    for (int i = 0; i < numNodes; i++) {
      if (!directed) {
        if (i < node) {
          if (adjacencyMatrix[i * numNodes + node] != DBL_MAX) {
            adjacentNodes.push_back(i);
          }
        } else {
          if (adjacencyMatrix[node * numNodes + i] != DBL_MAX) {
            adjacentNodes.push_back(i);
          }
        }
      } else if (adjacencyMatrix[node * numNodes + i] != DBL_MAX) {
        adjacentNodes.push_back(i);
      }
    }
    adjacencyCache[node] = adjacentNodes;
    return adjacencyCache[node];
  }

  std::vector<SuperGraph::edge_t> AdjacentMatrixGraph::getEdges() {
    throw std::runtime_error("Not implemented yet.");
  }

  double AdjacentMatrixGraph::getWeight(const int u, const int v) {
    if (!directed && v < u) return adjacencyMatrix[v * numNodes + u];
    return adjacencyMatrix[u * numNodes + v];
  }
}
