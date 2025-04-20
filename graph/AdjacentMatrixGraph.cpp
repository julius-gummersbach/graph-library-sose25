#include <string>
#include <sstream>
#include "AdjacentMatrixGraph.h"

#include <iostream>

namespace graph {
  using namespace std;

  void AdjacentMatrixGraph::initializeFromInput(istream &input) {
    input >> numNodes;
    adjacencyMatrix = new double[numNodes * numNodes];
    int a, b;
    while (input >> a >> b) {
      if (a < 0 || a >= numNodes || b < 0 || b >= numNodes) {
        cerr << "Invalid edge in input: " << a << " -> " << b << endl;
        continue;
      }
      if (!directed && b < a) {
        adjacencyMatrix[b * numNodes + a] = 1;
      } else {
        adjacencyMatrix[a * numNodes + b] = 1;
      }
    }
    initialized = true;
  }

  const vector<int>& AdjacentMatrixGraph::getAdjacentNodes(int node) {
    if (!initialized) {
      throw runtime_error("Graph not initialized.");
    }
    if (adjacencyCache.contains(node)) {
      return adjacencyCache.at(node);
    }
    vector<int> adjacentNodes;
    for (int i = 0; i < numNodes; i++) {
      if (!directed) {
        if (i < node) {
          if (adjacencyMatrix[i * numNodes + node] != 0) {
            adjacentNodes.push_back(i);
          }
        } else {
          if (adjacencyMatrix[node * numNodes + i] != 0) {
            adjacentNodes.push_back(i);
          }
        }
      } else if (adjacencyMatrix[node * numNodes + i] != 0) {
        adjacentNodes.push_back(i);
      }
    }
    adjacencyCache[node] = adjacentNodes;
    return adjacencyCache[node];
  }

  std::vector<std::array<int, 2>> AdjacentMatrixGraph::getEdgesSortedByWeight() {
    throw runtime_error("Not implemented yet.");
  }

  double AdjacentMatrixGraph::getWeight(std::array<int, 2> edge) {
    throw runtime_error("Not implemented yet.");
  }
}
