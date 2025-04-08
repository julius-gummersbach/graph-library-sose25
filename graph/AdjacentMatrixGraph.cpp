#include <string>
#include <sstream>
#include "AdjacentMatrixGraph.h"

#include <iostream>

namespace graph {
  using namespace std;

  AdjacentMatrixGraph::AdjacentMatrixGraph(istream &input) : SuperGraph(input) {
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
  }

  AdjacentMatrixGraph::~AdjacentMatrixGraph() {
    delete[] adjacencyMatrix;
  }

  const vector<int>& AdjacentMatrixGraph::getAdjacentNodes(int node) {
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
}
