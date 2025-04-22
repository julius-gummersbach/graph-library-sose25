#include "EdgeListGraph.h"

namespace graph {

  /*
   * currently only supports weighted graphs
   */
  void EdgeListGraph::initializeFromInput(std::istream& input) {
    input >> numNodes;
    int a, b;
    double weight;
    while (input >> a >> b >> weight) {
      auto edge = std::tuple{a, b, weight};
      edgeList.push_back(edge);
    }
    initialized = true;
  }

  std::vector<EdgeListGraph::edge_t> EdgeListGraph::getEdges() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    return edgeList;
  }

  double EdgeListGraph::getWeight(int u, int v) {
    throw std::runtime_error("Not implemented yet.");
  }

  const std::vector<int>& EdgeListGraph::getAdjacentNodes(int node) {
    throw std::runtime_error("Not implemented yet.");
  }
}
