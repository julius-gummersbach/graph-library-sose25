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
      auto edge = std::array{a, b};
      edgeList.push_back(edge);
      weights[edge] = weight;
    }
    initialized = true;
  }

  std::vector<EdgeListGraph::edge_t> EdgeListGraph::getEdges() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    return edgeList;
  }

  double EdgeListGraph::getWeight(const edge_t edge) {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    return weights[edge];
  }

  const std::vector<int>& EdgeListGraph::getAdjacentNodes(int node) {
    return {};
  }
}
