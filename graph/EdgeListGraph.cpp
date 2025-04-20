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

  std::vector<EdgeListGraph::edge_t> EdgeListGraph::getEdgesSortedByWeight() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    std::vector<edge_t> sortedEdges = edgeList;
    std::ranges::sort(sortedEdges, [this](const auto &e1, const auto &e2) {
      return this->getWeight(e1) < this->getWeight(e2);
    });
    return sortedEdges;
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
