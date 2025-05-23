#include "EdgeListGraph.h"

#include "../edge/SingleWeightedEdge.h"

namespace graph {

  /*
   * currently only supports weighted graphs
   */
  void EdgeListGraph::initializeFromInput(std::istream& input) {
    weighted = true;
    input >> numNodes;
    int u, v;
    double w;
    while (input >> u >> v >> w) {
      edgeList.push_back(std::make_shared<const edge::SingleWeightedEdge>(u, v, w));
    }
    initialized = true;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> EdgeListGraph::getEdges() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    return edgeList;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> EdgeListGraph::getAdjacent(int node) {
    throw std::runtime_error("Not implemented yet.");
  }

  std::shared_ptr<const edge::SuperEdge> EdgeListGraph::getEdge(int u, int v) {
    throw std::runtime_error("Not implemented yet.");
  }
}
