#include "EdgeListGraph.h"

namespace graph {
  EdgeListGraph::EdgeListGraph(std::istream& input) : SuperGraph(input) {}
  EdgeListGraph::~EdgeListGraph() = default;

  const std::vector<int>& EdgeListGraph::getAdjacentNodes(int node) {
    return {};
  }
}