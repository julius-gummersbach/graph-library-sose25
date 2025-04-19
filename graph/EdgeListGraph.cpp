#include "EdgeListGraph.h"

namespace graph {
  void EdgeListGraph::initializeFromInput(std::istream& input) {
    input >> numNodes;

    int a, b;
    while (input >> a >> b) {
      // todo
    }
    initialized = true;
  }

  const std::vector<int>& EdgeListGraph::getAdjacentNodes(int node) {
    return {};
  }
}
