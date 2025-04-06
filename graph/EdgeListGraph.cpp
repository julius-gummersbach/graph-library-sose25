#include "EdgeListGraph.h"

namespace graph {
  EdgeListGraph::EdgeListGraph(std::ifstream& input) : SuperGraph(input) {
    };
  EdgeListGraph::~EdgeListGraph(){
    };

  std::vector<int> EdgeListGraph::getAdjacentNodes(int node) const {
    return std::vector<int>();
  };
}