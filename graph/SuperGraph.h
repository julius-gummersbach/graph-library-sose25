//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>

namespace graph {
  class SuperGraph {
  public:
    SuperGraph();
    const int numNodes;

    std::vector<int> getAdjacentNodes(int node) const;
  };
}

#endif //SuperGraph_H
