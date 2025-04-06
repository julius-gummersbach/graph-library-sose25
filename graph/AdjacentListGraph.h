//
// Created by Julius Gummersbach on 06.04.25.
//

#ifndef ADJACENTLISTGRAPH_H
#define ADJACENTLISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class AdjacentListGraph : public SuperGraph {
    public:
      explicit AdjacentListGraph(std::ifstream& input);
      ~AdjacentListGraph() override;
      [[nodiscard]] std::vector<int> getAdjacentNodes(int node) const override;
    private:
      std::vector<int>* adjacencyList;
  };
}


#endif //ADJACENTLISTGRAPH_H
