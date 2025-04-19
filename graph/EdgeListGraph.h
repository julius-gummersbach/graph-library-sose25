//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      void initializeFromInput(std::istream& input) override;
      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
    private:
      std::vector<std::vector<int>> adjacencyList;
  };
}


#endif //EDGELISTGRAPH_H
