//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      explicit EdgeListGraph(std::ifstream& input);
      ~EdgeListGraph() override;
      [[nodiscard]] std::vector<int> getAdjacentNodes(int node) const override;
  };
}


#endif //EDGELISTGRAPH_H
