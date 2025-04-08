//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      explicit EdgeListGraph(std::istream& input);
      ~EdgeListGraph() override;
      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
  };
}


#endif //EDGELISTGRAPH_H
