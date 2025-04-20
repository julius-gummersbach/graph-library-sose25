//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      using edge_t = std::array<int, 2>;
      void initializeFromInput(std::istream& input) override;

      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
      std::vector<edge_t> getEdgesSortedByWeight() override;
      double getWeight(edge_t edge) override;

    private:
      std::vector<edge_t> edgeList;  // int array with two entries: node -> node
      std::map<edge_t, double> weights;
  };
}


#endif //EDGELISTGRAPH_H
