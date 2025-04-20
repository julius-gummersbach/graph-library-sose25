//
// Created by Julius Gummersbach on 06.04.25.
//

#ifndef ADJACENTLISTGRAPH_H
#define ADJACENTLISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class AdjacentListGraph : public SuperGraph {
    public:
      void initializeFromInput(std::istream& input) override;
      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
      std::vector<std::array<int, 2>> getEdgesSortedByWeight() override;
      double getWeight(std::array<int, 2> edge) override;
    private:
      std::vector<std::vector<int>> adjacencyList;

  };
}


#endif //ADJACENTLISTGRAPH_H
