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
      std::vector<std::array<int, 2>> getEdges() override;
      double getWeight(std::array<int, 2> edge) override;
    private:
      std::vector<std::vector<int>> adjacencyList;
      std::vector<std::vector<double>> weights;

  };
}


#endif //ADJACENTLISTGRAPH_H
