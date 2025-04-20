//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef ADJACENTMATRIXGRAPH_H
#define ADJACENTMATRIXGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class AdjacentMatrixGraph : public SuperGraph {
  public:
    void initializeFromInput(std::istream& input) override;
    [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
    std::vector<std::array<int, 2>> getEdgesSortedByWeight() override;
    double getWeight(std::array<int, 2> edge) override;
  private:
    double* adjacencyMatrix;
    std::map<int, std::vector<int>> adjacencyCache;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
