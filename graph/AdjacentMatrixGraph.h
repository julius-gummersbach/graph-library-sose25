//
// Created by Julius Gummersbach on 04.04.25.
//

#ifndef ADJACENTMATRIXGRAPH_H
#define ADJACENTMATRIXGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class AdjacentMatrixGraph : public SuperGraph {
  public:
    explicit AdjacentMatrixGraph(std::ifstream& input);
    ~AdjacentMatrixGraph() override;
    [[nodiscard]] std::vector<int> getAdjacentNodes(int node) const override;
  private:
    double* adjacencyMatrix;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
