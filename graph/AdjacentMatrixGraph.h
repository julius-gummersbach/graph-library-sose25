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
    explicit AdjacentMatrixGraph(std::ifstream& input);
    ~AdjacentMatrixGraph() override;
    [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
  private:
    double* adjacencyMatrix;
    std::map<int, std::vector<int>> adjacencyCache;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
