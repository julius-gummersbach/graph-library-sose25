#ifndef ADJACENTMATRIXGRAPH_H
#define ADJACENTMATRIXGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class AdjacentMatrixGraph : public SuperGraph {
  public:
    void initializeFromInput(std::istream& input) override;
    [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
    std::vector<edge_t> getEdges() override;
    double getWeight(int u, int v) override;
  private:
    double* adjacencyMatrix;
    std::map<int, std::vector<int>> adjacencyCache;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
