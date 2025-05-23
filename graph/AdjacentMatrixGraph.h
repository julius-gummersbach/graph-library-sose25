#ifndef ADJACENTMATRIXGRAPH_H
#define ADJACENTMATRIXGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class AdjacentMatrixGraph : public SuperGraph {
  public:
    void initializeFromInput(std::istream& input) override;
    [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
    std::vector<std::shared_ptr<const edge::SuperEdge>> getEdges() override;
    std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
  private:
    std::vector<std::shared_ptr<const edge::SuperEdge>> adjacencyMatrix;
    std::map<int, std::vector<std::shared_ptr<const edge::SuperEdge>>> adjacencyCache;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
