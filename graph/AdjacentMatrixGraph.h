#ifndef ADJACENTMATRIXGRAPH_H
#define ADJACENTMATRIXGRAPH_H

#include <map>

#include "SuperGraph.h"
#include "../edge/CostCapEdge.h"

namespace graph {
  class AdjacentMatrixGraph : public SuperGraph {
  public:
    explicit AdjacentMatrixGraph();
    /**
     * creates a new directed, weighted graph from the given edgeList
     * @param edgeList
     */
    explicit AdjacentMatrixGraph(const std::vector<std::shared_ptr<const edge::SuperEdge>>& edgeList, bool isDirected);
    void initializeFromInput(std::istream& input) override;
    [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
    std::shared_ptr<const std::vector<std::shared_ptr<const edge::SuperEdge>>> getEdges() override;
    std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
    void addEdge(std::shared_ptr<const edge::SuperEdge> edge) override;
    double getBalance(int node) override;
  private:
    std::vector<std::shared_ptr<const edge::SuperEdge>> adjacencyMatrix;
    std::map<int, std::vector<std::shared_ptr<const edge::SuperEdge>>> adjacencyCache;
  };
}


#endif //ADJACENTMATRIXGRAPH_H
