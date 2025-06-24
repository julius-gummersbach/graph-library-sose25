#ifndef ADJACENTLISTGRAPH_H
#define ADJACENTLISTGRAPH_H

#include <map>

#include "SuperGraph.h"
#include "../edge/CostCapEdge.h"

namespace graph {
  class AdjacentListGraph : public SuperGraph {
    public:
      explicit AdjacentListGraph();
      /**
       * @param mstEdgeList must be an edge list of an MST
       */
      explicit AdjacentListGraph(const std::vector<std::shared_ptr<const edge::CostCapEdge>>& mstEdgeList);
      explicit AdjacentListGraph(const std::vector<std::shared_ptr<const edge::SuperEdge>>& edgeList, bool isDirected);
      void initializeFromInput(std::istream& input) override;

      [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
      std::vector<std::shared_ptr<const edge::SuperEdge>> getEdges() override;
      std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
      void addEdge(std::shared_ptr<const edge::SuperEdge> edge) override;
      double getBalance(int node) override;
    private:
      std::vector<std::vector<std::shared_ptr<const edge::SuperEdge>>> adjacencyList;
  };
}


#endif //ADJACENTLISTGRAPH_H
