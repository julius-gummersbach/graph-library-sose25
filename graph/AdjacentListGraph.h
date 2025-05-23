#ifndef ADJACENTLISTGRAPH_H
#define ADJACENTLISTGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class AdjacentListGraph : public SuperGraph {
    public:
      explicit AdjacentListGraph();
      /**
       * @param mstEdgeList must be an edge list of an MST
       */
      explicit AdjacentListGraph(const std::vector<std::shared_ptr<edge::SuperEdge>>& mstEdgeList);
      void initializeFromInput(std::istream& input) override;

      [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
      std::vector<std::shared_ptr<const edge::SuperEdge>> getEdges() override;
      std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
    private:
      std::vector<std::map<int, std::shared_ptr<const edge::SuperEdge>>> adjacencyList;
  };
}


#endif //ADJACENTLISTGRAPH_H
