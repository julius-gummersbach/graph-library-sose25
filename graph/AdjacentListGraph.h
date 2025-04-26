#ifndef ADJACENTLISTGRAPH_H
#define ADJACENTLISTGRAPH_H

#include "SuperGraph.h"

namespace graph {
  class AdjacentListGraph : public SuperGraph {
    public:
      explicit AdjacentListGraph();
      /**
       * @param mstEdgeList must be an edge list of an MST
       */
      explicit AdjacentListGraph(const std::vector<edge_t>& mstEdgeList);

      void initializeFromInput(std::istream& input) override;
      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
      std::vector<edge_t> getEdges() override;
      double getWeight(int u, int v) override;
    private:
      std::vector<std::vector<int>> adjacencyList;
      std::vector<std::vector<double>> weights;
  };
}


#endif //ADJACENTLISTGRAPH_H
