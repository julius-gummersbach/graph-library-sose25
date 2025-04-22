#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      void initializeFromInput(std::istream& input) override;

      [[nodiscard]] const std::vector<int>& getAdjacentNodes(int node) override;
      std::vector<edge_t> getEdges() override;
      double getWeight(int u, int v) override;
    private:
      std::vector<edge_t> edgeList;
  };
}


#endif //EDGELISTGRAPH_H
