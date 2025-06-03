#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include <map>

#include "SuperGraph.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      void initializeFromInput(std::istream& input) override;

    [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
    std::vector<std::shared_ptr<const edge::SuperEdge>> getEdges() override;
    std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
    void addEdge(std::shared_ptr<const edge::SuperEdge> edge) override;
    private:
      std::vector<std::shared_ptr<const edge::SuperEdge>> edgeList;
  };
}


#endif //EDGELISTGRAPH_H
