#ifndef EDGELISTGRAPH_H
#define EDGELISTGRAPH_H

#include <map>

#include "SuperGraph.h"
#include "../edge/CostCapEdge.h"

namespace graph {
  class EdgeListGraph : public SuperGraph {
    public:
      void initializeFromInput(std::istream& input) override;
      explicit EdgeListGraph();
      explicit EdgeListGraph(const std::vector<std::shared_ptr<const edge::CostCapEdge>>& list, const bool isDirected) {
        for (const auto& e : list) {
          edgeList.push_back(e);
          numNodes = std::max(numNodes, std::max(e->getFrom(), e->getTo()) + 1);
        }
        initialized = true;
        weighted = true;
        directed = isDirected;
      }
      [[nodiscard]] std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) override;
      std::shared_ptr<const std::vector<std::shared_ptr<const edge::SuperEdge>>> getEdges() override;
      std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) override;
      void addEdge(std::shared_ptr<const edge::SuperEdge> edge) override;
      void removeEdge(const std::shared_ptr<const edge::SuperEdge>& edge);
      double getBalance(int node) override;
    private:
      std::vector<std::shared_ptr<const edge::SuperEdge>> edgeList;
      std::vector<double> balance;
  };
}


#endif //EDGELISTGRAPH_H
