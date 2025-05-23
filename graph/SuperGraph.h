#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>
#include <fstream>

#include "../edge/SuperEdge.h"

namespace graph {
  class SuperGraph {
  public:
    explicit SuperGraph() = default;
    virtual ~SuperGraph();

    int numNodes = 0;
    bool directed = false;
    bool weighted = false;
    bool initialized = false;

    virtual void initializeFromInput(std::istream& input) = 0;
    [[nodiscard]] virtual std::vector<std::shared_ptr<const edge::SuperEdge>> getAdjacent(int node) = 0;

    virtual std::vector<std::shared_ptr<const edge::SuperEdge>> getEdges() = 0;
    virtual std::shared_ptr<const edge::SuperEdge> getEdge(int u, int v) = 0;
  };
}

#endif //SuperGraph_H
