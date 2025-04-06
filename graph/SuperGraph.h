#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>
#include <fstream>

namespace graph {
  class SuperGraph {
  public:
    explicit SuperGraph(std::ifstream& input);
    virtual ~SuperGraph() = default;

    int numNodes;

    [[nodiscard]] virtual std::vector<int> getAdjacentNodes(int node) const = 0;
  };
}

#endif //SuperGraph_H
