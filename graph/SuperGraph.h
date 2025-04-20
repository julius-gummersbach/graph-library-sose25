#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>
#include <fstream>

namespace graph {
  class SuperGraph {
  public:
    explicit SuperGraph() = default;
    virtual ~SuperGraph();

    int numNodes = 0;
    bool directed = false;
    bool initialized = false;

    virtual void initializeFromInput(std::istream& input) = 0;
    [[nodiscard]] const virtual std::vector<int>& getAdjacentNodes(int node) = 0;

    virtual std::vector<std::array<int, 2>> getEdgesSortedByWeight() = 0;
    virtual double getWeight(std::array<int, 2> edge) = 0;
  };
}

#endif //SuperGraph_H
