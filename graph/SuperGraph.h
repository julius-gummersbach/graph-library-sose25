#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>
#include <fstream>

namespace graph {
  class SuperGraph {
  public:
    explicit SuperGraph() = default;
    virtual ~SuperGraph();
    typedef std::array<int, 2> edge_t;

    int numNodes = 0;
    bool directed = false;
    bool weighted = false;
    bool initialized = false;

    virtual void initializeFromInput(std::istream& input) = 0;
    [[nodiscard]] const virtual std::vector<int>& getAdjacentNodes(int node) = 0;

    virtual std::vector<edge_t> getEdges() = 0;
    virtual double getWeight(edge_t edge) = 0;
  };
}

#endif //SuperGraph_H
