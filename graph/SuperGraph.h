#ifndef SuperGraph_H
#define SuperGraph_H
#include <vector>
#include <fstream>

namespace graph {
  class SuperGraph {
  public:
    explicit SuperGraph() = default;
    virtual ~SuperGraph();
    typedef std::tuple<int, int, double> edge_t;

    int numNodes = 0;
    bool directed = false;
    bool weighted = false;
    bool initialized = false;

    virtual void initializeFromInput(std::istream& input) = 0;
    [[nodiscard]] const virtual std::vector<int>& getAdjacentNodes(int node) = 0;

    virtual std::vector<edge_t> getEdges() = 0;
    virtual double getWeight(int u, int v) = 0;
  };
}

#endif //SuperGraph_H
