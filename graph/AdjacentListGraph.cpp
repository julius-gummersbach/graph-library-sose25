#include "AdjacentListGraph.h"

namespace graph {

  using namespace std;

  AdjacentListGraph::AdjacentListGraph(ifstream& input) : SuperGraph(input) {
    input >> numNodes;
    adjacencyList = new vector<int>[numNodes];
    int a, b;
    while (input >> a >> b) {
      adjacencyList[a].push_back(b);
    }
    input.close();
  }

  AdjacentListGraph::~AdjacentListGraph() {
    delete[] adjacencyList;
  }

  vector<int> AdjacentListGraph::getAdjacentNodes(int node) const {
    return adjacencyList[node];
  }
}