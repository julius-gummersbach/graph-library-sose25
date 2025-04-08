#include "AdjacentListGraph.h"
#include <iostream>
namespace graph {

  using namespace std;

  AdjacentListGraph::AdjacentListGraph(ifstream& input) : SuperGraph(input) {
    input >> numNodes;
    adjacencyList.resize(numNodes);
    int a, b;
    while (input >> a >> b) {
      if (a < 0 || a >= numNodes || b < 0 || b >= numNodes) {
        cerr << "Invalid edge in input: " << a << " -> " << b << endl;
        continue;
      }
      adjacencyList[a].push_back(b);
      if (!directed) {
        adjacencyList[b].push_back(a);
      }
    }
  }

  AdjacentListGraph::~AdjacentListGraph() = default;

  const vector<int>& AdjacentListGraph::getAdjacentNodes(int node) {
    return adjacencyList[node];
  }
}