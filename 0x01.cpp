#include <vector>

#include "graph/SuperGraph.h"

using namespace std;

/**
 * @param visited array denoting which nodes were already visited
 * @return the order the nodes were visited in
 */
vector<int> depthFirstSearch(graph::SuperGraph &graph, const int startNode, vector<bool> &visited) {
  vector<int> subGraph;
  subGraph.reserve(graph.numNodes / 2);
  stack<int> stack{};
  stack.push(startNode);
  while (!stack.empty()) {
    int currentNode = stack.top();
    stack.pop();
    subGraph.push_back(currentNode);
    visited[currentNode] = true;
    for (const vector<int> &adjacent = graph.getAdjacentNodes(currentNode); int node: adjacent) {
      if (visited[node]) continue;
      stack.push(node);
    }
  }
  return subGraph;
}

/**
 * @return number of connected components graph consists of
 */
double getConnectedComponents(graph::SuperGraph &graph) {
  double connectedComponents = 0;
  vector visited(graph.numNodes, false);

  for (int node = 0; node < graph.numNodes; node++) {
    if (visited[node]) continue;
    connectedComponents++;
    depthFirstSearch(graph, node, visited);
  }
  return connectedComponents;
}
