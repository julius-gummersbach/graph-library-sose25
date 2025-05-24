#include <unordered_set>
#include <vector>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"

using namespace std;

/**
 *
 * @param graph a weighted graph
 * @param startNode node to start shortest-path-search from
 * @return vector that contains the shortest path from startNode to node i at index i, and an empty path at index startNode
 */
vector<vector<shared_ptr<const edge::WeightedEdge>>> dijkstra(graph::SuperGraph &graph, int startNode) {
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;

  vector<shared_ptr<const edge::SuperEdge>> shortestPath(graph.numNodes, nullptr);

  unordered_set<int> toVisit;
  toVisit.reserve(graph.numNodes);
  for (int i = 0; i < graph.numNodes; i++) {
    toVisit.insert(i);
  }

  while (!toVisit.empty()) {
    int currentNode = *toVisit.begin();
    for (const int v : toVisit) {
      if (shortestDistance[v] < shortestDistance[currentNode]) currentNode = v;
    }
    toVisit.erase(currentNode);
    for (const auto& edge: graph.getAdjacent(currentNode)) {
      double newDistance = shortestDistance[currentNode] + dynamic_pointer_cast<const edge::WeightedEdge>(edge)->getWeight();
      if (newDistance < shortestDistance[edge->getTo()]) {
        shortestDistance[edge->getTo()] = newDistance;
        shortestPath[edge->getTo()] = edge;
      }
    }
  }
  vector<vector<shared_ptr<const edge::WeightedEdge>>> result;
  result.reserve(graph.numNodes);
  for (int i = 0; i < graph.numNodes; i++) {
    if (i == startNode) {
      result.emplace_back();
      continue;
    };
    vector<shared_ptr<const edge::WeightedEdge>> path;
    path.reserve(graph.numNodes);
    int currentNode = i;
    while (currentNode != startNode) {
      path.insert(path.begin(), dynamic_pointer_cast<const edge::WeightedEdge>(shortestPath[currentNode]));
      currentNode = shortestPath[currentNode]->getFrom();
    }
    result.push_back(path);
  }
  return result;
}
