#include <unordered_set>
#include <vector>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"

using namespace std;


vector<vector<shared_ptr<const edge::WeightedEdge>>> buildPathsFromEdgeList(const graph::SuperGraph &graph, const int startNode, const vector<shared_ptr<const edge::SuperEdge>> &shortestPath) {
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

/**
 * @param graph a weighted graph
 * @param startNode node to start shortest-path-search from
 * @return vector that contains the shortest path from startNode to node i at index i, and an empty path at index startNode
 */
vector<vector<shared_ptr<const edge::WeightedEdge>>> dijkstra(graph::SuperGraph &graph, const int startNode) {
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;

  vector<shared_ptr<const edge::SuperEdge>> predecessorList(graph.numNodes, nullptr);

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
        predecessorList[edge->getTo()] = edge;
      }
    }
  }
  return buildPathsFromEdgeList(graph, startNode, predecessorList);
}

vector<vector<shared_ptr<const edge::WeightedEdge>>> mooreBellmanFord(graph::SuperGraph &graph, const int startNode) {
  auto edgeList = graph.getEdges();
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;
  vector<shared_ptr<const edge::SuperEdge>> predecessorList(graph.numNodes, nullptr);

  if (!graph.directed) {
    const auto edgeListCopy = edgeList;
    for (const auto& edge : edgeListCopy) {
      edgeList.push_back(make_shared<edge::WeightedEdge>(edge->getTo(), edge->getFrom(), dynamic_pointer_cast<const edge::WeightedEdge>(edge)->getWeight()));
    }
  }

  bool didUpdate = true;
  for (int i = 0; i < graph.numNodes; i++) {
    if (!didUpdate) break;
    didUpdate = false;
    for (const auto& edge : edgeList){
      double newDistance = shortestDistance[edge->getFrom()] + dynamic_pointer_cast<const edge::WeightedEdge>(edge)->getWeight();
      if (newDistance < shortestDistance[edge->getTo()]) {
        if (i == graph.numNodes - 1) throw std::invalid_argument("Graph contains negative-weight cycle");
        didUpdate = true;
        shortestDistance[edge->getTo()] = newDistance;
        predecessorList[edge->getTo()] = edge;
      }
    }
  }
  return buildPathsFromEdgeList(graph, startNode, predecessorList);
}

