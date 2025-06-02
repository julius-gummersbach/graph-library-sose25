#include <unordered_set>
#include <vector>
#include <boost/heap/fibonacci_heap.hpp>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"

using namespace std;


map<int, vector<shared_ptr<const edge::WeightedEdge>>> buildPathsFromEdgeList(const graph::SuperGraph &graph, const int startNode, const vector<shared_ptr<const edge::WeightedEdge>> &predecessorList) {
  map<int, vector<shared_ptr<const edge::WeightedEdge>>> result;
  for (int i = 0; i < graph.numNodes; i++) {
    if (predecessorList[i] == nullptr) continue;
    vector<shared_ptr<const edge::WeightedEdge>> path;
    path.reserve(graph.numNodes);
    int currentNode = i;
    while (currentNode != startNode) {
      path.insert(path.begin(), predecessorList[currentNode]);
      currentNode = predecessorList[currentNode]->getFrom();
    }
    result[i] = path;
  }
  return result;
}

/**
 * @param graph a weighted graph
 * @param startNode node to start shortest-path-search from
 * @return map that contains the shortest path from startNode to node i at index i,
 *         an empty path at index startNode and has no entries for unreachable nodes.
 */
map<int, vector<shared_ptr<const edge::WeightedEdge>>> dijkstra(graph::SuperGraph &graph, const int startNode) {
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;

  vector<shared_ptr<const edge::WeightedEdge>> predecessorList(graph.numNodes, nullptr);

  auto cmp = [&shortestDistance](const int& lhs, const int& rhs) {
    return shortestDistance[lhs] > shortestDistance[rhs];
  };
  using Heap = boost::heap::fibonacci_heap<int, boost::heap::compare<decltype(cmp)>, boost::heap::mutable_<true>>;
  Heap toVisit(cmp);
  std::unordered_map<int, Heap::handle_type> handles;
  for (int i = 0; i < graph.numNodes; i++) {
    handles[i] = toVisit.push(i);
  }

  while (!toVisit.empty()) {
    const int currentNode = toVisit.top();
    toVisit.pop();
    if (shortestDistance[currentNode] == INFINITY) break;  // rest of graph is unreachable
    for (const auto& edge: graph.getAdjacent(currentNode)) {
      const auto weightedEdge = static_pointer_cast<const edge::WeightedEdge>(edge);
      double newDistance = shortestDistance[currentNode] + weightedEdge->getWeight();
      if (newDistance < shortestDistance[weightedEdge->getTo()]) {
        shortestDistance[weightedEdge->getTo()] = newDistance;
        if (!toVisit.empty()) toVisit.decrease(handles[weightedEdge->getTo()]);
        predecessorList[weightedEdge->getTo()] = weightedEdge;
      }
    }
  }
  return buildPathsFromEdgeList(graph, startNode, predecessorList);
}

/**
 * @param graph a weighted graph
 * @param startNode node to start shortest-path-search from
 * @return map that contains the shortest path from startNode to node i at index i,
 *         an empty path at index startNode and has no entries for unreachable nodes.
 */
map<int, vector<shared_ptr<const edge::WeightedEdge>>> mooreBellmanFord(graph::SuperGraph &graph, const int startNode) {
  vector<shared_ptr<const edge::WeightedEdge>> edgeList;
  for (const auto& edge : graph.getEdges()) {
    edgeList.push_back(static_pointer_cast<const edge::WeightedEdge>(edge));
  }
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;
  vector<shared_ptr<const edge::WeightedEdge>> predecessorList(graph.numNodes, nullptr);

  if (!graph.directed) {
    const auto edgeListCopy = edgeList;
    edgeList.reserve(edgeList.size() * 2);
    for (const auto& edge : edgeListCopy) {
      edgeList.push_back(make_shared<edge::WeightedEdge>(edge->getTo(), edge->getFrom(), edge->getWeight()));
    }
  }

  for (int i = 0; i < graph.numNodes; i++) {
    bool didUpdate = false;
    for (const auto& edge : edgeList){
      double newDistance = shortestDistance[edge->getFrom()] + edge->getWeight();
      if (newDistance < shortestDistance[edge->getTo()]) {
        if (i == graph.numNodes - 1) throw std::invalid_argument("Graph contains negative-weight cycle");
        didUpdate = true;
        shortestDistance[edge->getTo()] = newDistance;
        predecessorList[edge->getTo()] = edge;
      }
    }
    if (!didUpdate) break;
  }
  return buildPathsFromEdgeList(graph, startNode, predecessorList);
}
