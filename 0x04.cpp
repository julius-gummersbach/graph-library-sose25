#include <unordered_set>
#include <vector>
#include <boost/heap/fibonacci_heap.hpp>

#include "edge/CostCapEdge.h"
#include "graph/SuperGraph.h"


static map<int, vector<shared_ptr<const edge::CostCapEdge>>> buildPathsFromEdgeList(const graph::SuperGraph &graph, const int startNode, const vector<shared_ptr<const edge::CostCapEdge>> &predecessorList) {
  map<int, vector<shared_ptr<const edge::CostCapEdge>>> result;
  for (int i = 0; i < graph.numNodes; i++) {
    if (predecessorList[i] == nullptr) continue;
    vector<shared_ptr<const edge::CostCapEdge>> path;
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

static map<int, vector<shared_ptr<const edge::CostCapEdge>>> buildNegativeCycleFromEdgeList(const graph::SuperGraph &graph, const int searchStartNode, const vector<shared_ptr<const edge::CostCapEdge>> &predecessorList) {
  map<int, vector<shared_ptr<const edge::CostCapEdge>>> result;
  unordered_set<int> visited;
  vector<shared_ptr<const edge::CostCapEdge>> path;
  path.reserve(graph.numNodes);

  int currentNode = searchStartNode;
  while (!visited.contains(currentNode)) {
    visited.insert(currentNode);
    path.insert(path.begin(), predecessorList[currentNode]);
    currentNode = predecessorList[currentNode]->getFrom();
  }
  while (path.back()->getTo() != currentNode) path.pop_back();

  result[-1] = path;
  return result;
}

/**
 * @param graph a weighted graph
 * @param startNode node to start shortest-path-search from
 * @return map that contains the shortest path from startNode to node i at index i,
 *         an empty path at index startNode and has no entries for unreachable nodes.
 */
static map<int, vector<shared_ptr<const edge::CostCapEdge>>> dijkstra(graph::SuperGraph &graph, const int startNode) {
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;

  vector<shared_ptr<const edge::CostCapEdge>> predecessorList(graph.numNodes, nullptr);

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
      const auto weightedEdge = static_pointer_cast<const edge::CostCapEdge>(edge);
      double newDistance = shortestDistance[currentNode] + weightedEdge->getCost();
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
 * @return Map that contains the shortest path from startNode to node i at index i,
 *         an empty path at index startNode and has no entries for unreachable nodes.
 *         If the graph contains a negative-weight cycle, the map contains this cycle at index -1 and no other entries
 */
static map<int, vector<shared_ptr<const edge::CostCapEdge>>> mooreBellmanFord(graph::SuperGraph &graph, const int startNode) {
  vector<shared_ptr<const edge::CostCapEdge>> edgeList;
  const auto superEdgeList = graph.getEdges();
  for (const auto& edge : *superEdgeList) {
    edgeList.push_back(static_pointer_cast<const edge::CostCapEdge>(edge));
  }
  vector<double> shortestDistance(graph.numNodes, INFINITY);
  shortestDistance[startNode] = 0;
  vector<shared_ptr<const edge::CostCapEdge>> predecessorList(graph.numNodes, nullptr);

  if (!graph.directed) {
    const auto edgeListCopy = edgeList;
    edgeList.reserve(edgeList.size() * 2);
    for (const auto& edge : edgeListCopy) {
      edgeList.push_back(make_shared<edge::CostCapEdge>(edge->getTo(), edge->getFrom(), edge->getCost()));
    }
  }

  bool negativeCycle = false;
  int negativeCycleSearchStart = -1;
  for (int i = 0; i < graph.numNodes; i++) {
    bool didUpdate = false;
    for (const auto& edge : edgeList){
      double newDistance = shortestDistance[edge->getFrom()] + edge->getCost();
      if (newDistance < shortestDistance[edge->getTo()]) {
        if (i == graph.numNodes - 1) {
          negativeCycle = true;
          negativeCycleSearchStart = edge->getTo();
          break;
        }
        didUpdate = true;
        shortestDistance[edge->getTo()] = newDistance;
        predecessorList[edge->getTo()] = edge;
      }
    }
    if (!didUpdate) break;
  }
  if (negativeCycle) {
    return buildNegativeCycleFromEdgeList(graph, negativeCycleSearchStart, predecessorList);
  }
  return buildPathsFromEdgeList(graph, startNode, predecessorList);
}
