#include <map>

#include "edge/CostCapEdge.h"
#include "edge/CostCapEdgeHasher.h"
#include "graph/SuperGraph.h"
#include "helper/AugmentingPathFinder.h"
#include "helper/BfsAugmentingPathFinder.h"


/**
 * Calculates the maximum flow in a graph
 * @param graph the underlying, directed graph with no double edges. it will be modified to a residual graph by this algorithm
 * @param pathFinder a strategy to find augmenting paths
 * @param source source of the flow network
 * @param target sink of the flow network
 * @return the maximum flow from source to sink and a map that maps a flow value to each edge
 */
static pair<double, unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher>> fordFulkerson(graph::SuperGraph& graph, const helper::AugmentingPathFinder* pathFinder, const int source, const int target) {
  unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher> flow;
  // create residual graph
  const auto edges = graph.getEdges();
  for (const auto& edge : *edges) {
    auto forwardEdgePtr = static_pointer_cast<const edge::CostCapEdge>(edge);
    if (forwardEdgePtr->getCapacity() < 0) throw std::invalid_argument("negative capacities are not allowed");
    if (forwardEdgePtr->getCapacity() > 0) {
      auto backwardEdgePtr = make_shared<const edge::CostCapEdge>(edge->getTo(), edge->getFrom(), 0);
      graph.addEdge(backwardEdgePtr);
      flow[*forwardEdgePtr] = 0;
      flow[*backwardEdgePtr] = 0;
    }
  }
  double currentFlow = 0;
  while (true) {
    auto [path, bottleneck]
      = pathFinder->getAugmentingPath(graph, flow, source, target);
    if (path.empty()) break;
    for (const auto& edge : path) {
      flow[*edge] += bottleneck;
      const auto& complementaryEdge
        = static_pointer_cast<const edge::CostCapEdge>(graph.getEdge(edge->getTo(), edge->getFrom()));
      flow[*complementaryEdge] -= bottleneck;
    }
    currentFlow += bottleneck;
  }
  return make_pair(currentFlow, flow);
}

/**
 * Calculates the maximum flow in a graph using Ford-Fulkerson with bfs for finding the augmenting paths
 * @param graph the underlying, directed graph with no double edges. it will be modified to a residual graph by this algorithm
 * @param source source of the flow network
 * @param target sink of the flow network
 * @return the maximum flow from source to sink and a map that maps a flow value to each edge
 */
static pair<double, unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher>> edmondsKarp(graph::SuperGraph& graph, const int source, const int target) {
  const helper::AugmentingPathFinder* bfsAugmenter = new helper::BfsAugmentingPathFinder();
  const auto flow = fordFulkerson(graph, bfsAugmenter, source, target);
  delete bfsAugmenter;
  return flow;
}
