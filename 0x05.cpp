#include <map>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"
#include "helper/AugmentingPathFinder.h"
#include "helper/BfsAugmentingPathFinder.h"

using namespace std;

// todo Note mündliches Argumentieren besprechen
// todo Praktikumserklärung hatte keinen Sound, war stumm

/**
 * Calculates the maximum flow in a graph
 * @param graph the underlying, directed graph with no double edges. it will be modified to a residual graph by this algorithm
 * @param pathFinder a strategy to find augmenting paths
 * @param source source of the flow network
 * @param sink sink of the flow network
 * @return the maximum flow from source to sink
 */
static double fordFulkerson(graph::SuperGraph& graph, const helper::AugmentingPathFinder* pathFinder, const int source, const int sink) {
  const auto edgeList = graph.getEdges();
  map<shared_ptr<const edge::WeightedEdge>, double> flow;
  for (const auto& edge : edgeList) {
    auto forwardEdgePtr = static_pointer_cast<const edge::WeightedEdge>(edge);
    if (forwardEdgePtr->getWeight() < 0) throw std::invalid_argument("negative capacities are not allowed");
    if (forwardEdgePtr->getWeight() > 0) {
      auto backwardEdgePtr = make_shared<const edge::WeightedEdge>(edge->getTo(), edge->getFrom(), 0);
      graph.addEdge(backwardEdgePtr);
      flow[forwardEdgePtr] = 0;
      flow[backwardEdgePtr] = 0;
    }
  }
  double currentFlow = 0;
  while (true) {
    auto [path, bottleneck]
      = pathFinder->getAugmentingPath(graph, flow, source, sink);
    if (bottleneck == 0) break;
    for (const auto& edge : path) {
      flow[edge] += bottleneck;
      const auto& complementaryEdge
        = static_pointer_cast<const edge::WeightedEdge>(graph.getEdge(edge->getTo(), edge->getFrom()));
      flow[complementaryEdge] -= bottleneck;
    }
    currentFlow += bottleneck;
  }
  return currentFlow;
}

/**
 * Calculates the maximum flow in a graph using Ford-Fulkerson with bfs for finding the augmenting paths
 * @param graph the underlying, directed graph with no double edges. it will be modified to a residual graph by this algorithm
 * @param source source of the flow network
 * @param sink sink of the flow network
 * @return the maximum flow from source to sink
 */
static double edmondsKarp(graph::SuperGraph& graph, const int source, const int sink) {
  const helper::AugmentingPathFinder* bfsAugmenter = new helper::BfsAugmentingPathFinder();
  const auto flow = fordFulkerson(graph, bfsAugmenter, source, sink);
  delete bfsAugmenter;
  return flow;
}
