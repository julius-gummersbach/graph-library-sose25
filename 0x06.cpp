#include <utility>

#include "edge/CostCapEdge.h"
#include "edge/CostCapEdgeHasher.h"
#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"


/**
 * Calculates the minimum cost of a flow that satisfies the balance values in the given graph
 */
static double cycleCancelling(graph::SuperGraph& graph) {
  double balanceSum = 0.0;
  bool existsSource = false;
  for (int i = 0; i < graph.numNodes; i++) {
    balanceSum += graph.getBalance(i);
    existsSource = existsSource || graph.getBalance(i) > 0;
  }
  if (abs(balanceSum) > DBL_EPSILON) {
    throw invalid_argument("Balance values inconsistent");
  }
  if (!existsSource) {
    throw invalid_argument("All balance values are 0, no b-flow possible");
    // or return (0;0)?
  }

  double costResult = 0;
  const auto edgeList = graph.getEdges();

  // add a pseudo-source and -target to graph according to balance values
  auto flowEdgeList = *edgeList;
  int pseudoSource = graph.numNodes;
  int pseudoTarget = graph.numNodes + 1;
  for (int node = 0; node < graph.numNodes; node++) {
    auto balance = graph.getBalance(node);
    if (balance > 0) {
      flowEdgeList.push_back(make_shared<edge::CostCapEdge>(pseudoSource, node, 0, balance));
    }
    else if (balance < 0) {
      flowEdgeList.push_back(make_shared<edge::CostCapEdge>(node, pseudoTarget, 0, -balance));
    }
  }
  auto flowGraph = graph::AdjacentMatrixGraph(flowEdgeList, true);

  // calculate flow
  const auto maxFlowResult = edmondsKarp(flowGraph, pseudoSource, pseudoTarget);

  unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher> maxFlowDistribution;
  for (auto const&[edge, flow] : get<1>(maxFlowResult)) {
    maxFlowDistribution[edge] = flow;
  }

  // check flow and update cost
  for (const auto& superEdge : flowEdgeList) {
    auto edge = static_pointer_cast<const edge::CostCapEdge>(superEdge);
    costResult += edge->getCost() * maxFlowDistribution[*edge];
    if (edge->getFrom() == pseudoSource && abs(maxFlowDistribution[*edge] - edge->getCapacity()) > DBL_EPSILON) {
      // one of the pseudo-edges is not fully used
      throw invalid_argument("No b-flow possible");
    }
  }

  // find and cancel cycles
  while (true) {
    // create cycle-checking graph
    vector<shared_ptr<const edge::CostCapEdge>> cycleCheckingEdgeList;
    for (const auto& superEdge : *edgeList) {
      auto edge = static_pointer_cast<const edge::CostCapEdge>(superEdge);
      if (edge->getCapacity() - maxFlowDistribution[*edge] > 0) {  // Vorwärts-Kanten, die noch Kapazität haben
        cycleCheckingEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          edge->getFrom(),
          edge->getTo(),
          edge->getCost(),
          edge->getCapacity() - maxFlowDistribution[*edge]));
      }
      if (maxFlowDistribution[*edge] > 0) {  // Rückwärts-Kanten für die Vorwärts-Kanten, auf denen Fluss fließt
        cycleCheckingEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          edge->getTo(),
          edge->getFrom(),
          -edge->getCost(),
          maxFlowDistribution[*edge]));
      }
    }
    // add pseudo-start node
    auto startNode = graph.numNodes;
    for (int i =0; i < graph.numNodes; i++) {
      cycleCheckingEdgeList.push_back(std::make_shared<edge::CostCapEdge>(startNode, i, 0, 0));
    }
    auto cycleCheckingGraph = graph::EdgeListGraph(cycleCheckingEdgeList, graph.directed);

    // look for negative cycles, if none is found, terminate
    auto mooreBellmanFordResult = mooreBellmanFord(cycleCheckingGraph, startNode);
    if (!mooreBellmanFordResult.contains(-1)) {
      break;
    }
    auto negativeCycle = mooreBellmanFordResult[-1];

    // calculate bottleneck
    double bottleneck = INFINITY;
    for (const auto& edge : negativeCycle) {
      bottleneck = min(bottleneck, edge->getCapacity());
    }
    // update flow and cost
    for (const auto& edge : negativeCycle) {
      if (maxFlowDistribution.contains(*edge)) {
        maxFlowDistribution[*edge] += bottleneck;
        costResult += bottleneck * edge->getCost();
      } else {  // edge is backward edge
        auto forwardEdge = static_pointer_cast<const edge::CostCapEdge>(graph.getEdge(edge->getTo(), edge->getFrom()));
        maxFlowDistribution[*forwardEdge] -= bottleneck;
        costResult -= bottleneck * forwardEdge->getCost();
      }
     // costResult += bottleneck * edge->getCost();  // could also do this. if edge is backward edge, its cost will be inverted
    }
  }
  return costResult;
}


/**
 * Calculates the minimum cost of a flow that satisfies the balance values in the given graph
 */
static double successiveShortestPath(graph::SuperGraph& graph) {
  double balanceSum = 0.0;
  bool existsSource = false;
  for (int i = 0; i < graph.numNodes; i++) {
    balanceSum += graph.getBalance(i);
    existsSource = existsSource || graph.getBalance(i) > 0;
  }
  if (abs(balanceSum) > DBL_EPSILON) {
    throw invalid_argument("Balance values inconsistent");
  }
  if (!existsSource) {
    throw invalid_argument("All balance values are 0, no b-flow possible");
  }

  // Initialize Flow and balance values
  const auto edgeList = graph.getEdges();
  unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher> flow;
  vector currentBalance(graph.numNodes, 0.0);
  for (const auto& superEdge : *edgeList) {
    const auto edge = static_pointer_cast<const edge::CostCapEdge>(superEdge);
    if (edge->getCost() >= 0) flow[*edge] = 0.0;
    else flow[*edge] = edge->getCapacity();
    currentBalance[edge->getFrom()] += flow[*edge];  // balance ist im .txt Input andersrum als im Vortrag. Im .txt. Input ist es der Fluss-Output eines Knotens, im Vortrag wars das Fluss-Input
    currentBalance[edge->getTo()] -= flow[*edge];
  }

  while (true) {
    // Find a supply node
    int supplyNode = -1;
    for (int i = 0; i < graph.numNodes; i++) {
      if (graph.getBalance(i) > currentBalance[i]) {
        supplyNode = i;
        break;
      }
    }
    if (supplyNode == -1) break; // done

    // create shortestPath graph
    vector<shared_ptr<const edge::CostCapEdge>> shortestPathGraphEdgeList;
    for (const auto& superEdge : *edgeList) {
      auto edge = static_pointer_cast<const edge::CostCapEdge>(superEdge);
      if (edge->getCapacity() - flow[*edge] > 0) {  // Vorwärts-Kanten, die noch Kapazität haben
        shortestPathGraphEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          edge->getFrom(),
          edge->getTo(),
          edge->getCost(),
          edge->getCapacity() - flow[*edge]));
      }
      if (flow[*edge] > 0) {  // Rückwärts-Kanten für die Vorwärts-Kanten, auf denen Fluss fließt
        shortestPathGraphEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          edge->getTo(),
          edge->getFrom(),
          -edge->getCost(),
          flow[*edge]));
      }
    }
    auto shortestPathGraph = graph::EdgeListGraph(shortestPathGraphEdgeList, graph.directed);
    auto shortestPathResult = mooreBellmanFord(shortestPathGraph, supplyNode);

    bool targetFound = false;
    vector<shared_ptr<const edge::CostCapEdge>> augmentingPath;
    for (int i = 0; i < graph.numNodes; i++) {
      if (i != supplyNode && shortestPathResult.contains(i) && graph.getBalance(i) < currentBalance[i]) {
        targetFound = true;
        augmentingPath = shortestPathResult[i];
        break;
      }
    }
    if (!targetFound) {
      throw invalid_argument("No b-flow possible");
    }
    auto targetNode = augmentingPath.back()->getTo();

    // Determine bottleneck (min residual capacity, supply, and demand)
    double bottleneck = min(
      abs(graph.getBalance(supplyNode) - currentBalance[supplyNode]),
      abs(graph.getBalance(targetNode)-currentBalance[targetNode])
      );
    for (const auto& edge : augmentingPath) {
      bottleneck = min(bottleneck, edge->getCapacity());
    }

    // Apply flow and update balances
    for (const auto& edge : augmentingPath) {
      if (flow.contains(*edge)) {
        flow[*edge] += bottleneck;
      } else {
        auto forwardEdge = static_pointer_cast<const edge::CostCapEdge>(graph.getEdge(edge->getTo(), edge->getFrom()));
        flow[*forwardEdge] -= bottleneck;
      }
    }
    currentBalance[supplyNode] += bottleneck;
    currentBalance[targetNode] -= bottleneck;
  }
  // Compute total cost
  double costValue = 0.0;
  for (const auto& superEdge : *edgeList) {
    auto edge = static_pointer_cast<const edge::CostCapEdge>(superEdge);
    costValue += flow[*edge] * edge->getCost();
  }
  for (int i = 0; i < graph.numNodes; i++) {
    if (graph.getBalance(i) != currentBalance[i]) {
      throw invalid_argument("No b-flow possible");
    }
  }
  return costValue;
}
