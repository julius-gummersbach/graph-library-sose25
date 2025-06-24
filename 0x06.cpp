#include <utility>

#include "edge/CostCapEdge.h"
#include "edge/CostCapEdgeHasher.h"
#include "graph/SuperGraph.h"
#include "graph/EdgeListGraph.h"

/**
 * Cycle-Cancelling:
 *
 * Idee:
 * Konstruktor nutzt Balance-Werte um super-quelle und super-senke zu erstellen, verwirft sie danach
 * Ist aber nicht gut, da wir für SSP später die Balancen noch brauchen.
 * Also stattdessen: optionales array im graph, das die Balancen speichert, algorithmus erstellt super-quellen und -senken
 *
 * Fluss wird Algorithmus-Intern gespeichert
 * Refaktor Weighted Edge: Cost-Cap-Edge
 * Wenn alter graph mit nur einem Wert: Cost und Cap beide dieser Wert
 * bei neuen Graphen: setze die jeweiligen Werte.
 *
 * Algorithmus guckt einmal am Anfang:
 * Alle Cap aller von s ausgehenden Kanten = Cap aller zu t eingehenden Kanten? Wenn ja, existiert B-Fluss
 * Dann nach max-flow algo: sind alle von s ausgehenden Kanten ausgelastet?
 *
 * Dann auf Residualgraphen nach negativen zyklen suchen.
 * Problem: nutze Cap-flow als metrik... aaargh
 * kriege negativen Zykel zurück (erfordert auch refactoring...)
 * bestimme kleinste reale Kapazität entlang des Zykels und augmentiere darum
 *
 * Successive-Shortest-Path:
 * Wenn Wege von Pseudo-Quellen zu Pseudo-Senken gesucht werden, kann abgebrochen werden, sobald von einer Quelle kein Weg gefunden wird.
 * Denn dann kann diese Balance nie ausgeglichen werden, wie in der VL besprochen
 *
 */

static pair<double, double> cycleCancelling(graph::SuperGraph& graph) {
  double balanceSum = 0.0;
  bool existsSource = false;
  for (int i = 0; i < graph.numNodes; i++) {
    balanceSum += graph.getBalance(i);
    existsSource = existsSource || graph.getBalance(i) > 0;
  }
  if (abs(balanceSum) > DBL_EPSILON) {
    throw invalid_argument("Balance values inconsistent, no b-flow possible");
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
  double flowResult = get<0>(maxFlowResult);

  unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher> maxFlowDistribution;
  for (auto const&[edge, flow] : get<1>(maxFlowResult)) {
    maxFlowDistribution[edge] = flow;
  }

  // check flow and update cost
  for (const auto& edge : flowEdgeList) {
    auto costCapEdge = static_pointer_cast<const edge::CostCapEdge>(edge);
    costResult += costCapEdge->getCost() * maxFlowDistribution[*costCapEdge];
    if (costCapEdge->getFrom() == pseudoSource && abs(maxFlowDistribution[*costCapEdge] - costCapEdge->getCapacity()) > DBL_EPSILON) {
      // one of the pseudo-edges is not fully used
      throw invalid_argument("No b-flow possible");
    }
  }

  // find and cancel cycles
  while (true) {
    // create cycle-checking graph
    vector<shared_ptr<const edge::CostCapEdge>> cycleCheckingEdgeList;
    for (const auto& edge : *edgeList) {
      auto costCapEdge = static_pointer_cast<const edge::CostCapEdge>(edge);
      if (costCapEdge->getCapacity() - maxFlowDistribution[*costCapEdge] > 0) {  // Vorwärts-Kanten, die noch Kapazität haben
        cycleCheckingEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          costCapEdge->getFrom(),
          costCapEdge->getTo(),
          costCapEdge->getCost(),
          costCapEdge->getCapacity() - maxFlowDistribution[*costCapEdge]));
      }
      if (maxFlowDistribution[*costCapEdge] > 0) {  // Rückwärts-Kanten für die Vorwärts-Kanten, auf denen Fluss fließt
        cycleCheckingEdgeList.push_back(std::make_shared<edge::CostCapEdge>(
          costCapEdge->getTo(),
          costCapEdge->getFrom(),
          -costCapEdge->getCost(),
          maxFlowDistribution[*costCapEdge]));
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
    if (!mooreBellmanFordResult.contains(-1)) break;
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
        auto realEdge = static_pointer_cast<const edge::CostCapEdge>(cycleCheckingGraph.getEdge(edge->getFrom(), edge->getTo()));
        maxFlowDistribution[*realEdge] -= bottleneck;
        costResult -= bottleneck * realEdge->getCost();
      }
    }
  }
  return make_pair(flowResult, costResult);
}
