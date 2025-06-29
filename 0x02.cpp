#include <vector>

#include "edge/CostCapEdge.h"
#include "graph/SuperGraph.h"
#include "helper/UnionFind.h"

/**
 * Performs Prim algorithm
 * @graph a weighted graph
 * @return edge list of the MST
 */
vector<shared_ptr<const edge::CostCapEdge>> getMSTPrim(graph::SuperGraph &graph) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  struct CompareEdgePtrs {
    bool operator()(const shared_ptr<const edge::CostCapEdge>& a,
                    const shared_ptr<const edge::CostCapEdge>& b) const {
      return !(*a < *b);
    }
  };

  std::priority_queue<
      shared_ptr<const edge::CostCapEdge>,
      std::vector<shared_ptr<const edge::CostCapEdge>>,
      CompareEdgePtrs
  > frontier;

  for (const auto& edge: graph.getAdjacent(0)) {
    frontier.push(std::dynamic_pointer_cast<const edge::CostCapEdge>(edge));
  }

  vector<shared_ptr<const edge::CostCapEdge>> mst;
  mst.reserve(graph.numNodes - 1);
  vector found(graph.numNodes, false);
  found[0] = true;
  while (mst.size() < graph.numNodes - 1) {
    auto cheapestOutgoingEdge = frontier.top(); frontier.pop();
    int outNode = !graph.directed && found[cheapestOutgoingEdge->getTo()] ? cheapestOutgoingEdge->getFrom() : cheapestOutgoingEdge->getTo();
    if (!found[outNode]) {
      found[outNode] = true;
      mst.push_back(cheapestOutgoingEdge);
      for (const auto& adjacentEdge: graph.getAdjacent(outNode)) {
        frontier.push(std::dynamic_pointer_cast<const edge::CostCapEdge>(adjacentEdge));
      }
    }
  }
  return mst;
}

/**
 * Performs Kruskal algorithm
 * @return edge list of the MST
 */
vector<shared_ptr<const edge::CostCapEdge>> getMSTKruskal(graph::SuperGraph &graph) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  struct CompareEdgePtrs {
    bool operator()(const shared_ptr<const edge::CostCapEdge>& a,
                    const shared_ptr<const edge::CostCapEdge>& b) const {
      return !(*a < *b);
    }
  };

  std::priority_queue<
      shared_ptr<const edge::CostCapEdge>,
      std::vector<shared_ptr<const edge::CostCapEdge>>,
      CompareEdgePtrs
  > sortedEdges;

  const auto edges = graph.getEdges();
  for (const auto& edge: *edges) {
    sortedEdges.push(std::dynamic_pointer_cast<const edge::CostCapEdge>(edge));
  }

  vector<shared_ptr<const edge::CostCapEdge>> mst;
  mst.reserve(graph.numNodes - 1);
  helper::UnionFind uf(graph.numNodes);
  while (mst.size() < graph.numNodes - 1) {
    if (sortedEdges.empty()) {
      throw std::invalid_argument("Graph is not connected");
    }
    auto edge = sortedEdges.top(); sortedEdges.pop();
    if (uf.unionSets(edge->getFrom(), edge->getTo())) {
      mst.push_back(edge);
    }
  }
  return mst;
}
