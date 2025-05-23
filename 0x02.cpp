#include <vector>

#include "edge/WeightedEdge.h"
#include "graph/SuperGraph.h"
#include "helper/UnionFind.h"

using namespace std;

/**
 * Performs Prim algorithm
 * @graph a weighted graph
 * @return edge list of the MST
 */
vector<shared_ptr<const edge::WeightedEdge>> getMSTPrim(graph::SuperGraph &graph) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  struct CompareEdgePtrs {
    bool operator()(const shared_ptr<const edge::WeightedEdge>& a,
                    const shared_ptr<const edge::WeightedEdge>& b) const {
      return *a < *b;
    }
  };

  std::priority_queue<
      shared_ptr<const edge::WeightedEdge>,
      std::vector<shared_ptr<const edge::WeightedEdge>>,
      CompareEdgePtrs
  > frontier;

  for (const auto& edge: graph.getAdjacent(0)) {
    frontier.push(std::dynamic_pointer_cast<const edge::WeightedEdge>(edge));
  }

  vector<shared_ptr<const edge::WeightedEdge>> mst;
  vector found(graph.numNodes, false);
  found[0] = true;
  while (mst.size() < graph.numNodes - 1) {
    auto cheapestOutgoingEdge = frontier.top(); frontier.pop();
    int v = cheapestOutgoingEdge->getTo();
    if (!found[v]) {
      found[v] = true;
      mst.push_back(cheapestOutgoingEdge);
      for (const auto& adjacentEdge: graph.getAdjacent(v)) {
        if (found[adjacentEdge->getTo()]) continue;
        frontier.push(std::dynamic_pointer_cast<const edge::WeightedEdge>(adjacentEdge));
      }
    }
  }
  return mst;
}

/**
 * Performs Kruskal algorithm
 * @return edge list of the MST
 */
vector<shared_ptr<const edge::WeightedEdge>> getMSTKruskal(graph::SuperGraph &graph) {
  if (!graph.weighted) {
    throw std::invalid_argument("Graph is not weighted");
  }
  struct CompareEdgePtrs {
    bool operator()(const shared_ptr<const edge::WeightedEdge>& a,
                    const shared_ptr<const edge::WeightedEdge>& b) const {
      return *a < *b;
    }
  };

  std::priority_queue<
      shared_ptr<const edge::WeightedEdge>,
      std::vector<shared_ptr<const edge::WeightedEdge>>,
      CompareEdgePtrs
  > sortedEdges;

  auto edges = graph.getEdges();
  for (const auto& edge: edges) {
    sortedEdges.push(std::dynamic_pointer_cast<const edge::WeightedEdge>(edge));
  }

  vector<shared_ptr<const edge::WeightedEdge>> mst;
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
