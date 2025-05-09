#include <vector>

#include "graph/SuperGraph.h"
#include "helper/UnionFind.h"

using namespace std;
using edge_t = graph::SuperGraph::edge_t;

/**
 * Performs Prim algorithm
 * @return edge list of the MST
 */
vector<edge_t> getMSTPrim(graph::SuperGraph &graph) {
  auto edgeComparator = [](const edge_t& lhs, const edge_t& rhs)
  {
    return get<2>(lhs) > get<2>(rhs);
  };

  priority_queue<edge_t, vector<edge_t>, decltype(edgeComparator)> pq(edgeComparator);
  for (const auto node: graph.getAdjacentNodes(0)) {
    pq.emplace(0, node, graph.getWeight(0, node));
  }

  vector<edge_t> mst;
  vector found(graph.numNodes, false);
  found[0] = true;
  while (mst.size() < graph.numNodes - 1) {
    auto edge = pq.top(); pq.pop();
    int v = get<1>(edge);
    if (!found[v]) {
      found[v] = true;
      mst.push_back(edge);
      for (const auto node: graph.getAdjacentNodes(v)) {
        if (found[node]) continue;
        pq.emplace(v, node, graph.getWeight(v, node));
      }
    }
  }
  return mst;
}

/**
 * Performs Kruskal algorithm
 * @return edge list of the MST
 */
vector<edge_t> getMSTKruskal(graph::SuperGraph &graph) {
  auto cmp = [](const edge_t& lhs, const edge_t& rhs)
  {
    return get<2>(lhs) > get<2>(rhs);
  };

  priority_queue<edge_t, vector<edge_t>, decltype(cmp)> pq(cmp);
  auto edges = graph.getEdges();
  for (const auto edge: edges) {
    pq.emplace(edge);
  }

  vector<edge_t> mst;
  helper::UnionFind uf(graph.numNodes);
  while (mst.size() < graph.numNodes - 1) {
    if (pq.empty()) {
      throw std::invalid_argument("Graph is not connected");
    }
    auto edge = pq.top(); pq.pop();
    if (uf.unionSets(get<0>(edge), get<1>(edge))) {
      mst.push_back(edge);
    }
  }
  return mst;
}
