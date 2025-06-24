#include "AdjacentListGraph.h"

#include <iostream>
#include <sstream>

#include "../edge/CostCapEdge.h"

namespace graph {
  using namespace std;

  AdjacentListGraph::AdjacentListGraph() = default;

  AdjacentListGraph::AdjacentListGraph(const vector<shared_ptr<const edge::CostCapEdge>>& mstEdgeList) {
    numNodes = mstEdgeList.size() + 1;
    adjacencyList.resize(numNodes);
    for (const auto& edge : mstEdgeList) {
      adjacencyList[edge->getFrom()].push_back(edge);
      if (!directed) {
        adjacencyList[edge->getTo()].push_back(edge);
      }
    }
    initialized = true;
  }

  AdjacentListGraph::AdjacentListGraph(const vector<shared_ptr<const edge::SuperEdge>>& edgeList, const bool isDirected) {
    adjacencyList.resize(edgeList.size());
    weighted = dynamic_pointer_cast<const edge::CostCapEdge>(edgeList[0]) != nullptr;
    directed = isDirected;
    for (const auto& edge : edgeList) {
      numNodes = max(numNodes, max(edge->getFrom(), edge->getTo()) + 1);
      adjacencyList[edge->getFrom()].push_back(edge);
      if (!directed) {
        adjacencyList[edge->getTo()].push_back(edge);
      }
    }
    initialized = true;
  }

  void AdjacentListGraph::initializeFromInput(istream &input) {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string line;
    std::getline(input, line);
    std::istringstream ls(line);
    ls >> numNodes;

    adjacencyList.resize(numNodes);

    std::getline(input, line);
    ls = std::istringstream(line);
    int u, v;
    double w;
    ls >> u >> v;
    if (ls >> w) {
      weighted = true;
    }

    do {
      if (line.empty()) continue;
      ls = std::istringstream(line);
      ls >> u >> v;
      if (weighted) {
        ls >> w;
        adjacencyList[u].push_back(std::make_shared<const edge::CostCapEdge>(u,v, w));
        if (!directed) {
          adjacencyList[v].push_back(std::make_shared<const edge::CostCapEdge>(v,u, w));
        }
      } else {
        adjacencyList[u].push_back(std::make_shared<const edge::SuperEdge>(u,v));
        if (!directed) {
          adjacencyList[v].push_back(std::make_shared<const edge::SuperEdge>(v,u));
        }
      }
    } while (std::getline(input, line));
    initialized = true;
  }


  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentListGraph::getAdjacent(const int node) {
    return adjacencyList[node];
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> AdjacentListGraph::getEdges() {
    throw runtime_error("Not implemented yet.");
    /* implemented, but not tested yet
    std::vector<std::array<int, 2>> edges;
    for (int i = 0; i < numNodes; i++) {
      for (int j = 0; j < adjacencyList[i].size(); j++) {
        if (adjacencyList[i][j] >= i) edges.push_back(std::array{i, adjacencyList[i][j]});
      }
    }
    return edges;*/
  }

  std::shared_ptr<const edge::SuperEdge> AdjacentListGraph::getEdge(const int u, const int v) {
    for (const auto& edge : adjacencyList[u]) {
      if (edge->getTo() == v) return edge;
    }
    if (!directed) {
      for (const auto& edge : adjacencyList[v]) {
        if (edge->getTo() == u) return edge;
      }
    }
    return nullptr;
  }

  void AdjacentListGraph::addEdge(std::shared_ptr<const edge::SuperEdge> edge) {
    throw std::runtime_error("Not implemented yet.");
  }

  double AdjacentListGraph::getBalance(int node) {
    throw std::runtime_error("Not implemented yet.");
  }
}
