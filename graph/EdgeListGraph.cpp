#include "EdgeListGraph.h"

#include "../edge/CostCapEdge.h"

namespace graph {

  EdgeListGraph::EdgeListGraph() = default;

  /*
   * currently only supports weighted graphs
   */
  void EdgeListGraph::initializeFromInput(std::istream& input) {
    weighted = true;
    input >> numNodes;

    const auto afterNumNodes = input.tellg();
    double temp;
    input >> temp;
    hasBalance = input.peek() == '\n' || input.peek() == '\r';
    input.seekg(afterNumNodes);

    if (hasBalance) {
      balance.reserve(numNodes);
      for (int i = 0; i < numNodes; i++) {
        double b;
        input >> b;
        balance.push_back(b);
      }
    }

    int u, v;
    double cost;
    double capacity;
    while (input >> u >> v >> cost) {
      if (input.peek() == '\n' || input.peek() == '\r') {
        edgeList.push_back(std::make_shared<const edge::CostCapEdge>(u, v, cost));
      } else {
        input >> capacity;
        edgeList.push_back(std::make_shared<const edge::CostCapEdge>(u, v, cost, capacity));
      }
    }
    initialized = true;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> EdgeListGraph::getEdges() {
    if (!initialized) {
      throw std::runtime_error("Graph not initialized.");
    }
    return edgeList;
  }

  std::vector<std::shared_ptr<const edge::SuperEdge>> EdgeListGraph::getAdjacent(int node) {
    throw std::runtime_error("Not implemented yet.");
  }

  std::shared_ptr<const edge::SuperEdge> EdgeListGraph::getEdge(const int u, const int v) {
    for (const auto& e : edgeList) {
      if (e->getFrom() == u && e->getTo() == v) {
        return e;
      }
    }
    throw std::runtime_error("Edge not found.");
  }

  void EdgeListGraph::addEdge(const std::shared_ptr<const edge::SuperEdge> edge) {
    edgeList.push_back(edge);
  }

  void EdgeListGraph::removeEdge(const std::shared_ptr<const edge::SuperEdge>& edge) {
    edgeList.erase(std::ranges::remove(edgeList, edge).begin());
  }

  double EdgeListGraph::getBalance(const int node) {
    if (!initialized) {
      throw std::runtime_error("Graph is not initialized.");
    }
    if (!hasBalance) {
      throw std::runtime_error("Graph has no balance.");
    }
    return balance[node];
  }

}
