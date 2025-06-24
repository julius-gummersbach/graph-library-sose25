#include "BfsAugmentingPathFinder.h"

#include <iostream>

namespace helper {
  using namespace std;
  pair<vector<shared_ptr<const edge::CostCapEdge>>, double> BfsAugmentingPathFinder::getAugmentingPath(
      graph::SuperGraph &graph,
      std::map<std::shared_ptr<const edge::CostCapEdge>, double> flow,
      const int source,
      const int sink
  ) const {
    auto predecessor = vector(graph.numNodes, -1);
    queue<int> frontier;
    auto visited = vector(graph.numNodes, false);
    frontier.push(source);
    bool sourceFound = false;
    while (!sourceFound && !frontier.empty()) {
      auto currentNode = frontier.front();
      for (auto& adjacent : graph.getAdjacent(currentNode)) {
        const auto& adjacentWeighted = static_pointer_cast<const edge::CostCapEdge>(adjacent);
        const auto capacity = adjacentWeighted->getCapacity() - flow[adjacentWeighted];
        if (capacity > 0 && !visited[adjacent->getTo()]) {
          frontier.push(adjacent->getTo());
          visited[adjacent->getTo()] = true;
          predecessor[adjacent->getTo()] = currentNode;
          if (adjacent->getTo() == sink) {
            sourceFound = true;
            break;
          }
        }
      }
      frontier.pop();
    }
    if (predecessor[sink] == -1) return make_pair<vector<shared_ptr<const edge::CostCapEdge>>, double>({}, 0.);
    vector<shared_ptr<const edge::CostCapEdge>> path{};  // hier könnte man noch ein reserve machen...
    int current = sink;
    double bottleneck = INFINITY;
    while (current != source) {
      const auto& edge = static_pointer_cast<const edge::CostCapEdge>(graph.getEdge(predecessor[current], current));
      path.insert(path.begin(), edge);
      bottleneck = min(bottleneck, edge->getCapacity() - flow[edge]);
      current = predecessor[current];
    }
    return make_pair<vector<shared_ptr<const edge::CostCapEdge>>, double>(std::move(path), std::move(bottleneck));
  }
} // helper