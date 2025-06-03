#include "BfsAugmentingPathFinder.h"

namespace helper {
  using namespace std;
  pair<vector<shared_ptr<const edge::WeightedEdge>>, double> BfsAugmentingPathFinder::getAugmentingPath(
      graph::SuperGraph &graph,
      std::map<std::shared_ptr<const edge::WeightedEdge>, double> flow,
      const int source,
      const int sink
  ) const {
    double bottleneck = INFINITY;
    queue<vector<shared_ptr<const edge::WeightedEdge>>> frontier;
    auto visited = vector(graph.numNodes, false);
    frontier.push({make_shared<const edge::WeightedEdge>(source, source, INFINITY)});  // mock-edge to avoid code duplication
    while (!frontier.empty()) {
      auto& currentPath = frontier.front();
      const auto currentNode = currentPath.back()->getTo();
      if (visited[currentNode]) {
        frontier.pop();
        continue;
      }
      visited[currentNode] = true;
      for (auto& adjacent : graph.getAdjacent(currentNode)) {
        const auto& adjacentWeighted = static_pointer_cast<const edge::WeightedEdge>(adjacent);
        const auto capacity = adjacentWeighted->getWeight() - flow[adjacentWeighted];
        if (capacity > 0) {
          bottleneck = min(bottleneck, capacity);
          auto copyPath = currentPath;
          copyPath.reserve(graph.numNodes);
          copyPath.push_back(adjacentWeighted);
          if (adjacent->getTo() == sink) {
            copyPath.erase(copyPath.begin());  // delete mock-edge
            return make_pair(copyPath, bottleneck);
          }
          frontier.push(copyPath);
        }
      }
      frontier.pop();
    }
    return make_pair<vector<shared_ptr<const edge::WeightedEdge>>, double>({}, 0.);
  }
} // helper