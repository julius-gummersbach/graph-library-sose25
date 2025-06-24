//
// Created by Julius Gummersbach on 03.06.25.
//

#ifndef AUGMENTINGPATHFINDER_H
#define AUGMENTINGPATHFINDER_H
#include <map>
#include <vector>

#include "../graph/SuperGraph.h"
#include "../edge/CostCapEdge.h"
#include "../edge/CostCapEdgeHasher.h"

namespace helper {
  class AugmentingPathFinder {
  public:
    /**
     * @param graph The underlying graph
     * @param flow maps each edge to its current flow
     * @param source source node
     * @param sink sink node
     * @return Pair of an augmenting path and its bottleneck value.
     *         If there is no path, return an empty path and a bottleneck of 0
     */
    virtual std::pair<std::vector<std::shared_ptr<const edge::CostCapEdge> >, double> getAugmentingPath(
      graph::SuperGraph &graph,
      std::unordered_map<edge::CostCapEdge, double, edge::CostCapEdgeHasher> flow,
      int source,
      int sink
    ) const = 0;

    virtual ~AugmentingPathFinder() = default;
  };
} // helper

#endif //AUGMENTINGPATHFINDER_H
