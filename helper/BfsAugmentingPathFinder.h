//
// Created by Julius Gummersbach on 03.06.25.
//

#ifndef BFSAUGMENTINGPATHFINDER_H
#define BFSAUGMENTINGPATHFINDER_H
#include "AugmentingPathFinder.h"

namespace helper {
  class BfsAugmentingPathFinder : public AugmentingPathFinder {
  public:
    std::pair<std::vector<std::shared_ptr<const edge::CostCapEdge> >, double> getAugmentingPath(
      graph::SuperGraph &graph,
      std::map<std::shared_ptr<const edge::CostCapEdge>, double> flow,
      int source,
      int sink) const override;
  };
} // helper

#endif //BFSAUGMENTINGPATHFINDER_H
