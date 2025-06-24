//
// Created by Julius Gummersbach on 24.06.25.
//

#ifndef EDGEKEY_H
#define EDGEKEY_H
#include <functional>

#include "CostCapEdge.h"


namespace edge {
  struct CostCapEdgeHasher {
    size_t operator()(const CostCapEdge& e) const
    {
      return std::hash<int>()(e.getFrom()) ^ std::hash<int>()(e.getTo()) << 1;
    }
  };
}

#endif //EDGEKEY_H
