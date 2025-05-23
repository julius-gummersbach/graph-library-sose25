#ifndef WEIGHTEDEDGE_H
#define WEIGHTEDEDGE_H
#include <stdexcept>
#include <string>

#include "SuperEdge.h"

namespace edge {
  class WeightedEdge : public SuperEdge {
  public:
    WeightedEdge(const int from, const int to) : SuperEdge(from, to) {}
    ~WeightedEdge() override = default;

    virtual bool operator<(const WeightedEdge& other) const {
      throw std::runtime_error(
          "operator< not implemented for this subclass of WeightedEdge: " +
          std::string(typeid(*this).name())
      );
    }
  };
}

#endif //WEIGHTEDEDGE_H
