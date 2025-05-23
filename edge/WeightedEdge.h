#ifndef WEIGHTEDEDGE_H
#define WEIGHTEDEDGE_H
#include <stdexcept>
#include <string>

#include "SuperEdge.h"

namespace edge {
  class WeightedEdge : public SuperEdge {
  public:
    WeightedEdge(const int from, const int to, const double weight) : SuperEdge(from, to), weight(weight) {}
    ~WeightedEdge() override = default;

    [[nodiscard]] double getWeight() const { return weight; }

    virtual bool operator<(const WeightedEdge& other) const {
      return this->weight < other.weight;
    }
  private:
    double weight;
  };
}

#endif //WEIGHTEDEDGE_H
