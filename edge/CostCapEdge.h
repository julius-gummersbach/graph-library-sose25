#ifndef WEIGHTEDEDGE_H
#define WEIGHTEDEDGE_H
#include <stdexcept>
#include <string>

#include "SuperEdge.h"

namespace edge {
  class CostCapEdge : public SuperEdge {
  public:
    CostCapEdge(const int from, const int to, const double costAndCap) : SuperEdge(from, to), cost(costAndCap), capacity(costAndCap) {}
    CostCapEdge(const int from, const int to, const double cost, const double capacity) : SuperEdge(from, to), cost(cost), capacity(capacity) {}
    ~CostCapEdge() override = default;

    [[nodiscard]] double getCost() const { return cost; }
    [[nodiscard]] double getCapacity() const { return capacity; }

    virtual bool operator<(const CostCapEdge& other) const {
      return this->cost < other.cost;
    }
  private:
    double cost;
    double capacity;
  };
}

#endif //WEIGHTEDEDGE_H
