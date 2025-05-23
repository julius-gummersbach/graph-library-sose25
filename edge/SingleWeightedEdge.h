#ifndef SINGLEWEIGHTEDEDGE_H
#define SINGLEWEIGHTEDEDGE_H
#include "WeightedEdge.h"


namespace edge {
  class SingleWeightedEdge : public WeightedEdge {
  public:
    SingleWeightedEdge(const int from, const int to, const double weight) : WeightedEdge(from, to), weight(weight) {}

    [[nodiscard]] double getWeight() const { return weight; }

    bool operator<(const SingleWeightedEdge& other) const {
      return this->weight < other.weight;
    }

    bool operator<(const WeightedEdge& other) const override {
      const auto otherEdge = dynamic_cast<const SingleWeightedEdge*>(&other);
      if (!otherEdge) {
        return WeightedEdge::operator<(other);  // Delegate to base: will throw
      }
      return *this < *otherEdge;
    }
  private:
    double weight;
  };
}


#endif //SINGLEWEIGHTEDEDGE_H
