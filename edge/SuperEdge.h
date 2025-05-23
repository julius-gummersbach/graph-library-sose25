#ifndef SUPEREDGE_H
#define SUPEREDGE_H

namespace edge {
  class SuperEdge {
  public:
    virtual ~SuperEdge() = default;

    SuperEdge(const int from, const int to) : from(from), to(to) {}
    [[nodiscard]] int getFrom() const { return from; }
    [[nodiscard]] int getTo() const { return to; }
  private:
    int from;
    int to;
  };
}
#endif //SUPEREDGE_H
