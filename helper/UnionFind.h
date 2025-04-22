#ifndef UNIONFIND_H
#define UNIONFIND_H

#include <vector>

namespace helper {

class UnionFind {
public:
  explicit UnionFind(int size);

  int getParent(int element);

  /*
   * returns true if sets were unionized, false if they were already the same set
   */
  bool unionSets(int e1, int e2);
private:
  int size;
  std::vector<int> parents;
};

} // helper

#endif //UNIONFIND_H
