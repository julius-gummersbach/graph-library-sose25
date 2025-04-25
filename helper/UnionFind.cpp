#include "UnionFind.h"

namespace helper {

  UnionFind::UnionFind(const int size) : size(size), parents(size) {
    for (int i = 0; i < size; i++) parents[i] = i;
  }

  int UnionFind::getParent(int element) {
    if (parents[element] != element) {
      parents[element] = getParent(parents[element]);
    }
    return parents[element];
  }

  bool UnionFind::unionSets(const int e1, const int e2) {
    const int p1 = getParent(e1);
    const int p2 = getParent(e2);
    if (p1 == p2) return false;
    parents[p1] = p2;
    return true;
  }
}