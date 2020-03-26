#pragma once

#include <vector>

namespace cdrm_legged
{
/**
 * Models a disjoint set of integers data structure.
 */
class DisjointSets
{
public:
  DisjointSets(int size);
  void merge(int a, int b);
  int parent(int i) const { return parent_[i]; }
  bool connected(int a, int b) const;

private:
  std::vector<int> parent_;
};
}
