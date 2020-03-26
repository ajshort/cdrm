#include <cdrm_legged/disjoint_sets.h>

#include <algorithm>
#include <numeric>

namespace cdrm_legged
{
DisjointSets::DisjointSets(int size) : parent_(size)
{
  std::iota(parent_.begin(), parent_.end(), 0);
}

bool DisjointSets::connected(int a, int b) const
{
  return parent_[a] == parent_[b];
}

void DisjointSets::merge(int a, int b)
{
  // Get the root parent of a.
  while (parent_[a] != a)
    a = parent_[a];

  parent_[b] = a;
}
}
