#include <cdrm/cdrm.h>

namespace cdrm
{
Cdrm::Cdrm(double resolution) : resolution_(resolution)
{
}

bool Cdrm::save(const std::string &filename) const
{
  return false;
}

bool Cdrm::load(const std::string &filename)
{
  return false;
}
}
