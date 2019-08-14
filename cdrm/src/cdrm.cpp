#include <cdrm/cdrm.h>

namespace cdrm
{
Cdrm::Cdrm(double resolution) : resolution_(resolution)
{
}

Key Cdrm::pointToKey(const Eigen::Vector3d &p) const
{
  Key k;
  k[0] = static_cast<std::int16_t>(std::round(p(0) / resolution_));
  k[1] = static_cast<std::int16_t>(std::round(p(1) / resolution_));
  k[2] = static_cast<std::int16_t>(std::round(p(2) / resolution_));
  return k;
}

Eigen::Vector3d Cdrm::keyToPoint(const Key &k) const
{
  Eigen::Vector3d p;
  p(0) = resolution_ * k[0];
  p(1) = resolution_ * k[1];
  p(2) = resolution_ * k[2];
  return p;
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
