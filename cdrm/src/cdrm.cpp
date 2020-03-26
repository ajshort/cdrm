#include <cdrm/cdrm.h>

#include <cdrm/cdrm_serialisation.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <ros/console.h>

#include <fstream>

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

const Eigen::VectorXd &Cdrm::getVertexConfig(const VertexDescriptor &v) const
{
  return roadmap_[v].q_;
}

bool Cdrm::save(const std::string &filename) const
{
  try
  {
    std::ofstream ofs(filename);
    boost::archive::binary_oarchive oa(ofs);
    oa << *this;
  }
  catch (const boost::archive::archive_exception &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  return true;
}

bool Cdrm::load(const std::string &filename)
{
  try
  {
    std::ifstream ifs(filename);
    boost::archive::binary_iarchive ia(ifs);
    ia >> *this;
  }
  catch (const boost::archive::archive_exception &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  return true;
}
}
