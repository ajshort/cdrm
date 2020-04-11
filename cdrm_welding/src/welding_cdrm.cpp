#include <cdrm_welding/welding_cdrm.h>

#include <cdrm/cdrm_serialisation.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <ros/console.h>

#include <fstream>

namespace cdrm_welding
{
bool WeldingCdrm::load(const std::string &filename)
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

bool WeldingCdrm::save(const std::string &filename) const
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
}

namespace boost
{
namespace serialization
{
template <class Archive>
void serialize(Archive &ar, cdrm_welding::WeldingCdrm &cdrm, const unsigned int version)
{
  ar & cdrm.nozzle_cdrm_;
  ar & cdrm.tool_cdrm_;
  ar & cdrm.robot_cdrm_;
}
}
}
