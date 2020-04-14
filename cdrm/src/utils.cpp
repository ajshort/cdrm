#include <cdrm/utils.h>

#include <ros/platform.h>

namespace cdrm
{
Eigen::Isometry3d interpolateTfs(const Eigen::Isometry3d &from, const Eigen::Isometry3d &to, double t)
{
  Eigen::Quaterniond from_quat(from.rotation());
  Eigen::Quaterniond to_quat(to.rotation());

  Eigen::Isometry3d result;
  result.linear() = from_quat.slerp(t, to_quat).toRotationMatrix();
  result.translation() = from.translation() + (to.translation() - from.translation()) * t;
  return result;
}

std::string getRosHome()
{
  std::string home;

  if (ros::get_environment_variable(home, "ROS_HOME"))
    return home;

  ros::get_environment_variable(home, "HOME");
  return home + "/.ros/";
}
}
