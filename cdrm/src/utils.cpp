#include <cdrm/utils.h>

#include <ros/platform.h>

namespace cdrm
{
std::string getRosHome()
{
  std::string home;

  if (ros::get_environment_variable(home, "ROS_HOME"))
    return home;

  ros::get_environment_variable(home, "HOME");
  return home + "/.ros/";
}
}
