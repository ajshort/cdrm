#include <cdrm_welding/weld_planner.h>

#include <ros/time.h>

namespace cdrm_welding
{
WeldPlanner::WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model) : robot_model_(robot_model)
{
}

bool WeldPlanner::plan(cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res)
{
  ros::Time start_time = ros::Time::now();
  ros::Time limit_time = start_time + ros::Duration(req.planning_timeout);

  // Get the CDRM for the toolpath and filter it.
  // Get the CDRM for the manipulator and filter the toolpath CDRM edges to reachable ones only.
  // See if we can create a path from near the start to near the end.

  return false;
}
}
