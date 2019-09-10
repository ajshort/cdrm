#pragma once

#include <cdrm_welding_msgs/PlanWeld.h>
#include <moveit/macros/class_forward.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}
}

namespace cdrm_welding
{
/**
 * Plans a single weld.
 */
class WeldPlanner
{
public:
  explicit WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model);

  bool plan(cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res);

private:
  moveit::core::RobotModelConstPtr robot_model_;
};
}
