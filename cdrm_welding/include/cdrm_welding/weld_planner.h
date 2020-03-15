#pragma once

#include <cdrm_welding_msgs/PlanWeld.h>
#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(LinkModel);
MOVEIT_CLASS_FORWARD(RobotModel);
}
}

namespace cdrm_welding
{
class Weld;

/**
 * Plans a single weld.
 */
class WeldPlanner
{
public:
  WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model,
              const ros::Publisher &target_publisher);

  bool plan(cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res);

private:
  void publishTargets(const Weld &weld);

  moveit::core::RobotModelConstPtr robot_model_;
  ros::Publisher target_publisher_;

  const moveit::core::LinkModel *nozzle_link_;
  const moveit::core::LinkModel *workpiece_link_;
};
}
