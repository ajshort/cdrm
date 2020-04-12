#pragma once

#include <cdrm_welding_msgs/PlanWeld.h>
#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(JointModelGroup);
MOVEIT_CLASS_FORWARD(LinkModel);
MOVEIT_CLASS_FORWARD(RobotModel);
}
}

namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);
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
  WeldPlanner(const planning_scene::PlanningSceneConstPtr &planning_scene,
              const ros::Publisher &target_publisher);

  bool plan(const cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res);

private:
  void publishTargets(const Weld &weld);

  const planning_scene::PlanningSceneConstPtr planning_scene_;
  const moveit::core::RobotModelConstPtr robot_model_;
  ros::Publisher target_publisher_;

  const moveit::core::JointModelGroup *planning_group_;
  const moveit::core::LinkModel *nozzle_link_;
};
}
