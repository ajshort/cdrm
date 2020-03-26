#include <class_loader/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <ros/console.h>
#include <ros/node_handle.h>

namespace pi = planning_interface;

namespace cdrm_legged
{
/**
 * Exposes a moveit planner plugin.
 */
class PlannerManager : public pi::PlannerManager
{
public:
  std::string getDescription() const override { return "CDRM Legged Planner"; }

  bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    robot_model_ = model;

    return true;
  }

  pi::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                            const pi::MotionPlanRequest &req,
                                            moveit_msgs::MoveItErrorCodes &error_code) const override
  {
    return nullptr;
  }

  bool canServiceRequest(const pi::MotionPlanRequest &req) const override { return true; }

private:
  ros::NodeHandle nh_ = ros::NodeHandle("~");
  moveit::core::RobotModelConstPtr robot_model_;
};
}

CLASS_LOADER_REGISTER_CLASS(cdrm_legged::PlannerManager, planning_interface::PlannerManager);
