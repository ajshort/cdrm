#include <cdrm_welding/weld_planner.h>

#include <actionlib/server/simple_action_server.h>
#include <cdrm_welding/welding_cdrm_generator.h>
#include <cdrm_welding_msgs/GenerateWeldingCdrmAction.h>
#include <cdrm_welding_msgs/PlanWeld.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace cdrm_welding
{
class Server
{
public:
  Server(const moveit::core::RobotModelConstPtr &robot_model)
    : nh_("~")
    , robot_model_(robot_model)
    , psm_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"))
    , generate_server_(nh_, "generate_welding_cdrm", std::bind(&Server::generateWeldingCdrm, this, std::placeholders::_1), false)
    , plan_service_(nh_.advertiseService("plan_weld", &Server::planWeld, this))
    , targets_publisher_(nh_.advertise<visualization_msgs::MarkerArray>("target_markers", 1))
    , display_trajectory_publisher_(nh_.advertise<moveit_msgs::DisplayTrajectory>("display_planned_path", 1))
  {
    generate_server_.start();
  }

private:
  void generateWeldingCdrm(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal)
  {
    WeldingCdrmGenerator generator(robot_model_, generate_feedback_, generate_result_);
    bool success = generator.generate(goal, [this] { return generate_server_.isPreemptRequested(); });

    if (success)
    {
      cdrm_welding_msgs::GenerateWeldingCdrmResult result;
      generate_server_.setSucceeded(result);
    }
    else if (generate_server_.isPreemptRequested())
    {
      generate_server_.setPreempted();
    }
    else
    {
      generate_server_.setAborted();
    }
  }

  bool planWeld(cdrm_welding_msgs::PlanWeld::Request &req,
                cdrm_welding_msgs::PlanWeld::Response &res)
  {
    planning_scene_monitor::LockedPlanningSceneRW locked_scene(psm_);
    bool success = WeldPlanner(locked_scene, targets_publisher_).plan(req, res);

    if (!success)
      return false;

    display_trajectory_publisher_.publish(res.trajectory);

    return true;
  }

  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  actionlib::SimpleActionServer<cdrm_welding_msgs::GenerateWeldingCdrmAction> generate_server_;
  cdrm_welding_msgs::GenerateWeldingCdrmFeedback generate_feedback_;
  cdrm_welding_msgs::GenerateWeldingCdrmResult generate_result_;
  ros::ServiceServer plan_service_;
  ros::Publisher targets_publisher_;
  ros::Publisher display_trajectory_publisher_;
};
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cdrm_welding_node");
  ros::NodeHandle node_handle;

  robot_model_loader::RobotModelLoader loader("robot_description");
  const auto &robot_model = loader.getModel();

  if (!robot_model)
  {
    ROS_FATAL("The robot model could not be loaded from the robot_description param");
    return -1;
  }

  cdrm_welding::Server server(robot_model);

  ros::spin();
}
