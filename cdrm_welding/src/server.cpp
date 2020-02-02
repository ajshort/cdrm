#include <cdrm_welding/weld_planner.h>

#include <actionlib/server/simple_action_server.h>
#include <cdrm_welding_msgs/GenerateWeldingCdrmAction.h>
#include <cdrm_welding_msgs/PlanWeld.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>

namespace cdrm_welding
{
class Server
{
public:
  Server(const moveit::core::RobotModelConstPtr &robot_model)
    : nh_("~")
    , robot_model_(robot_model)
    , generate_server_(nh_, "generate_welding_cdrm", std::bind(&Server::generateWeldingCdrm, this, std::placeholders::_1), false)
    , plan_service_(nh_.advertiseService("plan_weld", &Server::planWeld, this))
  {
  }

private:
  void generateWeldingCdrm(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal)
  {
  }

  bool planWeld(cdrm_welding_msgs::PlanWeld::Request &req,
                cdrm_welding_msgs::PlanWeld::Response &res)
  {
    return WeldPlanner(robot_model_).plan(req, res);
  }

  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;
  actionlib::SimpleActionServer<cdrm_welding_msgs::GenerateWeldingCdrmAction> generate_server_;
  cdrm_welding_msgs::GenerateWeldingCdrmFeedback generate_feedback_;
  cdrm_welding_msgs::GenerateWeldingCdrmResult generate_result_;
  ros::ServiceServer plan_service_;
};
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cdrm_welding_server");
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
