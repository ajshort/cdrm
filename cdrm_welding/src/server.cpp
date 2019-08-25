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
    , plan_server_(nh_, "plan_weld", std::bind(&Server::planWeld, this, std::placeholders::_1), false)
  {
    plan_server_.start();
  }

private:
  void planWeld(const cdrm_welding_msgs::PlanWeldGoalConstPtr &goal)
  {
  }

  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;

  actionlib::SimpleActionServer<cdrm_welding_msgs::PlanWeldAction> plan_server_;
  cdrm_welding_msgs::PlanWeldFeedback plan_feedback_;
  cdrm_welding_msgs::PlanWeldResult plan_result_;
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
