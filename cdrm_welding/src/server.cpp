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
    , plan_service_(nh_.advertiseService("plan_weld", &Server::planWeld, this))
  {
  }

private:
  bool planWeld(cdrm_welding_msgs::PlanWeld::Request &req,
                cdrm_welding_msgs::PlanWeld::Response &res)
  {
    return false;
  }

  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;
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
