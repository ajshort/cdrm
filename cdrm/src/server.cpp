#include <cdrm/cdrm.h>
#include <cdrm/generator.h>

#include <actionlib/server/simple_action_server.h>
#include <cdrm_msgs/GenerateCdrmAction.h>
#include <cdrm_msgs/ProcessFile.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>

namespace cdrm
{
/**
 * Exposes CDRM generation and persistence as actions/services.
 */
class Server
{
public:
  Server(const moveit::core::RobotModelConstPtr &robot_model)
    : nh_("~")
    , robot_model_(robot_model)
    , generate_server_(nh_, "generate_cdrm", std::bind(&Server::generateCdrm, this, std::placeholders::_1), false)
    , load_server_(nh_.advertiseService("load_cdrm", &Server::loadCdrm, this))
    , save_server_(nh_.advertiseService("save_cdrm", &Server::saveCdrm, this))
  {
    generate_server_.start();
  }

private:
  void generateCdrm(const cdrm_msgs::GenerateCdrmGoalConstPtr &goal)
  {
    const auto start = ros::Time::now();

    Generator generator(robot_model_);
    generator.setGoal(goal);

    generator.setProgressCallback([this](double progress) {
      generate_feedback_.progress = progress;
      generate_server_.publishFeedback(generate_feedback_);
    });

    const auto is_canceled = [this] { return generate_server_.isPreemptRequested(); };
    cdrm_ = generator.generate(is_canceled);

    if (cdrm_)
    {
      generate_result_.min_contact_distance = cdrm_->min_contact_distance_;
      generate_result_.max_contact_distance = cdrm_->max_contact_distance_;

      tf::vectorEigenToMsg(cdrm_->workspace_min_, generate_result_.workspace_min);
      tf::vectorEigenToMsg(cdrm_->workspace_max_, generate_result_.workspace_max);

      generate_result_.generation_time = (ros::Time::now() - start).toSec();

      generate_server_.setSucceeded(generate_result_);
    }
    else if (is_canceled())
    {
      generate_server_.setPreempted();
    }
    else
    {
      generate_server_.setAborted();
    }
  }

  bool loadCdrm(cdrm_msgs::ProcessFile::Request &req, cdrm_msgs::ProcessFile::Response &res)
  {
    cdrm_.reset(new Cdrm);
    return cdrm_->load(req.filename);
  }

  bool saveCdrm(cdrm_msgs::ProcessFile::Request &req, cdrm_msgs::ProcessFile::Response &res)
  {
    if (!cdrm_)
    {
      ROS_ERROR("A CDRM has not been generated or loaded");
      return false;
    }

    return cdrm_->save(req.filename);
  }

  ros::NodeHandle nh_;
  moveit::core::RobotModelConstPtr robot_model_;

  std::unique_ptr<Cdrm> cdrm_;

  actionlib::SimpleActionServer<cdrm_msgs::GenerateCdrmAction> generate_server_;
  cdrm_msgs::GenerateCdrmFeedback generate_feedback_;
  cdrm_msgs::GenerateCdrmResult generate_result_;

  ros::ServiceServer load_server_;
  ros::ServiceServer save_server_;
};
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cdrm_server");
  ros::NodeHandle node_handle;

  robot_model_loader::RobotModelLoader loader("robot_description");
  const auto &robot_model = loader.getModel();

  if (!robot_model)
  {
    ROS_FATAL("The robot model could not be loaded from the robot_description param");
    return -1;
  }

  cdrm::Server server(robot_model);
  ros::spin();
}
