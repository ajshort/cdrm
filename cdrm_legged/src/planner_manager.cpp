#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planning_context.h>

#include <cdrm/cdrm.h>
#include <class_loader/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <ros/console.h>
#include <ros/node_handle.h>

using planning_interface::MotionPlanRequest;
using planning_interface::PlanningContextPtr;

namespace cdrm_legged
{
/**
 * Exposes a MoveIt plugin.
 */
class PlannerManager : public planning_interface::PlannerManager
{
public:
  std::string getDescription() const override
  {
    return "CDRM";
  }

  bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
  {
    if (!ns.empty())
      nh_ = ros::NodeHandle(ns);

    robot_model_ = model;

    loadCdrms();
    loadPlannerConfigs();
    createPlanningContexts();

    if (contexts_.empty())
    {
      ROS_ERROR("No planning contexts were created");
      return false;
    }

    return true;
  }

  PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr &planning_scene,
                                        const MotionPlanRequest &req,
                                        moveit_msgs::MoveItErrorCodes &error_code) const override
  {
    auto it = contexts_.find(req.group_name);

    if (it == contexts_.end())
    {
      ROS_ERROR("Could not find planning context for group '%s'", req.group_name.c_str());
      return nullptr;
    }

    const auto &context = it->second;
    context->clear();
    context->setPlanningScene(planning_scene);
    context->setMotionPlanRequest(req);
    return context;
  }

  bool canServiceRequest(const MotionPlanRequest &req) const override
  {
    return true;
  }

private:
  /**
   * Loads CDRM instances from the specified filenames.
   */
  bool loadCdrms()
  {
    XmlRpc::XmlRpcValue filenames;

    if (!nh_.getParam("cdrms", filenames))
    {
      ROS_ERROR("No CDRM filenames specified to load on the parameter server");
      return false;
    }

    cdrms_.clear();

    for (auto it = filenames.begin(); it != filenames.end(); ++it)
    {
      const auto &filename = static_cast<std::string &>(it->second);
      std::unique_ptr<cdrm::Cdrm> cdrm(new cdrm::Cdrm);

      ROS_INFO_STREAM("Loading CDRM '" << it->first << "' from '" << filename.c_str() << "'...");

      if (cdrm->load(filename))
        cdrms_[it->first] = std::move(cdrm);
      else
        ROS_WARN_STREAM("Could not load CDRM '" << it->first << "'");
    }

    return true;
  }

  /**
   * Loads planner configs from the parameter server.
   */
  bool loadPlannerConfigs()
  {
    config_settings_.clear();

    for (const auto &group_name : robot_model_->getJointModelGroupNames())
    {
      XmlRpc::XmlRpcValue config_value;

      if (nh_.getParam("planner_configs/" + group_name, config_value))
      {
        auto &config = config_settings_[group_name];
        config.group = group_name;
        config.name = group_name;

        for (auto it = config_value.begin(); it != config_value.end(); ++it)
        {
          if (it->second.getType() == XmlRpc::XmlRpcValue::TypeString)
            config.config[it->first] = static_cast<std::string>(it->second);
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            config.config[it->first] = std::to_string(static_cast<double>(it->second));
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
            config.config[it->first] = std::to_string(static_cast<int>(it->second));
          else if (it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
            config.config[it->first] = std::to_string(static_cast<bool>(it->second));
        }
      }
    }

    return true;
  }

  void createPlanningContexts()
  {
    contexts_.clear();

    for (const auto &group_name : robot_model_->getJointModelGroupNames())
    {
      XmlRpc::XmlRpcValue legs_value;

      if (!nh_.getParam("planner_configs/" + group_name + "/legs", legs_value))
        continue;

      const auto context = std::make_shared<PlanningContext>(group_name);
      for (auto it = legs_value.begin(); it != legs_value.end(); ++it)
      {
        const auto &leg_name = it->first;
        const auto *leg_group = robot_model_->getJointModelGroup(leg_name);

        if (!leg_group)
        {
          ROS_WARN("Could not find leg joint model group '%s'", leg_name.c_str());
          continue;
        }

        const auto &cdrm_name = static_cast<std::string &>(it->second);
        const auto cdrm_it = cdrms_.find(cdrm_name);

        if (cdrm_it != cdrms_.end())
          context->addLegModel(LegModel(robot_model_, leg_group, cdrm_it->second.get()));
        else
          ROS_WARN("No CDRM '%s' for leg '%s' in group '%s'", cdrm_name.c_str(), leg_name.c_str(), group_name.c_str());
      }

      // Make sure we loaded the expected number of legs.
      if (context->getLegModels().size() != static_cast<std::size_t>(legs_value.size()))
      {
        ROS_WARN("Could not create context for group '%s'", group_name.c_str());
        continue;
      }

      contexts_[group_name] = context;

      const auto config_it = config_settings_.find(group_name);

      if (config_it != config_settings_.end())
        context->setPlannerConfig(config_it->second.config);
    }
  }

  ros::NodeHandle nh_ = ros::NodeHandle("~");
  moveit::core::RobotModelConstPtr robot_model_;

  std::map<std::string, std::unique_ptr<cdrm::Cdrm>> cdrms_;
  std::map<std::string, std::shared_ptr<PlanningContext>> contexts_;
};
}

CLASS_LOADER_REGISTER_CLASS(cdrm_legged::PlannerManager, planning_interface::PlannerManager);
