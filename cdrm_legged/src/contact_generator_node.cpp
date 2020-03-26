#include <cdrm_legged/leg_config_generator.h>
#include <cdrm_legged/leg_model.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cdrm/cdrm.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>

static const std::string TOPIC = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update";
static const std::string MARKER_NAME = "JJ:start_base_link";

class ContactGeneratorNode
{
public:
  ContactGeneratorNode()
    : nh_("move_group")
    , subscriber_(nh_.subscribe(TOPIC, 100, &ContactGeneratorNode::onMarkerUpdated, this))
    , publisher_(nh_.advertise<visualization_msgs::Marker>("rviz_contacts", 0))
    , psm_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"))
  {
    psm_->startSceneMonitor();

    loadCdrms();
    createLegModels();
  }

private:
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

  void createLegModels()
  {
    planning_scene_monitor::LockedPlanningSceneRW lock(psm_);
    const auto &robot_model = lock->getRobotModel();

    for (const auto &group_name : robot_model->getJointModelGroupNames())
    {
      XmlRpc::XmlRpcValue legs_value;

      if (!nh_.getParam("planner_configs/" + group_name + "/legs", legs_value))
        continue;

      for (auto it = legs_value.begin(); it != legs_value.end(); ++it)
      {
        const auto &leg_name = it->first;
        const auto *leg_group = robot_model->getJointModelGroup(leg_name);

        if (!leg_group)
        {
          ROS_WARN("Could not find leg joint model group '%s'", leg_name.c_str());
          continue;
        }

        const auto &cdrm_name = static_cast<std::string &>(it->second);
        const auto cdrm_it = cdrms_.find(cdrm_name);

        if (cdrm_it != cdrms_.end())
          leg_models_.emplace_back(robot_model, leg_group, cdrm_it->second.get());
        else
          ROS_WARN("No CDRM '%s' for leg '%s' in group '%s'", cdrm_name.c_str(), leg_name.c_str(), group_name.c_str());
      }

      ROS_INFO("Created leg models for '%s'", group_name.c_str());
    }
  }

  void onMarkerUpdated(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg)
  {
    for (const auto &pose : msg->poses)
    {
      if (pose.name != MARKER_NAME)
        continue;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "contact_generator_node";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.025;
      marker.scale.y = 0.025;
      marker.scale.z = 0.025;
      marker.color.a = 1.0;
      marker.color.r = 1.0;

      Eigen::Isometry3d updated_tf;
      tf::poseMsgToEigen(pose.pose, updated_tf);

      if (body_tf_.isApprox(updated_tf))
        continue;

      planning_scene_monitor::LockedPlanningSceneRW lock(psm_);
      cdrm_legged::LegConfigGenerator generator(lock);

      for (const auto &model : leg_models_)
      {
        const auto contacts = generator.generateLegConfigs(updated_tf, model);

        for (const auto vertex : contacts.contacts_)
        {
          const Eigen::VectorXd &q = model.cdrm_->getVertexConfig(vertex);
          const Eigen::Isometry3d contact_tf = updated_tf * model.tf_ * model.getTipTransform(q);

          geometry_msgs::Point point;
          point.x = contact_tf.translation().x();
          point.y = contact_tf.translation().y();
          point.z = contact_tf.translation().z();
          marker.points.push_back(point);
        }
      }

      body_tf_ = updated_tf;

      publisher_.publish(marker);
      return;
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  Eigen::Isometry3d body_tf_;

  std::map<std::string, std::unique_ptr<cdrm::Cdrm>> cdrms_;
  std::vector<cdrm_legged::LegModel> leg_models_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "contact_generator_node");
  ContactGeneratorNode node;
  ros::spin();
}
