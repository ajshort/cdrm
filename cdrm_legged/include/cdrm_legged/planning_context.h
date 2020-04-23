#pragma once

#include <cdrm_legged/leg_configs.h>
#include <cdrm_legged/moveit_forward.h>
#include <cdrm_legged/transform_map.h>
#include <cdrm_legged/ompl_forward.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <moveit/planning_interface/planning_interface.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <memory>
#include <map>
#include <string>
#include <vector>

namespace cdrm
{
class Cdrm;
}

namespace cdrm_legged
{
class LegConfigGenerator;
class LegModel;
class StabilityChecker;

/**
 * Handles performing motion planning queries.
 */
class PlanningContext : public planning_interface::PlanningContext
{
  using LegModelVector = std::vector<LegModel, Eigen::aligned_allocator<LegModel>>;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlanningContext(const std::string &group);
  ~PlanningContext();

  bool solve(planning_interface::MotionPlanResponse &res) override;
  bool solve(planning_interface::MotionPlanDetailedResponse &res) override;

  bool terminate() override;
  void clear() override;

  void setPlannerConfig(std::map<std::string, std::string> planner_config)
  {
    planner_config_ = std::move(planner_config);
  }

  const Eigen::Vector3d &getWorkspaceMin() const
  {
    return workspace_min_;
  }

  const Eigen::Vector3d &getWorkspaceMax() const
  {
    return workspace_max_;
  }

  const moveit::core::RobotModelConstPtr &getRobotModel() const;

  const moveit::core::JointModelGroup *getGroup() const
  {
    return group_;
  }

  const moveit::core::FloatingJointModel *getBodyJoint() const
  {
    return body_joint_;
  }

  const LegModelVector &getLegModels() const
  {
    return leg_models_;
  }

  std::size_t getNumLegs() const;

  void addLegModel(const LegModel &leg_model);

  /**
   * Generates leg configs for a body transform, caching them.
   */
  const std::vector<LegConfigs> &generateLegConfigs(const Eigen::Isometry3d &body_tf);

  const StabilityChecker *getStabilityChecker() const
  {
    return stability_checker_.get();
  }

  const moveit::core::RobotStateConstPtr &getStartState() const
  {
    return start_state_;
  }

  const ompl::base::SpaceInformationPtr &getSpaceInformation() const
  {
    return space_info_;
  }

private:
  void configure();

  void setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &params);
  void setStartState(const moveit_msgs::RobotState &state);
  void setGoalConstraints(const std::vector<moveit_msgs::Constraints> &constraints);

  robot_trajectory::RobotTrajectoryPtr pathOmplToMoveIt(const ompl::geometric::PathGeometric &path) const;

  std::map<std::string, std::string> planner_config_;
  LegModelVector leg_models_;

  const moveit::core::JointModelGroup *group_ = nullptr;
  const moveit::core::FloatingJointModel *body_joint_ = nullptr;

  std::unique_ptr<StabilityChecker> stability_checker_;

  Eigen::Vector3d workspace_min_;
  Eigen::Vector3d workspace_max_;

  ompl::base::StateSpacePtr state_space_;
  ompl::base::SpaceInformationPtr space_info_;
  ompl::geometric::SimpleSetupPtr simple_setup_;
  ompl::base::PlannerTerminationCondition ptc_;

  moveit::core::RobotStateConstPtr start_state_;

  ros::NodeHandle node_handle_;
  ros::Publisher all_contacts_publisher_;

  std::unique_ptr<LegConfigGenerator> leg_config_generator_;

  unordered_map_Isometry3d<std::vector<LegConfigs>> generated_leg_configs_;
};
}
