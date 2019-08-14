#pragma once

#include "cdrm/roadmap.h"

#include <Eigen/Core>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ros/message_forward.h>
#include <ros/time.h>

#include <memory>

namespace cdrm_msgs
{
ROS_DECLARE_MESSAGE(GenerateCdrmGoal);
}

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(JointModelGroup);
MOVEIT_CLASS_FORWARD(LinkModel);
MOVEIT_CLASS_FORWARD(RobotModel);
}
}

namespace ompl
{
namespace base
{
class State;
}
}

namespace cdrm
{
class Cdrm;

/**
 * Creates a CDRM from a robot model.
 */
class Generator
{
public:
  using CancelRequestedCallback = std::function<bool()>;
  using ProgressCallback = std::function<void(double)>;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Generator(const moveit::core::RobotModelConstPtr &robot_model);
  ~Generator();

  void setGoal(const cdrm_msgs::GenerateCdrmGoalConstPtr &goal) { goal_ = goal; }

  void setProgressCallback(ProgressCallback progress_callback) { progress_callback_ = progress_callback; }

  /**
   * Builds and returns a new CDRM instance.
   */
  std::unique_ptr<Cdrm> generate(CancelRequestedCallback is_canceled_callback);

private:
  VertexDescriptor addVertex(const ompl::base::State *s);
  EdgeDescriptor addEdge(const VertexDescriptor &a, const VertexDescriptor &b);

  bool isStateValid(const ompl::base::State *s);

  void updateProgress(double progress);

  const moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScene planning_scene_;

  cdrm_msgs::GenerateCdrmGoalConstPtr goal_;
  ProgressCallback progress_callback_;

  const moveit::core::JointModelGroup *jmg_;
  std::vector<const moveit::core::LinkModel *> link_models_;
  Eigen::Isometry3d origin_tf_inv_;
  collision_detection::AllowedCollisionMatrix acm_;

  std::unique_ptr<Cdrm> cdrm_;

  ros::Time last_progress_time_ = ros::Time(0);
};
}
