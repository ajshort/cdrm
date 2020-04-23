#include <cdrm_legged/leg_model.h>

#include <cdrm/cdrm.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <cassert>

namespace cdrm_legged
{
LegModel::LegModel(const moveit::core::RobotModelConstPtr &robot_model, const moveit::core::JointModelGroup *jmg,
                   const cdrm::Cdrm *cdrm)
  : jmg_(jmg)
  , tip_link_(jmg->getLinkModels().back())
  , cdrm_(cdrm)
  , tf_(getLegOriginTf(robot_model))
  , state_(robot_model)
{
  assert(jmg);
  assert(cdrm);
}

Eigen::Isometry3d LegModel::getTipTransform(const Eigen::VectorXd &q) const
{
  state_.setJointGroupPositions(jmg_, q);
  state_.update();

  return tf_.inverse() * state_.getGlobalLinkTransform(tip_link_);
}

Eigen::Isometry3d LegModel::getTipTransform(unsigned int vertex) const
{
  return getTipTransform(cdrm_->getVertexConfig(vertex));
}

Eigen::Isometry3d LegModel::getLegOriginTf(const moveit::core::RobotModelConstPtr &robot_model) const
{
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  state.update();

  // TODO the body origin is not neccesarily world origin.
  return state.getGlobalLinkTransform(jmg_->getLinkModels().front());
}
}
