#pragma once

#include <cdrm_legged/moveit_forward.h>

#include <Eigen/Geometry>

#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/robot_state.h>

namespace cdrm
{
class Cdrm;
}

namespace cdrm_legged
{
/**
 * Contains information about a single leg.
 */
class LegModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegModel(const moveit::core::RobotModelConstPtr &robot_model, const moveit::core::JointModelGroup *jmg,
           const cdrm::Cdrm *cdrm);

  /**
   * Gets the position of the foot tip relative to body given a configuration.
   */
  Eigen::Isometry3d getTipTransform(const Eigen::VectorXd &q) const;

  const moveit::core::JointModelGroup *jmg_;
  const moveit::core::LinkModel *tip_link_;

  const cdrm::Cdrm *cdrm_;

  /**
   * The transform from the body to the leg origin.
   */
  const Eigen::Isometry3d tf_;

  collision_detection::AllowedCollisionMatrix acm_;

private:
  /**
   * Gets the transform from body to leg origin.
   */
  Eigen::Isometry3d getLegOriginTf(const moveit::core::RobotModelConstPtr &robot_model) const;

  mutable moveit::core::RobotState state_;
};
}
