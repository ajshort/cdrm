#pragma once

#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/StateValidityChecker.h>

namespace cdrm_legged
{
class PlanningContext;

/**
 * Validates that a state does not collide the body and that all legs are within reachability limits.
 */
class StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(PlanningContext *context);
  ~StateValidityChecker();

  bool isValid(const ompl::base::State *state) const override;

private:
  bool isWithinLegsWorkspace() const;
  bool isBodyInCollision() const;
  bool allLegsHaveConfigs(const Eigen::Isometry3d &body_tf) const;

  PlanningContext *context_;

  collision_detection::AllowedCollisionMatrix body_acm_;
  mutable moveit::core::RobotState state_;
};
}
