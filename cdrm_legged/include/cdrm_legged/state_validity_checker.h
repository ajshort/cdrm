#pragma once

#include <cdrm_legged/transform_map.h>

#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/StateValidityChecker.h>

namespace cdrm_legged
{
class LegConfigGenerator;
struct LegConfigs;
class PlanningContext;

/**
 * Validates that a state does not collide the body and that all legs are within reachability limits.
 */
class StateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  StateValidityChecker(const PlanningContext *context);
  ~StateValidityChecker();

  bool isValid(const ompl::base::State *state) const override;

private:
  bool isWithinLegsWorkspace() const;
  bool isBodyInCollision() const;
  bool allLegsHaveConfigs(const Eigen::Isometry3d &body_tf) const;

  const PlanningContext *context_;

  collision_detection::AllowedCollisionMatrix body_acm_;
  mutable moveit::core::RobotState state_;
  mutable unordered_map_Isometry3d<std::vector<LegConfigs>> state_leg_configs_;
  std::unique_ptr<LegConfigGenerator> leg_config_generator_;
};
}
