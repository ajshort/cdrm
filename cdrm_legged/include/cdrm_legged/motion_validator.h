#pragma once

#include <cdrm_legged/leg_configs.h>
#include <cdrm_legged/moveit_forward.h>
#include <cdrm_legged/transform_map.h>

#include <Eigen/Geometry>
#include <ompl/base/MotionValidator.h>
#include <random_numbers/random_numbers.h>

#include <array>
#include <map>
#include <memory>
#include <vector>

namespace cdrm_legged
{
class PlanningContext;

/**
 * Validates that the full robot can move between two states.
 */
class MotionValidator : public ompl::base::MotionValidator
{
public:
  MotionValidator(PlanningContext *context);
  ~MotionValidator();

  bool checkMotion(const ompl::base::State *a, const ompl::base::State *b) const override;
  bool checkMotion(const ompl::base::State *a, const ompl::base::State *b,
                   std::pair<ompl::base::State *, double> &last_valid) const override;

  const std::vector<LegConfigs> &getReachableContacts(const Eigen::Isometry3d &body_tf) const
  {
    return reachable_contacts_.at(body_tf);
  }

  const std::vector<moveit::core::RobotState> &getValidStates(const Eigen::Isometry3d &body_tf) const
  {
    return valid_states_.at(body_tf);
  }

private:
  bool generateValidStates(const Eigen::Isometry3d &body_tf) const;

  PlanningContext *context_;

  mutable random_numbers::RandomNumberGenerator rng_;
  mutable unordered_map_Isometry3d<std::vector<LegConfigs>> reachable_contacts_;
  mutable unordered_map_Isometry3d<std::vector<moveit::core::RobotState>> valid_states_;
};
}
