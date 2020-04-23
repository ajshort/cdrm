#pragma once

#include <cdrm_legged/leg_configs.h>
#include <cdrm_legged/transform_map.h>

#include <ompl/base/MotionValidator.h>

#include <Eigen/Geometry>

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

private:
  PlanningContext *context_;

  mutable unordered_map_Isometry3d<std::vector<LegConfigs>> reachable_contacts_;
};
}
