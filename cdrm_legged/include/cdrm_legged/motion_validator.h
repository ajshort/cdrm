#pragma once

#include <ompl/base/MotionValidator.h>

#include <Eigen/Geometry>

#include <array>
#include <map>
#include <memory>
#include <vector>

namespace cdrm_legged
{
struct LegConfigs;
class LegConfigGenerator;
class PlanningContext;

/**
 * Validates that the full robot can move between two states.
 */
class MotionValidator : public ompl::base::MotionValidator
{
public:
  MotionValidator(const PlanningContext *context);
  ~MotionValidator();

  bool checkMotion(const ompl::base::State *a, const ompl::base::State *b) const override;
  bool checkMotion(const ompl::base::State *a, const ompl::base::State *b,
                   std::pair<ompl::base::State *, double> &last_valid) const override;

  const PlanningContext *context_;
};
}
