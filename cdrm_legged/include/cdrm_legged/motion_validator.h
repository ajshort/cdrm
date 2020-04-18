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

private:
  /**
   * Checks if each leg has at least one valid foothold, memoising results.
   */
  bool hasValidLegConfigs(const std::array<double, 7> &s, const Eigen::Isometry3d &tf) const;

  const PlanningContext *context_;
  std::unique_ptr<LegConfigGenerator> leg_config_generator_;
  mutable std::map<std::array<double, 7>, std::vector<LegConfigs>> leg_configs_;
};
}
