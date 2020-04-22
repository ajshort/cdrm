#pragma once

#include "legplan/stability_checker.h"

#include <memory>

namespace robust_equilibrium
{
class StaticEquilibrium;
}

namespace legplan
{
/**
 * Stability checker using Del Prete's robust equilibrium library.
 */
class RobustEquilibriumStabilityChecker : public StabilityChecker
{
public:
  explicit RobustEquilibriumStabilityChecker(const PlanningContext *context);

  bool isStable(const moveit::core::RobotState &state, const Contacts &contacts) const override;

private:
  std::unique_ptr<robust_equilibrium::StaticEquilibrium> m_equilibrium;
};
}
