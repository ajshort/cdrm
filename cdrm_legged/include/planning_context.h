#pragma once

#include <moveit/planning_interface/planning_interface.h>

namespace cdrm_legged
{
/**
 * Handles performing motion planning queries.
 */
class PlanningContext : public planning_interface::PlanningContext
{
public:
  bool solve(MotionPlanResponse& &res) override;
  bool solve(MotionPlanDetailedResponse &res) override;

  bool terminate() override;
  void clear() override;
};
}
