#pragma once

#include <moveit/planning_interface/planning_interface.h>

namespace legplan
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
