#pragma once

#include <cdrm_legged/moveit_forward.h>

namespace cdrm_legged
{
class Planner;
class PlanningContext;

/**
 * Plans motions for individual legs.
 */
class LegPlanner
{
public:
  LegPlanner(const PlanningContext *context, const Planner *planner);

  /**
   * Plans retreat, intermediate and approach motions.
   */
  void planLegMotions(const robot_trajectory::RobotTrajectory &input);

  const robot_trajectory::RobotTrajectoryPtr &getMoveItTrajectory() const { return moveit_; }

private:
  const PlanningContext *context_;
  const Planner *planner_;

  robot_trajectory::RobotTrajectoryPtr moveit_;
};
}
