#include <cdrm_legged/leg_planner.h>

#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planner.h>
#include <cdrm_legged/planning_context.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ros/console.h>

namespace rt = robot_trajectory;

namespace cdrm_legged
{
LegPlanner::LegPlanner(const PlanningContext *context, const Planner *planner) : context_(context), planner_(planner)
{
}

void LegPlanner::planLegMotions(const rt::RobotTrajectory &input)
{
  trajectory_processing::IterativeParabolicTimeParameterization time_param;

  moveit_.reset(new rt::RobotTrajectory(input.getRobotModel(), input.getGroupName()));
  moveit_->addSuffixWayPoint(input.getWayPoint(0), 0);

  const auto &models = context_->getLegModels();

  std::vector<std::string> all_leg_names;
  all_leg_names.reserve(models.size());

  for (const auto &model : models) {
    all_leg_names.push_back(model.jmg_->getName());
  }

  for (std::size_t i = 1; i < input.getWayPointCount(); ++i)
  {
    // Figure out which leg has moved (if any).
    int leg = -1;

    const auto &from = input.getWayPoint(i - 1);
    const auto &to = input.getWayPoint(i);

    for (std::size_t l = 0; l < context_->getNumLegs(); ++l)
    {
      const auto *tip = models[l].tip_link_;
      double norm = (from.getGlobalLinkTransform(tip).translation() - to.getGlobalLinkTransform(tip).translation()).norm();

      if (norm < 0.001)
        continue;

      if (leg != -1)
        ROS_WARN_STREAM("More than one leg is moving at state " << i);

      leg = l;
    }

    if (leg == -1)
    {
      moveit_->addSuffixWayPoint(to, 0);

      rt::RobotTrajectory body_traj(input.getRobotModel(), input.getGroupName());
      body_traj.addSuffixWayPoint(from, 0);
      body_traj.addSuffixWayPoint(to, 0);

      time_param.computeTimeStamps(body_traj, 0.25, 0.25);

      continue;
    }

    const auto &leg_model = models[leg];
    auto work_state = from;

    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace);

    for (const auto *bounds : leg_model.jmg_->getActiveJointModelsBounds())
      space->as<ompl::base::RealVectorStateSpace>()->addDimension((*bounds)[0].min_position_, (*bounds)[0].max_position_);

    ompl::geometric::SimpleSetup ss(space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);

    from.copyJointGroupPositions(leg_model.jmg_, start->values);
    to.copyJointGroupPositions(leg_model.jmg_, goal->values);

    ss.setStateValidityChecker([&](const ompl::base::State *s) {
      work_state.setJointGroupPositions(leg_model.jmg_, s->as<ompl::base::RealVectorStateSpace::StateType>()->values);
      work_state.update();

      collision_detection::CollisionRequest req;
      collision_detection::CollisionResult res;

      context_->getPlanningScene()->checkCollision(req, res, work_state, leg_model.acm_);
      return !res.collision;
    });
    ss.setStartAndGoalStates(start, goal);

    Eigen::Isometry3d global_body_tf = to.getJointTransform(context_->getBodyJoint());
    Eigen::Isometry3d global_goal_tf = to.getGlobalLinkTransform(leg_model.tip_link_);

    rt::RobotTrajectory leg_robot_traj(input.getRobotModel(), input.getGroupName());

    if (ss.solve(1.0))
    {
      rt::RobotTrajectory leg_traj(input.getRobotModel(), leg_model.jmg_->getName());

      ss.simplifySolution(0.2);
      const auto &path = ss.getSolutionPath();

      for (std::size_t i = 0; i < path.getStateCount(); ++i)
      {
        work_state.setJointGroupPositions(leg_model.jmg_, path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values);
        work_state.update();

        moveit_->addSuffixWayPoint(work_state, 0);
      }
    }
    else
    {
      moveit_->addSuffixWayPoint(to, 0);
    }

    time_param.computeTimeStamps(leg_robot_traj, 0.5, 0.5);
  }
}
}
