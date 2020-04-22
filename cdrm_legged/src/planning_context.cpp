#include <cdrm_legged/planning_context.h>

#include <cdrm_legged/conversions.h>
#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/leg_planner.h>
#include <cdrm_legged/motion_validator.h>
#include <cdrm_legged/stability_checker.h>
#include <cdrm_legged/state_validity_checker.h>

#include <cdrm/cdrm.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <visualization_msgs/Marker.h>

namespace mc = moveit::core;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace rt = robot_trajectory;

namespace cdrm_legged
{
PlanningContext::PlanningContext(const std::string &group)
  : planning_interface::PlanningContext("CDRM", group)
  , state_space_(new ob::SE3StateSpace)
  , space_info_(new ob::SpaceInformation(state_space_))
  , simple_setup_(new og::SimpleSetup(space_info_))
  , ptc_(ob::plannerAlwaysTerminatingCondition())
  , node_handle_("cdrm_legged")
  , all_contacts_publisher_(node_handle_.advertise<visualization_msgs::Marker>("all_contacts", 1))
{
}

PlanningContext::~PlanningContext()
{
}

bool PlanningContext::solve(planning_interface::MotionPlanResponse &res)
{
  planning_interface::MotionPlanDetailedResponse detailed;

  auto start_time = ros::Time::now();
  bool result = solve(detailed);

  if (result)
    res.trajectory_ = detailed.trajectory_.back();

  res.planning_time_ = (ros::Time::now() - start_time).toSec();
  res.error_code_ = detailed.error_code_;

  return result;
}

bool PlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res)
{
  configure();

  const auto status = simple_setup_->solve(ptc_);

  if (status)
  {
    robot_trajectory::RobotTrajectoryPtr trajectory(new robot_trajectory::RobotTrajectory(getRobotModel(), group_));

    robot_state::RobotState robot_state(getRobotModel());
    robot_state.setToDefaultValues();

    for (const auto *state : simple_setup_->getSolutionPath().getStates())
    {
      Eigen::Isometry3d body_tf = transformOmplToEigen(state);
      robot_state.setJointPositions(getBodyJoint(), body_tf);
      robot_state.update();

      trajectory->addSuffixWayPoint(robot_state, 1.0);
    }

    res.trajectory_.push_back(trajectory);
    res.description_.push_back("plan");
    res.processing_time_.push_back(simple_setup_->getLastPlanComputationTime());

    // TODO simplify path.
    // TODO interpolate path.

    res.error_code_.val  = res.error_code_.SUCCESS;
  }
  else
  {
    res.error_code_.val = res.error_code_.PLANNING_FAILED;
  }

  // Publish all contacts as dots.
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time();
  // marker.ns = "cdrm_legged";
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::SPHERE_LIST;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.scale.x = 0.025;
  // marker.scale.y = 0.025;
  // marker.scale.z = 0.025;
  // marker.color.a = 1.0;
  // marker.color.r = 1.0;

  // for (const auto &contact : planner->getAllCreatedContacts())
  // {
  //   geometry_msgs::Point point;
  //   point.x = contact.x();
  //   point.y = contact.y();
  //   point.z = contact.z();
  //   marker.points.push_back(point);
  // }

  // all_contacts_publisher_.publish(marker);

  return status;
}

bool PlanningContext::terminate()
{
  ptc_.terminate();
  return true;
}

void PlanningContext::clear()
{
  group_ = nullptr;
  body_joint_ = nullptr;

  stability_checker_.reset();

  simple_setup_->setStateValidityChecker(ob::StateValidityCheckerPtr());
  simple_setup_->setOptimizationObjective(ob::OptimizationObjectivePtr());

  workspace_min_ = Eigen::Vector3d::Zero();
  workspace_max_ = Eigen::Vector3d::Zero();
  state_space_->as<ob::SE3StateSpace>()->setBounds(ob::RealVectorBounds(3));

  start_state_ = nullptr;
  simple_setup_->clearStartStates();
  simple_setup_->setGoal(nullptr);

  ptc_ = ob::plannerAlwaysTerminatingCondition();
}

const mc::RobotModelConstPtr &PlanningContext::getRobotModel() const
{
  return getPlanningScene()->getRobotModel();
}

std::size_t PlanningContext::getNumLegs() const
{
  return leg_models_.size();
}

void PlanningContext::addLegModel(const LegModel &leg_model)
{
  leg_models_.push_back(leg_model);
}

void PlanningContext::configure()
{
  group_ = getRobotModel()->getJointModelGroup(getGroupName());
  body_joint_ = static_cast<const mc::FloatingJointModel *>(getRobotModel()->getRootJoint());

  stability_checker_.reset(new SupportPolygonStability(this));

  // TODO move this somewhere else
  // Set up the ACM for each leg.
  for (auto &leg_model : leg_models_)
  {
    leg_model.acm_ = getPlanningScene()->getAllowedCollisionMatrix();

    for (const auto &link_name : getRobotModel()->getLinkModelNamesWithCollisionGeometry())
    {
      // TODO the last link collision or not should be configurable.
      if (!leg_model.jmg_->hasLinkModel(link_name) || link_name == leg_model.jmg_->getLinkModels().back()->getName())
        leg_model.acm_.setDefaultEntry(link_name, true);
    }
  }

  // Simple setup
  ob::StateValidityCheckerPtr state_validator(new StateValidityChecker(this));
  simple_setup_->setStateValidityChecker(state_validator);

  ob::MotionValidatorPtr motion_validator(new MotionValidator(this));
  space_info_->setMotionValidator(motion_validator);

  ob::OptimizationObjectivePtr optim(new ob::PathLengthOptimizationObjective(space_info_));
  simple_setup_->setOptimizationObjective(optim);

  // Request
  const auto &req = getMotionPlanRequest();

  setWorkspaceBounds(req.workspace_parameters);
  setStartState(req.start_state);
  setGoalConstraints(req.goal_constraints);

  ptc_ = ob::timedPlannerTerminationCondition(req.allowed_planning_time);

  // Planner
  ob::PlannerPtr planner(new og::RRT(space_info_));
  simple_setup_->setPlanner(planner);

  for (const auto &param : planner_config_)
  {
    if (planner->params().hasParam(param.first))
      planner->params()[param.first] = std::stod(param.second);
  }
}

void PlanningContext::setWorkspaceBounds(const moveit_msgs::WorkspaceParameters &params)
{
  tf::vectorMsgToEigen(params.min_corner, workspace_min_);
  tf::vectorMsgToEigen(params.max_corner, workspace_max_);

  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, params.min_corner.x);
  bounds.setLow(1, params.min_corner.y);
  bounds.setLow(2, params.min_corner.z);

  bounds.setHigh(0, params.max_corner.x);
  bounds.setHigh(1, params.max_corner.y);
  bounds.setHigh(2, params.max_corner.z);

  state_space_->as<ob::SE3StateSpace>()->setBounds(bounds);
}

void PlanningContext::setStartState(const moveit_msgs::RobotState &state)
{
  start_state_ = getPlanningScene()->getCurrentStateUpdated(state);
  const Eigen::Isometry3d &tf = start_state_->getJointTransform(getBodyJoint());

  ob::ScopedState<> ompl_state(space_info_);
  transformEigenToOmpl(tf, ompl_state.get());
  simple_setup_->addStartState(ompl_state);
}

void PlanningContext::setGoalConstraints(const std::vector<moveit_msgs::Constraints> &constraints)
{
  mc::RobotState robot_state(getRobotModel());
  ob::ScopedState<> goal_state(space_info_);

  for (const auto &constraint : constraints)
  {
    for (const auto &joint : constraint.joint_constraints)
      robot_state.setVariablePosition(joint.joint_name, joint.position);
  }

  robot_state.update();

  // TODO this should use a goal region (constraint tolerance).
  transformEigenToOmpl(robot_state.getJointTransform(getBodyJoint()), goal_state.get());
  simple_setup_->setGoalState(goal_state);
}
}
