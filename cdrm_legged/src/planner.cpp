#include <cdrm_legged/planner.h>

#include <cdrm_legged/conversions.h>
#include <cdrm_legged/disjoint_sets.h>
#include <cdrm_legged/leg_config_generator.h>
#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planning_context.h>
#include <cdrm_legged/stability_checker.h>

#include <cdrm/cdrm.h>

#include <angles/angles.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/config/SelfConfig.h>

namespace mc = moveit::core;
namespace ob = ompl::base;

namespace cdrm_legged
{
struct Planner::Node
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO the number of leg configs shouldn't be hard-coded.
  Node(const ob::SpaceInformationPtr &si) : state_(si), leg_configs_(4)
  {
  }

  /**
   * The SE(3) body state.
   */
  ob::ScopedState<ob::SE3StateSpace> state_;

  /**
   * Leg configs for each leg (some may not be populated).
   */
  std::vector<std::unique_ptr<LegConfigs>> leg_configs_;

  EigenSTL::vector_Vector3d surface_normals_;
  EigenSTL::vector_Vector3d preceding_surface_normals_;

  /**
   * The full-body state.
   */
  Eigen::VectorXd config_;
  Eigen::VectorXd preceding_config_;

  /**
   * The parent node in the tree.
   */
  Node *parent_ = nullptr;
};

Planner::Planner(const PlanningContext *context)
  : ob::Planner(context->getSpaceInformation(), "CDRM")
  , context_(context)
  , leg_configs_(new LegConfigGenerator(context_->getPlanningScene()))
  , state_(context->getRobotModel())
{
  specs_.approximateSolutions = true;
  specs_.directed = true;

  declareParam<double>("range", this, &Planner::setRange, &Planner::getRange, "0.:1.:10000.");
  declareParam<double>("goal_bias", this, &Planner::setGoalBias, &Planner::getGoalBias, "0.:.05:1.");
}

Planner::~Planner()
{
  clear();
}

ob::PlannerStatus Planner::solve(const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();

  const auto &legs = context_->getLegModels();
  const auto &space = si_->getStateSpace();
  auto *goal = pdef_->getGoal().get();
  auto *goal_sampleable = dynamic_cast<ob::GoalSampleableRegion *>(goal);

  while (const auto *start = pis_.nextStart())
  {
    // Copy the overall start state and transform the body to the generated start.
    const Eigen::Isometry3d body_tf = transformOmplToEigen(start);

    state_ = *context_->getStartState();
    state_.setJointPositions(context_->getBodyJoint(), body_tf);
    state_.update();

    auto *node = new Node(si_);
    node->state_ = start;

    for (const auto &leg : legs)
      node->surface_normals_.push_back(-state_.getGlobalLinkTransform(leg.tip_link_).linear().col(2));

    nn_->add(node);

    state_.copyJointGroupPositions(context_->getGroup(), node->config_);
  }

  if (nn_->size() == 0)
  {
    OMPL_ERROR("No valid start states");
    return ob::PlannerStatus::INVALID_START;
  }

  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  OMPL_INFORM("Starting planning with %u start states", nn_->size());

  Node *solution = nullptr;
  bool approximate = true;
  double approximate_dist = std::numeric_limits<double>::infinity();

  // Grow the tree until we reach the goal, or the time has elapsed.
  std::unique_ptr<Node> sampled(new Node(si_));

  while (!ptc)
  {
    if (goal_sampleable && rng_.uniform01() < goal_bias_ && goal_sampleable->canSample())
      goal_sampleable->sampleGoal(sampled->state_.get());
    else
      sampler_->sampleUniform(sampled->state_.get());

    sampled->state_->rotation().setIdentity();

    auto *nearest = nn_->nearest(sampled.get());
    double dist = sampled->state_.distance(nearest->state_);

    if (dist > range_)
      space->interpolate(nearest->state_.get(), sampled->state_.get(), range_ / dist, sampled->state_.get());

    if (!si_->isValid(sampled->state_.get()))
      continue;

    if (!checkMotion(nearest, sampled.get()))
      continue;

    // If the state is valid, then release it and add it to the tree.
    auto *added = sampled.release();
    added->parent_ = nearest;
    nn_->add(added);

    sampled.reset(new Node(si_));

    if (goal->isSatisfied(added->state_.get(), &dist))
    {
      solution = added;
      approximate = false;
      break;
    }

    if (dist < approximate_dist)
    {
      solution = added;
      approximate_dist = dist;
    }
  }

  OMPL_INFORM("Finished planning with %u states", nn_->size());

  if (solution)
  {
    std::vector<const Node *> nodes;

    for (; solution != nullptr; solution = solution->parent_)
      nodes.push_back(solution);

    std::reverse(nodes.begin(), nodes.end());

    auto path = std::make_shared<ompl::geometric::PathGeometric>(si_);

    for (const auto *node : nodes)
      path->append(node->state_.get());

    pdef_->addSolutionPath(path, approximate, approximate_dist, getName());

    createTrajectory(nodes);

    return ob::PlannerStatus(true, approximate);
  }

  return ob::PlannerStatus::TIMEOUT;
}

void Planner::setup()
{
  ob::Planner::setup();

  ompl::tools::SelfConfig config(si_, getName());
  config.configurePlannerRange(range_);

  nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Node *>(this));
  nn_->setDistanceFunction([](const Node *a, const Node *b) { return a->state_.distance(b->state_); });
}

void Planner::clear()
{
  ob::Planner::clear();

  std::vector<Node *> nodes;
  nn_->list(nodes);
  nn_->clear();

  for (auto *node : nodes)
    delete node;
}

bool Planner::checkMotion(Node *from, Node *to)
{
  Eigen::Isometry3d body_tf = transformOmplToEigen(to->state_.get());
  to->surface_normals_ = from->surface_normals_;

  return true;

  if (checkTransition(from->config_, body_tf, to))
  {
    state_.copyJointGroupPositions(context_->getGroup(), to->config_);
    return true;
  }

  const auto &legs = context_->getLegModels();
  const auto *stability = context_->getStabilityChecker();

  // Figure out which leg we can reposition.
  Eigen::Isometry3d from_body_tf = transformOmplToEigen(from->state_.get());

  // TODO this is a copy and paste of createContact basically
  for (int i = 0; i < legs.size(); ++i)
  {
    const auto &leg = legs[i];

    state_.setJointGroupPositions(context_->getGroup(), from->config_);
    state_.update();

    all_contacts_.push_back(state_.getGlobalLinkTransform(leg.tip_link_).translation());

    Contacts reduced_contacts;

    for (int j = 0; j < context_->getNumLegs(); ++j)
    {
      if (i == j)
        continue;

      reduced_contacts.contacts_.push_back(i);
      reduced_contacts.normals_.push_back(from->surface_normals_[j]);
    }

    if (!stability->isStable(state_, reduced_contacts))
      continue;

    // if (!from->leg_configs_[i])
      from->leg_configs_[i].reset(new LegConfigs(std::move(leg_configs_->generateLegConfigs(from_body_tf, legs[i]))));

    const auto &configs = *from->leg_configs_[i];
    const auto &contacts = configs.contacts_;
    const auto &roadmap = leg.cdrm_->roadmap_;

    // Ensure there's at least one contact.
    if (contacts.empty())
      continue;

    // TODO check contacts are reachable through the roadmap from prev

    collision_detection::CollisionRequest req;
    req.group_name = leg.jmg_->getName();

    for (std::size_t candidate_index = 0; candidate_index < contacts.size(); ++candidate_index)
    {
      auto candidate = contacts[candidate_index];
      state_.setJointGroupPositions(leg.jmg_, roadmap[candidate].q_);
      state_.update();

      Contacts contacts;

      state_.setJointGroupPositions(leg.jmg_, roadmap[candidate].q_);

      // TODO normals should be fixed.
      for (int i = 0; i < context_->getNumLegs(); ++i)
      {
        contacts.contacts_.push_back(i);
        contacts.normals_.push_back(from->surface_normals_[i]);
      }

      collision_detection::CollisionResult res;
      context_->getPlanningScene()->checkSelfCollision(req, res, state_);

      if (res.collision)
        continue;

      if (!stability->isStable(state_, contacts))
        continue;

      if (!offsetContact(state_, leg, configs.normals_[candidate_index], 0.001))
        continue;

      Eigen::VectorXd intermediate;
      state_.copyJointGroupPositions(context_->getGroup(), intermediate);

      if (checkTransition(intermediate, body_tf, to))
      {
        to->preceding_config_ = intermediate;
        to->preceding_surface_normals_ = to->surface_normals_;
        to->preceding_surface_normals_[i] = configs.normals_[candidate_index];
        state_.copyJointGroupPositions(context_->getGroup(), to->config_);
        return true;
      }
      else
      {
        state_.setJointGroupPositions(context_->getGroup(), from->config_);
      }
    }
  }

  return false;
}

// bool Planner::createReposition()
// {
// }

bool Planner::checkTransition(const Eigen::VectorXd &from_config, const Eigen::Isometry3d &body_tf, Node *node)
{
  const auto &legs = context_->getLegModels();
  const auto *stability = context_->getStabilityChecker();

  // Get the previous position for each foot tip.
  state_.setJointGroupPositions(context_->getGroup(), from_config);
  state_.update();

  EigenSTL::vector_Isometry3d from_tips(legs.size());

  for (std::size_t i = 0; i < legs.size(); ++i)
    from_tips[i] = state_.getGlobalLinkTransform(legs[i].tip_link_);

  // Check that from the previous state only one contact is broken.
  state_.setJointPositions(context_->getBodyJoint(), body_tf);
  state_.update();

  int broken_index = -1;

  for (std::size_t i = 0; i < legs.size(); ++i)
  {
    const auto &leg = legs[i];

    Eigen::VectorXd qprev;
    state_.copyJointGroupPositions(leg.jmg_, qprev);

    if (state_.setFromIK(leg.jmg_, from_tips[i]))
    {
      state_.update();

      Eigen::VectorXd qcurr;
      state_.copyJointGroupPositions(leg.jmg_, qcurr);

      // Ensure we don't flip the femur sign / configuration.
      if (qprev(1) * qcurr(1) >= 0)
      {
        collision_detection::CollisionRequest req;
        req.group_name = leg.jmg_->getName();
        collision_detection::CollisionResult res;

        // context_->getPlanningScene()->checkSelfCollision(req, res, state_);

        if (!res.collision)
        {
          // Ensure we don't exceed the gimbal 30 deg limit.
          const Eigen::Vector3d direction = state_.getGlobalLinkTransform(leg.tip_link_).linear().col(2);
          const Eigen::Vector3d &normal = node->surface_normals_[i];
          const double angle = std::acos(direction.dot(-normal));

          if (angle < angles::from_degrees(30))
            continue;
        }
      }
    }

    // More than one contact is already broken, we can't move two legs at once.
    if (broken_index >= 0)
      return false;

    broken_index = i;
  }

  // ROS_INFO_STREAM(__LINE__);

  state_.update();

  // Check if the IK-generated soln is in self-collision.
  // collision_detection::CollisionRequest req;
  // collision_detection::CollisionResult res;

  // context_->getPlanningScene()->checkSelfCollision(req, res, state_);

  // if (res.collision)
  //   return false;

  // ROS_INFO_STREAM(__LINE__);

  if (broken_index >= 0)
  {
    // Contacts contacts(context_);
    // contacts.in_contact_.set();

    // TODO ensure stability when broken not set.
    // contacts.in_contact_[broken] = false;

    // ROS_INFO_STREAM(__LINE__);

    if (!createContact(body_tf, broken_index, node))
      return false;
  }
  else
  {
    // ROS_INFO_STREAM(__LINE__);
    // TODO check stability
    // state_.update();

    // Contacts contacts(context_);
    // contacts.in_contact_.set();

    // if (!stability->isStable(state_, contacts))
    //   valid = false;
  }

  return true;
}

bool Planner::createContact(const Eigen::Isometry3d &body_tf, int leg_index, Node *node)
{
  const auto &leg = context_->getLegModels()[leg_index];

  // if (!node->leg_configs_[leg_index])
    node->leg_configs_[leg_index].reset(new LegConfigs(leg_configs_->generateLegConfigs(body_tf, leg)));

  const auto &configs = *node->leg_configs_[leg_index];

  const auto &contacts = configs.contacts_;
  const auto &roadmap = leg.cdrm_->roadmap_;

  // Ensure there's at least one contact.
  if (contacts.empty())
    return false;

  // Sort by heuristic.
  const auto *stability = context_->getStabilityChecker();

  // TODO check contacts are reachable through the roadmap from prev

  for (std::size_t candidate_index = 0; candidate_index < contacts.size(); ++candidate_index)
  {
    auto candidate = contacts[candidate_index];
    state_.setJointPositions(context_->getBodyJoint(), body_tf);
    state_.setJointGroupPositions(leg.jmg_, roadmap[candidate].q_);
    state_.update();

    state_.setJointGroupPositions(leg.jmg_, roadmap[candidate].q_);

    all_contacts_.push_back(state_.getGlobalLinkTransform(leg.tip_link_).translation());

    // const auto tip_tf = state_.getGlobalLinkTransform(leg.tip_link_).translation();

    // if (std::abs(tip_tf.y()) > 0.2)
    // {
    //   ROS_INFO_STREAM("sad face");
    // }

    Contacts contacts;

    for (int i = 0; i < context_->getNumLegs(); ++i)
    {
      contacts.contacts_.push_back(i);
      // TODO
      // contacts.normals_.push_back();
    }

    contacts.normals_ = node->surface_normals_;
    contacts.normals_[leg_index] = configs.normals_[candidate_index];

    if (!stability->isStable(state_, contacts))
      continue;

    collision_detection::CollisionRequest req;
    req.group_name = leg.jmg_->getName();
    collision_detection::CollisionResult res;

    context_->getPlanningScene()->checkSelfCollision(req, res, state_);

    if (res.collision)
      continue;

    if (!offsetContact(state_, leg, configs.normals_[candidate_index], 0.001))
      continue;

    node->surface_normals_[leg_index] = configs.normals_[candidate_index];

    return true;
  }

  return false;
}

// bool Planner::repositionContact()

void Planner::createTrajectory(const std::vector<const Node *> &nodes)
{
  state_ = *context_->getStartState();
  trajectory_.reset(new robot_trajectory::RobotTrajectory(context_->getRobotModel(), context_->getGroupName()));
  trajectory_surface_normals_.clear();

  for (const auto *node : nodes)
  {
    if (node->preceding_config_.size() > 0)
    {
      state_.setJointGroupPositions(context_->getGroup(), node->preceding_config_);
      state_.update();

      trajectory_->addSuffixWayPoint(state_, 0);
      trajectory_surface_normals_.push_back(node->preceding_surface_normals_);
    }

    state_.setJointGroupPositions(context_->getGroup(), node->config_);
    state_.update();

    if (trajectory_->getWayPointCount() > 0)
    {
      auto intermediate = trajectory_->getLastWayPoint();

      // Add an intermediate state if we need it.
      if (!state_.getJointTransform(context_->getBodyJoint()).isApprox(intermediate.getJointTransform(context_->getBodyJoint())))
      {
        for (const auto &leg : context_->getLegModels())
        {
          if (!intermediate.setFromIK(leg.jmg_, state_.getGlobalLinkTransform(leg.tip_link_)))
            ROS_WARN("Unexpected leg IK failure");
        }

        trajectory_->addSuffixWayPoint(intermediate, 0);
        trajectory_surface_normals_.push_back(node->surface_normals_);
      }
    }

    trajectory_->addSuffixWayPoint(state_, 0);
    trajectory_surface_normals_.push_back(node->surface_normals_);
  }
}

bool Planner::offsetContact(moveit::core::RobotState &state, const LegModel &leg,
                            const Eigen::Vector3d &surface_normal, double step) const
{
  Eigen::Isometry3d original = state.getGlobalLinkTransform(leg.tip_link_);

  collision_detection::CollisionRequest req;
  req.group_name = leg.jmg_->getName();

  for (double offset = 0; offset < 0.015; offset += step)
  {
    const Eigen::Translation3d trans(surface_normal * offset);

    if (!state.setFromIK(leg.jmg_, original * trans))
      return false;

    // ROS_INFO_STREAM(offset);

    collision_detection::CollisionResult res;
    context_->getPlanningScene()->checkCollision(req, res, state);

    if (!res.collision)
    {
      // ROS_INFO_STREAM("offs");
      return true;
    }
  }

  return false;
}
}
