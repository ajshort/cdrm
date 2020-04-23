#include <cdrm_legged/motion_validator.h>

#include <cdrm_legged/conversions.h>
#include <cdrm_legged/leg_config_generator.h>
#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planning_context.h>
#include <cdrm_legged/stability_checker.h>

#include <cdrm/cdrm.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>

namespace mc = moveit::core;
namespace ob = ompl::base;

namespace cdrm_legged
{
MotionValidator::MotionValidator(PlanningContext *context)
  : ob::MotionValidator(context->getSpaceInformation())
  , context_(context)
{
}

MotionValidator::~MotionValidator()
{
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b) const
{
  Eigen::Isometry3d body_from = transformOmplToEigen(a);
  Eigen::Isometry3d body_to = transformOmplToEigen(b);

  // Make sure the robot is collision free and has footholds at b.
  if (!si_->isValid(b))
    return false;

  const auto &leg_models = context_->getLegModels();

  // Do we have a reachable contacts entry for the from position? If we don't, it must be the start so everything
  // is reachable.
  if (!reachable_contacts_.count(body_from))
  {
    reachable_contacts_[body_from] = context_->generateLegConfigs(body_from);

    if (!generateValidStates(body_from))
      return false;
  }

  const std::vector<LegConfigs> &prev_contacts = reachable_contacts_.at(body_from);
  const std::vector<LegConfigs> &all_contacts = context_->generateLegConfigs(body_to);

  std::vector<LegConfigs> reachable_contacts(context_->getNumLegs());

  for (std::size_t leg = 0; leg < context_->getNumLegs(); ++leg)
  {
    const auto &leg_model = context_->getLegModels()[leg];
    const auto &from = prev_contacts[leg];
    const auto &all = all_contacts[leg];
    auto &reachable = reachable_contacts[leg];

    for (std::size_t i = 0; i < all.contacts_.size(); ++i)
    {
      // Get the closest configuration from a reachable contact from the candidate to a contact in the previous
      // state.
      double score = std::numeric_limits<double>::max();

      for (const auto &contact : from.contacts_)
      {
        const Eigen::Vector3d contact_translation = leg_model.getTipTransform(contact).translation();

        for (const auto &reachable : all.reachable_[i])
        {
          const Eigen::Vector3d reachable_translation = leg_model.getTipTransform(reachable).translation();
          const double dist = (contact_translation - reachable_translation).norm();

          if (dist < score)
            score = dist;
        }
      }

      // If it's close, we assume we can reach it.
      if (score <= 0.1)
      {
        reachable.contacts_.push_back(all.contacts_[i]);
        reachable.normals_.push_back(all.normals_[i]);
        reachable.reachable_.push_back(all.reachable_[i]);
      }
    }
  }

  // Check we have reachable contacts for all legs still.
  bool all_reachable = std::all_of(reachable_contacts.begin(), reachable_contacts.end(), [](const LegConfigs &lc)
  {
    return !lc.contacts_.empty();
  });

  if (!all_reachable)
    return false;

  reachable_contacts_[body_to] = std::move(reachable_contacts);

  return generateValidStates(body_to);
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b,
                                  std::pair<ob::State *, double> &last_valid) const
{
  throw "not implemented";
}

bool MotionValidator::generateValidStates(const Eigen::Isometry3d &body_tf) const
{
  moveit::core::RobotState robot_state = context_->getPlanningScene()->getCurrentState();
  robot_state.setJointPositions(context_->getBodyJoint(), body_tf);

  const auto &reachable_contacts = reachable_contacts_.at(body_tf);

  // Make sure we can generate states which are (a) stable with all combinations if only 3 legs (since we have to)
  // lift one up, and (b) free of self collision.
  auto &valid = valid_states_[body_tf];

  for (int attempt = 0; attempt < 100; ++attempt)
  {
    for (std::size_t leg = 0; leg < context_->getNumLegs(); ++leg)
    {
      const auto &leg_model = context_->getLegModels()[leg];
      const auto &candidates = reachable_contacts[leg].contacts_;
      const auto index = rng_.uniformInteger(0, candidates.size() - 1);
      robot_state.setJointGroupPositions(leg_model.jmg_, leg_model.cdrm_->getVertexConfig(candidates[index]));
    }

    robot_state.update();

    // Contacts contacts;

    // for (std::size_t leg = 0; leg < context_->getNumLegs(); ++leg)
    //   contacts.contacts_.push_back(leg);

    // if (!context_->getStabilityChecker()->isStable(robot_state, contacts))
    //   continue;

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    context_->getPlanningScene()->checkSelfCollision(req, res, robot_state);

    if (res.collision)
      continue;

    valid.push_back(robot_state);
  }

  return !valid.empty();
}
}
