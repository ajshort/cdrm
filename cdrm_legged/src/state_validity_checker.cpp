#include "cdrm_legged/state_validity_checker.h"

#include "cdrm_legged/conversions.h"
#include "cdrm_legged/leg_model.h"
#include "cdrm_legged/planning_context.h"
#include <cdrm_legged/leg_config_generator.h>

#include <cdrm/cdrm.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

namespace cdrm_legged
{
StateValidityChecker::StateValidityChecker(const PlanningContext *context)
  : ompl::base::StateValidityChecker(context->getSpaceInformation())
  , context_(context)
  , body_acm_(context_->getPlanningScene()->getAllowedCollisionMatrix())
  , state_(context_->getRobotModel())
  , leg_config_generator_(new LegConfigGenerator(context->getPlanningScene()))
{
  // Set up the ACM to only check collisions with body links.
  const auto *body_link = context_->getBodyJoint()->getChildLinkModel();
  const auto &fixed_tfs = body_link->getAssociatedFixedTransforms();

  for (const auto *link : context_->getRobotModel()->getLinkModelsWithCollisionGeometry())
  {
    if (link != body_link && fixed_tfs.find(link) == fixed_tfs.end())
      body_acm_.setDefaultEntry(link->getName(), true);
  }

  // Work state.
  state_.setToDefaultValues();
  state_.update();
}

StateValidityChecker::~StateValidityChecker()
{
}

bool StateValidityChecker::isValid(const ompl::base::State *state) const
{
  const Eigen::Isometry3d body_tf = transformOmplToEigen(state);

  state_.setJointPositions(context_->getBodyJoint(), body_tf);
  state_.update();

  if (!isWithinLegsWorkspace())
    return false;

  if (isBodyInCollision())
    return false;

  if (!allLegsHaveConfigs(body_tf))
    return false;

  return true;
}

// TODO this should check if each of the legs is capable of reaching the environment.
bool StateValidityChecker::isWithinLegsWorkspace() const
{
  return true;
}

bool StateValidityChecker::isBodyInCollision() const
{
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  context_->getPlanningScene()->checkCollision(req, res, state_, body_acm_);

  return res.collision;
}

bool StateValidityChecker::allLegsHaveConfigs(const Eigen::Isometry3d &body_tf) const
{
  const auto &leg_models = context_->getLegModels();
  auto &leg_configs = state_leg_configs_[body_tf];

  if (leg_configs.empty())
  {
    leg_configs.resize(leg_models.size());

    for (std::size_t i = 0; i < leg_models.size(); ++i)
      leg_configs[i] = leg_config_generator_->generateLegConfigs(body_tf, leg_models[i]);
  }

  return std::all_of(leg_configs.begin(), leg_configs.end(), [](const LegConfigs &lc)
  {
    return !lc.contacts_.empty();
  });
}
}
