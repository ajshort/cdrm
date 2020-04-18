#include "cdrm_legged/state_validity_checker.h"

#include "cdrm_legged/conversions.h"
#include "cdrm_legged/leg_model.h"
#include "cdrm_legged/planning_context.h"

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
  const auto value = stateOmplToMoveIt(state);

  state_.setJointPositions(context_->getBodyJoint(), value.data());
  state_.update();

  return isWithinLegsWorkspace() && !isBodyInCollision();
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
}
