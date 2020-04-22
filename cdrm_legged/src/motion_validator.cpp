#include <cdrm_legged/motion_validator.h>

#include <cdrm_legged/conversions.h>
#include <cdrm_legged/leg_config_generator.h>
#include <cdrm_legged/leg_model.h>
#include <cdrm_legged/planning_context.h>

#include <ros/console.h>

namespace mc = moveit::core;
namespace ob = ompl::base;

namespace cdrm_legged
{
MotionValidator::MotionValidator(const PlanningContext *context)
  : ob::MotionValidator(context->getSpaceInformation())
  , context_(context)
{
}

MotionValidator::~MotionValidator()
{
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b) const
{
  return true;
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b,
                                  std::pair<ob::State *, double> &last_valid) const
{
  throw "not implemented";
}
}
