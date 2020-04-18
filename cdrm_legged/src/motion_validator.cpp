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
  , leg_config_generator_(new LegConfigGenerator(context->getPlanningScene()))
{
}

MotionValidator::~MotionValidator()
{
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b) const
{
  const auto sa = stateOmplToMoveIt(a);
  const auto sb = stateOmplToMoveIt(b);

  const auto tfa = transformOmplToEigen(a);
  const auto tfb = transformOmplToEigen(b);

  if (!hasValidLegConfigs(sa, tfa) || !hasValidLegConfigs(sb, tfb))
    return false;

  return true;
}

bool MotionValidator::checkMotion(const ob::State *a, const ob::State *b,
                                  std::pair<ob::State *, double> &last_valid) const
{
  throw "not implemented";
}

bool MotionValidator::hasValidLegConfigs(const std::array<double, 7> &s, const Eigen::Isometry3d &tf) const
{
  auto &configs = leg_configs_[s];
  const auto &legs = context_->getLegModels();

  for (std::size_t i = 0; i < legs.size(); ++i)
  {
    if (configs.size() < i - 1)
      configs.push_back(leg_config_generator_->generateLegConfigs(tf, legs[i]));

    if (configs[i].contacts_.empty())
      return false;
  }

  return true;
}
}
