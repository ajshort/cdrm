#include <cdrm_legged/conversions.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

namespace cdrm_legged
{
Eigen::Isometry3d transformOmplToEigen(const ompl::base::State *state)
{
  const auto *s = state->as<ompl::base::SE3StateSpace::StateType>();

  return Eigen::Translation3d(s->getX(), s->getY(), s->getZ()) *
         Eigen::Quaterniond(s->rotation().w, s->rotation().x, s->rotation().y, s->rotation().z);
}

Eigen::Isometry3d transformMoveItToEigen(const std::array<double, 7> &s)
{
  return Eigen::Translation3d(s[0], s[1], s[2]) * Eigen::Quaterniond(s[6], s[3], s[4], s[5]);
}

std::array<double, 7> stateOmplToMoveIt(const ompl::base::State *state)
{
  const auto *s = state->as<ompl::base::SE3StateSpace::StateType>();
  const auto &r = s->rotation();

  return { s->getX(), s->getY(), s->getZ(), r.x, r.y, r.z, r.w };
}

void transformEigenToOmpl(const Eigen::Isometry3d &tf, ompl::base::State *state)
{
  auto *s = state->as<ompl::base::SE3StateSpace::StateType>();
  const Eigen::Quaterniond quat(tf.linear());

  s->setXYZ(tf.translation().x(), tf.translation().y(), tf.translation().z());
  s->rotation().w = quat.w();
  s->rotation().x = quat.x();
  s->rotation().y = quat.y();
  s->rotation().z = quat.z();
}
}
