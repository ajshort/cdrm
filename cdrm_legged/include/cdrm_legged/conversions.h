#pragma once

#include <cdrm_legged/ompl_forward.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>

namespace cdrm_legged
{
/**
 * Converts an SE(3) state to an Eigen transform.
 */
Eigen::Isometry3d transformOmplToEigen(const ompl::base::State *state);

/**
 * Converts an array of values to a tf.
 */
Eigen::Isometry3d transformMoveItToEigen(const std::array<double, 7> &s);

/**
 * Converts an SE(3) state to a moveit floating joint value.
 */
std::array<double, 7> stateOmplToMoveIt(const ompl::base::State *state);

/**
 * Converts an Eigen transform to an SE(3) state.
 */
void transformEigenToOmpl(const Eigen::Isometry3d &tf, ompl::base::State *state);
}
