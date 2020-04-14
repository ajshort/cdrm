#pragma once

#include <Eigen/Geometry>

#include <string>

namespace cdrm
{
/**
 * Linterp between two TFs.
 */
Eigen::Isometry3d interpolateTfs(const Eigen::Isometry3d &from, const Eigen::Isometry3d &to, double t);

/**
 * Gets the ROS home directory.
 */
std::string getRosHome();
}
