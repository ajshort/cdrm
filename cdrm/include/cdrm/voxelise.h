#pragma once

#include <Eigen/Geometry>

namespace moveit
{
namespace core
{
class LinkModel;
class RobotState;
}
}

namespace shapes
{
class Mesh;
}

namespace cdrm
{
/**
 * Invoked when a voxel is generated.
 */
using VoxelCallback = std::function<void(const Eigen::Vector3d &p, const Eigen::Vector3d &n)>;

/**
 * Voxelises a mesh at a specified resolution.
 */
void voxelise(const shapes::Mesh &mesh, double resolution, const VoxelCallback &callback,
              const Eigen::Isometry3d &tf = Eigen::Isometry3d::Identity(), const Eigen::Vector3d *min_bound = nullptr,
              const Eigen::Vector3d *max_bound = nullptr);

/**
 * Voxelises all meshes in a joint model group.
 */
void voxelise(const moveit::core::RobotState &state, const std::vector<const moveit::core::LinkModel *> &links,
              double resolution, const VoxelCallback &callback,
              const Eigen::Isometry3d &tf = Eigen::Isometry3d::Identity());
}
