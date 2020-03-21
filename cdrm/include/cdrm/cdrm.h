#pragma once

#include <cdrm/roadmap.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <unordered_map>

namespace cdrm
{
/**
 * A key which corresponds to a CDRM W-space cell.
 */
using Key = std::array<std::int16_t, 3>;

/**
 * The main Contact Dynamic Roadmap (CDRM) data structure.
 */
class Cdrm
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cdrm(double resolution = 0.01);

  Key pointToKey(const Eigen::Vector3d &p) const;
  Eigen::Vector3d keyToPoint(const Key &k) const;

  bool save(const std::string &filename) const;
  bool load(const std::string &filename);

  double resolution_;
  Roadmap roadmap_;

  std::multimap<Key, VertexDescriptor> colliding_vertices_;
  std::multimap<Key, EdgeDescriptor> colliding_edges_;
  std::multimap<Key, VertexDescriptor> contacts_;

  // An AABB of all voxelisations.
  Eigen::AlignedBox3d aabb_;

  Eigen::Vector3d workspace_min_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d workspace_max_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

  double min_contact_distance_ = std::numeric_limits<double>::max();
  double max_contact_distance_ = 0;
};
}
