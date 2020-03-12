#pragma once

#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <Eigen/Geometry>

namespace cdrm_welding
{
struct Target
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position;
  Eigen::Vector3d direction;
};

/**
 * A weld constructed from targets.
 */
class Weld
{
public:
  double getLength() const;

  std::size_t getNumTargets() const { return positions_.size(); }

  void addTarget(const Eigen::Vector3d &position, const Eigen::Vector3d &direction);

  Eigen::Isometry3d getTargetTransform(std::size_t i) const;
  Eigen::Isometry3d getTransform(double t) const;

private:
  EigenSTL::vector_Vector3d positions_;
  EigenSTL::vector_Vector3d directions_;
};
}
