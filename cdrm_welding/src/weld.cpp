#include <cdrm_welding/weld.h>

namespace cdrm_welding
{
double Weld::getLength() const
{
  double len = 0.0;

  for (std::size_t i = 1; i < positions_.size(); ++i)
    len += (positions_[i - 1] - positions_[i]).norm();

  return len;
}

void Weld::addTarget(const Eigen::Vector3d &position, const Eigen::Vector3d &direction)
{
  positions_.push_back(position);
  directions_.push_back(direction);
}

Eigen::Isometry3d Weld::getTargetTransform(std::size_t i) const
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

  tf.translation() = positions_[i];

  // X points along the weld.
  if (i == positions_.size() - 1)
    tf.linear.col(0) = (positions_[i] - positions_[i - 1]).normalized();
  else
    tf.linear.col(0) = (positions_[i + 1] - positions_[i]).normalized();

  // Z points in the weld direction.
  tf.linear().col(2) = directions_[i];

  // Y is orthogonal.
  tf.linear().col(1) = tf.linear().col(0).cross(tf.linear().col(2));

  return tf;
}
}
