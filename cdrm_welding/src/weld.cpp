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
    tf.linear().col(0) = (positions_[i] - positions_[i - 1]).normalized();
  else
    tf.linear().col(0) = (positions_[i + 1] - positions_[i]).normalized();

  // Z points in the weld direction.
  tf.linear().col(2) = directions_[i].normalized();

  // Y is orthogonal.
  tf.linear().col(1) = tf.linear().col(0).cross(tf.linear().col(2)).normalized();

  // Also re-calculate X to make sure mit matches.
  tf.linear().col(0) = tf.linear().col(1).cross(tf.linear().col(2)).normalized();

  return tf;
}

Eigen::Isometry3d Weld::getTransform(double t) const
{
  if (t <= 0.0) {
    return getTargetTransform(0);
  }

  if (t >= 1.0) {
    return getTargetTransform(positions_.size() - 1);
  }

  double dist = t * getLength();
  double upto = 0.0;

  std::size_t i;

  for (i = 1; i < positions_.size(); ++i) {
    double seg = (positions_[i] - positions_[i - 1]).norm();

    if (upto + seg >= dist) {
      Eigen::Isometry3d a = getTargetTransform(i - 1);
      Eigen::Isometry3d b = getTargetTransform(i);

      // Interpolate the two transforms.
      const double between = (dist - upto) / seg;

      Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
      res.translation() = a.translation() + (b.translation() - a.translation()) * between;

      Eigen::Quaterniond qa(a.linear());
      Eigen::Quaterniond qb(b.linear());
      res.linear() = qa.slerp(between, qb).toRotationMatrix();

      return res;
    }

    upto += seg;
  }

  return getTargetTransform(positions_.size() - 1);
}
}
