#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/functional/hash/hash.hpp>

#include <unordered_map>

namespace cdrm_legged
{
struct TransformHash
{
  std::size_t operator()(const Eigen::Isometry3d &tf) const
  {
    std::size_t seed = 0;

    for (std::size_t i = 0; i < 3; ++i)
      boost::hash_combine(seed, tf.translation()(i));

    for (Eigen::Index i = 0; i < tf.linear().size(); ++i)
      boost::hash_combine(seed, *(tf.linear().data() + i));

    return seed;
  }
};

struct TransformApproxEqual
{
  bool operator()(const Eigen::Isometry3d &a, const Eigen::Isometry3d &b) const
  {
    return a.isApprox(b);
  }
};

template <typename V>
using unordered_map_Isometry3d = std::unordered_map<Eigen::Isometry3d, V, TransformHash, TransformApproxEqual>;
}
