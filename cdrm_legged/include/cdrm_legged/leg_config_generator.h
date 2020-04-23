#pragma once

#include <cdrm_legged/disjoint_sets.h>
#include <cdrm_legged/leg_configs.h>
#include <cdrm_legged/moveit_forward.h>

#include <boost/dynamic_bitset.hpp>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <Eigen/Geometry>

#include <set>
#include <vector>

namespace cdrm_legged
{
class LegModel;
class PlanningContext;

/**
 * Generates leg configs for each leg given a body pose.
 */
class LegConfigGenerator
{
public:
  /**
   * A heuristic to order contacts.
   */
  using FootholdHeuristic = std::function<bool(const LegModel &leg, unsigned int a, double contact_angle_a,
                                               unsigned int b, double contact_angle_b)>;

  explicit LegConfigGenerator(const planning_scene::PlanningSceneConstPtr &scene);

  /**
   * Generates configs for leg number @a i.
   */
  LegConfigs generateLegConfigs(const Eigen::Isometry3d &body_tf, const LegModel &leg) const;

private:
  const planning_scene::PlanningSceneConstPtr planning_scene_;
  FootholdHeuristic heuristic_;
};
}
