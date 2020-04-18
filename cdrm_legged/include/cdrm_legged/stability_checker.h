#pragma once

#include <cdrm_legged/moveit_forward.h>

#include <eigen_stl_containers/eigen_stl_map_container.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <map>
#include <vector>

namespace cdrm_legged
{
class LegModel;
class PlanningContext;

/**
 * Contains information about the current contacts.
 */
struct Contacts
{
  std::vector<unsigned int> contacts_;
  EigenSTL::vector_Vector3d normals_;
};

using map_string_Isometry3d = std::map<std::string,
                                       Eigen::Affine3d,
                                       std::less<std::string>,
                                       Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>>;

/**
 * Checks if a robot state is statically stable.
 */
class StabilityChecker
{
public:
  StabilityChecker(const PlanningContext *context);
  virtual ~StabilityChecker();

  /**
   * Returns true if @a state is stable with @a contacts in contact with the environment.
   */
  virtual bool isStable(const moveit::core::RobotState &state, const Contacts &contacts) const = 0;

protected:
  Eigen::Vector3d getCom(const moveit::core::RobotState &state) const;
  double getRobotMass() const { return robot_mass_; }

  const PlanningContext *context_;
private:
  double robot_mass_ = 0;

  std::map<std::string, double> link_mass_;
  map_string_Isometry3d link_mass_origins_;
};

/**
 * Simple support polygon approximation.
 */
class SupportPolygonStability : public StabilityChecker
{
public:
  using StabilityChecker::StabilityChecker;

  bool isStable(const moveit::core::RobotState &state, const Contacts &contacts) const override;
};
}
