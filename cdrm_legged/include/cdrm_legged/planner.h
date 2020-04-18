#pragma once

#include <cdrm_legged/moveit_forward.h>

#include <Eigen/Geometry>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <moveit/robot_state/robot_state.h>
#include <ompl/base/Planner.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>

#include <memory>
#include <vector>

namespace cdrm_legged
{
class LegConfigGenerator;
class LegModel;
class PlanningContext;

/**
 * The main motion planner implementation.
 */
// TODO check if we can use a standard OMPL implementation and then use RRT-connect.
class Planner : public ompl::base::Planner
{
public:
  explicit Planner(const PlanningContext *context);
  ~Planner();

  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

  void setup() override;
  void clear() override;

  double getGoalBias() const
  {
    return goal_bias_;
  }

  void setGoalBias(double goal_bias)
  {
    goal_bias_ = goal_bias;
  }

  double getRange() const
  {
    return range_;
  }

  void setRange(double range)
  {
    range_ = range;
  }

  const robot_trajectory::RobotTrajectoryPtr &getRobotTrajectory() const
  {
    return trajectory_;
  }

  const EigenSTL::vector_Vector3d &getSurfaceNormals(std::size_t trajectory_index) const
  {
    return trajectory_surface_normals_[trajectory_index];
  }

  const EigenSTL::vector_Vector3d &getAllCreatedContacts() const
  {
    return all_contacts_;
  }

private:
  struct Node;

  bool checkMotion(Node *from, Node *to);
  bool checkTransition(const Eigen::VectorXd &from_config, const Eigen::Isometry3d &body_tf, Node *node);
  bool createContact(const Eigen::Isometry3d &body_tf, int leg_index, Node *node);
  bool repositionContact();

  void createTrajectory(const std::vector<const Node *> &nodes);

  bool offsetContact(moveit::core::RobotState &state, const LegModel &leg,
                     const Eigen::Vector3d &surface_normal, double step) const;

  const PlanningContext *context_;

  double goal_bias_ = 0.05;
  double range_ = 1;

  ompl::base::StateSamplerPtr sampler_;
  ompl::RNG rng_;

  std::unique_ptr<ompl::NearestNeighbors<Node *>> nn_;
  std::unique_ptr<LegConfigGenerator> leg_configs_;

  moveit::core::RobotState state_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;

  std::vector<EigenSTL::vector_Vector3d> trajectory_surface_normals_;
  EigenSTL::vector_Vector3d all_contacts_;
};
}
