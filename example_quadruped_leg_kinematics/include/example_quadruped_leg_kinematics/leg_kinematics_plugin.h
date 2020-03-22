#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <kdl/frames.hpp>

namespace example_quadruped_leg_kinematics
{
/// Implements forwards and inverse kinematics for a single leg.
class LegKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool initialize(const std::string &robot_description,
                  const std::string &group_name,
                  const std::string &base_frame,
                  const std::string &tip_frame,
                  double search_discretization) override;

  const std::vector<std::string> &getJointNames() const override;
  const std::vector<std::string> &getLinkNames() const override;

  bool getPositionFK(const std::vector<std::string> &link_names,
                     const std::vector<double> &joint_angles,
                     std::vector<geometry_msgs::Pose> &poses) const override;

  bool getPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      std::vector<double> &solution,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override
  {
    return searchPositionIK(ik_pose, ik_seed_state, -1, solution, error_code, options);
  }

  bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      double timeout,
      std::vector<double> &solution,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override
  {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, 0, error_code);
  }

  bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      double timeout,
      const std::vector<double> &consistency_limits,
      std::vector<double> &solution,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override
  {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, 0, error_code, options);
  }

  bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      double timeout,
      std::vector<double> &solution,
      const IKCallbackFn &solution_callback,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override
  {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, std::vector<double>(), solution, solution_callback,
                            error_code, options);
  }

  bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      double timeout,
      const std::vector<double> &consistency_limits,
      std::vector<double> &solution,
      const IKCallbackFn &solution_callback,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<double> link_lengths_;
  std::vector<std::pair<double, double>> joint_limits_;

  KDL::Frame base_origin_inv_;
};
}
