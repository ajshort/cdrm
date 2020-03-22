#include "example_quadruped_leg_kinematics/leg_kinematics_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <urdf/model.h>

#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

namespace example_quadruped_leg_kinematics
{
static KDL::Frame poseUrdfToKdl(const urdf::Pose &p)
{
  KDL::Rotation r = KDL::Rotation::Quaternion(p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w);
  KDL::Vector v(p.position.x, p.position.y, p.position.z);

  return KDL::Frame(r, v);
}

bool LegKinematicsPlugin::initialize(const std::string &robot_description, const std::string &group_name,
                                     const std::string &base_frame, const std::string &tip_frame,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  // Load the robot model.
  urdf::Model urdf_model;

  if (!urdf_model.initParam(robot_description))
    return false;

  // Traverse from the tip to the base frame.
  auto link = urdf_model.getLink(tip_frame);

  while (link && link->name != base_frame)
  {
    auto joint = link->parent_joint;
    auto origin_tf = poseUrdfToKdl(joint->parent_to_joint_origin_transform);

    if (joint && joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_names_.push_back(joint->name);

      if (joint->safety)
        joint_limits_.push_back(std::make_pair(joint->safety->soft_lower_limit, joint->safety->soft_upper_limit));
      else
        joint_limits_.push_back(std::make_pair(joint->limits->lower, joint->limits->upper));
    }

    link_names_.push_back(link->name);
    link_lengths_.push_back(origin_tf.p.Norm());

    link = link->getParent();

    if (link->name == base_frame)
      base_origin_inv_ = origin_tf.Inverse();
  }

  // Reverse the data to get it from base to tip.
  std::reverse(link_names_.begin(), link_names_.end());
  std::reverse(link_lengths_.begin(), link_lengths_.end());
  std::reverse(joint_names_.begin(), joint_names_.end());
  std::reverse(joint_limits_.begin(), joint_limits_.end());

  return joint_names_.size() == 3;
}

const std::vector<std::string> &LegKinematicsPlugin::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string> &LegKinematicsPlugin::getLinkNames() const
{
  return link_names_;
}

bool LegKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                        const std::vector<double> &joint_angles,
                                        std::vector<geometry_msgs::Pose> &poses) const
{
  return false;
}

bool LegKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                           double timeout, const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution, const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  error_code.val = error_code.NO_IK_SOLUTION;

  // Get the target point in the leg frame.
  KDL::Frame ik_frame(KDL::Vector(ik_pose.position.x, ik_pose.position.y, ik_pose.position.z));
  KDL::Frame target = base_origin_inv_ * ik_frame;

  // Use IKFast to get the solutions.
  ikfast::IkSolutionList<IkReal> solutions;

  if (!ComputeIk(target.p.data, target.M.data, nullptr, solutions))
    return false;

  // Get the solution closest to the seed state.
  std::vector<std::vector<double>> candidates;

  for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
  {
    std::vector<double> candidate;
    solutions.GetSolution(i).GetSolution(candidate, std::vector<double>());
    candidates.push_back(candidate);
  }

  std::sort(candidates.begin(), candidates.end(), [&](const std::vector<double> &a, const std::vector<double> &b) {
    double a_dist = 0, b_dist = 0;

    for (unsigned int i = 0; i < 3; ++i)
    {
      a_dist += std::pow(ik_seed_state[i] - a[i], 2);
      b_dist += std::pow(ik_seed_state[i] - b[i], 2);
    }

    return a_dist < b_dist;
  });

  if (!candidates.empty())
  {
    error_code.val = error_code.SUCCESS;
    solution = std::move(candidates.front());

    return true;
  }

  return false;
}
}

PLUGINLIB_EXPORT_CLASS(example_quadruped_leg_kinematics::LegKinematicsPlugin, kinematics::KinematicsBase);
