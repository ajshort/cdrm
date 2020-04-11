#include <cdrm_welding/kinematic_utils.h>

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>

namespace cdrm_welding
{
void applyNozzleConfigurationToState(moveit::core::RobotState &state,
                                     const Eigen::VectorXd &q,
                                     const moveit::core::LinkModel *ee)
{
  // The three dimensions are Rx, Ry, CTWD. We want to place the nozzle at origin and then
  // transform it by these amounts and voxelise it.
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitX()));
  tf.rotate(Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitY()));
  tf.translation() -= q(2) * tf.linear().col(2);

  state.updateStateWithLinkAt(ee, tf, true);
}

void applyToolConfigurationToState(moveit::core::RobotState &state,
                                   const Eigen::VectorXd &q,
                                   const moveit::core::LinkModel *ee)
{
  // Dimensions are Rx, Ry, Rz and CTWD.
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitX()));
  tf.rotate(Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitY()));
  tf.rotate(Eigen::AngleAxisd(q(2), Eigen::Vector3d::UnitZ()));
  tf.translation() -= q(3) * tf.linear().col(2);

  state.updateStateWithLinkAt(ee, tf, true);
}
}
