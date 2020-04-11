#pragma once

#include <Eigen/Core>

namespace moveit
{
namespace core
{
class LinkModel;
class RobotState;
}
}

namespace cdrm_welding
{
void applyNozzleConfigurationToState(moveit::core::RobotState &state,
                                     const Eigen::VectorXd &q,
                                     const moveit::core::LinkModel *ee);

void applyToolConfigurationToState(moveit::core::RobotState &state,
                                   const Eigen::VectorXd &q,
                                   const moveit::core::LinkModel *ee);
}
