#pragma once

#include <cdrm_welding_msgs/GenerateWeldingCdrmAction.h>

#include <moveit/macros/class_forward.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);
}
}

namespace ompl
{
namespace base
{
class State;
}
}

namespace cdrm
{
class Cdrm;
}

namespace cdrm_welding
{
/**
 * Generates a composite CDRM suitable for planning welds.
 */
class WeldingCdrmGenerator
{
public:
  WeldingCdrmGenerator(const moveit::core::RobotModelConstPtr &robot_model,
                       cdrm_welding_msgs::GenerateWeldingCdrmFeedback &feedback,
                       cdrm_welding_msgs::GenerateWeldingCdrmResult &result);

  ~WeldingCdrmGenerator();

  void generate(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal);

private:
  void generateNozzleCdrm();

  bool isNozzleStateValid(const ompl::base::State *state) const;

  moveit::core::RobotModelConstPtr robot_model_;

  cdrm_welding_msgs::GenerateWeldingCdrmFeedback feedback_;
  cdrm_welding_msgs::GenerateWeldingCdrmResult result_;
  cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr goal_;

  std::unique_ptr<cdrm::Cdrm> nozzle_cdrm_;
};
}
