#pragma once

#include <cdrm/roadmap.h>
#include <cdrm_welding_msgs/GenerateWeldingCdrmAction.h>
#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(JointModelGroup);
MOVEIT_CLASS_FORWARD(LinkModel);
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

namespace cdrm_welding
{
class WeldingCdrm;

/**
 * Generates a composite CDRM suitable for planning welds.
 */
class WeldingCdrmGenerator
{
public:
  using CancelledFn = std::function<bool()>;

  WeldingCdrmGenerator(const moveit::core::RobotModelConstPtr &robot_model,
                       cdrm_welding_msgs::GenerateWeldingCdrmFeedback &feedback,
                       cdrm_welding_msgs::GenerateWeldingCdrmResult &result);

  ~WeldingCdrmGenerator();

  void generate(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal, const CancelledFn &is_cancelled);

private:
  void generateNozzleCdrm(const CancelledFn &is_cancelled);
  void generateRobotCdrm(const CancelledFn &is_cancelled);

  bool isNozzleStateValid(const ompl::base::State *state) const;

  cdrm::VertexDescriptor addNozzleVertex(const ompl::base::State *s);
  cdrm::EdgeDescriptor addNozzleEdge(const cdrm::VertexDescriptor &a, const cdrm::VertexDescriptor &b);

  moveit::core::RobotModelConstPtr robot_model_;
  planning_scene::PlanningScene planning_scene_;

  cdrm_welding_msgs::GenerateWeldingCdrmFeedback feedback_;
  cdrm_welding_msgs::GenerateWeldingCdrmResult result_;
  cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr goal_;

  const moveit::core::JointModelGroup *end_effector_;
  const moveit::core::LinkModel *flange_link_;
  const moveit::core::LinkModel *nozzle_link_;
  std::unique_ptr<WeldingCdrm> cdrm_;
};
}
