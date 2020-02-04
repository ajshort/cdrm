#include <cdrm_welding/welding_cdrm_generator.h>

#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>

#include <moveit/robot_model/robot_model.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ros/console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace cdrm_welding
{
WeldingCdrmGenerator::WeldingCdrmGenerator(const moveit::core::RobotModelConstPtr &robot_model,
                                           cdrm_welding_msgs::GenerateWeldingCdrmFeedback &feedback,
                                           cdrm_welding_msgs::GenerateWeldingCdrmResult &result)
  : robot_model_(robot_model), feedback_(feedback), result_(result)
{
}

WeldingCdrmGenerator::~WeldingCdrmGenerator()
{
}

void WeldingCdrmGenerator::generate(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal)
{
  goal_ = goal;

  generateNozzleCdrm();
}

void WeldingCdrmGenerator::generateNozzleCdrm()
{
  // Get the joint and link models.
  const auto *group = robot_model_->getJointModelGroup(goal_->nozzle_group_name);

  if (!group)
  {
    ROS_ERROR("Could not find the nozzle joint  model group '%s'", goal_->nozzle_group_name.c_str());
    return;
  }

  ROS_INFO("Generating CDRM for nozzle...");

  nozzle_cdrm_.reset(new cdrm::Cdrm(goal_->nozzle_resolution));

  ob::StateSpacePtr state_space(new ob::RealVectorStateSpace);
  state_space->as<ob::RealVectorStateSpace>()->addDimension("rx", goal_->rx.min, goal_->rx.max);
  state_space->as<ob::RealVectorStateSpace>()->addDimension("ry", goal_->ry.min, goal_->ry.max);
  state_space->as<ob::RealVectorStateSpace>()->addDimension("ctwd", goal_->ctwd.min, goal_->ctwd.max);

  ob::SpaceInformationPtr space_info(new ob::SpaceInformation(ob::StateSpacePtr(state_space)));
  space_info->setStateValidityChecker(std::bind(&WeldingCdrmGenerator::isNozzleStateValid, this, std::placeholders::_1));
  space_info->setup();

  ROS_INFO("Generating nozzle PRM...");

  og::PRM prm(space_info);
  prm.setMaxNearestNeighbors(goal_->roadmap_k);
  prm.setProblemDefinition(ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info)));

  ob::PlannerTerminationCondition termination([&, this] { return prm.milestoneCount() >= goal_->roadmap_size; });
  prm.growRoadmap(termination);

  ROS_INFO("Generated nozzle PRM, generating W-space mapping...");

  ob::PlannerData planner_data(space_info);
  prm.getPlannerData(planner_data);

  std::vector<cdrm::VertexDescriptor> vertices(planner_data.numVertices());

  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    const auto *state = planner_data.getVertex(i).getState();
    // vertices[i] = addVertex(state);

    std::vector<unsigned int> edges;
    planner_data.getEdges(i, edges);

    for (const auto &j : edges)
    {
      // if (j < i)
      //   addEdge(vertices[j], vertices[i]);
    }
  }
}

bool WeldingCdrmGenerator::isNozzleStateValid(const ompl::base::State *state) const
{
}
}
