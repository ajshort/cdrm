#include <cdrm_welding/welding_cdrm_generator.h>

#include <cdrm_welding/welding_cdrm.h>

#include <cdrm/cdrm.h>
#include <cdrm/cdrm_serialisation.h>
#include <cdrm/voxelise.h>

#include <moveit/robot_model/robot_model.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ros/console.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <fstream>
#include <iostream>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace cdrm_welding
{
WeldingCdrmGenerator::WeldingCdrmGenerator(const moveit::core::RobotModelConstPtr &robot_model,
                                           cdrm_welding_msgs::GenerateWeldingCdrmFeedback &feedback,
                                           cdrm_welding_msgs::GenerateWeldingCdrmResult &result)
  : robot_model_(robot_model)
  , planning_scene_(robot_model_)
  , feedback_(feedback)
  , result_(result)
{
}

WeldingCdrmGenerator::~WeldingCdrmGenerator()
{
}

void WeldingCdrmGenerator::generate(const cdrm_welding_msgs::GenerateWeldingCdrmGoalConstPtr &goal,
                                    const CancelledFn &is_cancelled)
{
  goal_ = goal;
  cdrm_ = std::unique_ptr<WeldingCdrm>(new WeldingCdrm);

  ROS_INFO("Generating welding CDRM");

  generateNozzleCdrm(is_cancelled);

  // Save the information.
  ROS_INFO("Saving welding CDRM to '%s'...", goal->filename.c_str());

  if (!cdrm_->save(goal->filename))
    return;

  ROS_INFO("Finished generating welding CDRM");
}

void WeldingCdrmGenerator::generateNozzleCdrm(const CancelledFn &is_cancelled)
{
  end_effector_ = robot_model_->getEndEffector(goal_->end_effector_name);

  if (!end_effector_)
  {
    ROS_ERROR("Could not find the end effector group '%s'", goal_->end_effector_name.c_str());
    return;
  }

  // Get the flange link.
  const auto &parent = end_effector_->getEndEffectorParentGroup();
  const auto *parent_group = robot_model_->getJointModelGroup(parent.first);

  if (!parent_group)
  {
    ROS_ERROR("Could not find the end effector parent group '%s'", parent.first.c_str());
    return;
  }

  if (!(flange_link_ = parent_group->getLinkModel(parent.second)))
  {
    ROS_ERROR("Could not find the end effector parent link '%s'", parent.second.c_str());
    return;
  }

  // Get the nozzle link.
  if (!(nozzle_link_ = robot_model_->getLinkModel(goal_->nozzle_link_name)))
  {
    ROS_ERROR("Could not find the nozzle link '%s'", goal_->nozzle_link_name.c_str());
    return;
  }

  ROS_INFO("Generating CDRM for nozzle...");

  cdrm_->nozzle_cdrm_ = cdrm::Cdrm(goal_->nozzle_resolution);

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

  auto last_update = ros::Time::now();

  for (std::size_t i = 0; i < vertices.size(); ++i)
  {
    const auto *state = planner_data.getVertex(i).getState();
    vertices[i] = addNozzleVertex(state);

    std::vector<unsigned int> edges;
    planner_data.getEdges(i, edges);

    for (const auto &j : edges)
    {
      if (j < i)
        addNozzleEdge(vertices[j], vertices[i]);
    }

    if ((ros::Time::now() - last_update) > ros::Duration(5.0)) {
      ROS_INFO("Processed %lu / %lu CDRM vertices", i + 1, vertices.size());
      last_update = ros::Time::now();
    }

    if (is_cancelled())
      return;
  }

  ROS_INFO("Finished generating nozzle CDRM");
}

static void applyNozzleConfigurationToState(moveit::core::RobotState &state,
                                            const Eigen::VectorXd q,
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

cdrm::VertexDescriptor WeldingCdrmGenerator::addNozzleVertex(const ob::State *s)
{
  auto &nozzle_cdrm = cdrm_->nozzle_cdrm_;

  Eigen::VectorXd config(3);

  for (int i = 0; i < 3; ++i)
    config(i) = (*s->as<ob::RealVectorStateSpace::StateType>())[i];

  auto &robot_state = planning_scene_.getCurrentStateNonConst();
  applyNozzleConfigurationToState(robot_state, config, nozzle_link_);
  robot_state.update();

  // We use the flange as the contact position.
  Eigen::Vector3d contact = robot_state.getGlobalLinkTransform(flange_link_).translation();

  // We don't need to check for duplicates as each vertex descriptor is only processed once.
  const auto vertex = boost::add_vertex(cdrm::Vertex{config}, nozzle_cdrm.roadmap_);
  const auto contact_key = nozzle_cdrm.pointToKey(contact);
  nozzle_cdrm.contacts_.insert(std::make_pair(contact_key, vertex));

  // Update the workspace and contact bounds.
  const auto distance = contact.norm();

  nozzle_cdrm.min_contact_distance_ = std::min(nozzle_cdrm.min_contact_distance_, distance);
  nozzle_cdrm.max_contact_distance_ = std::max(nozzle_cdrm.max_contact_distance_, distance);

  nozzle_cdrm.workspace_min_ = contact.cwiseMin(nozzle_cdrm.workspace_min_);
  nozzle_cdrm.workspace_max_ = contact.cwiseMax(nozzle_cdrm.workspace_max_);

  // Voxelise the state.
  const auto callback = [this, &vertex, &nozzle_cdrm](const Eigen::Vector3d &p, const Eigen::Vector3d &n)
  {
    auto key = nozzle_cdrm.pointToKey(p);
    auto existing = nozzle_cdrm.colliding_vertices_.equal_range(key);
    bool found = false;

    for (auto it = existing.first; it != existing.second; ++it)
    {
      if (it->second == vertex)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      nozzle_cdrm.aabb_.extend(p);
      nozzle_cdrm.colliding_vertices_.insert({key, vertex});
    }
  };
  cdrm::voxelise(robot_state, { nozzle_link_ }, goal_->nozzle_resolution, callback, Eigen::Isometry3d());

  return vertex;
}

cdrm::EdgeDescriptor WeldingCdrmGenerator::addNozzleEdge(const cdrm::VertexDescriptor &a,
                                                         const cdrm::VertexDescriptor &b)
{
  auto &nozzle_cdrm = cdrm_->nozzle_cdrm_;

  const auto edge = boost::add_edge(a, b, nozzle_cdrm.roadmap_).first;

  moveit::core::RobotState sa(robot_model_);
  moveit::core::RobotState sb(robot_model_);

  // TODO
  return edge;

  sa.setToDefaultValues();
  applyNozzleConfigurationToState(sa, nozzle_cdrm.roadmap_[a].q_, nozzle_link_);
  sa.update();

  sb.setToDefaultValues();
  applyNozzleConfigurationToState(sb, nozzle_cdrm.roadmap_[b].q_, nozzle_link_);
  sb.update();

  // Get the displacement the two nozzle links positions.
  const Eigen::Isometry3d &ta = sa.getGlobalLinkTransform(nozzle_link_->getParentLinkModel());
  const Eigen::Isometry3d &tb = sb.getGlobalLinkTransform(nozzle_link_->getParentLinkModel());

  const double max_displacement = (ta.translation() - tb.translation()).norm();

  auto steps = static_cast<unsigned int>(max_displacement / goal_->nozzle_resolution);
  auto state = sa;

  const auto callback = [this, &edge, &nozzle_cdrm](const Eigen::Vector3d &p, const Eigen::Vector3d &n) {
    auto key = nozzle_cdrm.pointToKey(p);
    auto existing = nozzle_cdrm.colliding_edges_.equal_range(key);
    bool found = false;

    for (auto it = existing.first; it != existing.second; ++it)
    {
      if (it->second == edge)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      nozzle_cdrm.aabb_.extend(p);
      nozzle_cdrm.colliding_edges_.insert({key, edge});
    }
  };

  for (unsigned int i = 0; i <= steps; ++i)
  {
    sa.interpolate(sb, static_cast<double>(i) / steps, state, end_effector_);
    state.update();

    cdrm::voxelise(state, { nozzle_link_ }, goal_->nozzle_resolution, callback);
  }

  return edge;
}

bool WeldingCdrmGenerator::isNozzleStateValid(const ompl::base::State *state) const
{
  // Since we are only checking the nozzle link, it can't collide with anything - just
  // return true.
  return true;
}
}
