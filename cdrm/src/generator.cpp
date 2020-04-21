#include <cdrm/generator.h>

#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>

#include <cdrm_msgs/GenerateCdrmGoal.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ros/console.h>

#include <queue>
#include <utility>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace cdrm
{
Generator::Generator(const moveit::core::RobotModelConstPtr &robot_model)
    : robot_model_(robot_model), planning_scene_(robot_model)
{
}

Generator::~Generator()
{
}

class GeneratorStateSpace : public ob::RealVectorStateSpace
{
public:
  GeneratorStateSpace(const robot_model::RobotModelConstPtr &robot_model,
                      const moveit::core::JointModelGroup *joint_model_group)
    : robot_model_(robot_model)
    , joint_model_group_(joint_model_group)
    , tip_link_(joint_model_group_->getLinkModels().back())
  {
  }

  double distance(const ob::State *a, const ob::State *b) const override
  {
    moveit::core::RobotState sa(robot_model_);
    moveit::core::RobotState sb(robot_model_);

    sa.setToDefaultValues();
    sa.setJointGroupPositions(joint_model_group_, a->as<RealVectorStateSpace::StateType>()->values);
    sa.update();

    sb.setToDefaultValues();
    sb.setJointGroupPositions(joint_model_group_, b->as<RealVectorStateSpace::StateType>()->values);
    sb.update();

    const Eigen::Isometry3d &ta = sa.getGlobalLinkTransform(tip_link_);
    const Eigen::Isometry3d &tb = sb.getGlobalLinkTransform(tip_link_);
    return (ta.translation() - tb.translation()).norm();
  }

private:
  const robot_model::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup *joint_model_group_;
  const moveit::core::LinkModel *tip_link_;
};

std::unique_ptr<Cdrm> Generator::generate(CancelRequestedCallback is_canceled_callback)
{
  // Get the joint and link models.
  if (!(jmg_ = robot_model_->getJointModelGroup(goal_->group_name)))
  {
    ROS_ERROR("Could not find the joint model group '%s'", goal_->group_name.c_str());
    return nullptr;
  }

  link_models_ = jmg_->getLinkModels();

  if (!goal_->collide_tip_link)
    link_models_.pop_back();

  // Get the transform to the group origin.
  auto &robot_state = planning_scene_.getCurrentStateNonConst();
  robot_state.update();

  const Eigen::Isometry3d &origin_tf = robot_state.getGlobalLinkTransform(jmg_->getLinkModels().front());
  origin_tf_inv_ = origin_tf.inverse();

  // Set up the ACM to only check for self-collisions in the group.
  acm_ = planning_scene_.getAllowedCollisionMatrix();

  for (const auto &link_name : robot_model_->getLinkModelNamesWithCollisionGeometry())
  {
    if (!jmg_->hasLinkModel(link_name))
      acm_.setDefaultEntry(link_name, true);
  }

  ROS_INFO("Generating CDRM...");

  // Create the CDRM and generate the roadmap.
  cdrm_.reset(new Cdrm(goal_->resolution));

  ob::StateSpacePtr state_space(new GeneratorStateSpace(robot_model_, jmg_));

  for (const auto *bounds : jmg_->getActiveJointModelsBounds())
    state_space->as<ob::RealVectorStateSpace>()->addDimension((*bounds)[0].min_position_, (*bounds)[0].max_position_);

  ob::SpaceInformationPtr space_info(new ob::SpaceInformation(ob::StateSpacePtr(state_space)));
  space_info->setStateValidityChecker(std::bind(&Generator::isStateValid, this, std::placeholders::_1));
  space_info->setup();

  og::PRM prm(space_info);
  prm.setMaxNearestNeighbors(goal_->roadmap_k);
  prm.setProblemDefinition(ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info)));

  ob::PlannerTerminationCondition cancel_ptc(is_canceled_callback);
  ob::PlannerTerminationCondition size_ptc([&, this] {
    const auto num_milestones = prm.milestoneCount();
    updateProgress(0.5 * num_milestones / goal_->roadmap_size);
    return num_milestones >= goal_->roadmap_size;
  });
  prm.growRoadmap(ob::plannerOrTerminationCondition(cancel_ptc, size_ptc));

  ob::PlannerData planner_data(space_info);
  prm.getPlannerData(planner_data);
  const auto num_vertices = planner_data.numVertices();

  ROS_INFO("Generated C-space roadmap, generating W-space mapping...");

  // Create the W-space mapping.
  std::vector<VertexDescriptor> vertices(num_vertices);

  for (unsigned int i = 0; i < num_vertices; ++i)
  {
    if (is_canceled_callback())
      return nullptr;

    const auto *state = planner_data.getVertex(i).getState();
    vertices[i] = addVertex(state);

    std::vector<unsigned int> edges;
    planner_data.getEdges(i, edges);

    for (const auto &j : edges)
    {
      if (j < i)
        addEdge(vertices[j], vertices[i]);
    }

    updateProgress(0.5 * (1 + static_cast<double>(i + 1) / num_vertices));
  }

  ROS_INFO("CDRM generated");
  return std::move(cdrm_);
}

VertexDescriptor Generator::addVertex(const ob::State *s)
{
  auto &robot_state = planning_scene_.getCurrentStateNonConst();
  robot_state.setJointGroupPositions(jmg_, s->as<ob::RealVectorStateSpace::StateType>()->values);
  robot_state.update();

  // Get the C-space and W-space values.
  const auto *tip_link = jmg_->getLinkModels().back();
  Eigen::Vector3d contact = (origin_tf_inv_ * robot_state.getGlobalLinkTransform(tip_link)).translation();
  Eigen::VectorXd config;
  robot_state.copyJointGroupPositions(jmg_, config);

  // We don't need to check for duplicates as each vertex descriptor is only processed once.
  const auto vertex = boost::add_vertex(Vertex{config}, cdrm_->roadmap_);
  const auto contact_key = cdrm_->pointToKey(contact);
  cdrm_->contacts_.insert(std::make_pair(contact_key, vertex));

  // Update the workspace and contact bounds.
  const auto distance = contact.norm();

  cdrm_->min_contact_distance_ = std::min(cdrm_->min_contact_distance_, distance);
  cdrm_->max_contact_distance_ = std::max(cdrm_->max_contact_distance_, distance);

  cdrm_->workspace_min_ = contact.cwiseMin(cdrm_->workspace_min_);
  cdrm_->workspace_max_ = contact.cwiseMax(cdrm_->workspace_max_);

  // Voxelise the state.
  const auto callback = [this, &vertex](const Eigen::Vector3d &p, const Eigen::Vector3d &n) {
    auto key = cdrm_->pointToKey(p);
    auto existing = cdrm_->colliding_vertices_.equal_range(key);
    bool found = false;

    for (auto it = existing.first; it != existing.second; ++it) {
      if (it->second == vertex) {
        found = true;
        break;
      }
    }

    if (!found)
    {
      cdrm_->aabb_.extend(p);
      cdrm_->colliding_vertices_.insert({key, vertex});
    }
  };
  voxelise(robot_state, link_models_, goal_->resolution, callback, origin_tf_inv_);

  return vertex;
}

EdgeDescriptor Generator::addEdge(const VertexDescriptor &a, const VertexDescriptor &b)
{
  const auto edge = boost::add_edge(a, b, cdrm_->roadmap_).first;

  if (!goal_->collide_edges)
    return edge;

  // What we have already seen in this edge.
  std::set<cdrm::Key> seen;

  // The two states we are moving between.
  moveit::core::RobotState sa(robot_model_);
  moveit::core::RobotState sb(robot_model_);

  sa.setToDefaultValues();
  sa.setJointGroupPositions(jmg_, cdrm_->roadmap_[a].q_);
  sa.update();

  sb.setToDefaultValues();
  sb.setJointGroupPositions(jmg_, cdrm_->roadmap_[b].q_);
  sb.update();

  const auto callback = [&, this](const Eigen::Vector3d &p, const Eigen::Vector3d &n)
  {
    auto key = cdrm_->pointToKey(p);

    if (seen.count(key))
      return;

    cdrm_->aabb_.extend(p);
    cdrm_->colliding_edges_.insert({key, edge});

    seen.insert(key);
  };

  // Voxelise the start and end.
  voxelise(sa, link_models_, goal_->resolution, callback, origin_tf_inv_);
  voxelise(sb, link_models_, goal_->resolution, callback, origin_tf_inv_);

  // Recursively bisect until we see nothing new.
  std::queue<std::pair<double, double>> queue;
  queue.push(std::make_pair(0.0, 1.0));

  moveit::core::RobotState mid_state = sa;

  while (!queue.empty())
  {
    const auto front = queue.front();
    const double mid = (front.first + front.second) / 2;
    queue.pop();

    sa.interpolate(sb, mid, mid_state, jmg_);
    mid_state.update();

    std::size_t before = seen.size();
    voxelise(mid_state, link_models_, goal_->resolution, callback, origin_tf_inv_);
    std::size_t after = seen.size();

    // Did we see anything new?
    if (before != after)
    {
      queue.push(std::make_pair(front.first, mid));
      queue.push(std::make_pair(mid, front.second));
    }
  }

  return edge;
}

bool Generator::isStateValid(const ob::State *s)
{
  auto &state = planning_scene_.getCurrentStateNonConst();
  state.setJointGroupPositions(jmg_, s->as<ob::RealVectorStateSpace::StateType>()->values);
  state.update();

  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  planning_scene_.checkSelfCollision(request, result, state, acm_);

  if (result.collision)
    return false;

  return true;
}

void Generator::updateProgress(double progress)
{
  if (progress >= 1.0 || (ros::Time::now() - last_progress_time_).toSec() > 0.5)
  {
    last_progress_time_ = ros::Time::now();
    ROS_INFO_STREAM("Generated " << (progress * 100) << "%");

    // if (progress_callback_ )
    //   progress_callback_(progress);
  }
}
}
