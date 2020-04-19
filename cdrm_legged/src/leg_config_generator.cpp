#include <cdrm_legged/leg_config_generator.h>

#include <cdrm_legged/leg_model.h>

#include <angles/angles.h>
#include <boost/graph/adjacency_list.hpp>
#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ros/console.h>

#include <queue>

namespace cdrm_legged
{
LegConfigGenerator::LegConfigGenerator(const planning_scene::PlanningSceneConstPtr &planning_scene)
  : planning_scene_(planning_scene)
{
  heuristic_ = [](const LegModel &, unsigned int, double angle_a, unsigned int, double angle_b) {
    return std::abs(angle_a) < std::abs(angle_b);
  };
}

LegConfigs LegConfigGenerator::generateLegConfigs(const Eigen::Isometry3d &body_tf, const LegModel &leg) const
{
  using NormalMap = std::map<int, Eigen::Vector3d, std::less<int>,
                             Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d>>>;

  const auto *cdrm = leg.cdrm_;

  LegConfigs configs;

  Eigen::Isometry3d leg_tf = body_tf * leg.tf_;
  Eigen::Isometry3d leg_tf_inv = leg_tf.inverse();

  std::vector<cdrm::VertexDescriptor> contacts;
  EigenSTL::vector_Vector3d contact_normals;
  std::set<cdrm::VertexDescriptor> colliding_vertices;
  std::set<cdrm::EdgeDescriptor> colliding_edges;

  const auto callback = [&](const Eigen::Vector3d &p, const Eigen::Vector3d &n)
  {
    const auto key = cdrm->pointToKey(leg_tf_inv * p);

    const auto cell_contacts = cdrm->contacts_.equal_range(key);
    const auto cell_vertices = cdrm->colliding_vertices_.equal_range(key);
    const auto cell_edges = cdrm->colliding_edges_.equal_range(key);

    for (auto it = cell_contacts.first; it != cell_contacts.second; ++it)
    {
      contacts.push_back(it->second);
      contact_normals.push_back(n);
    }

    for (auto it = cell_vertices.first; it != cell_vertices.second; ++it)
      colliding_vertices.insert(it->second);

    for (auto it = cell_edges.first; it != cell_edges.second; ++it)
      colliding_edges.insert(it->second);
  };

  Eigen::Vector3d min = leg_tf.translation().array() - leg.cdrm_->max_contact_distance_;
  Eigen::Vector3d max = leg_tf.translation().array() + leg.cdrm_->max_contact_distance_;

  for (const auto &object : *(planning_scene_->getWorld()))
  {
    const auto &shapes = object.second->shapes_;
    const auto &poses = object.second->shape_poses_;

    for (std::size_t i = 0; i < shapes.size(); ++i)
    {
      if (shapes[i]->type != shapes::MESH)
        continue;

      const auto *mesh = static_cast<const shapes::Mesh *>(shapes[i].get());
      cdrm::voxelise(*mesh, leg.cdrm_->resolution_, callback, poses[i], &min, &max);
    }
  }

  for (std::size_t i = 0; i < contacts.size(); ++i)
  {
    const auto contact = contacts[i];

    if (colliding_vertices.count(contact))
      continue;

    // Get the angle between the foot and the surface normal.
    const Eigen::VectorXd &q = leg.cdrm_->getVertexConfig(contact);
    const Eigen::Vector3d direction = (body_tf * leg.tf_ * leg.getTipTransform(q)).linear().col(2);
    const Eigen::Vector3d &normal = contact_normals[i];
    const double angle = std::acos(direction.dot(-normal));

    if (angle > angles::from_degrees(45))
      continue;

    configs.contacts_.push_back(contact);
    configs.normals_.push_back(normal);
  }

  // Sort the contacts by heuristic.
  // std::sort(configs.contacts_.begin(), configs.contacts_.end(), [&, this](unsigned int a, unsigned int b) {
  //   return heuristic_(leg, a, contact_angles.at(a), b, contact_angles.at(b));
  // });

  // Generate disjoint sets, starting at contacts (as that is what we want to connect to).
  unsigned int num_vertices = boost::num_vertices(leg.cdrm_->roadmap_);

  DisjointSets disjoint(num_vertices);

  std::queue<int> queue;
  std::set<int> seen;

  for (const auto &contact : configs.contacts_)
  {
    queue.push(contact);
    seen.insert(contact);
  }

  while (!queue.empty())
  {
    auto source = queue.front();
    auto out = boost::out_edges(source, cdrm->roadmap_);
    queue.pop();

    for (auto it = out.first; it != out.second; ++it)
    {
      auto target = boost::target(*it, cdrm->roadmap_);

      if (seen.count(target))
        continue;

      if (colliding_edges.count(*it))
        continue;

      seen.insert(target);

      disjoint.merge(source, target);
    }
  }

  configs.reachable_.resize(configs.contacts_.size());

  // Copy it across to the reachable configurations.
  for (unsigned int i = 0; i < num_vertices; ++i)
  {
    for (std::size_t j = 0; j < configs.contacts_.size(); ++j)
    {
      if (disjoint.connected(i, configs.contacts_[j]))
        configs.reachable_[j].insert(i);
    }
  }

  return configs;
}
}
