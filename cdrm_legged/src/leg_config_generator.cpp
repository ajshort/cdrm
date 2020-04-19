#include <cdrm_legged/leg_config_generator.h>

#include <cdrm_legged/leg_model.h>

#include <angles/angles.h>
#include <boost/graph/adjacency_list.hpp>
#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <ros/console.h>

#include <queue>

namespace cdrm_legged
{
LegConfigs::LegConfigs(std::size_t size) : free_(size), disjoints_(size)
{
  free_.set();
}

LegConfigGenerator::LegConfigGenerator(const planning_scene::PlanningSceneConstPtr &planning_scene) : planning_scene_(planning_scene)
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
  const auto num_vertices = boost::num_vertices(cdrm->roadmap_);

  LegConfigs configs(num_vertices);

  Eigen::Isometry3d leg_tf = body_tf * leg.tf_;
  Eigen::Isometry3d leg_tf_inv = leg_tf.inverse();

  std::set<unsigned int> contacts;
  NormalMap normals;
  std::map<int, double> contact_angles;

  const auto callback = [&](const Eigen::Vector3d &p, const Eigen::Vector3d &n) {
    const auto key = cdrm->pointToKey(leg_tf_inv * p);

    const auto cell_contacts = cdrm->contacts_.equal_range(key);
    const auto cell_colliding = cdrm->colliding_vertices_.equal_range(key);

    for (auto it = cell_contacts.first; it != cell_contacts.second; ++it)
    {
      contacts.insert(it->second);
      normals[it->second] = n;
    }

    for (auto it = cell_colliding.first; it != cell_colliding.second; ++it)
      configs.free_[it->second] = false;
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

  for (const auto &contact : contacts)
  {
    if (!configs.free_[contact])
      continue;

    // Get the angle between the foot and the surface normal.
    const Eigen::VectorXd &q = leg.cdrm_->getVertexConfig(contact);
    const Eigen::Vector3d direction = (body_tf * leg.tf_ * leg.getTipTransform(q)).linear().col(2);
    const Eigen::Vector3d &normal = normals.at(contact);
    const double angle = std::acos(direction.dot(-normal));

    if (angle > angles::from_degrees(45))
      continue;

    contact_angles[contact] = angle;

    configs.contacts_.push_back(contact);
    configs.free_[contact] = false;
  }

  // Sort the contacts by heuristic.
  std::sort(configs.contacts_.begin(), configs.contacts_.end(), [&, this](unsigned int a, unsigned int b) {
    return heuristic_(leg, a, contact_angles.at(a), b, contact_angles.at(b));
  });

  // Populate the normal information.
  for (const auto contact : configs.contacts_)
    configs.normals_.push_back(normals.at(contact));

  // TODO this should be done lazily
  // Generate disjoint sets, starting at contacts (as that is what we want to connect to).
  // for (const auto &contact : configs.contacts_)
  // {
  //   std::queue<int> queue;
  //   queue.push(contact);

  //   while (!queue.empty())
  //   {
  //     configs.disjoints_.merge(contact, queue.front());

  //     auto out = boost::out_edges(queue.front(), cdrm->roadmap_);
  //     queue.pop();

  //     for (auto it = out.first; it != out.second; ++it)
  //     {
  //       auto target = boost::target(*it, cdrm->roadmap_);

  //       if (!configs.free_[target])
  //         continue;

  //       if (!configs.disjoints_.connected(contact, target))
  //         queue.push(target);
  //     }
  //   }
  // }

  return configs;
}
}
