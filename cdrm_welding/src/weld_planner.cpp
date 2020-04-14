#include <cdrm_welding/weld_planner.h>

#include <cdrm/cdrm.h>
#include <cdrm/utils.h>
#include <cdrm/voxelise.h>
#include <cdrm_welding/kinematic_utils.h>
#include <cdrm_welding/weld.h>
#include <cdrm_welding/welding_cdrm.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <ros/console.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <iostream>

namespace cdrm_welding
{
WeldPlanner::WeldPlanner(const planning_scene::PlanningSceneConstPtr &planning_scene,
                         const ros::Publisher &target_publisher)
  : planning_scene_(planning_scene)
  , robot_model_(planning_scene_->getRobotModel())
  , target_publisher_(target_publisher)
{
}

template <typename T>
struct FilterPredicate
{
  FilterPredicate()
  {
  }

  FilterPredicate(const std::set<T> &invalid) : invalid_(&invalid)
  {
  }

  template <typename Vertex>
  bool operator()(const Vertex &v) const
  {
    return invalid_->count(v) == 0;
  }

  const std::set<T> *invalid_ = nullptr;
};

bool WeldPlanner::plan(const cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res)
{
  ROS_INFO("Received weld planning request");

  // We load the CDRM - note we don't count this as part of the planning time. This should really be cached or
  // pre-loaded when the mode is created.
  WeldingCdrm cdrm;

  ROS_INFO("Loading CDRM...");

  if (!cdrm.load(req.cdrm_filename))
  {
    ROS_ERROR("Could not load the welding CDRM from '%s'", req.cdrm_filename.c_str());
    return false;
  }

  if (!(planning_group_ = robot_model_->getJointModelGroup(req.planning_group_name)))
  {
    ROS_ERROR("Could not find the planning group '%s'", req.planning_group_name.c_str());
    return false;
  }

  if (!(robot_group_ = robot_model_->getJointModelGroup(req.robot_group_name)))
  {
    ROS_ERROR("Could not find the robot group '%s'", req.robot_group_name.c_str());
    return false;
  }

  if (!(nozzle_link_ = robot_model_->getLinkModel(req.nozzle_link_name)))
  {
    ROS_ERROR("Could not find the nozzle link '%s'", req.nozzle_link_name.c_str());
    return false;
  }

  // Create weld from the targets.
  Weld weld;

  for (std::size_t i = 0; i < req.weld_points.size(); ++i)
  {
    Eigen::Vector3d position;
    Eigen::Vector3d direction;

    tf::pointMsgToEigen(req.weld_points[i], position);
    tf::vectorMsgToEigen(req.weld_directions[i], direction);

    weld.addTarget(position, direction);
  }

  if (weld.getLength() == 0)
  {
    ROS_ERROR("Weld has zero length");
    return false;
  }

  // Publish the targets as a line strip.
  publishTargets(weld);

  // Start the actual planning process.
  ROS_INFO("Starting planning...");

  ros::Time start_time = ros::Time::now();
  ros::Time limit_time = start_time + ros::Duration(req.planning_timeout);

  // Create a robot state.
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();

  // Voxelise around the toolpath every 5mm in its local frame.
  const auto &nozzle_cdrm = cdrm.nozzle_cdrm_;

  const int steps = static_cast<int>(std::ceil(weld.getLength() / 0.005));
  const double step_size = weld.getLength() / steps;

  std::vector<std::set<cdrm::Key>> nozzle_voxelised(steps + 1);

  // We voxelise the AABB around the nozzle.
  const Eigen::Vector3d &min_bound = cdrm.nozzle_cdrm_.aabb_.min();
  const Eigen::Vector3d &max_bound = cdrm.nozzle_cdrm_.aabb_.max();

  for (int step = 0; step <= steps; ++step)
  {
    const double t = static_cast<double>(step) / steps;

    for (const auto &object : *(planning_scene_->getWorld()))
    {
      const auto &shapes = object.second->shapes_;
      const auto &poses = object.second->shape_poses_;

      for (std::size_t i = 0; i < shapes.size(); ++i)
      {
        if (shapes[i]->type != shapes::MESH)
          continue;

        const Eigen::Isometry3d weld_tf = poses[i] * weld.getTransform(t);
        const auto *mesh = static_cast<const shapes::Mesh *>(shapes[i].get());

        cdrm::voxelise(
          *mesh,
          nozzle_cdrm.resolution_,
          [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
            nozzle_voxelised[step].insert(nozzle_cdrm.pointToKey(p));
          },
          weld_tf.inverse(),
          &min_bound,
          &max_bound
        );
      }
    }
  }

  // Now that we know which cells are occupied at each step, we need to convert this into the vertices and edges
  // which are invalid.
  std::vector<std::set<cdrm::VertexDescriptor>> occupied_nozzle_vertices(nozzle_voxelised.size());
  std::vector<std::set<cdrm::EdgeDescriptor>> occupied_nozzle_edges(nozzle_voxelised.size());

  for (std::size_t i = 0; i < nozzle_voxelised.size(); ++i)
  {
    for (const auto &key : nozzle_voxelised[i])
    {
      auto v_range = cdrm.nozzle_cdrm_.colliding_vertices_.equal_range(key);
      auto e_range = cdrm.nozzle_cdrm_.colliding_edges_.equal_range(key);

      for (auto it = v_range.first; it != v_range.second; ++it)
        occupied_nozzle_vertices[i].insert(it->second);

      for (auto it = e_range.first; it != e_range.second; ++it)
        occupied_nozzle_edges[i].insert(it->second);
    }
  }

  // We use the occupied cells at each pair of points to filter a CDRM to generate reachable pairs.
  std::vector<std::vector<int>> connected_components(steps);

  for (int i = 0; i < steps; ++i)
    connected_components[i].resize(boost::num_vertices(cdrm.nozzle_cdrm_.roadmap_));

  for (int i = 0; i < steps; ++i)
  {
    using VertexFilter = FilterPredicate<cdrm::VertexDescriptor>;
    using EdgeFilter = FilterPredicate<cdrm::EdgeDescriptor>;
    using FilteredGraph = boost::filtered_graph<cdrm::Roadmap, EdgeFilter, VertexFilter>;

    // Merge the inaccessible items from the from and the to item.
    std::set<cdrm::VertexDescriptor> invalid_vertices;
    std::set<cdrm::EdgeDescriptor> invalid_edges;

    std::set_union(occupied_nozzle_vertices[i].begin(),
                   occupied_nozzle_vertices[i].end(),
                   occupied_nozzle_vertices[i + 1].begin(),
                   occupied_nozzle_vertices[i + 1].end(),
                   std::inserter(invalid_vertices, invalid_vertices.begin()));

    std::set_union(occupied_nozzle_edges[i].begin(),
                   occupied_nozzle_edges[i].end(),
                   occupied_nozzle_edges[i + 1].begin(),
                   occupied_nozzle_edges[i + 1].end(),
                   std::inserter(invalid_edges, invalid_edges.begin()));

    // Create a roadmap of items which are accessible from steps[i] and steps[i + 1].
    const auto &from = nozzle_voxelised[i];
    const auto &to = nozzle_voxelised[i + 1];

    VertexFilter vertex_filter(invalid_vertices);
    EdgeFilter edge_filter(invalid_edges);
    FilteredGraph filtered(cdrm.nozzle_cdrm_.roadmap_, edge_filter, vertex_filter);

    // Then use this to create a connected components.
    boost::connected_components(filtered, &connected_components[i][0]);
  }

  // Start at the start, and for each vertex in the start try and generate a path to the end through the pairwise
  // nozzle CDRMs.
  std::vector<cdrm::VertexDescriptor> nozzle_path;

  const auto vertices = boost::vertices(cdrm.nozzle_cdrm_.roadmap_);

  for (int i = 0; i <= steps; ++i)
  {
    for (auto it = vertices.first; it != vertices.second; ++it)
    {
      if (occupied_nozzle_vertices[i].count(*it))
        continue;

      nozzle_path.push_back(*it);
      break;
    }
  }

  // Get where the robot CDRM is relative to.
  const auto *robot_origin_link = planning_group_->getLinkModels().front();
  const Eigen::Isometry3d &robot_origin_tf = planning_scene_->getCurrentState().getGlobalLinkTransform(robot_origin_link);

  // Voxelise the workspace at the robot's CDRM resolution.
  const auto &robot_cdrm = cdrm.robot_cdrm_;

  std::set<cdrm::Key> robot_voxelised;

  for (const auto &object : *(planning_scene_->getWorld()))
  {
    const auto &shapes = object.second->shapes_;
    const auto &poses = object.second->shape_poses_;

    for (std::size_t i = 0; i < shapes.size(); ++i)
    {
      if (shapes[i]->type != shapes::MESH)
        continue;

      const auto *mesh = static_cast<const shapes::Mesh *>(shapes[i].get());

      cdrm::voxelise(
        *mesh,
        robot_cdrm.resolution_,
        [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
          robot_voxelised.insert(robot_cdrm.pointToKey(p));
        },
        poses[i]
      );
    }
  }

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  // Figure out the nozzle to flange tf.
  const auto *flange_link = robot_group_->getLinkModels().back();
  const Eigen::Isometry3d nozzle_flange_tf = robot_state.getGlobalLinkTransform(nozzle_link_).inverse() *
                                             robot_state.getGlobalLinkTransform(flange_link);

  // Go through the flange transforms and attempt to generate a matching path using the robot CDRM which is collision
  // free.
  EigenSTL::vector_Isometry3d flange_tfs;

  for (int i = 0; i <= steps; ++i)
  {
    flange_tfs.push_back(weld.getTransform(static_cast<double>(i) / steps) * nozzle_flange_tf);
  }

  robot_trajectory::RobotTrajectory trajectory(robot_model_, req.planning_group_name);

  for (const Eigen::Isometry3d &flange_tf : flange_tfs)
  {
    const Eigen::Vector3d translation = robot_origin_tf.inverse() * flange_tf.translation();
    const auto key = robot_cdrm.pointToKey(translation);
    const auto found = robot_cdrm.contacts_.equal_range(key);

    // Find the pose which has the closest angle to the goal flange TF.
    auto best = found.first;
    double bestScore = std::numeric_limits<double>::max();

    for (auto it = found.first; it != found.second; ++it)
    {
      robot_state.setJointGroupPositions(planning_group_, robot_cdrm.roadmap_[it->second].q_);
      robot_state.update();

      const Eigen::Isometry3d &close_flange_tf = robot_state.getGlobalLinkTransform(flange_link);
      Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(flange_tf.linear().col(2), close_flange_tf.linear().col(2));
      Eigen::AngleAxisd aa(quat);

      if (std::abs(aa.angle()) < bestScore) {
        best = it;
        bestScore = std::abs(aa.angle());
      }
    }

    robot_state.setJointGroupPositions(planning_group_, robot_cdrm.roadmap_[best->second].q_);
    robot_state.setFromIK(robot_group_, flange_tf);
    robot_state.update();

    trajectory.addSuffixWayPoint(robot_state, step_size / req.welding_speed);
  }

  ros::Duration planning_duration = ros::Time::now() - start_time;

  ROS_INFO_STREAM("Finished planning after " << planning_duration.toSec() << "s");

  res.success = true;
  res.planning_time = planning_duration.toSec();
  res.trajectory.model_id = robot_model_->getName();

  moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), res.trajectory.trajectory_start);

  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);
  res.trajectory.trajectory.push_back(trajectory_msg);

  return true;
}

void WeldPlanner::publishTargets(const Weld &weld)
{
  visualization_msgs::MarkerArray marker_array;

  // Draw a Z marker for each target.
  for (std::size_t i = 0; i < weld.getNumTargets(); ++i)
  {
    Eigen::Isometry3d tf = weld.getTargetTransform(i);
    tf *= Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    tf::poseEigenToMsg(tf, marker.pose);

    marker.scale.x = 0.05;
    marker.scale.y = 0.0025;
    marker.scale.z = 0.0025;

    marker.color.a = 1;
    marker.color.b = 1;

    marker_array.markers.push_back(marker);
  }

  target_publisher_.publish(marker_array);
}
}
