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
#include <geometric_shapes/shapes.h>
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
WeldPlanner::WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model,
                         const ros::Publisher &target_publisher)
  : robot_model_(robot_model)
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

  if (!(workpiece_link_ = robot_model_->getLinkModel(req.workpiece_link_name)))
  {
    ROS_ERROR("Could not find the workpiece link '%s'", req.workpiece_link_name.c_str());
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
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();

  const auto &tool_cdrm = cdrm.tool_cdrm_;

  // Figure out the AABB which encompasses the welding path as well as the tool.
  Eigen::AlignedBox3d aabb;

  for (std::size_t i = 0; i < weld.getNumTargets(); ++i)
  {
    const Eigen::Isometry3d tf = robot_state.getGlobalLinkTransform(workpiece_link_) * weld.getTargetTransform(i);
    aabb.extend(tf * tool_cdrm.aabb_.min());
    aabb.extend(tf * tool_cdrm.aabb_.max());
  }

  // Voxelise the workpiece in the desired region.
  ROS_INFO_STREAM("Voxelising workpiece...");

  std::set<cdrm::Key> workpiece_voxels;

  const auto &workpiece_shapes = workpiece_link_->getShapes();
  const auto &workpiece_poses = workpiece_link_->getCollisionOriginTransforms();

  for (std::size_t i = 0; i < workpiece_shapes.size(); ++i)
  {
    if (workpiece_shapes[i]->type != shapes::MESH)
      continue;

    const Eigen::Isometry3d workpiece_tf = robot_state.getGlobalLinkTransform(workpiece_link_) * workpiece_poses[i];
    const auto *mesh = static_cast<const shapes::Mesh *>(workpiece_shapes[i].get());

    cdrm::voxelise(
      *mesh,
      tool_cdrm.resolution_,
      [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
        workpiece_voxels.insert(tool_cdrm.pointToKey(p));
      },
      workpiece_tf,
      &aabb.min(),
      &aabb.max()
    );
  }

  // Now that we know which cells are occupied at each step, we need to convert this into the vertices and edges
  // which are invalid.
  const int steps = static_cast<int>(std::ceil(weld.getLength() / 0.005));
  const double step_size = weld.getLength() / steps;

  ROS_INFO_STREAM("Filtering roadmap...");

  std::vector<std::set<cdrm::VertexDescriptor>> occupied_nozzle_vertices(steps + 1);
  std::vector<std::set<cdrm::EdgeDescriptor>> occupied_nozzle_edges(steps + 1);

  for (std::size_t i = 0; i <= steps; ++i)
  {
    const Eigen::Isometry3d &workpiece_tf = robot_state.getGlobalLinkTransform(workpiece_link_);
    const Eigen::Isometry3d weld_tf = weld.getTransform(static_cast<double>(i) / steps);

    for (const auto &key : workpiece_voxels)
    {
      const Eigen::Vector3d point = (workpiece_tf * weld_tf).inverse() * tool_cdrm.keyToPoint(key);
      const auto transformed_key = tool_cdrm.pointToKey(point);

      if (!tool_cdrm.aabb_.contains(point))
        continue;

      auto vertices = tool_cdrm.colliding_vertices_.equal_range(key);
      auto edges = tool_cdrm.colliding_edges_.equal_range(key);

      for (auto it = vertices.first; it != vertices.second; ++it)
        occupied_nozzle_vertices[i].insert(it->second);

      for (auto it = edges.first; it != edges.second; ++it)
        occupied_nozzle_edges[i].insert(it->second);
    }
  }

  ROS_INFO_STREAM("Creating connected components...");

  // We use the occupied cells at each pair of points to filter a CDRM to generate reachable pairs.
  std::vector<std::vector<int>> connected_components(steps);

  for (int i = 0; i < steps; ++i)
    connected_components[i].resize(boost::num_vertices(tool_cdrm.roadmap_));

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
    VertexFilter vertex_filter(invalid_vertices);
    EdgeFilter edge_filter(invalid_edges);
    FilteredGraph filtered(tool_cdrm.roadmap_, edge_filter, vertex_filter);

    // Then use this to create a connected components.
    boost::connected_components(filtered, &connected_components[i][0]);
  }

  // Start at the start, and for each vertex in the start try and generate a path to the end through the pairwise
  // nozzle CDRMs.
  std::vector<cdrm::VertexDescriptor> tool_path;

  const auto vertices = boost::vertices(tool_cdrm.roadmap_);

  for (int i = 0; i <= steps; ++i)
  {
    for (auto it = vertices.first; it != vertices.second; ++it)
    {
      if (occupied_nozzle_vertices[i].count(*it))
        continue;

      tool_path.push_back(*it);
      break;
    }
  }

  // Get where the robot CDRM is relative to.
  const auto *robot_origin_link = planning_group_->getLinkModels().front();
  const Eigen::Isometry3d &robot_origin_tf = robot_state.getGlobalLinkTransform(robot_origin_link);

  // Voxelise the workspace at the robot's CDRM resolution.
  const auto &robot_cdrm = cdrm.robot_cdrm_;

  std::set<cdrm::Key> robot_voxelised;

  // TODO
  // for (const auto &object : *(planning_scene_->getWorld()))
  // {
  //   const auto &shapes = object.second->shapes_;
  //   const auto &poses = object.second->shape_poses_;

  //   for (std::size_t i = 0; i < shapes.size(); ++i)
  //   {
  //     if (shapes[i]->type != shapes::MESH)
  //       continue;

  //     const auto *mesh = static_cast<const shapes::Mesh *>(shapes[i].get());

  //     cdrm::voxelise(
  //       *mesh,
  //       robot_cdrm.resolution_,
  //       [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
  //         robot_voxelised.insert(robot_cdrm.pointToKey(p));
  //       },
  //       poses[i]
  //     );
  //   }
  // }

  // Figure out the nozzle to flange tf.
  const auto *flange_link = robot_group_->getLinkModels().back();
  const Eigen::Isometry3d nozzle_flange_tf = robot_state.getGlobalLinkTransform(nozzle_link_).inverse() *
                                             robot_state.getGlobalLinkTransform(flange_link);

  // Go through the flange transforms and attempt to generate a matching path using the robot CDRM which is collision
  // free.
  EigenSTL::vector_Isometry3d flange_tfs;

  for (std::size_t i = 0; i < tool_path.size(); ++i)
  {
    const cdrm::VertexDescriptor vertex = tool_path[i];
    const Eigen::VectorXd &q = tool_cdrm.roadmap_[vertex].q_;

    Eigen::Isometry3d tf = weld.getTransform((i * step_size) / weld.getLength());
    tf.rotate(Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitX()));
    tf.rotate(Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitY()));
    tf.rotate(Eigen::AngleAxisd(q(2), Eigen::Vector3d::UnitZ()));
    tf.translation() -= q(3) * tf.linear().col(2);

    flange_tfs.push_back(tf * nozzle_flange_tf);
  }

  robot_trajectory::RobotTrajectory trajectory(robot_model_, req.planning_group_name);

  for (const Eigen::Isometry3d &flange_tf : flange_tfs)
  {
    const Eigen::Vector3d translation = robot_origin_tf.inverse() * flange_tf.translation();
    const auto key = robot_cdrm.pointToKey(translation);
    const auto found = robot_cdrm.contacts_.equal_range(key);

    // Find the pose which has the closest angle to the goal flange TF.
    bool solved = false;
    auto best = found.first;
    double bestScore = std::numeric_limits<double>::max();

    for (auto it = found.first; it != found.second; ++it)
    {
      robot_state.setJointGroupPositions(planning_group_, robot_cdrm.roadmap_[it->second].q_);
      robot_state.update();

      if (!robot_state.setFromIK(robot_group_, flange_tf))
        continue;

      solved = true;

      robot_state.update();

      const Eigen::Isometry3d &close_flange_tf = robot_state.getGlobalLinkTransform(flange_link);
      Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(flange_tf.linear().col(2), close_flange_tf.linear().col(2));
      Eigen::AngleAxisd aa(quat);

      if (std::abs(aa.angle()) < bestScore) {
        best = it;
        bestScore = std::abs(aa.angle());
      }
    }

    if (!solved)
      continue;

    robot_state.setJointGroupPositions(planning_group_, robot_cdrm.roadmap_[best->second].q_);
    robot_state.update();
    robot_state.setFromIK(robot_group_, flange_tf);
    robot_state.update();

    trajectory.addSuffixWayPoint(robot_state, step_size / req.welding_speed);
  }

  ros::Duration planning_duration = ros::Time::now() - start_time;

  ROS_INFO_STREAM("Finished planning after " << planning_duration.toSec() << "s");

  res.success = true;
  res.planning_time = planning_duration.toSec();
  res.trajectory.model_id = robot_model_->getName();

  if (!trajectory.empty())
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
