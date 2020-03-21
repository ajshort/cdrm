#include <cdrm_welding/weld_planner.h>

#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>
#include <cdrm_welding/weld.h>
#include <cdrm_welding/welding_cdrm.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

namespace cdrm_welding
{
WeldPlanner::WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model,
                         const ros::Publisher &target_publisher)
  : robot_model_(robot_model), target_publisher_(target_publisher)
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
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();

  // Voxelise around the toolpath every 5mm in its local frame.
  cdrm::Cdrm nozzle_cdrm(cdrm.nozzle_cdrm_.resolution_);

  const int steps = static_cast<int>(std::ceil(weld.getLength() / 0.005));

  std::vector<std::set<cdrm::Key>> nozzle_voxelised(steps + 1);

  // We voxelise the AABB around the nozzle.
  const Eigen::Vector3d &min_bound = cdrm.nozzle_cdrm_.aabb_.min();
  const Eigen::Vector3d &max_bound = cdrm.nozzle_cdrm_.aabb_.max();

  for (int step = 0; step <= steps; ++step)
  {
    const double t = static_cast<double>(step) / steps;
    const Eigen::Isometry3d &workpiece_tf = state.getGlobalLinkTransform(workpiece_link_);
    const Eigen::Isometry3d weld_tf = workpiece_tf * weld.getTransform(t);

    cdrm::voxelise(
      state,
      std::vector<const moveit::core::LinkModel *>({ workpiece_link_ }),
      nozzle_cdrm.resolution_,
      [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
        nozzle_voxelised[step].insert(nozzle_cdrm.pointToKey(p));
      },
      weld_tf.inverse(),
      &min_bound,
      &max_bound
    );
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

  // Get the CDRM for the toolpath and filter it.
  // Get the CDRM for the manipulator and filter the toolpath CDRM edges to reachable ones only.
  // See if we can create a path from near the start to near the end.

  ROS_INFO_STREAM("Finished planning after " << (ros::Time::now() - start_time).toSec() << "s");

  return false;
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
