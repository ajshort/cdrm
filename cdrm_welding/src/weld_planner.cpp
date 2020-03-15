#include <cdrm_welding/weld_planner.h>

#include <cdrm/cdrm.h>
#include <cdrm/voxelise.h>
#include <cdrm_welding/weld.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/console.h>
#include <ros/time.h>
#include <visualization_msgs/MarkerArray.h>

namespace cdrm_welding
{
WeldPlanner::WeldPlanner(const moveit::core::RobotModelConstPtr &robot_model,
                         const ros::Publisher &target_publisher)
  : robot_model_(robot_model), target_publisher_(target_publisher)
{
}

bool WeldPlanner::plan(cdrm_welding_msgs::PlanWeld::Request &req, cdrm_welding_msgs::PlanWeld::Response &res)
{
  ros::Time start_time = ros::Time::now();
  ros::Time limit_time = start_time + ros::Duration(req.planning_timeout);

  ROS_DEBUG("Received weld planning request");

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

  // Create a robot state.
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();

  // Voxelise around the toolpath every 5mm in its local frame.
  const int steps = static_cast<int>(std::ceil(weld.getLength() / 5.0));

  std::vector<std::set<cdrm::Key>> nozzle_voxelised;
  nozzle_voxelised.reserve(steps + 1);

  for (int step = 0; step <= steps; ++step)
  {
    const double t = static_cast<double>(step) / steps;
    const Eigen::Isometry3d tf = weld.getTransform(t);

    // cdrm::voxelise(
    //   state,
    //   links,
    //   0.001,
    //   [&](const Eigen::Vector3d &p, const Eigen::Vector3d &) {
    //   },
    //   tf,
    //   min,
    //   max
    // );
  }

  // Get the CDRM for the toolpath and filter it.
  // Get the CDRM for the manipulator and filter the toolpath CDRM edges to reachable ones only.
  // See if we can create a path from near the start to near the end.

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
