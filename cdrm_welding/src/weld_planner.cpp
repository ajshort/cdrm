#include <cdrm_welding/weld_planner.h>

#include <cdrm_welding/weld.h>

#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>

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

  for (std::size_t i = 0; i < req.weld_points.size(); ++i) {
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
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.color.a = 1.0;
  marker.color.r = 1.0;

  for (std::size_t i = 0; i < weld.getNumTargets(); ++i) {
    geometry_msgs::Point point;
    tf::pointEigenToMsg(weld.getTargetTransform(i).translation(), point);
    marker.points.push_back(point);
  }

  target_publisher_.publish(marker);

  // Get the CDRM for the toolpath and filter it.
  // Get the CDRM for the manipulator and filter the toolpath CDRM edges to reachable ones only.
  // See if we can create a path from near the start to near the end.

  return false;
}
}
