#pragma once

#include <moveit/macros/class_forward.h>
#include <ros/message_forward.h>

namespace moveit
{
namespace core
{
MOVEIT_CLASS_FORWARD(RobotModel);
MOVEIT_CLASS_FORWARD(RobotState);

class FloatingJointModel;
class JointModelGroup;
class LinkModel;
}
}

namespace moveit_msgs
{
ROS_DECLARE_MESSAGE(Constraints);
}

namespace planning_scene
{
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace robot_trajectory
{
MOVEIT_CLASS_FORWARD(RobotTrajectory);
}
