#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <gtest/gtest.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

TEST(KinematicsTest, testKinematics)
{
  // static const int NUM_IK_TESTS = 1000;

  // // Create a planning scene for kinematics.
  // robot_model_loader::RobotModelLoader loader("robot_description");
  // robot_model::RobotModelPtr robot_model = loader.getModel();

  // ASSERT_TRUE(static_cast<bool>(robot_model));

  // robot_state::RobotState robot_state(robot_model);
  // robot_state.setToDefaultValues();

  // const auto *group = robot_state.getJointModelGroup("al");
  // const auto &solver = group->getSolverInstance();

  // // Generate random configurations are ensure IK generates correct solutions.
  // for (int i = 0; i < NUM_IK_TESTS; ++i)
  // {
  //   robot_state.setToRandomPositions(group);
  //   robot_state.update();

  //   const Eigen::Affine3d &tf = robot_state.getFrameTransform("al_foot_link");

  //   Eigen::Affine3d solver_tf = tf;
  //   robot_state.setToIKSolverFrame(solver_tf, solver);

  //   geometry_msgs::Pose pose;
  //   tf::poseEigenToMsg(solver_tf, pose);

  //   std::vector<double> seed_state;
  //   robot_state.copyJointGroupPositions(group, seed_state);

  //   std::vector<double> solution;
  //   moveit_msgs::MoveItErrorCodes error_code;

  //   EXPECT_TRUE(solver->getPositionIK(pose, seed_state, solution, error_code));
  //   EXPECT_EQ(error_code.SUCCESS, error_code.val);

  //   if (error_code.val == error_code.SUCCESS)
  //   {
  //     EXPECT_NEAR(seed_state[0], solution[0], 0.0001);
  //     EXPECT_NEAR(seed_state[1], solution[1], 0.0001);
  //     EXPECT_NEAR(seed_state[2], solution[2], 0.0001);

  //     robot_state.setJointGroupPositions(group, solution);
  //     robot_state.update();

  //     EXPECT_TRUE(robot_state.getFrameTransform("al_foot_link").isApprox(tf));
  //   }
  // }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "kinematics_test");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
