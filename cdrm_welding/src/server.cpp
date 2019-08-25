#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cdrm_server");
  ros::NodeHandle node_handle;
  ros::spin();
}
