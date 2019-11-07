#include <ros/ros.h>
#include "urVision/RosPackageTemplate.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urVision");
  ros::NodeHandle nodeHandle("~");

  urVision::RosPackageTemplate rosPackageTemplate(nodeHandle);

  ros::spin();
  return 0;
}
