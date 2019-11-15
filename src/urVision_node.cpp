// Project internal includes
#include "urVision/RosPackageTemplate.hpp"
#include "PlantDetect.h"
#include "ImageConverter.cpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "urVision");
	ros::NodeHandle nodeHandle("~");

	// Stub class
	urVision::RosPackageTemplate rosPackageTemplate(nodeHandle);

	// Instantiate imageConverter class
	ImageConverter imageConverter(nodeHandle);
	// Initialize the imageConverter
	// This will start the main vision pipeline
	if (!imageConverter.initialize())
	{
		ROS_ERROR("Error in call to initialize ImageConverter.");
		ros::requestShutdown();
	}

	ros::spin();
	return 0;
}