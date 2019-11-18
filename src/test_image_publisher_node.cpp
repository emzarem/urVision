#include "urVision/RosPackageTemplate.hpp"
#include "PlantDetector.h"
#include "ImageConverter.cpp"

#include <ros/ros.h>

using namespace cv;
using namespace std;

// Constants
const float X_SCALE = (float)0.15;
const float Y_SCALE = (float)0.15;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_image_publisher");
	ros::NodeHandle nodeHandle("~");

	std::string test_image_file;

	// Check for test image specified
	if (nodeHandle.getParam("test_image", test_image_file))
	{
		cv_bridge::CvImage cv_image;
		cv_image.image = cv::imread(test_image_file, IMREAD_COLOR);

		if (cv_image.image.empty())
		{
			ROS_ERROR("Cannot read image file: %s\n", test_image_file.c_str());
			ros::requestShutdown();
		}
		
		// Resize image
		resize(cv_image.image , cv_image.image, Size(0, 0), X_SCALE, Y_SCALE, INTER_LINEAR);

		cv_image.encoding = "bgr8";
		sensor_msgs::Image ros_image;
		cv_image.toImageMsg(ros_image);

		ros::Publisher pub = nodeHandle.advertise<sensor_msgs::Image>("/test_image/image_raw", 1);
		ros::Rate loop_rate(5);

		while (nodeHandle.ok()) 
		{
			pub.publish(ros_image);
			loop_rate.sleep();
		}
	}
	else
	{
		ROS_ERROR("Could not get test_image file!");
		ros::requestShutdown();
	}
}

