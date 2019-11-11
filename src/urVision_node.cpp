#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Project internal includes
#include "urVision/RosPackageTemplate.hpp"
#include "PlantDetect.h"

const std::string OPENCV_WINDOW = "urVision Window";

// Image Converter class
class ImageConverter
{
	ros::NodeHandle& m_nodeHandle;

	image_transport::ImageTransport m_imageTransport;
	image_transport::Subscriber m_imageSubscriber;
	image_transport::Publisher m_imagePublisher;  
	// For topic names
	std::string m_cameraName;
	std::string m_imageTopic;

	std::string m_publisherName;

	bool m_showWindow;

public:
	ImageConverter(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle), m_imageTransport(nodeHandle)
	{
		if (!readParameters()) {
			ROS_ERROR("Could not read parameters required for ImageConverter.");
			ros::requestShutdown();
		}

		// Subscribe to input video feed and publish output video feed
		m_imageSubscriber = m_imageTransport.subscribe(std::string(m_cameraName + m_imageTopic), 1, &ImageConverter::processImage, this);

		m_imagePublisher = m_imageTransport.advertise(m_publisherName, 1);

		if (m_showWindow)
		{
			cv::namedWindow(OPENCV_WINDOW);
		}

		  ROS_INFO("ImageConverter Pipeline started successfully.");
	}

	~ImageConverter()
	{
		if (m_showWindow)
		{
			cv::destroyWindow(OPENCV_WINDOW);
		}
	}
 
	void processImage(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}

		// Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		 cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		// Update GUI Window
		if (m_showWindow)
		{
			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		}

		cv::waitKey(3);

		// Output modified video stream
		m_imagePublisher.publish(cv_ptr->toImageMsg());
	}

	// Add parameters here
	bool readParameters()
	{
		if (!m_nodeHandle.getParam("camera_name", m_cameraName)) return false;
		if (!m_nodeHandle.getParam("image_topic", m_imageTopic)) return false;
		if (!m_nodeHandle.getParam("publisher_name", m_publisherName)) return false;
		if (!m_nodeHandle.getParam("show_img_window", m_showWindow)) return false;
		return true;
	}

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urVision");
  ros::NodeHandle nodeHandle("~");

  urVision::RosPackageTemplate rosPackageTemplate(nodeHandle);
  // Starts the image conversion pipeline
  ImageConverter imageConverter(nodeHandle);

  ros::spin();
  return 0;
}