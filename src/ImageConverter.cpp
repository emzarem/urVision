#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <urVision/weedDataArray.h>
#include <urVision/weedData.h>

const std::string OPENCV_WINDOW = "urVision Window";

// Image Converter class
// This class is the main pipeline of the vision node
class ImageConverter
{
	ros::NodeHandle& m_nodeHandle;

	image_transport::ImageTransport m_imageTransport;
	image_transport::Subscriber m_imageSubscriber;
	image_transport::Publisher m_imagePublisher;  

	ros::Publisher m_weedDataPublisher;  

	// For topic names
	std::string m_cameraName;
	std::string m_imageTopic;

	std::string m_imagePublisherName;
	std::string m_weedDataPublisherName;

	bool m_showWindow;

public:
	ImageConverter(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle), m_imageTransport(nodeHandle)
	{
		if (!readParameters()) {
			ROS_ERROR("Could not read parameters required for ImageConverter.");
			ros::requestShutdown();
		}
	}

	~ImageConverter()
	{
	}

	int initialize()
	{
		// Subscribe to input video feed and publish output video feed
		m_imageSubscriber = m_imageTransport.subscribe(std::string(m_cameraName + m_imageTopic), 1, &ImageConverter::processImage, this);

		m_imagePublisher = m_imageTransport.advertise(m_imagePublisherName, 1);

	  	m_weedDataPublisher = m_nodeHandle.advertise<urVision::weedDataArray>(m_weedDataPublisherName, 1);

		ROS_INFO("ImageConverter Pipeline started successfully!");

		return true;
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

		cv::waitKey(3);

		// Output modified video stream
		m_imagePublisher.publish(cv_ptr->toImageMsg());

		// Send the most recent weed data
		urVision::weedData weed_data;
		urVision::weedDataArray weed_msg;

		// Set the data
		// Something like:
		// weed_data.x_cm = ; weed_data.y_cm = , 
		// weed_msg.weeds.push_back(weed_data)

		// data.upperleft=0; data.lowerRight=100; data.color="red"; data.cameraID="one";
		// msg.images.push_back(data);
		// data.upperleft=1; data.lowerRight=101; data.color="blue"; data.cameraID="two";
		// msg.images.push_back(data);

		m_weedDataPublisher.publish(weed_msg);
	}

	// Image converter specific parameters
	bool readParameters()
	{
		if (!m_nodeHandle.getParam("camera_name", m_cameraName)) return false;
		if (!m_nodeHandle.getParam("image_topic", m_imageTopic)) return false;
		if (!m_nodeHandle.getParam("image_publisher", m_imagePublisherName)) return false;
		if (!m_nodeHandle.getParam("weed_data_publisher", m_weedDataPublisherName)) return false;
		if (!m_nodeHandle.getParam("show_img_window", m_showWindow)) return false;
		return true;
	}

};
