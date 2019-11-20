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
	bool m_haveFrame;
	uint64_t m_frameNum;

	PlantDetector* m_detector;

	VisionParams m_visionParams;

	int fovWidthCm, fovHeightCm, maxWeedSizeCm, minWeedSizeCm, defaultWeedSizeCm;

	float scaleFactorX, scaleFactorY, sizeScale;

public:
	ImageConverter(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle), m_imageTransport(nodeHandle), m_haveFrame(false), m_frameNum(0)
	{
		if (!readGeneralParameters()) {
			ROS_ERROR("Could not read general parameters required for ImageConverter.");
			ros::requestShutdown();
		}

		if (!readVisionParameters())
		{
			ROS_ERROR("Could not read vision parameters required for ImageConverter.");
			ros::requestShutdown();
		}

		// Create plant detector
		m_detector = new PlantDetector(m_showWindow);
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
		// Update framenumber
		m_frameNum++;
		// Get image from cv_bridge
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

		// Get current frame
		Mat currentFrame = cv_ptr->image;

		// If this is the first frame
		if (!m_haveFrame)
		{
			m_visionParams.frameSize = currentFrame.size();

			// These are our scaling operations
			scaleFactorX = ((float)fovWidthCm) / m_visionParams.frameSize.width;
			scaleFactorY = ((float)fovHeightCm) / m_visionParams.frameSize.height;
			sizeScale = (scaleFactorX + scaleFactorY) / 2;
			
			ROS_INFO("FrameSize is (width, height): (%i, %i)", 
				m_visionParams.frameSize.width, m_visionParams.frameSize.height);
			ROS_INFO("ScaleFactors -- (xScale, yScale, sizeScale): ( %f, %f, %f)", scaleFactorX, scaleFactorY, sizeScale);

			m_visionParams.defaultWeedThreshold = ((float)defaultWeedSizeCm ) / sizeScale;
			m_visionParams.minWeedSize = ((float)minWeedSizeCm ) / sizeScale;
			m_visionParams.maxWeedSize = ((float)maxWeedSizeCm ) / sizeScale;

			// Now we have a frame (only had to to this initialization once)
			m_haveFrame = true;
			if (!m_detector->init(m_visionParams))
			{
				ROS_ERROR("Could not initialize PlantDetector instance!");
				ros::requestShutdown();
			}
		}

		// Do processing on this frame
		if (!m_detector->processFrame(currentFrame))
		{
			ROS_ERROR("Error processing frame: %i\n", (int)m_frameNum);
			ros::requestShutdown();
		}

		ROS_INFO("Processed Frame: %i", (int)m_frameNum);

		// Get the weed list for this frame!	
		vector<KeyPoint> weedList = m_detector->getWeedList();
		for (vector<KeyPoint>::iterator it = weedList.begin(); it != weedList.end(); ++it)
		{
			/* Populating weedData list to be published */
			// urVision::weedData weed_data;
			// urVision::weedDataArray weed_msg;

			// Set the data
			// Something like:
			// weed_data.x_cm = ; weed_data.y_cm = , 
			// weed_msg.weeds.push_back(weed_data)

			// data.upperleft=0; data.lowerRight=100; data.color="red"; data.cameraID="one";
			// msg.images.push_back(data);
			// data.upperleft=1; data.lowerRight=101; data.color="blue"; data.cameraID="two";
			// msg.images.push_back(data);

			// m_weedDataPublisher.publish(weed_msg);
		}

		// Publish weed keypoints drawn on original image
		Mat im_with_keypoints;
		drawKeypoints(currentFrame, weedList, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv_ptr->image = im_with_keypoints;
		m_imagePublisher.publish(cv_ptr->toImageMsg());
	}

	// General parameters for this node
	bool readGeneralParameters()
	{
		if (!m_nodeHandle.getParam("camera_name", m_cameraName)) return false;
		if (!m_nodeHandle.getParam("image_topic", m_imageTopic)) return false;
		if (!m_nodeHandle.getParam("image_publisher", m_imagePublisherName)) return false;
		if (!m_nodeHandle.getParam("weed_data_publisher", m_weedDataPublisherName)) return false;
		if (!m_nodeHandle.getParam("show_img_window", m_showWindow)) return false;

		return true;
	}

		// Image converter specific parameters
	bool readVisionParameters()
	{
		// Read vision parameters
		if (!m_nodeHandle.getParam("fov_width_cm", fovWidthCm)) return false;
		if (!m_nodeHandle.getParam("fov_height_cm", fovHeightCm)) return false;
		if (!m_nodeHandle.getParam("min_weed_size_cm", minWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("max_weed_size_cm", maxWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("default_weed_size_threshold_cm", defaultWeedSizeCm)) return false;

		if (!m_nodeHandle.getParam("min_accumulator_size", m_visionParams.minAccumulatorSize)) return false;
		if (!m_nodeHandle.getParam("max_accumulator_size", m_visionParams.maxAccumulatorSize)) return false;

		return true;
	}

};
