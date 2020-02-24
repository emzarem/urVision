#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// For spatial mapping utilities
#include "SpatialMapper.h"

// Msg types
#include <urVision/weedDataArray.h>
#include <urVision/weedData.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <urGovernor/FetchWeed.h>

const std::string OPENCV_WINDOW = "urVision Window";

// Image Converter class
// This class is the main pipeline of the vision node
class ImageConverter
{
	ros::NodeHandle& m_nodeHandle;

	image_transport::ImageTransport m_imageTransport;

	// Subscribers
	image_transport::Subscriber m_imageSubscriber;

	// Publishers
	image_transport::Publisher m_imagePublisher;  
	ros::Publisher m_weedDataPublisher;
	ros::Publisher m_weedThresholdPublisher;

	// Service clients
	ros::ServiceClient m_queryWeedClient;

	// For topic names
	std::string m_cameraName;
	std::string m_imageTopic;

	std::string m_imagePublisherName;
	std::string m_weedDataPublisherName;
	std::string m_weedThresholdPublisherName;

	std::string queryWeedServiceName;

	bool m_showWindow;
	bool m_haveFrame;
	uint64_t m_frameNum;

	int m_frameLogInterval;

	PlantDetector* m_detector;

	SpatialMapper* m_spatialMapper;

	VisionParams m_visionParams;

	int maxWeedSizeCm, minWeedSizeCm, defaultWeedSizeCm, defaultCropSizeCm;

	// For performance logging
	ros::WallTime start_, end_;

public:
	ImageConverter(ros::NodeHandle& nodeHandle)
	: m_nodeHandle(nodeHandle), m_imageTransport(nodeHandle), m_haveFrame(false), m_frameNum(0), m_spatialMapper(NULL)
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

	  	m_weedThresholdPublisher = m_nodeHandle.advertise<std_msgs::Float32>(m_weedThresholdPublisherName, 1);

		m_queryWeedClient = m_nodeHandle.serviceClient<urGovernor::FetchWeed>(queryWeedServiceName);

		ROS_INFO("ImageConverter Pipeline started successfully!");

		return true;
	}
 
	void processImage(const sensor_msgs::ImageConstPtr& msg)
	{
		// For calling back to tracker node show current valid weed.
    	urGovernor::FetchWeed queryWeedSrv;
		queryWeedSrv.request.caller = 1; // we don't need this

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

			m_spatialMapper = new SpatialMapper(m_nodeHandle, m_visionParams.frameSize.width, m_visionParams.frameSize.height);

			m_visionParams.defaultWeedThreshold = ((float)defaultWeedSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.defaultCropThreshold = ((float)defaultCropSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.minWeedSize = ((float)minWeedSizeCm ) / m_spatialMapper->sizeScale;
			m_visionParams.maxWeedSize = ((float)maxWeedSizeCm ) / m_spatialMapper->sizeScale;

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

		// Every 10 frames, log processing rate
		if ((m_frameNum % m_frameLogInterval) == 1)
		{
			end_ = ros::WallTime::now();

			double frame_rate_hz = ((double)m_frameLogInterval) / ((end_ - start_).toNSec() * 1e-9);
			ROS_INFO("Processed Frame: %i, running @ %f Hz", (int)m_frameNum, (float)frame_rate_hz);

			start_ = ros::WallTime::now();		
		}

		// msg array to publish
		urVision::weedDataArray weed_msg;	

		// Get the weed list for this frame!
		vector<KeyPoint> weedList = m_detector->getWeedList();
		for (vector<KeyPoint>::iterator it = weedList.begin(); it != weedList.end(); ++it)
		{
			// Populating weedData list to be published
			urVision::weedData weed_data;

			// Do spatial mapping conversion
			if (m_spatialMapper->keypointToReferenceFrame(*it, weed_data))
			{
				weed_msg.weeds.push_back(weed_data);
			}
			else
			{
				ROS_ERROR("Spatial Mapping could not be performed on point (in image) (x,y)=(%f,%f)", it->pt.x, it->pt.y);
			}			
		}

		// Publish weeddaata
		if (weed_msg.weeds.size() > 0)
		{
			weed_msg.header.stamp = ros::Time::now();
			m_weedDataPublisher.publish(weed_msg);
		}

		// Publish weed keypoints drawn on original image
		Mat im_with_keypoints;
		drawKeypoints(m_detector->greenFrame, weedList, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		// Show the current valid weed (next to be harvested) if there is one 
        if (m_queryWeedClient.call(queryWeedSrv))
        {
			KeyPoint nextValid;
			
			// Do spatial mapping conversion
			if (m_spatialMapper->referenceFrameToKeypoint(queryWeedSrv.response.weed, nextValid))
			{
				// Draw red crosshair on next weed to target!
				cv::drawMarker(im_with_keypoints, cv::Point(nextValid.pt.x, nextValid.pt.y),  
								cv::Scalar(0, 0, 255), MARKER_CROSS, nextValid.size*2, 5);
			}
			else
			{
				ROS_ERROR("[REVERSE] Spatial Mapping could not be performed on weed (from tracker) (x,y)=(%f,%f)", queryWeedSrv.response.weed.x_cm, queryWeedSrv.response.weed.y_cm);
			}
		}

		// Publish the output image with keypoints, bounding boxes, etc.
		cv_ptr->image = im_with_keypoints;
		m_imagePublisher.publish(cv_ptr->toImageMsg());

		// Publish the current otsuThreshold
		std_msgs::Float32 otsuThreshold;
		otsuThreshold.data = (float)(m_detector->getWeedThreshold() * m_spatialMapper->sizeScale);
		m_weedThresholdPublisher.publish(otsuThreshold);

		// Done processing this frame
		return;
	}

	// General parameters for this node
	bool readGeneralParameters()
	{
		if (!m_nodeHandle.getParam("camera_name", m_cameraName)) return false;
		if (!m_nodeHandle.getParam("image_topic", m_imageTopic)) return false;
		if (!m_nodeHandle.getParam("image_publisher", m_imagePublisherName)) return false;
		if (!m_nodeHandle.getParam("weed_data_publisher", m_weedDataPublisherName)) return false;
		if (!m_nodeHandle.getParam("weed_threshold_publisher", m_weedThresholdPublisherName)) return false;
		if (!m_nodeHandle.getParam("show_img_window", m_showWindow)) return false;

		if (!m_nodeHandle.getParam("frames_log_interval", m_frameLogInterval)) return false;

		if (!m_nodeHandle.getParam("query_weed_service", queryWeedServiceName)) return false;

		return true;
	}

	// Image converter specific parameters
	bool readVisionParameters()
	{
		// Read vision parameters
		if (!m_nodeHandle.getParam("min_weed_size_cm", minWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("max_weed_size_cm", maxWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("default_weed_size_threshold_cm", defaultWeedSizeCm)) return false;
		if (!m_nodeHandle.getParam("default_crop_size_threshold_cm", defaultCropSizeCm)) return false;

		if (!m_nodeHandle.getParam("min_accumulator_size", m_visionParams.minAccumulatorSize)) return false;
		if (!m_nodeHandle.getParam("max_accumulator_size", m_visionParams.maxAccumulatorSize)) return false;

		if (!m_nodeHandle.getParam("blur_kernel_size", m_visionParams.blurSize)) return false;
		if (!m_nodeHandle.getParam("low_hue", m_visionParams.lowH)) return false;
		if (!m_nodeHandle.getParam("low_sat", m_visionParams.lowS)) return false;
		if (!m_nodeHandle.getParam("low_value", m_visionParams.lowV)) return false;
		if (!m_nodeHandle.getParam("high_hue", m_visionParams.highH)) return false;
		if (!m_nodeHandle.getParam("morph_size", m_visionParams.morphSize)) return false;
		if (!m_nodeHandle.getParam("morph_closing_iterations", m_visionParams.morphClosingIters)) return false;
		if (!m_nodeHandle.getParam("morph_opening_iterations", m_visionParams.morphOpeningIters)) return false;
		
		if (!m_nodeHandle.getParam("min_circularity", m_visionParams.minCircularity)) return false;
		if (!m_nodeHandle.getParam("min_convexity", m_visionParams.minConvexity)) return false;
		if (!m_nodeHandle.getParam("min_inertia_ratio", m_visionParams.minInertiaRatio)) return false;

		return true;
	}

};
